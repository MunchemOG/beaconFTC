package org.firstinspires.ftc.teamcode.beacon.parked;

import org.firstinspires.ftc.teamcode.beacon.FieldFix;
import org.firstinspires.ftc.teamcode.beacon.FieldFixSource;
import org.firstinspires.ftc.teamcode.beacon.FrameMath;
import org.firstinspires.ftc.teamcode.beacon.PinpointPoseSource;

import android.os.SystemClock;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

/**
 * PARKED (2026-11-17): AprilTag code from the old odometry tier. Beacon uses no AprilTags; this
 * class is not wired into BeaconRig and is kept compiling and tested only so it can be revived.
 *
 * A {@link FieldFixSource} backed by a Limelight 3A running an AprilTag pipeline (MegaTag1 and
 * MegaTag2). What becomes a fix, and why, is in {@link LimelightFixFilter}.
 *
 * <h3>Threads</h3>
 * Everything that talks to the Limelight runs on one background thread
 * ("BeaconLimelight"), never on the OpMode loop, because the SDK's
 * {@code updateRobotOrientation} and {@code pipelineSwitch} are HTTP POSTs with a 15 s read
 * timeout (Limelight3A source). Each cycle (at {@code limelight.headingHz}): switch to the
 * configured pipeline if the results say we are on another one (retried every second); send
 * the localizer's heading if it is TRACKING; read the SDK's cached result (polled by the SDK
 * at 100 Hz on its own thread) and run the filter; publish the fix through a volatile field.
 * {@link #latestFix()} is a volatile read.
 *
 * <h3>Lifecycle</h3>
 * {@link #stop()} is safe twice and before start. It does not wait for a POST that is hung
 * inside the SDK; instead the thread checks a generation number after every call and exits
 * without publishing anything if it belongs to a stopped run, so a thread can never feed an
 * old Limelight into a new OpMode. If the Limelight is unplugged, its last result stays cached
 * in the SDK and its staleness grows; the filter rejects it as stale and the telemetry says so.
 */
public final class LimelightTagSource implements FieldFixSource {

    /** What this class needs from the Limelight; lets tests run without hardware. */
    public interface Device {
        /** The newest cached result as plain numbers, or null. Must not block. */
        LimelightFixFilter.Reading read();
        /** Sends the robot heading (DEGREES, field frame) for MegaTag2. May block. */
        boolean sendHeadingDegrees(double deg);
        /** Asks the Limelight to switch pipeline. May block. */
        boolean switchPipeline(int index);
    }

    /** Supplies the robot's field heading in radians, or NaN if it is not trustworthy. */
    public interface HeadingProvider {
        double headingRadiansOrNaN();
    }

    private static final long PIPELINE_RETRY_NS = 1000000000L;

    private final Device device;
    private final PinpointPoseSource.TimeSource time;
    private final LimelightTagConfig cfg;
    private final LimelightFixFilter filter;
    private final long periodNs;

    private volatile HeadingProvider headingProvider;
    private volatile FieldFix fix;
    private volatile int generation;
    private volatile boolean running;
    private Thread worker;

    // worker-thread state
    private long lastHeadingOkNs = Long.MIN_VALUE;
    private long nextPipelineTryNs = Long.MIN_VALUE;
    private double lastSentDeg = Double.NaN;

    // diagnostics (written by the worker, read by telemetry)
    private volatile String reason = "not started";
    private volatile long headingSends, headingFailures, pipelineSwitches, cycles;
    private volatile double lastStalenessMs = Double.NaN;
    private volatile int lastPipeline = -1;
    private volatile int lastTagCount;
    private volatile double mt2EchoErrDeg = Double.NaN;
    private volatile LimelightFixFilter.Reading lastReading;

    public LimelightTagSource(Device device, PinpointPoseSource.TimeSource time,
                              LimelightTagConfig cfg) {
        this.device = device;
        this.time = time;
        this.cfg = cfg;
        this.filter = new LimelightFixFilter(cfg);
        this.periodNs = (long) (1e9 / cfg.headingHz);
    }

    /** Production constructor around an SDK Limelight (not yet started is fine). */
    public LimelightTagSource(Limelight3A limelight, LimelightTagConfig cfg) {
        this(new SdkDevice(limelight), new PinpointPoseSource.TimeSource() {
            @Override
            public long nowNs() {
                return SystemClock.elapsedRealtimeNanos();
            }
        }, cfg);
    }

    public void setHeadingProvider(HeadingProvider p) {
        headingProvider = p;
    }

    /** Starts the background thread. Safe to call twice. */
    public synchronized void start() {
        if (running) {
            return;
        }
        running = true;
        final int gen = ++generation;
        reason = "starting";
        worker = new Thread(new Runnable() {
            @Override
            public void run() {
                while (running && generation == gen) {
                    long t0 = time.nowNs();
                    try {
                        step(gen);
                    } catch (RuntimeException e) {
                        reason = "Limelight thread error: " + e;
                    }
                    long sleepNs = periodNs - (time.nowNs() - t0);
                    if (sleepNs > 0) {
                        try {
                            Thread.sleep(sleepNs / 1000000L, (int) (sleepNs % 1000000L));
                        } catch (InterruptedException e) {
                            return;
                        }
                    }
                }
            }
        }, "BeaconLimelight");
        worker.setDaemon(true);
        worker.start();
    }

    /** Stops the background thread without waiting for a hung network call. Idempotent. */
    public synchronized void stop() {
        running = false;
        generation++;
        if (worker != null) {
            worker.interrupt();
            worker = null;
        }
        reason = "stopped";
    }

    public boolean isRunning() {
        return running;
    }

    /**
     * One cycle of the background thread. Public for tests, which call it directly instead of
     * starting the thread. {@code gen} must be the current generation or nothing is published.
     */
    public void step(int gen) {
        cycles++;
        long now = time.nowNs();

        LimelightFixFilter.Reading r = device.read();
        if (r != null) {
            lastStalenessMs = r.stalenessMs;
            lastPipeline = r.pipelineIndex;
            lastTagCount = r.tagCount;
        }
        lastReading = r;

        // 1. Pipeline: switch only if a fresh result says we are on the wrong one.
        if (r != null && r.stalenessMs <= cfg.maxStalenessMs && r.pipelineIndex != cfg.pipeline
                && now >= nextPipelineTryNs) {
            nextPipelineTryNs = now + PIPELINE_RETRY_NS;
            device.switchPipeline(cfg.pipeline);
            pipelineSwitches++;
            if (generation != gen) {
                return;
            }
        }

        // 2. Heading for MegaTag2, only while the localizer's heading is trustworthy.
        HeadingProvider hp = headingProvider;
        double h = Double.NaN;
        if (hp != null) {
            try {
                h = hp.headingRadiansOrNaN();
            } catch (RuntimeException e) {
                h = Double.NaN;
            }
        }
        boolean tracking = !Double.isNaN(h) && !Double.isInfinite(h);
        if (generation != gen) {
            return;     // a stopped run must not send the Limelight a stale heading
        }
        if (tracking) {
            double deg = Math.toDegrees(h);
            if (device.sendHeadingDegrees(deg)) {
                lastHeadingOkNs = time.nowNs();
                lastSentDeg = deg;
                headingSends++;
            } else {
                headingFailures++;
            }
            if (generation != gen) {
                return;
            }
        }

        // 3. Turn the newest result into a fix.
        if (r != null && r.hasMt2 && !Double.isNaN(lastSentDeg) && r.valid) {
            mt2EchoErrDeg = Math.toDegrees(FrameMath.angleDiff(r.mt2HeadingRad,
                    Math.toRadians(lastSentDeg)));
        }
        FieldFix f = filter.evaluate(r, now, tracking, lastHeadingOkNs);
        reason = filter.lastReason();
        if (f != null && generation == gen) {
            fix = f;
        }
    }

    /** Current generation, for tests that call {@link #step(int)}. */
    public int generation() {
        return generation;
    }

    @Override
    public FieldFix latestFix() {
        return fix;
    }

    // ------------------------------------------------------------ diagnostics

    /** Why the last result did or did not become a fix. */
    public String getReason() {
        return reason;
    }

    public double getLastStalenessMs() {
        return lastStalenessMs;
    }

    public int getLastPipeline() {
        return lastPipeline;
    }

    public int getLastTagCount() {
        return lastTagCount;
    }

    public long getHeadingSends() {
        return headingSends;
    }

    public long getHeadingFailures() {
        return headingFailures;
    }

    public long getFixesProduced() {
        return filter.producedCount();
    }

    public double getLastLatencyMs() {
        return filter.lastLatencyMs();
    }

    /** The last raw reading (for the calibration OpMode). May be null. */
    public LimelightFixFilter.Reading getLastReading() {
        return lastReading;
    }

    /**
     * MegaTag2's reported yaw minus the heading we last sent, degrees. It should stay near 0.
     * A large or growing value means the Limelight is not using our heading as we think
     * (wrong unit, wrong frame): see the calibration OpMode.
     */
    public double getMt2EchoErrorDeg() {
        return mt2EchoErrDeg;
    }

    public String debugLine() {
        return String.format("LL pipe=%d tags=%d age=%.0fms lat=%.0fms hdg ok/fail=%d/%d "
                        + "fixes=%d | %s", lastPipeline, lastTagCount, lastStalenessMs,
                filter.lastLatencyMs(), headingSends, headingFailures, filter.producedCount(),
                reason);
    }

    // ------------------------------------------------------------ real device

    /** Adapter around the SDK class. Loaded only on the robot. */
    static final class SdkDevice implements Device {
        private final Limelight3A ll;

        SdkDevice(Limelight3A ll) {
            this.ll = ll;
        }

        @Override
        public LimelightFixFilter.Reading read() {
            LLResult res = ll.getLatestResult();
            if (res == null) {
                return null;
            }
            LimelightFixFilter.Reading r = new LimelightFixFilter.Reading();
            r.valid = res.isValid();
            r.pipelineIndex = res.getPipelineIndex();
            r.llTimestamp = res.getTimestamp();
            r.stalenessMs = res.getStaleness();
            r.captureLatencyMs = res.getCaptureLatency();
            r.targetingLatencyMs = res.getTargetingLatency();
            r.tagCount = res.getBotposeTagCount();
            r.avgDistM = res.getBotposeAvgDist();     // metres per the LLResult Javadoc
            Pose3D mt2 = res.getBotpose_MT2();
            if (mt2 != null) {
                Position p = mt2.getPosition().toUnit(DistanceUnit.METER);
                r.hasMt2 = true;
                r.mt2X = p.x;
                r.mt2Y = p.y;
                r.mt2HeadingRad = mt2.getOrientation().getYaw(AngleUnit.RADIANS);
            }
            Pose3D mt1 = res.getBotpose();
            if (mt1 != null) {
                Position p = mt1.getPosition().toUnit(DistanceUnit.METER);
                r.hasMt1 = true;
                r.mt1X = p.x;
                r.mt1Y = p.y;
                r.mt1HeadingRad = mt1.getOrientation().getYaw(AngleUnit.RADIANS);
            }
            return r;
        }

        @Override
        public boolean sendHeadingDegrees(double deg) {
            return ll.updateRobotOrientation(deg);
        }

        @Override
        public boolean switchPipeline(int index) {
            return ll.pipelineSwitch(index);
        }
    }
}
