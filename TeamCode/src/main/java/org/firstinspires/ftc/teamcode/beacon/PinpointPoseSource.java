package org.firstinspires.ftc.teamcode.beacon;

import android.content.Context;
import android.os.SystemClock;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A {@link PoseSource} backed by a goBILDA Pinpoint odometry computer (two odometry pods plus
 * its own IMU). It reports the ROBOT's planar pose directly ({@link #reportsRobotPose()} is
 * true), so {@code T_robot_cam} plays no part.
 *
 * <h3>Units and conventions (from the SDK 12.0 driver source, GoBildaPinpointDriver)</h3>
 * The driver keeps position in millimetres and heading in radians (unnormalized; it keeps
 * counting turns); {@code getPosX/getPosY(DistanceUnit)} convert, {@code getHeading(AngleUnit)}
 * wraps to +/-180 degrees. Frame: x forward, y left (the Y pod must count up strafing LEFT),
 * heading counter-clockwise positive, origin wherever {@code resetPosAndIMU()} was called.
 * Pod offsets: X pod offset = how far LEFT of the tracking point the forward pod is; Y pod
 * offset = how far FORWARD the strafe pod is. The calibration OpMode checks every one of these
 * signs on the robot, because a doc that says it is not a robot that shows it.
 *
 * <h3>Status mapping</h3>
 * <ul>
 *   <li>READY: TRACKING sample.</li>
 *   <li>NOT_READY, CALIBRATING: INITIALIZING (the IMU is finding its zero; the robot must be
 *       still). If either appears AFTER the device was READY, the Pinpoint rebooted (a
 *       brown-out): its position restarted at 0,0,0 and it lost the offsets and directions we
 *       sent. When it is READY again this class re-sends the configuration and increments the
 *       world epoch, so the localizer continues from its last pose instead of jumping.</li>
 *   <li>FAULT_BAD_READ: the driver detected a corrupt I2C read and repeated the last good
 *       values. No new sample is published (the values are stale), the read is counted; if
 *       bad reads continue, the localizer's source timeout reports LOST.</li>
 *   <li>Pod faults, IMU runaway, an exception from the I2C read: LOST sample.</li>
 * </ul>
 * A READY sample that moved faster than any FTC robot can (position or heading) since the
 * previous one is treated as a world reset too (epoch + 1).
 *
 * <h3>Cost and timing</h3>
 * {@link #poll()} does the one bulk I2C read (about 1 to 3 ms on a Control Hub) and stamps
 * the sample at the middle of that read on {@code SystemClock.elapsedRealtimeNanos()}.
 * {@link #latest()} is a volatile read.
 */
public final class PinpointPoseSource implements PoseSource {

    /** What this class needs from the Pinpoint; lets tests run without hardware. */
    public interface Reader {
        /** One bulk read of the device. May throw on an I2C failure. */
        void update();
        /** Name of the driver's DeviceStatus, e.g. "READY", "CALIBRATING". */
        String statusName();
        /** Robot x in metres (forward at reset). */
        double xMetres();
        /** Robot y in metres (left at reset). */
        double yMetres();
        /** Robot heading in radians, counter-clockwise positive (any wrapping). */
        double headingRadians();
        /** The Pinpoint's own loop rate in Hz (diagnostic; about 1500 is healthy). */
        double loopHz();
        /** Re-sends pod offsets, encoder resolution and directions (after a reboot). */
        void reconfigure();
    }

    /** Nanosecond clock; the production one is {@code SystemClock.elapsedRealtimeNanos()}. */
    public interface TimeSource {
        long nowNs();
    }

    /** No FTC robot moves this fast; a bigger step between reads is a reset or garbage. */
    static final double MAX_PLAUSIBLE_SPEED_MPS = 5.0;
    /** Same for turning (rad/s). The driver's own corruption check allows 20 turns/s. */
    static final double MAX_PLAUSIBLE_OMEGA_RADPS = 25.0;
    /** Slack added to the plausibility limits so read jitter never trips them. */
    static final double JUMP_SLACK_M = 0.05;
    static final double JUMP_SLACK_RAD = 0.2;

    static final int K_READY = 0, K_WARMING = 1, K_BAD_READ = 2, K_FAULT = 3;

    private final Reader reader;
    private final TimeSource time;
    private volatile Pose3dStamped latest =
            new Pose3dStamped(0, 0, 0, 0, 0, 0, 1, 0, BeaconStatus.INVALID);
    private long lastT;
    private volatile boolean started;

    private int epoch;
    private boolean everReady;
    private boolean rebootSuspected;
    private boolean havePrev;
    private double prevX, prevY, prevH;
    private long prevT;

    private volatile String statusName = "NOT STARTED";
    private volatile String lastError = "";
    private volatile double loopHz;
    private volatile long reads, badReads, faults, exceptions, reboots, jumps;

    public PinpointPoseSource(Reader reader, TimeSource time) {
        this.reader = reader;
        this.time = time;
    }

    /**
     * Production constructor around a configured driver. Call {@link #configure} (or let
     * {@link BeaconRig} do it) before starting.
     */
    public PinpointPoseSource(GoBildaPinpointDriver driver, BeaconHardwareConfig hc) {
        this(new DriverReader(driver, hc), new TimeSource() {
            @Override
            public long nowNs() {
                return SystemClock.elapsedRealtimeNanos();
            }
        });
    }

    /** Maps a driver DeviceStatus name to one of the K_ kinds. Package-private for tests. */
    static int classify(String status) {
        if ("READY".equals(status)) {
            return K_READY;
        }
        if ("NOT_READY".equals(status) || "CALIBRATING".equals(status)) {
            return K_WARMING;
        }
        if ("FAULT_BAD_READ".equals(status)) {
            return K_BAD_READ;
        }
        return K_FAULT;     // pods missing, IMU runaway, unknown
    }

    @Override
    public void start(Context ctx, BeaconConfig cfg) throws BeaconException {
        started = true;
    }

    @Override
    public void stop() {
        started = false;
    }

    @Override
    public boolean reportsRobotPose() {
        return true;
    }

    @Override
    public void poll() {
        if (!started) {
            return;
        }
        long t0 = time.nowNs();
        try {
            reader.update();
        } catch (RuntimeException e) {
            exceptions++;
            lastError = "I2C read failed: " + e;
            statusName = "READ EXCEPTION";
            publish(stamp(t0, time.nowNs()), BeaconStatus.LOST, 0, 0, 0);
            return;
        }
        long t = stamp(t0, time.nowNs());
        reads++;
        String st = reader.statusName();
        statusName = st;
        loopHz = reader.loopHz();
        switch (classify(st)) {
            case K_READY:
                handleReady(t);
                break;
            case K_WARMING:
                if (everReady) {
                    rebootSuspected = true;   // READY before, warming now: it restarted
                }
                havePrev = false;
                publish(t, BeaconStatus.INITIALIZING, 0, 0, 0);
                break;
            case K_BAD_READ:
                badReads++;   // values are a copy of the last good read: publish nothing
                break;
            default:
                faults++;
                lastError = "Pinpoint fault: " + st;
                havePrev = false;
                publish(t, BeaconStatus.LOST, 0, 0, 0);
                break;
        }
    }

    private void handleReady(long t) {
        double x = reader.xMetres();
        double y = reader.yMetres();
        double h = reader.headingRadians();
        if (!fin(x) || !fin(y) || !fin(h)) {
            badReads++;
            return;
        }
        everReady = true;
        if (rebootSuspected) {
            rebootSuspected = false;
            reboots++;
            epoch++;
            lastError = "Pinpoint restarted (power?); configuration re-sent, pose continued";
            try {
                reader.reconfigure();
            } catch (RuntimeException e) {
                lastError = "Pinpoint restarted and reconfigure failed: " + e;
            }
        } else if (havePrev) {
            double dt = (t - prevT) * 1e-9;
            double step = Math.hypot(x - prevX, y - prevY);
            double turn = Math.abs(FrameMath.angleDiff(h, prevH));
            if (step > MAX_PLAUSIBLE_SPEED_MPS * dt + JUMP_SLACK_M
                    || turn > MAX_PLAUSIBLE_OMEGA_RADPS * dt + JUMP_SLACK_RAD) {
                jumps++;
                epoch++;
                lastError = String.format("Pinpoint pose jumped %.2f m / %.2f rad in one read",
                        step, turn);
            }
        }
        havePrev = true;
        prevX = x;
        prevY = y;
        prevH = h;
        prevT = t;
        publish(t, BeaconStatus.TRACKING, x, y, h);
    }

    private long stamp(long t0, long t1) {
        long t = t0 + (t1 - t0) / 2;     // the read happened between t0 and t1
        if (t <= lastT) {
            t = lastT + 1;               // contract: strictly increasing
        }
        lastT = t;
        return t;
    }

    private void publish(long t, int status, double x, double y, double h) {
        latest = new Pose3dStamped(x, y, 0, 0, 0, Math.sin(0.5 * h), Math.cos(0.5 * h), t,
                status, epoch);
    }

    @Override
    public Pose3dStamped latest() {
        return latest;
    }

    @Override
    public boolean isTracking() {
        return latest.status == BeaconStatus.TRACKING;
    }

    /** Driver status name of the last read ("READY", "CALIBRATING", ...). */
    public String getStatusName() {
        return statusName;
    }

    public boolean isReady() {
        return started && "READY".equals(statusName);
    }

    public long getBadReads() {
        return badReads;
    }

    public long getReboots() {
        return reboots;
    }

    public long getJumps() {
        return jumps;
    }

    public long getExceptions() {
        return exceptions;
    }

    public double getLoopHz() {
        return loopHz;
    }

    public String getLastError() {
        return lastError;
    }

    @Override
    public String debugLine() {
        StringBuilder sb = new StringBuilder(96);
        sb.append("pinpoint ").append(statusName)
                .append(String.format(" %.0fHz", loopHz))
                .append(" reads=").append(reads);
        if (badReads > 0) {
            sb.append(" bad=").append(badReads);
        }
        if (faults > 0 || exceptions > 0) {
            sb.append(" faults=").append(faults).append(" exc=").append(exceptions);
        }
        if (reboots > 0 || jumps > 0) {
            sb.append(" reboots=").append(reboots).append(" jumps=").append(jumps);
        }
        if (!lastError.isEmpty()) {
            sb.append(" | ").append(lastError);
        }
        return sb.toString();
    }

    /**
     * Raw Pinpoint values straight from the driver (cached from the last read; no I2C), in
     * inches and degrees, plus the raw pod encoder counts. Empty when not a real driver.
     * Compare with the Beacon pose to tell a pod, IMU or configuration problem apart.
     */
    public String rawLine() {
        return reader instanceof DriverReader ? ((DriverReader) reader).raw() : "";
    }

    private static boolean fin(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    // ------------------------------------------------------------ real driver

    /** Adapter around the SDK driver. Loaded only on the robot. */
    static final class DriverReader implements Reader {
        private final GoBildaPinpointDriver d;
        private final BeaconHardwareConfig hc;

        DriverReader(GoBildaPinpointDriver d, BeaconHardwareConfig hc) {
            this.d = d;
            this.hc = hc;
        }

        @Override
        public void update() {
            d.update();
        }

        @Override
        public String statusName() {
            GoBildaPinpointDriver.DeviceStatus s = d.getDeviceStatus();
            return s == null ? "NULL" : s.name();
        }

        @Override
        public double xMetres() {
            return d.getPosX(DistanceUnit.METER);
        }

        @Override
        public double yMetres() {
            return d.getPosY(DistanceUnit.METER);
        }

        @Override
        public double headingRadians() {
            return d.getHeading(AngleUnit.RADIANS);
        }

        @Override
        public double loopHz() {
            return d.getFrequency();
        }

        String raw() {
            return String.format("x %.2f in  y %.2f in  h %.1f deg | encX %d  encY %d | "
                            + "yawScalar %.4f",
                    d.getPosX(DistanceUnit.INCH), d.getPosY(DistanceUnit.INCH),
                    d.getHeading(AngleUnit.DEGREES), d.getEncoderX(), d.getEncoderY(),
                    d.getYawScalar());
        }

        /** Sends offsets, resolution, directions and (if set) the yaw scalar. I2C writes. */
        @Override
        public void reconfigure() {
            d.setOffsets(hc.xPodOffsetMm, hc.yPodOffsetMm, DistanceUnit.MM);
            if (hc.customTicksPerMm > 0) {
                d.setEncoderResolution(hc.customTicksPerMm, DistanceUnit.MM);
            } else {
                d.setEncoderResolution(hc.pods);
            }
            d.setEncoderDirections(hc.xDirection, hc.yDirection);
            if (hc.yawScalar > 0) {
                d.setYawScalar(hc.yawScalar);
            }
        }
    }
}
