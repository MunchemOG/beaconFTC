package org.firstinspires.ftc.teamcode.beacon.vision;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconException;
import org.firstinspires.ftc.teamcode.beacon.BeaconLocalizer;
import org.firstinspires.ftc.teamcode.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.beacon.FieldFix;
import org.firstinspires.ftc.teamcode.beacon.FieldFixSource;
import org.firstinspires.ftc.teamcode.beacon.Pose2dStamped;
import org.firstinspires.ftc.teamcode.beacon.Pose3dStamped;
import org.firstinspires.ftc.teamcode.beacon.PoseSource;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * Beacon's vision fixes for {@link BeaconLocalizer}: feeds frames and Pinpoint odometry to the
 * native core ({@link BeaconCore}, normally {@link NativeBeacon}) and publishes its fixes as
 * {@link FieldFix}es. Plan task J2.
 *
 * <h3>Threads (the OpMode thread does nothing here except a volatile read in latestFix)</h3>
 * <ul>
 *   <li>"BeaconVisionFeed" (this class): creates the core (reads the field model and the
 *       camera calibration from the Hub's storage), then every {@code vision.odomPollMs}:
 *       pushes the Pinpoint source's newest sample (a volatile read of
 *       {@link PoseSource#latest()}; the OpMode's {@code update()} is never touched), sends
 *       the start pose when needed, and collects a new fix if the core has one.</li>
 *   <li>The frame source's decode thread: {@link FrameSource.Listener#onFrame} pushes each
 *       frame into the core (the core copies it and never blocks; if it is busy the frame is
 *       dropped and counted).</li>
 * </ul>
 * {@code beacon_destroy} must not run while another call is in progress: every call holds the
 * read side of a lock (the frame push only tries it, so it can never block the decode thread)
 * and the core is destroyed under the write side. {@code beacon_destroy} also waits for the
 * frame the core is working on (tens of ms), so {@link #stop()} never calls it on the caller's
 * thread (usually the OpMode thread): it hands the shutdown to a "BeaconVisionStop" thread and
 * returns at once. A later {@link #start()} waits (on its own feeder thread) for that.
 *
 * <h3>Start pose</h3>
 * The core needs the team's start pose in the field frame. It is taken from the localizer
 * (attach it with {@link #attachLocalizer}) whenever the localizer has a new aligned pose that
 * reflects a new {@code setPose} call.
 *
 * <h3>Pinpoint reboot</h3>
 * When the Pinpoint's world epoch changes (it rebooted and restarted at 0, 0, 0) the feeder
 * calls {@code beacon_reset_odom_world(t)} BEFORE pushing the first sample of the new world,
 * on the same thread that pushes odometry. The core then re-anchors the new world to its own
 * last field pose (as the localizer does); no start pose is re-sent.
 *
 * <h3>Field mode only</h3>
 * The core's explore mode (params {@code "mode": "explore"}) publishes poses in its own map
 * frame, not the field frame. Such poses must never reach the localizer, so a params file
 * that selects any mode other than "field" keeps vision OFF, and every fix carrying the
 * bench-only {@code FIX_NO_ODOM} bit is dropped.
 *
 * <h3>Failure is a status, never an exception</h3>
 * Library not loaded, vision disabled, no camera calibration, no field model, the core
 * refusing to start: {@link #latestFix()} stays null (pure Pinpoint) and
 * {@link #debugLine()} says which, starting with "vision: ".
 */
public final class VisionFixSource implements FieldFixSource {

    private final FrameSource frames;
    private final PoseSource odom;
    private final BeaconConfig cfg;
    private final BeaconCore core;
    private final boolean nativeLoaded;
    private final long periodNs;
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();

    private volatile BeaconLocalizer localizer;
    private volatile boolean running;
    private volatile int generation;
    private Thread feeder;
    private Thread stopper;

    private volatile boolean coreRunning;
    private boolean coreCreated, coreDestroyed;   // guarded by the write lock
    private volatile BeaconCore.Camera camera;
    private volatile FieldFix fix;
    private volatile String state;
    private volatile String coreLine = "";
    private volatile int coreStatus = BeaconStatus.INVALID;
    private volatile String lastFrameReject = "";

    // feeder-thread state
    private Pose3dStamped lastOdom;
    private long lastOdomT = Long.MIN_VALUE;
    private int lastEpoch = Integer.MIN_VALUE;
    private int sentSetPoseCount;
    private final long[] fixT = new long[1];
    private final double[] fixV = new double[7];

    // counters
    private volatile long odomPushed, framesPushed, framesBusy, framesRejected, startsSent;
    private volatile long odomResets;
    private volatile long fixesReceived, fixesDropped;

    /** The robot version: native core over JNI. */
    public VisionFixSource(FrameSource frames, PoseSource odom, BeaconConfig cfg) {
        this(frames, odom, cfg, new NativeBeacon(), NativeBeacon.isLoaded(),
                NativeBeacon.loadError());
    }

    /** With any core (tests use a fake; it counts as loaded). */
    public VisionFixSource(FrameSource frames, PoseSource odom, BeaconConfig cfg,
                           BeaconCore core) {
        this(frames, odom, cfg, core, true, null);
    }

    private VisionFixSource(FrameSource frames, PoseSource odom, BeaconConfig cfg,
                            BeaconCore core, boolean loaded, String loadError) {
        this.frames = frames;
        this.odom = odom;
        this.cfg = cfg;
        this.core = core;
        this.nativeLoaded = loaded;
        this.periodNs = (long) (cfg.vision.odomPollMs * 1e6);
        if (!cfg.vision.enabled) {
            state = "vision: disabled in beacon.json (vision.enabled)";
        } else if (!loaded) {
            state = "vision: native library not loaded (" + loadError + "); running on the "
                    + "Pinpoint alone";
        } else {
            state = "vision: not started";
        }
    }

    /** The localizer whose start pose the core should use. Call once, before start(). */
    public void attachLocalizer(BeaconLocalizer loc) {
        localizer = loc;
    }

    public boolean isNativeLoaded() {
        return nativeLoaded;
    }

    /** True once the core was created and until stop(). */
    public boolean isCoreRunning() {
        return coreRunning;
    }

    // ------------------------------------------------------------- lifecycle

    /** Starts the feeder thread (which creates the core). Returns at once. Idempotent. */
    public synchronized void start() {
        if (running || !cfg.vision.enabled || !nativeLoaded) {
            return;
        }
        running = true;
        final int gen = ++generation;
        final Thread previousStop = stopper;
        state = "vision: starting";
        feeder = new Thread(new Runnable() {
            @Override
            public void run() {
                if (previousStop != null) {
                    try {
                        previousStop.join();      // the old core must be gone first
                    } catch (InterruptedException e) {
                        return;
                    }
                }
                feedLoop(gen);
            }
        }, "BeaconVisionFeed");
        feeder.setDaemon(true);
        feeder.start();
    }

    /**
     * Stops feeding and destroys the core on a background thread; returns at once (safe on the
     * OpMode thread). Safe twice and before start.
     */
    public synchronized void stop() {
        boolean wasRunning = running;
        running = false;
        generation++;
        frames.setListener(null);
        final Thread t = feeder;
        feeder = null;
        if (t == null) {
            return;
        }
        t.interrupt();
        coreRunning = false;     // no more calls into the core from here on; destroy follows
        if (wasRunning) {
            state = "vision: stopped";
        }
        stopper = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    t.join(5000);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
                destroyCore();
            }
        }, "BeaconVisionStop");
        stopper.setDaemon(true);
        stopper.start();
    }

    /** Waits up to {@code ms} for a previous {@link #stop()} to finish; true if it has. */
    public boolean awaitStopped(long ms) throws InterruptedException {
        Thread s;
        synchronized (this) {
            s = stopper;
        }
        if (s != null) {
            s.join(ms);
            return !s.isAlive();
        }
        return true;
    }

    private void destroyCore() {
        lock.writeLock().lock();
        try {
            coreRunning = false;
            if (coreCreated && !coreDestroyed) {
                coreDestroyed = true;
                try {
                    core.destroy();
                } catch (RuntimeException e) {
                    state = "vision: core crashed on destroy: " + e;
                }
            }
        } finally {
            lock.writeLock().unlock();
        }
    }

    private boolean alive(int gen) {
        return running && generation == gen;
    }

    // ----------------------------------------------------------------- feeder

    private void feedLoop(int gen) {
        if (!createCore(gen)) {
            return;
        }
        frames.setListener(new FrameSource.Listener() {
            @Override
            public void onFrame(GrayFrame f) {
                pushFrame(f);
            }
        });
        long nextLine = 0;
        while (alive(gen)) {
            long t0 = System.nanoTime();
            lock.readLock().lock();
            try {
                if (!coreRunning) {
                    return;
                }
                feedOdom();
                sendStartPoseIfNeeded();
                collectFix();
                if (t0 >= nextLine) {
                    coreStatus = core.status();
                    coreLine = core.debugLine();
                    nextLine = t0 + 250000000L;
                }
            } catch (RuntimeException e) {
                state = "vision: feeder error: " + e;
            } finally {
                lock.readLock().unlock();
            }
            long sleep = periodNs - (System.nanoTime() - t0);
            if (sleep > 0) {
                try {
                    Thread.sleep(sleep / 1000000L, (int) (sleep % 1000000L));
                } catch (InterruptedException e) {
                    return;
                }
            }
        }
    }

    private boolean createCore(int gen) {
        String fieldJson;
        BeaconCore.Camera cam;
        String params = null;
        try {
            cam = BeaconCore.Camera.parse(read(cfg.vision.cameraFile, "camera calibration"));
        } catch (BeaconException e) {
            state = "vision: OFF, no usable camera calibration: " + e.getMessage();
            return false;
        }
        try {
            fieldJson = read(cfg.vision.fieldFile, "field model");
        } catch (BeaconException e) {
            state = "vision: OFF, no field model: " + e.getMessage();
            return false;
        }
        if (cfg.vision.paramsFile.length() > 0) {
            try {
                params = read(cfg.vision.paramsFile, "core params");
            } catch (BeaconException e) {
                state = "vision: OFF, " + e.getMessage();
                return false;
            }
            String mode = paramsMode(params);
            if (!"field".equals(mode)) {
                state = "vision: OFF, " + cfg.vision.paramsFile + " sets mode \"" + mode
                        + "\"; only \"field\" mode gives field-frame fixes (explore mode "
                        + "poses are in its own map frame)";
                return false;
            }
        }
        camera = cam;
        String[] err = new String[1];
        boolean ok;
        try {
            ok = core.create(fieldJson, cam, cfg.cameraMount, params, err);
        } catch (RuntimeException e) {
            ok = false;
            err[0] = e.toString();
        }
        if (!ok) {
            state = "vision: OFF, core failed to start: " + err[0];
            return false;
        }
        lock.writeLock().lock();
        try {
            coreCreated = true;
            coreDestroyed = false;
            if (!alive(gen)) {
                coreDestroyed = true;     // stop() came while we were creating
                core.destroy();
                return false;
            }
            coreRunning = true;
        } finally {
            lock.writeLock().unlock();
        }
        state = "vision: running";
        return true;
    }

    /** The core's "mode" from a params JSON: "field" when absent; the reason if unreadable. */
    @SuppressWarnings("unchecked")
    static String paramsMode(String paramsJson) {
        Object o;
        try {
            o = org.firstinspires.ftc.teamcode.beacon.MiniJson.parse(paramsJson);
        } catch (BeaconException e) {
            return "unreadable params JSON: " + e.getMessage();
        }
        if (!(o instanceof java.util.Map)) {
            return "params JSON is not an object";
        }
        Object m = ((java.util.Map<String, Object>) o).get("mode");
        if (m == null) {
            return "field";
        }
        return m instanceof String ? (String) m : String.valueOf(m);
    }

    private static String read(String path, String what) throws BeaconException {
        File f = new File(path);
        if (!f.isFile()) {
            throw new BeaconException(what + " not found at " + path);
        }
        InputStream in = null;
        try {
            in = new FileInputStream(f);
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buf = new byte[8192];
            int n;
            while ((n = in.read(buf)) > 0) {
                out.write(buf, 0, n);
                if (out.size() > 16 << 20) {
                    throw new BeaconException(path + " is too large");
                }
            }
            return new String(out.toByteArray(), "UTF-8");
        } catch (IOException e) {
            throw new BeaconException("cannot read " + what + " " + path + ": " + e.getMessage());
        } finally {
            if (in != null) {
                try {
                    in.close();
                } catch (IOException ignored) {
                    // done anyway
                }
            }
        }
    }

    /** Newest Pinpoint sample to the core (raw odometry world, yaw-only quaternion). */
    private void feedOdom() {
        Pose3dStamped s = odom.latest();
        if (s == null || s == lastOdom) {
            return;
        }
        lastOdom = s;
        if (s.status != BeaconStatus.TRACKING || s.tNs <= lastOdomT || !s.isFinite()) {
            return;
        }
        if (lastEpoch != Integer.MIN_VALUE && s.worldEpoch != lastEpoch) {
            core.resetOdomWorld(s.tNs);   // Pinpoint rebooted: new world starts with this sample
            odomResets++;
        }
        lastEpoch = s.worldEpoch;
        lastOdomT = s.tNs;
        double yaw = 2 * Math.atan2(s.qz, s.qw);
        core.pushOdom(s.tNs, s.x, s.y, yaw);
        odomPushed++;
    }

    private void sendStartPoseIfNeeded() {
        BeaconLocalizer loc = localizer;
        if (loc == null) {
            return;
        }
        int count = loc.getPoseSetPoseCount();   // read BEFORE the pose (see its Javadoc)
        Pose2dStamped p = loc.getPose();
        if (count != sentSetPoseCount && loc.isAligned() && p.status == BeaconStatus.TRACKING) {
            core.setStartPose(p.tNs, p.x, p.y, p.heading);
            sentSetPoseCount = count;
            startsSent++;
        }
    }

    private void collectFix() {
        if (!core.latestFix(fixT, fixV)) {
            return;
        }
        fixesReceived++;
        double x = fixV[0], y = fixV[1], h = fixV[2], sxy = fixV[3], sh = fixV[4];
        int kind = (int) fixV[6];
        if (!finite(x) || !finite(y) || !finite(h) || (kind & BeaconCore.FIX_NO_ODOM) != 0
                || fixT[0] <= 0) {
            fixesDropped++;
            return;
        }
        fix = new FieldFix(x, y, h, fixT[0], true, true, kindName(kind), sigma(sxy), sigma(sh));
    }

    private static double sigma(double s) {
        return (finite(s) && s >= 0) ? s : Double.NaN;
    }

    static String kindName(int kind) {
        StringBuilder sb = new StringBuilder();
        if ((kind & BeaconCore.FIX_EDGES) != 0) {
            sb.append("edges");
        }
        if ((kind & BeaconCore.FIX_POINTS) != 0) {
            sb.append(sb.length() > 0 ? "+" : "").append("points");
        }
        if ((kind & BeaconCore.FIX_SEAMS) != 0) {
            sb.append(sb.length() > 0 ? "+" : "").append("seams");
        }
        return sb.length() == 0 ? "vision" : sb.toString();
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    // ----------------------------------------------------------- decode thread

    private void pushFrame(GrayFrame f) {
        BeaconCore.Camera cam = camera;
        if (!coreRunning || cam == null) {
            return;
        }
        if (f.width != cam.width || f.height != cam.height) {
            framesRejected++;
            lastFrameReject = "frame " + f.width + "x" + f.height + " but camera.json is "
                    + cam.width + "x" + cam.height;
            return;
        }
        if (!lock.readLock().tryLock()) {
            return;       // being destroyed: never block the decode thread
        }
        try {
            if (!coreRunning) {
                return;
            }
            int r = core.pushFrame(f.tNs, f.gray, f.width, f.height, f.width);
            if (r == BeaconCore.OK) {
                framesPushed++;
            } else if (r == BeaconCore.DROPPED) {
                framesBusy++;
            } else {
                framesRejected++;
                lastFrameReject = "core rejected a " + f.width + "x" + f.height + " frame";
            }
        } finally {
            lock.readLock().unlock();
        }
    }

    // --------------------------------------------------------------- results

    @Override
    public FieldFix latestFix() {
        return fix;
    }

    /** The core's status (BeaconCore.STATUS_*, equal to BeaconStatus); INVALID when off. */
    public int status() {
        return coreRunning ? coreStatus : BeaconCore.STATUS_INVALID;
    }

    /** Times the core was told the odometry world restarted (Pinpoint reboots). */
    public long getOdomResets() {
        return odomResets;
    }

    public long getOdomPushed() {
        return odomPushed;
    }

    public long getFramesPushed() {
        return framesPushed;
    }

    /** Frames the core dropped because it was still busy with the previous one. */
    public long getFramesBusy() {
        return framesBusy;
    }

    public long getFramesRejected() {
        return framesRejected;
    }

    public long getFixesReceived() {
        return fixesReceived;
    }

    /** Fixes from the core that were not passed on (non-finite, bench-only kind). */
    public long getFixesDropped() {
        return fixesDropped;
    }

    public long getStartPosesSent() {
        return startsSent;
    }

    /** One telemetry line, always starting with "vision: ". */
    public String debugLine() {
        if (!coreRunning) {
            return state;
        }
        StringBuilder sb = new StringBuilder(200);
        sb.append("vision: ").append(BeaconStatus.name(coreStatus))
                .append(" | frames ").append(framesPushed).append(" (busy ").append(framesBusy)
                .append(", rejected ").append(framesRejected).append(") odom ")
                .append(odomPushed).append(" | fixes ").append(fixesReceived);
        if (fixesDropped > 0) {
            sb.append(" (dropped ").append(fixesDropped).append(')');
        }
        if (!lastFrameReject.isEmpty()) {
            sb.append(" | ").append(lastFrameReject);
        }
        if (!coreLine.isEmpty()) {
            sb.append(" | ").append(coreLine);
        }
        return sb.toString();
    }
}
