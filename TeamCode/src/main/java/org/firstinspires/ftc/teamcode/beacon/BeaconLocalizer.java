package org.firstinspires.ftc.teamcode.beacon;

import android.content.Context;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Turns the raw poses of a {@link PoseSource} into a 2-D robot pose in the FIELD frame.
 * Call {@link #update()} once per OpMode loop; read {@link #getPose()},
 * {@link #getVelocity()} and {@link #getStatus()} afterwards. Not thread-safe for writes:
 * {@code update}, {@code setPose}, {@code onStart} belong to the OpMode thread. The getters
 * read volatile immutable objects and may be called from any thread.
 *
 * <h3>Pipeline (per new source sample)</h3>
 * <ol>
 *   <li>The source pose is converted to the robot's centre of rotation in the source world:
 *       for a robot-pose source (odometry) it already is; for a camera source
 *       {@code T_sw_robot = T_sw_cam * inverse(T_robot_cam)}.</li>
 *   <li>Project to the plane: x, y, and heading = angle of the robot's +x axis projected on
 *       the horizontal plane.</li>
 *   <li>Apply the planar field alignment {@code (ax, ay, ayaw)}:
 *       {@code field = Rot(ayaw) * sw + (ax, ay)}, {@code heading = sw_yaw + ayaw}. It is set
 *       by {@link #setPose} or by the first full field fix.</li>
 *   <li>A fresh field fix ({@link FieldFix}, from Beacon's vision) adjusts the ALIGNMENT,
 *       never the output directly. It is compared
 *       with the pose the localizer had AT THE FIX'S CAPTURE TIME (interpolated history), so
 *       camera latency does not become a position error. The heading error (if the fix
 *       carries heading) is removed by rotating the alignment about the robot's position at
 *       the fix time, then the position error (if it carries position) by translating it.
 *       Gains: for a fix that carries its uncertainty (sigma), {@code p^2 / (p^2 + sigma^2)}
 *       with p = {@code fix.priorSigmaXYM} / {@code fix.priorSigmaHeadingRad}, so a precise fix
 *       moves the pose more than a vague one; for a fix without sigma, {@code fix.gainXY} /
 *       {@code fix.gainHeading}. The correction persists, so later motion continues
 *       smoothly from the corrected frame.</li>
 * </ol>
 *
 * <h3>Status</h3>
 * TRACKING only when the source is tracking, its sample was valid and the alignment is set.
 * INITIALIZING until then. LOST when the source says so, when no new sample has arrived for
 * {@code sourceTimeoutS}, or when a sample was rejected. While not TRACKING the last good
 * pose is held (stamped with the time of that good sample), the velocity reads zero, and
 * nothing non-finite is ever published.
 *
 * <h3>Fix rules</h3>
 * Rejected: fixes older than the retained history ({@code fix.historyS}, sized by TIME so it
 * covers the vision latency at any Pinpoint rate up to 4 kHz); fixes stamped more than
 * {@code sourceTimeoutS} ahead of the newest sample (a clock mix-up; this does NOT block later
 * fixes; a fix only slightly ahead is compared with the newest sample); fixes taken while the
 * robot moved faster than {@code fix.maxSpeedMps} or turned faster than
 * {@code fix.maxOmegaRadPs} (measured over +/- 20 ms around the capture time); fixes further
 * than {@code fix.maxJumpM} / {@code fix.maxJumpRad} from the pose at that time. The jump
 * test is waived, with gain 1,
 * only (a) for the fix that first sets the alignment and (b) for the first fix captured within
 * {@code fix.recoveryWindowS} after tracking resumed or the source world reset.
 *
 * <h3>Source discontinuities</h3>
 * A change of {@link Pose3dStamped#worldEpoch} means the source's world restarted. The
 * localizer then re-aligns so the first new sample continues from the last output
 * (extrapolated with the last velocity across short gaps) and opens the recovery window. If
 * samples keep arriving with earlier timestamps for longer than {@code sourceTimeoutS}, the
 * source's time base is accepted as changed: history is cleared and the window opens.
 *
 * <h3>Match start (rule G304: motionless after INIT)</h3>
 * Call {@link #setPose} during INIT with the start pose and {@link #onStart} when START is
 * pressed. If the source is tracking at START, the start pose is re-asserted against the
 * newest sample (the robot has not moved since INIT, so this removes any drift accumulated
 * while waiting). If the source is not yet tracking at START, the start pose EXPIRES: when
 * tracking begins later the robot may already have moved, so the localizer stays
 * INITIALIZING until a full field fix sets the alignment.
 */
public final class BeaconLocalizer {

    /** Time source for the "source has gone quiet" checks; replaceable in tests. */
    public interface Clock {
        long nanoTime();
    }

    private static final Clock SYSTEM_CLOCK = new Clock() {
        @Override
        public long nanoTime() {
            return System.nanoTime();
        }
    };

    /** Highest source sample rate the time-sized history is dimensioned for (Hz). */
    static final double MAX_SAMPLE_HZ = 4000;
    /** Speed / turn rate at a fix's time are measured over +/- this window. */
    private static final long MOTION_WINDOW_NS = 20000000L;
    /** Largest accepted deviation of |q| from 1. */
    private static final double QUAT_NORM_TOL = 0.01;
    /** Coordinates beyond this (metres) are garbage, not a robot on a 3.6 m field. */
    private static final double MAX_COORD_M = 1e4;

    // source sample states
    private static final int ST_NONE = 0;      // nothing received yet
    private static final int ST_OK = 1;
    private static final int ST_SRC_INIT = 2;
    private static final int ST_SRC_LOST = 3;
    private static final int ST_SRC_INVALID = 4;
    private static final int ST_BAD = 5;       // rejected sample

    private final PoseSource source;
    private final BeaconConfig cfg;
    private final FieldFixSource fixes;
    private final PoseLogWriter log;
    private final Clock clock;
    private final long timeoutNs;
    private final long recoveryNs;
    private final long historyNs;

    // precomputed from T_robot_cam (identity for robot-pose sources)
    private final double pcrX, pcrY, pcrZ;
    private final double c0x, c0y, c0z;

    private final double[] m = new double[9];

    // source tracking
    private Pose3dStamped lastSeen;
    private long lastTNs = Long.MIN_VALUE;      // newest sample time seen (any status)
    private long lastGoodTNs = 0;               // newest TRACKING sample used
    private long lastNewLocalNs;
    private int state = ST_NONE;
    private boolean swValid;
    private double swX, swY, swYaw, swZ;
    private boolean backwardsActive;
    private long backwardsSinceLocalNs;
    private boolean haveEpoch;
    private int lastGoodEpoch;

    // recovery window
    private boolean gapSinceGood;   // a non-tracking or bad sample happened while aligned
    private boolean needRecover;
    private long recoverUntilTNs = Long.MIN_VALUE;

    // history of good samples in the source world (newest at ringHead-1); sized from
    // fix.historyS so it spans that much time at up to MAX_SAMPLE_HZ
    private final int ring;
    private final long[] rt;
    private final double[] rx;
    private final double[] ry;
    private final double[] ryaw;
    private int ringHead;
    private int ringCount;

    // field alignment
    private volatile boolean alignSet;
    private double ax, ay, ayaw, cosA = 1, sinA = 0;
    private boolean pendingSet;
    private double pendX, pendY, pendH;
    private long pendingExpiresAfterTNs = Long.MAX_VALUE;
    private boolean motionStarted;
    private boolean startPoseHeld;
    private boolean startPoseExpired;
    private double startX, startY, startH;

    // output
    private double outX, outY, outH;
    private boolean dirty = true;
    private long logTNs;
    private volatile Pose2dStamped pose = new Pose2dStamped(0, 0, 0, 0, BeaconStatus.INITIALIZING);
    private volatile Velocity2d velocity = Velocity2d.ZERO;
    private int publishedStatus = BeaconStatus.INITIALIZING;

    // field fixes
    private volatile boolean fixBlend;
    private FieldFix lastFixObj;
    private long lastFixTNs = Long.MIN_VALUE;
    private double hX, hY, hYaw;        // scratch: history lookup result
    private double mSpeed, mOmega;      // scratch: motion at the fix time

    // diagnostics
    private long accepted, rejected, fixesApplied, fixesRejected;
    private long worldResets, timeBaseResets, startReasserts;
    private String lastReject = "";
    private String lastFixReject = "";
    private String lastFixKind = "";
    private String startError;
    private volatile int setPoseCount;
    private volatile int poseSetPoseCount;
    private volatile double lastCorrM, lastCorrRad;

    public BeaconLocalizer(PoseSource source, BeaconConfig cfg) {
        this(source, cfg, null, null, SYSTEM_CLOCK);
    }

    /**
     * @param fixes optional field fix source (null = no correction, pure odometry)
     * @param log  optional log writer (null = no logging); started by {@link #start}
     */
    public BeaconLocalizer(PoseSource source, BeaconConfig cfg, FieldFixSource fixes,
                           PoseLogWriter log) {
        this(source, cfg, fixes, log, SYSTEM_CLOCK);
    }

    public BeaconLocalizer(PoseSource source, BeaconConfig cfg, FieldFixSource fixes,
                           PoseLogWriter log, Clock clock) {
        if (source == null || cfg == null) {
            throw new IllegalArgumentException("source and cfg are required");
        }
        this.source = source;
        this.cfg = cfg;
        this.fixes = fixes;
        this.log = log;
        this.clock = clock;
        this.timeoutNs = (long) (cfg.sourceTimeoutS * 1e9);
        this.recoveryNs = (long) (cfg.fix.recoveryWindowS * 1e9);
        this.historyNs = (long) (cfg.fix.historyS * 1e9);
        this.ring = (int) Math.ceil(cfg.fix.historyS * MAX_SAMPLE_HZ) + 16;
        this.rt = new long[ring];
        this.rx = new double[ring];
        this.ry = new double[ring];
        this.ryaw = new double[ring];
        this.fixBlend = cfg.fix.enabled;

        double[] tRobotCam = source.reportsRobotPose() ? FrameMath.identity()
                : cfg.cameraMount.toTransform();
        double[] tCamRobot = new double[7];
        FrameMath.invert(tRobotCam, tCamRobot);
        pcrX = tCamRobot[0];
        pcrY = tCamRobot[1];
        pcrZ = tCamRobot[2];
        double[] rcr = new double[9];
        FrameMath.quatToMatrix(tCamRobot[3], tCamRobot[4], tCamRobot[5], tCamRobot[6], rcr);
        c0x = rcr[0];   // column 0 of R_cam_robot: robot +x axis in the source body frame
        c0y = rcr[3];
        c0z = rcr[6];
    }

    // ------------------------------------------------------------- lifecycle

    /**
     * Starts the log writer and the pose source. Never throws: on failure it records the
     * message ({@link #getStartError()}), reports INVALID and returns false.
     */
    public boolean start(Context ctx) {
        startError = null;
        if (log != null && !log.isRunning() && !log.start()) {
            startError = "log: " + log.getError();   // logging failure alone is not fatal
        }
        try {
            source.start(ctx, cfg);
            return startError == null;
        } catch (BeaconException e) {
            startError = e.getMessage();
        } catch (RuntimeException e) {
            startError = "pose source crashed on start: " + e;
        }
        dirty = true;
        state = ST_SRC_INVALID;
        return false;
    }

    /** Stops the source and closes the log. Safe to call twice. */
    public void stop() {
        try {
            source.stop();
        } catch (RuntimeException e) {
            startError = "pose source crashed on stop: " + e;
        }
        if (log != null) {
            log.close();
        }
    }

    // ---------------------------------------------------------------- setPose

    /**
     * Declares where the robot is right now in the field frame (metres, radians CCW).
     * Non-finite arguments are ignored (and counted as a rejection). Before START, if the
     * source has no usable sample yet, the pose is applied as soon as one arrives (see the
     * class Javadoc for what happens at START). After START it applies only if the source is
     * tracking now.
     */
    public void setPose(double xMetres, double yMetres, double headingRadians) {
        if (!finite(xMetres) || !finite(yMetres) || !finite(headingRadians)
                || Math.abs(xMetres) > MAX_COORD_M || Math.abs(yMetres) > MAX_COORD_M) {
            rejected++;
            lastReject = "setPose with non-finite or absurd argument";
            return;
        }
        double h = FrameMath.wrapAngle(headingRadians);
        setPoseCount++;
        outX = xMetres;
        outY = yMetres;
        outH = h;
        startX = xMetres;
        startY = yMetres;
        startH = h;
        startPoseHeld = true;
        startPoseExpired = false;
        if (swValid && state == ST_OK) {
            alignTo(swX, swY, swYaw, xMetres, yMetres, h);
            pendingSet = false;
            needRecover = false;
        } else {
            pendingSet = true;
            pendX = xMetres;
            pendY = yMetres;
            pendH = h;
            pendingExpiresAfterTNs = motionStarted ? lastTNs : Long.MAX_VALUE;
        }
        dirty = true;
    }

    /** Same as {@link #setPose(double, double, double)} using the SDK's pose class. */
    public void setPose(Pose2D p) {
        setPose(p.getX(DistanceUnit.METER), p.getY(DistanceUnit.METER),
                p.getHeading(AngleUnit.RADIANS));
    }

    /**
     * Call once when START is pressed (right after {@code waitForStart()}). From here on the
     * robot may move. See the class Javadoc ("Match start").
     */
    public void onStart() {
        motionStarted = true;
        if (pendingSet) {
            // Valid only for samples up to now; any later first sample may be after motion.
            pendingExpiresAfterTNs = lastTNs;
        } else if (startPoseHeld && alignSet && swValid && state == ST_OK) {
            alignTo(swX, swY, swYaw, startX, startY, startH);
            startReasserts++;
        }
        startPoseHeld = false;
    }

    // ----------------------------------------------------------------- update

    /**
     * Polls the source, reads the newest sample (and field fix), updates the pose and logs one
     * row if a new sample arrived. Never throws. Apart from the source's own device read,
     * cheap (no I/O, allocation only when a new pose is published).
     * @return the current pose, same as {@link #getPose()}
     */
    public Pose2dStamped update() {
        long now = clock.nanoTime();
        try {
            source.poll();
            Pose3dStamped s = source.latest();
            if (s != null && s != lastSeen) {
                lastSeen = s;
                if (processSample(s, now)) {
                    dirty = true;
                }
            }
        } catch (RuntimeException e) {
            // A misbehaving source must not take down the OpMode loop.
            rejected++;
            lastReject = "pose source threw: " + e;
            state = ST_BAD;
            if (alignSet) {
                gapSinceGood = true;
            }
            dirty = true;
        }
        if (fixes != null) {
            try {
                processFix();
            } catch (RuntimeException e) {
                // Only the fix is lost; the pose is fine (review finding 4).
                fixRejected("fix source threw: " + e);
            }
        }
        if (logTNs > 0) {
            if (log != null) {
                double h = outH;
                log.offer(logTNs, outX, outY, swZ, 0, 0, Math.sin(0.5 * h), Math.cos(0.5 * h),
                        alignSet ? BeaconStatus.TRACKING : BeaconStatus.INITIALIZING);
            }
            logTNs = 0;
        }
        return publish(now);
    }

    private boolean processSample(Pose3dStamped s, long now) {
        if (s.tNs <= 0 || s.tNs == lastTNs) {
            return false;   // placeholder or repeat of the same sample
        }
        if (s.tNs < lastTNs) {
            if (!backwardsActive) {
                backwardsActive = true;
                backwardsSinceLocalNs = now;
            }
            if (now - backwardsSinceLocalNs <= timeoutNs) {
                reject("source time went backwards");
                state = ST_BAD;
                return true;
            }
            // Backwards for longer than the timeout: the source's clock changed (or one
            // earlier sample was stamped in the future). Accept the new time base.
            timeBaseResets++;
            lastReject = "source time base changed; history cleared";
            clearHistory();
            lastFixTNs = Long.MIN_VALUE;
            lastFixObj = null;
            if (alignSet) {
                gapSinceGood = true;
            }
        }
        backwardsActive = false;
        lastTNs = s.tNs;
        lastNewLocalNs = now;

        if (s.status != BeaconStatus.TRACKING) {
            state = (s.status == BeaconStatus.INITIALIZING) ? ST_SRC_INIT
                    : (s.status == BeaconStatus.LOST ? ST_SRC_LOST : ST_SRC_INVALID);
            if (swValid) {
                clearHistory();
            }
            if (alignSet) {
                gapSinceGood = true;
            }
            return true;
        }
        if (!computeRobotInSourceWorld(s)) {
            state = ST_BAD;
            if (alignSet) {
                gapSinceGood = true;
            }
            return true;
        }
        state = ST_OK;
        accepted++;

        if (haveEpoch && s.worldEpoch != lastGoodEpoch) {
            worldResets++;
            lastReject = "source world reset (epoch " + lastGoodEpoch + " -> " + s.worldEpoch
                    + "); continuing from the last pose";
            Velocity2d v = velocity;
            clearHistory();
            if (alignSet && !pendingSet) {
                double px = outX, py = outY, ph = outH;
                double dt = (s.tNs - lastGoodTNs) * 1e-9;
                if (dt > 0 && dt <= cfg.sourceTimeoutS) {
                    double c = Math.cos(outH), sn = Math.sin(outH);
                    px += (c * v.vx - sn * v.vy) * dt;
                    py += (sn * v.vx + c * v.vy) * dt;
                    ph += v.omega * dt;
                }
                alignTo(swX, swY, swYaw, px, py, ph);
                gapSinceGood = true;
            }
        }
        haveEpoch = true;
        lastGoodEpoch = s.worldEpoch;

        if (gapSinceGood) {
            gapSinceGood = false;
            needRecover = true;
            recoverUntilTNs = s.tNs + recoveryNs;
        } else if (needRecover && s.tNs > recoverUntilTNs) {
            needRecover = false;    // window over: back to the normal jump test (finding 1)
        }

        swValid = true;
        pushHistory(s.tNs);
        lastGoodTNs = s.tNs;
        updateVelocity(s.tNs);
        if (pendingSet) {
            if (s.tNs > pendingExpiresAfterTNs) {
                pendingSet = false;
                startPoseExpired = true;
                lastReject = "start pose expired: source began tracking after START; "
                        + "waiting for a vision fix";
            } else {
                alignTo(swX, swY, swYaw, pendX, pendY, pendH);
                pendingSet = false;
                needRecover = false;
            }
        }
        if (alignSet) {
            computeOutput();
        }
        logTNs = s.tNs;    // row is written at the end of update(), after any field fix
        return true;
    }

    private void clearHistory() {
        swValid = false;
        ringCount = 0;
        velocity = Velocity2d.ZERO;
    }

    /** Fills swX, swY, swZ, swYaw from a TRACKING sample; false (and a reject) if bad. */
    private boolean computeRobotInSourceWorld(Pose3dStamped s) {
        if (!s.isFinite()) {
            reject("source pose is not finite");
            return false;
        }
        if (Math.abs(s.x) > MAX_COORD_M || Math.abs(s.y) > MAX_COORD_M
                || Math.abs(s.z) > MAX_COORD_M) {
            reject("source position is absurdly large");
            return false;
        }
        double n2 = s.qx * s.qx + s.qy * s.qy + s.qz * s.qz + s.qw * s.qw;
        double n = Math.sqrt(n2);
        if (Math.abs(n - 1.0) > QUAT_NORM_TOL) {
            reject("source quaternion not unit length (|q|=" + n + ")");
            return false;
        }
        double inv = 1.0 / n;
        FrameMath.quatToMatrix(s.qx * inv, s.qy * inv, s.qz * inv, s.qw * inv, m);
        // p_sw_robot = p_sw_body + R_sw_body * p_body_robot
        double x = s.x + m[0] * pcrX + m[1] * pcrY + m[2] * pcrZ;
        double y = s.y + m[3] * pcrX + m[4] * pcrY + m[5] * pcrZ;
        double z = s.z + m[6] * pcrX + m[7] * pcrY + m[8] * pcrZ;
        // robot +x axis in the source world: R_sw_body * (column 0 of R_body_robot)
        double fx = m[0] * c0x + m[1] * c0y + m[2] * c0z;
        double fy = m[3] * c0x + m[4] * c0y + m[5] * c0z;
        if (fx * fx + fy * fy < 1e-12) {
            reject("robot x axis is vertical in the source world (bad T_robot_cam or source)");
            return false;
        }
        swX = x;
        swY = y;
        swZ = z;
        swYaw = Math.atan2(fy, fx);
        return true;
    }

    private void reject(String why) {
        rejected++;
        lastReject = why;
    }

    private void pushHistory(long t) {
        rt[ringHead] = t;
        rx[ringHead] = swX;
        ry[ringHead] = swY;
        ryaw[ringHead] = swYaw;
        ringHead = (ringHead + 1) % ring;
        if (ringCount < ring) {
            ringCount++;
        }
    }

    private int ringIndex(int back) {   // back = 0 is the newest sample
        return (ringHead - 1 - back + 2 * ring) % ring;
    }

    /**
     * Robot-frame velocity from the last three samples in the SOURCE world (finite difference
     * from the oldest to the newest, heading taken from the middle one). Done before the field
     * alignment, so fix corrections cannot create velocity spikes.
     */
    private void updateVelocity(long t) {
        if (ringCount < 2) {
            return;
        }
        int i2 = ringIndex(0);
        int i0 = ringIndex(ringCount >= 3 ? 2 : 1);
        int im = ringIndex(ringCount >= 3 ? 1 : 0);
        double dt = (rt[i2] - rt[i0]) * 1e-9;
        if (!(dt > 0)) {
            return;
        }
        double vsx = (rx[i2] - rx[i0]) / dt;
        double vsy = (ry[i2] - ry[i0]) / dt;
        double omega = FrameMath.angleDiff(ryaw[i2], ryaw[i0]) / dt;
        double c = Math.cos(ryaw[im]);
        double sn = Math.sin(ryaw[im]);
        velocity = new Velocity2d(c * vsx + sn * vsy, -sn * vsx + c * vsy, omega, t);
    }

    // -------------------------------------------------------------- alignment

    /** Chooses the alignment so that source-world pose (sx, sy, syaw) maps to (X, Y, H). */
    private void alignTo(double sx, double sy, double syaw, double X, double Y, double H) {
        ayaw = FrameMath.wrapAngle(H - syaw);
        cosA = Math.cos(ayaw);
        sinA = Math.sin(ayaw);
        ax = X - (cosA * sx - sinA * sy);
        ay = Y - (sinA * sx + cosA * sy);
        alignSet = true;
        outX = X;
        outY = Y;
        outH = FrameMath.wrapAngle(H);
        dirty = true;
    }

    private void computeOutput() {
        outX = cosA * swX - sinA * swY + ax;
        outY = sinA * swX + cosA * swY + ay;
        outH = FrameMath.wrapAngle(swYaw + ayaw);
    }

    // -------------------------------------------------------------- field fix

    private void processFix() {
        FieldFix f = fixes.latestFix();
        if (f == null || f == lastFixObj) {
            return;
        }
        if (f.tNs <= lastFixTNs) {
            lastFixObj = f;     // same detection republished, not new
            return;
        }
        if (!swValid) {
            return;             // cannot compare now; keep it and try again next loop
        }
        long newest = rt[ringIndex(0)];
        if (f.tNs - newest > timeoutNs) {
            // Stamped in the future: a clock mix-up. Reject it WITHOUT advancing lastFixTNs,
            // so correct later fixes still get through (finding 3). A fix only slightly
            // ahead of the newest sample is compared with that newest sample.
            lastFixObj = f;
            fixRejected("fix stamped " + (f.tNs - newest) / 1000000 + " ms after the "
                    + "newest sample (clock mismatch?)");
            return;
        }
        lastFixObj = f;
        lastFixTNs = f.tNs;
        if (!fixBlend) {
            return;
        }
        if ((!f.hasPosition && !f.hasHeading) || !f.isFinite()
                || (f.hasPosition && (Math.abs(f.x) > MAX_COORD_M
                || Math.abs(f.y) > MAX_COORD_M))) {
            fixRejected("fix not valid");
            return;
        }
        if (!lookupHistory(f.tNs)) {
            fixRejected("fix older than history");
            return;
        }
        motionAt(f.tNs);
        if (mSpeed > cfg.fix.maxSpeedMps) {
            fixRejected(String.format("robot moving %.2f m/s at fix time", mSpeed));
            return;
        }
        if (Math.abs(mOmega) > cfg.fix.maxOmegaRadPs) {
            fixRejected(String.format("robot turning %.2f rad/s at fix time", mOmega));
            return;
        }
        applyFix(f);
    }

    /** Source-world robot pose at time t into hX, hY, hYaw. False if t is too old. */
    private boolean lookupHistory(long t) {
        int newest = ringIndex(0);
        if (rt[newest] - t > historyNs) {
            return false;
        }
        if (t >= rt[newest]) {
            hX = rx[newest];
            hY = ry[newest];
            hYaw = ryaw[newest];
            return true;
        }
        for (int back = 1; back < ringCount; back++) {
            int i = ringIndex(back);
            if (rt[i] <= t) {
                int j = ringIndex(back - 1);   // next newer sample
                double frac = (double) (t - rt[i]) / (double) (rt[j] - rt[i]);
                hX = rx[i] + frac * (rx[j] - rx[i]);
                hY = ry[i] + frac * (ry[j] - ry[i]);
                hYaw = ryaw[i] + frac * FrameMath.angleDiff(ryaw[j], ryaw[i]);
                return true;
            }
        }
        return false;
    }

    /**
     * Speed and turn rate of the robot around time t, into mSpeed / mOmega: the change between
     * the newest sample at or before t - 20 ms and the oldest at or after t + 20 ms (clamped to
     * the history). A window in time, not in samples, so the result does not depend on the
     * Pinpoint's rate (at 1.5 kHz three samples span only 2 ms). Zero if there is not enough
     * history.
     */
    private void motionAt(long t) {
        mSpeed = 0;
        mOmega = 0;
        if (ringCount < 2) {
            return;
        }
        int older = ringCount - 1;          // back-index; default: the oldest sample
        int newer = 0;                      // default: the newest sample
        for (int back = 0; back < ringCount; back++) {
            long tb = rt[ringIndex(back)];
            if (tb >= t + MOTION_WINDOW_NS) {
                newer = back;               // keeps moving toward t while still >= t + W
            }
            if (tb <= t - MOTION_WINDOW_NS) {
                older = back;
                break;
            }
        }
        if (older <= newer) {
            return;
        }
        int a = ringIndex(older);
        int b = ringIndex(newer);
        double dt = (rt[b] - rt[a]) * 1e-9;
        if (!(dt > 0)) {
            return;
        }
        mSpeed = Math.hypot(rx[b] - rx[a], ry[b] - ry[a]) / dt;
        mOmega = FrameMath.angleDiff(ryaw[b], ryaw[a]) / dt;
    }

    private void applyFix(FieldFix f) {
        if (!alignSet) {
            if (!(f.hasPosition && f.hasHeading)) {
                fixRejected("cannot set the alignment from a partial fix (" + f.kind + ")");
                return;
            }
            // First fix of the run (or rescue after an expired start pose): take it whole.
            alignTo(hX, hY, hYaw, f.x, f.y, f.heading);
            pendingSet = false;
            startPoseExpired = false;
            needRecover = false;
            startPoseHeld = false;
            fixApplied(f);
            return;
        }
        // Where the localizer thinks the robot was at the fix time, in the field frame.
        double px = cosA * hX - sinA * hY + ax;
        double py = sinA * hX + cosA * hY + ay;
        double ph = FrameMath.wrapAngle(hYaw + ayaw);
        double ex = f.hasPosition ? f.x - px : 0;
        double ey = f.hasPosition ? f.y - py : 0;
        double eh = f.hasHeading ? FrameMath.angleDiff(f.heading, ph) : 0;

        boolean waive = needRecover && f.tNs <= recoverUntilTNs;
        if (needRecover && !waive) {
            needRecover = false;            // window expired before any fix arrived
        }
        double gainXY = waive ? 1.0 : cfg.fix.gainXYFor(f.sigmaXY);
        double gainH = waive ? 1.0 : cfg.fix.gainHeadingFor(f.sigmaHeading);
        double dist = Math.sqrt(ex * ex + ey * ey);
        if (!waive && (dist > cfg.fix.maxJumpM || Math.abs(eh) > cfg.fix.maxJumpRad)) {
            fixRejected(String.format("fix %s jumps %.3f m / %.3f rad", f.kind, dist, eh));
            return;
        }

        // Rotate the alignment about the robot's position at the fix time (position is
        // unchanged by this), then translate by the gained position error.
        double dpsi = gainH * eh;
        double cd = Math.cos(dpsi);
        double sd = Math.sin(dpsi);
        double rx0 = ax - px;
        double ry0 = ay - py;
        ax = cd * rx0 - sd * ry0 + px + gainXY * ex;
        ay = sd * rx0 + cd * ry0 + py + gainXY * ey;
        ayaw = FrameMath.wrapAngle(ayaw + dpsi);
        cosA = Math.cos(ayaw);
        sinA = Math.sin(ayaw);
        lastCorrM = gainXY * dist;
        lastCorrRad = dpsi;
        needRecover = false;
        startPoseHeld = false;
        fixApplied(f);
    }

    private void fixApplied(FieldFix f) {
        fixesApplied++;
        lastFixKind = f.kind;
        computeOutput();    // from the newest sample, so there is no extra jump
        dirty = true;
    }

    private void fixRejected(String why) {
        fixesRejected++;
        lastFixReject = why;
    }

    // ---------------------------------------------------------------- publish

    private Pose2dStamped publish(long now) {
        int st = computeStatus(now);
        if (dirty || st != publishedStatus) {
            if (!finite(outX) || !finite(outY) || !finite(outH)) {
                // Never publish garbage: keep the previous pose and report LOST.
                reject("computed pose is not finite; holding the last pose");
                state = ST_BAD;
                Pose2dStamped old = pose;
                pose = new Pose2dStamped(old.x, old.y, old.heading, old.tNs, BeaconStatus.LOST);
                publishedStatus = BeaconStatus.LOST;
                dirty = false;
                return pose;
            }
            pose = new Pose2dStamped(outX, outY, outH, lastGoodTNs, st);
            poseSetPoseCount = setPoseCount;    // after the pose: see getPoseSetPoseCount
            publishedStatus = st;
            dirty = false;
        }
        return pose;
    }

    private int computeStatus(long now) {
        if (state == ST_NONE) {
            return BeaconStatus.INITIALIZING;
        }
        if (state == ST_SRC_INVALID) {
            return BeaconStatus.INVALID;
        }
        if (state == ST_BAD || now - lastNewLocalNs > timeoutNs) {
            return BeaconStatus.LOST;
        }
        if (state == ST_SRC_LOST) {
            return BeaconStatus.LOST;
        }
        if (state == ST_SRC_INIT) {
            return BeaconStatus.INITIALIZING;
        }
        return alignSet && !pendingSet ? BeaconStatus.TRACKING : BeaconStatus.INITIALIZING;
    }

    // ---------------------------------------------------------------- getters

    /** Latest field-frame pose of the robot's centre of rotation. Never null, never NaN. */
    public Pose2dStamped getPose() {
        return pose;
    }

    /** Robot-frame velocity (vx forward, vy left, omega CCW); zero unless TRACKING. */
    public Velocity2d getVelocity() {
        return pose.status == BeaconStatus.TRACKING ? velocity : Velocity2d.ZERO;
    }

    public int getStatus() {
        return pose.status;
    }

    /**
     * The field heading, or NaN when the localizer has no trustworthy heading (not TRACKING).
     * Safe from any thread.
     */
    public double getHeadingIfTrackingOrNaN() {
        Pose2dStamped p = pose;
        return p.status == BeaconStatus.TRACKING ? p.heading : Double.NaN;
    }

    /** Robot centre height in the source world (metres). Diagnostic only. */
    public double getRobotZ() {
        return swZ;
    }

    /**
     * Number of accepted {@link #setPose} calls so far. The vision source watches it to hand
     * every new start pose to the native core. Safe from any thread.
     */
    public int getSetPoseCount() {
        return setPoseCount;
    }

    /**
     * The {@link #getSetPoseCount()} value that the published pose already reflects. Written
     * right after the pose, so a reader on another thread that reads THIS first and then
     * {@link #getPose()} always gets a pose at least as new as that setPose.
     */
    public int getPoseSetPoseCount() {
        return poseSetPoseCount;
    }

    public boolean isAligned() {
        return alignSet && !pendingSet;
    }

    /** True if START came before the source tracked, so the start pose was discarded and a
     *  full field fix (or a new {@code setPose}) is needed. */
    public boolean isStartPoseExpired() {
        return startPoseExpired;
    }

    /** True while the post-loss recovery waiver is open. */
    public boolean isRecovering() {
        return needRecover;
    }

    public void setFixBlendEnabled(boolean enabled) {
        fixBlend = enabled;
    }

    public boolean isFixBlendEnabled() {
        return fixBlend;
    }

    public long getSamplesAccepted() {
        return accepted;
    }

    public long getSamplesRejected() {
        return rejected;
    }

    public long getFixesApplied() {
        return fixesApplied;
    }

    /** Position change the last applied fix made at its capture time (metres). */
    public double getLastCorrectionM() {
        return lastCorrM;
    }

    /** Heading change the last applied fix made (radians). */
    public double getLastCorrectionRad() {
        return lastCorrRad;
    }

    public long getFixesRejected() {
        return fixesRejected;
    }

    public long getWorldResets() {
        return worldResets;
    }

    public long getTimeBaseResets() {
        return timeBaseResets;
    }

    public long getStartReasserts() {
        return startReasserts;
    }

    public String getLastRejectReason() {
        return lastReject;
    }

    public String getLastFixRejectReason() {
        return lastFixReject;
    }

    /** Kind of the last applied fix ("MT2", "MT2+MT1h", ...), empty if none. */
    public String getLastFixKind() {
        return lastFixKind;
    }

    /** Non-null if {@link #start} failed or the source crashed on stop. */
    public String getStartError() {
        return startError;
    }

    public PoseSource getSource() {
        return source;
    }

    /** One line for telemetry. Allocates; call it for display, not in a tight loop. */
    public String debugLine() {
        Pose2dStamped p = pose;
        StringBuilder sb = new StringBuilder(200);
        sb.append(BeaconStatus.name(p.status))
                .append(String.format(" x=%.3f y=%.3f h=%.1fdeg", p.x, p.y,
                        Math.toDegrees(p.heading)))
                .append(" | ok=").append(accepted).append(" rej=").append(rejected);
        if (worldResets > 0 || timeBaseResets > 0) {
            sb.append(" | resets world=").append(worldResets).append(" time=")
                    .append(timeBaseResets);
        }
        if (fixes != null) {
            sb.append(" | fix ").append(fixBlend ? "on" : "off").append(" +").append(fixesApplied)
                    .append(" -").append(fixesRejected);
            if (needRecover) {
                sb.append(" (recovering)");
            }
        }
        if (startPoseExpired) {
            sb.append(" | START POSE EXPIRED: need a vision fix");
        }
        if (log != null) {
            sb.append(" | log drop=").append(log.getDroppedCount());
        }
        if (!lastReject.isEmpty()) {
            sb.append(" | last: ").append(lastReject);
        }
        if (startError != null) {
            sb.append(" | ERR: ").append(startError);
        }
        sb.append(" | src: ").append(source.debugLine());
        return sb.toString();
    }

    private static boolean finite(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}
