package org.firstinspires.ftc.teamcode.beacon;

import android.app.Activity;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.beacon.vision.LimelightFrameSource;
import org.firstinspires.ftc.teamcode.beacon.vision.LimelightSdk;
import org.firstinspires.ftc.teamcode.beacon.vision.VisionFixSource;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Wires Beacon together: goBILDA Pinpoint (dead wheels + IMU) as the continuous pose feeding
 * one {@link BeaconLocalizer}, corrected by Beacon's vision: the Limelight 3A's camera stream
 * ({@link LimelightFrameSource}) into the native core ({@link VisionFixSource}), whose
 * field fixes the localizer blends at their capture time. AprilTags are not used.
 *
 * <pre>
 *   // INIT (robot placed and still)
 *   BeaconRig rig = BeaconRig.create(hardwareMap, new BeaconHardwareConfig(), cfg);
 *   rig.start();                                    // starts reading at once
 *   rig.localizer.setPose(startX, startY, startH);  // field frame, metres, radians
 *   while (opModeInInit()) { rig.localizer.update(); rig.addTelemetry(telemetry); ... }
 *   // START
 *   rig.localizer.onStart();
 *   while (opModeIsActive()) { Pose2dStamped p = rig.localizer.update(); ... }
 *   rig.stop();
 * </pre>
 *
 * <p>{@link #create} never throws for missing hardware: a Pinpoint absent from the robot
 * configuration becomes a line in {@link #problems} and the rig reports INVALID; missing
 * vision (no native library, no camera calibration, no stream) leaves the rig on the Pinpoint
 * alone, with the reason on the "Vision" telemetry line. It sends the Pinpoint its
 * configuration and resets it, which starts the IMU calibration (about 0.25 s, the robot must
 * be still: it is, during INIT). Nothing in create waits on the network.
 *
 * <p>Lifecycle: {@link #start} and {@link #stop} are idempotent. The rig also registers with
 * the SDK's OpMode manager and stops itself when the OpMode that created it ends, so no
 * Beacon thread can outlive the OpMode even if {@code stop()} is forgotten.
 */
public final class BeaconRig {
    public final BeaconLocalizer localizer;
    /** Null if the Pinpoint is not in the hardware configuration. */
    public final PinpointPoseSource pinpoint;
    /** The Limelight camera stream; null if vision is off (useLimelight / vision.enabled). */
    public final LimelightFrameSource frames;
    /** Vision fixes; null if vision is off. */
    public final VisionFixSource vision;
    /** Human-readable setup problems for telemetry. Never null; empty when all is well. */
    public final List<String> problems;

    private final Limelight3A limelight;
    private boolean started;
    private long rateT0Ns;
    private long rateApplied0;
    private double fixRate;
    private OpModeManagerImpl manager;
    private OpModeManagerNotifier.Notifications autoStop;   // strong ref: the SDK holds it weakly

    private BeaconRig(BeaconLocalizer localizer, PinpointPoseSource pinpoint,
                      LimelightFrameSource frames, VisionFixSource vision, Limelight3A limelight,
                      List<String> problems) {
        this.localizer = localizer;
        this.pinpoint = pinpoint;
        this.frames = frames;
        this.vision = vision;
        this.limelight = limelight;
        this.problems = Collections.unmodifiableList(problems);
    }

    public static BeaconRig create(HardwareMap hw, BeaconHardwareConfig hc, BeaconConfig cfg) {
        List<String> problems = new ArrayList<String>();
        for (String w : cfg.warnings) {
            problems.add("beacon.json: " + w);
        }

        // ---- Pinpoint
        PoseSource src;
        PinpointPoseSource pp = null;
        GoBildaPinpointDriver driver = null;
        try {
            driver = hw.tryGet(GoBildaPinpointDriver.class, hc.pinpointName);
        } catch (RuntimeException e) {
            problems.add("Pinpoint '" + hc.pinpointName + "' could not be opened: " + e);
        }
        if (driver == null) {
            problems.add("No goBILDA Pinpoint named '" + hc.pinpointName + "' in the robot "
                    + "configuration. Add it (I2C port, type goBILDA Pinpoint) or fix "
                    + "BeaconHardwareConfig.pinpointName.");
            src = new MissingPoseSource("pinpoint '" + hc.pinpointName + "' not configured");
        } else {
            PinpointPoseSource.DriverReader reader =
                    new PinpointPoseSource.DriverReader(driver, hc);
            try {
                reader.reconfigure();
                driver.resetPosAndIMU();       // robot must be still: it is, during INIT
            } catch (RuntimeException e) {
                problems.add("Pinpoint did not accept its configuration (I2C?): " + e);
            }
            pp = new PinpointPoseSource(driver, hc);
            src = pp;
        }

        // ---- Vision: Limelight stream -> native core -> field fixes
        LimelightFrameSource frames = null;
        VisionFixSource vision = null;
        Limelight3A ll = null;
        if (hc.useLimelight && cfg.vision.enabled) {
            try {
                ll = hw.tryGet(Limelight3A.class, hc.limelightName);   // optional
            } catch (RuntimeException e) {
                ll = null;
            }
            frames = LimelightSdk.createFrameSource(cfg, ll);
            vision = new VisionFixSource(frames, src, cfg);
            if (!vision.isNativeLoaded()) {
                problems.add("Beacon vision unavailable (native library not loaded): running on "
                        + "the Pinpoint alone.");
            }
        }

        BeaconLocalizer loc = new BeaconLocalizer(src, cfg, vision,
                PoseLogWriter.forConfig(cfg, "beacon"));
        if (vision != null) {
            vision.attachLocalizer(loc);
        }
        return new BeaconRig(loc, pp, frames, vision, ll, problems);
    }

    /** Starts the Pinpoint source and, if available, the camera stream and vision core. */
    public synchronized void start() {
        if (started) {
            return;
        }
        started = true;
        localizer.start(null);
        if (vision != null && vision.isNativeLoaded()) {
            if (limelight != null) {
                try {
                    limelight.start();   // SDK result polling: only for the reported latency
                } catch (RuntimeException e) {
                    // the frame source falls back to the configured latency
                }
            }
            frames.start();
            vision.start();
        }
        registerAutoStop();
    }

    /** Stops everything. Safe to call twice, before start, and from {@code OpMode.stop()}. */
    public synchronized void stop() {
        stopAll();
        started = false;
        unregisterAutoStop();
    }

    private void stopAll() {
        if (vision != null) {
            vision.stop();
        }
        if (frames != null) {
            frames.stop();
        }
        if (limelight != null) {
            try {
                limelight.stop();
            } catch (RuntimeException e) {
                // stopping anyway
            }
        }
        localizer.stop();
    }

    /** True once the Pinpoint reports READY (its IMU calibration is done). */
    public boolean isPinpointReady() {
        return pinpoint != null && pinpoint.isReady();
    }

    private void registerAutoStop() {
        try {
            Activity a = AppUtil.getInstance().getRootActivity();
            OpModeManagerImpl mgr = a == null ? null : OpModeManagerImpl.getOpModeManagerOfActivity(a);
            if (mgr == null) {
                return;
            }
            autoStop = new OpModeManagerNotifier.Notifications() {
                @Override
                public void onOpModePreInit(OpMode opMode) {
                }

                @Override
                public void onOpModePreStart(OpMode opMode) {
                }

                @Override
                public void onOpModePostStop(OpMode opMode) {
                    // Called on the SDK's thread while it iterates its listener set, so do
                    // not unregister here; just stop (idempotent, bounded waits).
                    stopAll();
                }
            };
            mgr.registerListener(autoStop);
            manager = mgr;
        } catch (RuntimeException e) {
            // Not fatal: the OpMode's own stop() still stops everything.
        }
    }

    private void unregisterAutoStop() {
        try {
            if (manager != null && autoStop != null) {
                manager.unregisterListener(autoStop);
            }
        } catch (RuntimeException e) {
            // ignore
        }
        manager = null;
        autoStop = null;
    }

    /**
     * Standard Beacon telemetry block: status, pose, problems, Pinpoint and fix lines.
     * Allocates; fine at telemetry rate.
     */
    public void addTelemetry(Telemetry t) {
        Pose2dStamped p = localizer.getPose();
        t.addData("Beacon", "%s  x %.3f m  y %.3f m  h %.1f deg", BeaconStatus.name(p.status),
                p.x, p.y, Math.toDegrees(p.heading));
        if (localizer.isStartPoseExpired()) {
            t.addLine("!! START came before the Pinpoint was READY: pose unknown until a vision "
                    + "fix. Wait for READY in INIT next time.");
        }
        for (String s : problems) {
            t.addLine("!! " + s);
        }
        t.addData("Pinpoint", pinpoint == null ? "MISSING" : pinpoint.debugLine());
        if (vision != null) {
            t.addData("Vision", vision.debugLine());
            if (vision.isNativeLoaded()) {
                t.addData("Camera", frames.debugLine());
            }
        }
        long now = System.nanoTime();
        long applied = localizer.getFixesApplied();
        if (now - rateT0Ns > 1000000000L) {
            fixRate = rateT0Ns == 0 ? 0 : (applied - rateApplied0) / ((now - rateT0Ns) * 1e-9);
            rateT0Ns = now;
            rateApplied0 = applied;
        }
        t.addData("Fixes", "applied %d (%.1f/s)  rejected %d  last %s moved %.1f mm %.2f deg | %s",
                applied, fixRate, localizer.getFixesRejected(), localizer.getLastFixKind(),
                localizer.getLastCorrectionM() * 1000,
                Math.toDegrees(localizer.getLastCorrectionRad()),
                localizer.getLastFixRejectReason());
        if (!localizer.getLastRejectReason().isEmpty()) {
            t.addData("Last sample issue", localizer.getLastRejectReason());
        }
        if (localizer.getStartError() != null) {
            t.addData("Start error", localizer.getStartError());
        }
    }

    /**
     * Stand-in source when the Pinpoint is not configured. {@code start} fails, so the
     * localizer reports INVALID with this message as its start error; nothing ever throws
     * into the loop.
     */
    static final class MissingPoseSource implements PoseSource {
        private final String why;
        private final Pose3dStamped invalid =
                new Pose3dStamped(0, 0, 0, 0, 0, 0, 1, 0, BeaconStatus.INVALID);

        MissingPoseSource(String why) {
            this.why = why;
        }

        @Override
        public void start(android.content.Context ctx, BeaconConfig cfg) throws BeaconException {
            throw new BeaconException(why);
        }

        @Override
        public void stop() {
        }

        @Override
        public Pose3dStamped latest() {
            return invalid;
        }

        @Override
        public boolean isTracking() {
            return false;
        }

        @Override
        public String debugLine() {
            return why;
        }

        @Override
        public boolean reportsRobotPose() {
            return true;
        }
    }
}
