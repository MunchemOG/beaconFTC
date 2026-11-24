package org.firstinspires.ftc.teamcode.beacon.opmodes;

import android.os.Build;
import android.os.SystemClock;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconHardwareConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconRig;
import org.firstinspires.ftc.teamcode.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.beacon.FrameMath;
import org.firstinspires.ftc.teamcode.beacon.Pose2dStamped;
import org.firstinspires.ftc.teamcode.beacon.Pose3dStamped;
import org.firstinspires.ftc.teamcode.beacon.vision.CpuTime;
import org.firstinspires.ftc.teamcode.beacon.vision.DatasetRecorder;
import org.firstinspires.ftc.teamcode.beacon.vision.FrameSource;
import org.firstinspires.ftc.teamcode.beacon.vision.GrayFrame;
import org.firstinspires.ftc.teamcode.beacon.vision.LedLatencyMeter;
import org.firstinspires.ftc.teamcode.beacon.vision.LimelightFrameSource;
import org.firstinspires.ftc.teamcode.beacon.vision.LimelightSdk;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Day-one probe and data collection for Beacon (plan milestone 0, task J3). One OpMode, so a
 * single field session gets everything: is the Limelight stream reachable from the Hub, at
 * what rate, size, decode cost, latency and jitter; plus recordings (frames + Pinpoint) for
 * the desktop bench, tape-mark checks, a checkerboard sequence for intrinsics, and an LED
 * latency measurement.
 *
 * <h3>Setup</h3>
 * Robot configuration: the goBILDA Pinpoint (name in BeaconHardwareConfig, default
 * "pinpoint"); optionally the Limelight 3A (default "limelight"; without it the stream is
 * still tried at the built-in addresses, only the Limelight-reported latency is missing).
 * The Limelight should run a pipeline that sends a clean picture (no crosshair or overlays;
 * see tools/limelight/README.md). Optional beacon.json "probe": {"start": [x, y, h],
 * "marks": [[x, y, h], ...]} (field frame, metres, radians).
 *
 * <h3>Gamepad 1</h3>
 * <ul>
 *   <li>B: start / stop a recording (runs go to /sdcard/FIRST/beacon/runs/&lt;date&gt;/).</li>
 *   <li>X: start / stop a CHECKERBOARD recording (slow frame rate, "_checker" run) for
 *       intrinsics: wave a printed checkerboard in front of the camera, all corners and
 *       angles.</li>
 *   <li>A: record a tape-mark check: the robot is parked at the selected mark (dpad
 *       left/right selects); writes a marks.csv row with the mark's typed coordinates.</li>
 *   <li>Y: start / stop the LED latency test. Hold the Limelight lens a few cm from the
 *       Control Hub's status LED, LED near the image centre, until it says DONE; then put
 *       the latency it prints into beacon.json as limelight.latencyMs.</li>
 *   <li>dpad up: switch between the summary and the details page.</li>
 *   <li>Sticks: drive slowly, if the four drive motors are configured (after START).</li>
 * </ul>
 *
 * <p>Everything heavy runs on other threads: frame reading and decoding (the frame source),
 * PNG writing (the recorder), LED blinking (its own thread). The loop only reads the Pinpoint,
 * hands odometry rows to the recorder and draws telemetry.
 */
@TeleOp(name = "Beacon Probe", group = "Beacon")
public class BeaconProbeOpMode extends LinearOpMode {
    private BeaconConfig cfg;
    private BeaconRig rig;
    private Limelight3A ll;
    private LimelightFrameSource frames;
    private final LedLatencyMeter meter = new LedLatencyMeter();
    private HubLedBlinker blinker;
    private volatile DatasetRecorder recorder;
    private volatile boolean ledTest;
    private volatile DatasetRecorder closing;
    private boolean checkerRun;
    private volatile String recNote = "not recording";

    private double[] startPose;
    private long startPoseTNs;
    private long lastOdomT;
    private int lastEpoch = Integer.MIN_VALUE;
    private final StringBuilder odomEvents = new StringBuilder();
    private int markIdx;
    private int page;

    private boolean pA, pB, pX, pY, pL, pR, pU;
    private long cpuWallNs;
    private long cpuMs = -1;
    private double procCpuPct = -1;
    private double updMs, worstUpdMs;

    @Override
    public void runOpMode() {
        List<String> notes = new ArrayList<String>();
        cfg = Support.loadConfig(notes);
        BeaconHardwareConfig hc = new BeaconHardwareConfig();
        hc.useLimelight = false;            // the probe reads the stream itself
        rig = BeaconRig.create(hardwareMap, hc, cfg);
        try {
            ll = hardwareMap.tryGet(Limelight3A.class, hc.limelightName);
        } catch (RuntimeException e) {
            ll = null;
        }
        if (ll == null) {
            notes.add("No Limelight 3A named '" + hc.limelightName + "' in the configuration: "
                    + "trying the stream at the default addresses; no Limelight-reported "
                    + "latency.");
        }
        frames = LimelightSdk.createFrameSource(cfg, ll);
        frames.setListener(new FrameSource.Listener() {
            @Override
            public void onFrame(GrayFrame f) {   // decode thread
                DatasetRecorder r = recorder;
                if (r != null) {
                    r.offerFrame(f);
                }
                if (ledTest) {
                    meter.onFrame(f);
                }
            }
        });
        blinker = HubLedBlinker.create(hardwareMap, meter);
        OptionalDrive drive = new OptionalDrive(hardwareMap, hc);
        try {
            if (ll != null) {
                try {
                    ll.start();     // SDK result polling, for the reported latency
                } catch (RuntimeException e) {
                    notes.add("Limelight start failed: " + e);
                }
            }
            frames.start();
            rig.start();
            startPose = cfg.probe.start != null ? cfg.probe.start : new double[] {0, 0, 0};
            if (cfg.probe.start == null) {
                notes.add("No probe.start in beacon.json: start pose is (0, 0, 0).");
            }
            rig.localizer.setPose(startPose[0], startPose[1], startPose[2]);
            startPoseTNs = SystemClock.elapsedRealtimeNanos();

            while (opModeInInit()) {
                step(notes, null);
                idle();
            }
            rig.localizer.onStart();
            while (opModeIsActive()) {
                step(notes, drive);
            }
        } finally {
            stopRecording();
            if (blinker != null) {
                blinker.stop();
            }
            frames.stop();
            if (ll != null) {
                try {
                    ll.stop();
                } catch (RuntimeException ignored) {
                    // stopping anyway
                }
            }
            drive.stop();
            rig.stop();
        }
    }

    private void step(List<String> notes, OptionalDrive drive) {
        long t0 = System.nanoTime();
        rig.localizer.update();
        double u = (System.nanoTime() - t0) / 1e6;
        updMs = 0.95 * updMs + 0.05 * u;
        worstUpdMs = Math.max(worstUpdMs, u);
        feedOdom();
        buttons();
        if (drive != null) {
            drive.drive(gamepad1, 0.5);
        }
        updateCpu();
        if (page == 0) {
            summaryPage(notes);
        } else {
            detailsPage();
        }
        telemetry.update();
    }

    // ------------------------------------------------------------- recording

    private void feedOdom() {
        DatasetRecorder r = recorder;
        if (r == null || rig.pinpoint == null) {
            return;
        }
        Pose3dStamped s = rig.pinpoint.latest();
        if (s.status != BeaconStatus.TRACKING || s.tNs == lastOdomT) {
            return;
        }
        lastOdomT = s.tNs;
        if (lastEpoch != Integer.MIN_VALUE && s.worldEpoch != lastEpoch) {
            odomEvents.append("odometry world reset (epoch ").append(lastEpoch).append(" -> ")
                    .append(s.worldEpoch).append(") at t_ns ").append(s.tNs).append('\n');
        }
        lastEpoch = s.worldEpoch;
        // Raw Pinpoint world, robot centre; heading from the yaw-only quaternion.
        r.offerOdom(s.tNs, s.x, s.y, FrameMath.wrapAngle(2 * Math.atan2(s.qz, s.qw)));
    }

    private void startRecording(boolean checker) {
        if (closing != null) {
            recNote = "still saving the previous run, try again in a moment";
            return;
        }
        String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
        File dir = new File(cfg.record.dir, stamp + (checker ? "_checker" : ""));
        double hz = checker ? cfg.record.checkerboardHz : cfg.record.maxHz;
        DatasetRecorder r = new DatasetRecorder(dir, cfg.record.frameQueue, 16384, hz,
                cfg.cameraMount, new File(cfg.record.fieldFile));
        if (!r.start()) {
            recNote = "RECORDING FAILED: " + r.getError();
            return;
        }
        r.writeStart(startPoseTNs, startPose[0], startPose[1], startPose[2]);
        odomEvents.setLength(0);
        lastOdomT = 0;
        lastEpoch = Integer.MIN_VALUE;
        checkerRun = checker;
        recorder = r;
        recNote = (checker ? "CHECKERBOARD " : "") + "recording to " + dir.getPath();
    }

    /** Stops the recorder; the final writes happen on a helper thread, not in the loop. */
    private void stopRecording() {
        final DatasetRecorder r = recorder;
        if (r == null) {
            return;
        }
        recorder = null;
        r.writeText("probe_summary.txt", summaryText());
        closing = r;
        recNote = "saving " + r.getDir().getPath();
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                r.close();
                recNote = "saved " + r.getDir().getPath() + ": " + r.debugLine();
                closing = null;
            }
        }, "BeaconRecorderClose");
        t.setDaemon(false);   // let the files finish even if the OpMode ends
        t.start();
    }

    private void buttons() {
        if (gamepad1.b && !pB) {
            if (recorder == null) {
                startRecording(false);
            } else if (!checkerRun) {
                stopRecording();
            }
        }
        if (gamepad1.x && !pX) {
            if (recorder == null) {
                startRecording(true);
            } else if (checkerRun) {
                stopRecording();
            }
        }
        if (gamepad1.a && !pA) {
            recordMark();
        }
        if (gamepad1.y && !pY) {
            toggleLedTest();
        }
        int n = cfg.probe.marks.size();
        if (n > 0 && gamepad1.dpad_right && !pR) {
            markIdx = (markIdx + 1) % n;
        }
        if (n > 0 && gamepad1.dpad_left && !pL) {
            markIdx = (markIdx + n - 1) % n;
        }
        if (gamepad1.dpad_up && !pU) {
            page = 1 - page;
        }
        pA = gamepad1.a;
        pB = gamepad1.b;
        pX = gamepad1.x;
        pY = gamepad1.y;
        pL = gamepad1.dpad_left;
        pR = gamepad1.dpad_right;
        pU = gamepad1.dpad_up;
    }

    private void recordMark() {
        DatasetRecorder r = recorder;
        if (cfg.probe.marks.isEmpty()) {
            recNote = "no probe.marks in beacon.json; nothing recorded";
            return;
        }
        if (r == null) {
            recNote = "start a recording (B) before recording marks";
            return;
        }
        double[] m = cfg.probe.marks.get(markIdx);
        r.addMark(SystemClock.elapsedRealtimeNanos(), m[0], m[1], m[2]);
        Pose2dStamped p = rig.localizer.getPose();
        recNote = String.format(Locale.US, "mark %d recorded; Beacon says %.2f in, %.2f in, %.1f deg",
                markIdx, p.x / 0.0254, p.y / 0.0254, Math.toDegrees(p.heading));
    }

    private void toggleLedTest() {
        if (blinker == null) {
            recNote = "LED test needs the Control Hub's own module; none found";
            return;
        }
        if (blinker.isRunning()) {
            ledTest = false;
            blinker.stop();
        } else {
            blinker.start();
            ledTest = true;
        }
    }

    // ------------------------------------------------------------- telemetry

    private void updateCpu() {
        long now = System.nanoTime();
        if (now - cpuWallNs < 1000000000L) {
            return;
        }
        long ms = CpuTime.processMs();
        if (ms >= 0 && cpuMs >= 0 && cpuWallNs > 0) {
            procCpuPct = 100.0 * (ms - cpuMs) / ((now - cpuWallNs) / 1e6);
        }
        cpuMs = ms;
        cpuWallNs = now;
    }

    private void summaryPage(List<String> notes) {
        LimelightFrameSource f = frames;
        telemetry.addLine("B rec | X checkerboard rec | A mark | Y LED test | dpad-up details");
        if (f.isStreaming()) {
            telemetry.addData("Camera", "STREAMING %s", f.getWorkingUrl());
        } else {
            telemetry.addData("Camera", "NO STREAM: %s", f.getLastLoss());
        }
        telemetry.addData("Frames", "%.1f fps  %dx%d  jpg %.0f KB  xfer %.1f ms", f.getFps(),
                f.getWidth(), f.getHeight(), f.getJpegBytes() / 1024.0, f.getTransferMs());
        telemetry.addData("Decode", "%.1f ms avg, %.1f ms max/1s, decode thread %s",
                f.getDecodeMsAvg(), f.getDecodeMsMax(), pct(f.getDecodeCpuPct()));
        telemetry.addData("Latency", "%.1f ms, %s (capture = first byte - latency)",
                f.getLatencyMs(), GrayFrame.latencySourceName(f.getLatencySource()));
        telemetry.addData("Arrival", "jitter %.1f ms sd, max gap %.1f ms", f.getJitterMs(),
                f.getMaxGapMs());
        telemetry.addData("Dropped", "before decode %d, decode fail %d, bad parts %d, "
                        + "reconnects %d", f.getDroppedBeforeDecode(), f.getDecodeFailures(),
                f.getBadParts(), f.getReconnects());
        telemetry.addData("CPU", "app %s of one core (4 cores = 400%%)", pct(procCpuPct));
        DatasetRecorder r = recorder;
        telemetry.addData("Record", r != null ? r.debugLine() : recNote);
        if (r != null) {
            telemetry.addLine(recNote);
        }
        telemetry.addData("LED test", blinker == null ? "unavailable"
                : (blinker.isRunning() ? meter.statusLine() + String.format(Locale.US,
                " | LED cmd %.1f ms", blinker.getLastCommandMs()) : "off (Y starts)")
                + (blinker != null && !blinker.getError().isEmpty() ? " | " + blinker.getError()
                : ""));
        Pose2dStamped p = rig.localizer.getPose();
        telemetry.addData("Beacon", "%s  x %.2f in  y %.2f in  h %.1f deg | update %.2f ms (worst "
                        + "%.2f)", BeaconStatus.name(p.status), p.x / 0.0254, p.y / 0.0254,
                Math.toDegrees(p.heading), updMs, worstUpdMs);
        telemetry.addData("Pinpoint", rig.pinpoint == null ? "MISSING" : rig.pinpoint.debugLine());
        if (rig.pinpoint != null) {
            telemetry.addData("Pinpoint raw", rig.pinpoint.rawLine());
        }
        if (cfg.probe.marks.isEmpty()) {
            telemetry.addData("Marks", "none in beacon.json (probe.marks)");
        } else {
            double[] m = cfg.probe.marks.get(markIdx);
            telemetry.addData("Mark", "%d/%d at %.2f in, %.2f in, %.0f deg (dpad L/R)", markIdx + 1,
                    cfg.probe.marks.size(), m[0] / 0.0254, m[1] / 0.0254, Math.toDegrees(m[2]));
        }
        for (String n : notes) {
            telemetry.addLine("!! " + n);
        }
        for (String n : rig.problems) {
            telemetry.addLine("!! " + n);
        }
    }

    private void detailsPage() {
        LimelightFrameSource f = frames;
        telemetry.addLine("DETAILS (dpad-up: back)");
        telemetry.addData("Candidates", "%s", f.getCandidates());
        telemetry.addData("Tried", f.getAttemptsLine());
        telemetry.addData("Stream", f.debugLine());
        telemetry.addData("Server timestamp header", f.getTimestampHeader() == null ? "none"
                : f.getTimestampHeader());
        telemetry.addData("Hub", "Android %d, ABIs %s", Build.VERSION.SDK_INT,
                join(Build.SUPPORTED_ABIS));
        telemetry.addData("Limelight results", limelightLine());
        telemetry.addLine("Capture time = arrival of the frame's first byte (measured) minus "
                + "latency: limelight.latencyMs if set (measure it with the LED test), else the "
                + "Limelight's own cl+tl (a LOWER bound: leaves out half the exposure, JPEG "
                + "encode and transfer), else limelight.defaultLatencyMs (a guess).");
        telemetry.addData("Non-monotonic stamps fixed", f.getNonMonotonicStamps());
        telemetry.addData("Localizer", rig.localizer.debugLine());
    }

    private String limelightLine() {
        if (ll == null) {
            return "no Limelight in the configuration";
        }
        try {
            LLResult r = ll.getLatestResult();
            if (r == null) {
                return "no result";
            }
            return String.format(Locale.US, "cl %.1f ms tl %.1f ms age %d ms pipe %d (%s) "
                            + "addr %s", r.getCaptureLatency(), r.getTargetingLatency(),
                    r.getStaleness(), r.getPipelineIndex(), r.getPipelineType(),
                    LimelightSdk.configuredIp(ll));
        } catch (RuntimeException e) {
            return "error: " + e;
        }
    }

    private String summaryText() {
        StringBuilder sb = new StringBuilder(2048);
        LimelightFrameSource f = frames;
        sb.append("Beacon probe run summary\n");
        sb.append("stream: ").append(f.debugLine()).append('\n');
        sb.append("candidates: ").append(f.getCandidates()).append('\n');
        sb.append("tried: ").append(f.getAttemptsLine()).append('\n');
        sb.append("server timestamp header: ").append(f.getTimestampHeader()).append('\n');
        sb.append("limelight results: ").append(limelightLine()).append('\n');
        sb.append("config limelight.latencyMs: ").append(cfg.stream.latencyMs)
                .append(" defaultLatencyMs: ").append(cfg.stream.defaultLatencyMs).append('\n');
        sb.append("LED test: ").append(blinker != null && blinker.isRunning()
                ? meter.statusLine() : "not running").append('\n');
        sb.append("app CPU: ").append(pct(procCpuPct)).append(" decode thread CPU: ")
                .append(pct(f.getDecodeCpuPct())).append('\n');
        sb.append("hub: Android ").append(Build.VERSION.SDK_INT).append(" ABIs ")
                .append(join(Build.SUPPORTED_ABIS)).append('\n');
        sb.append("update(): avg ").append(updMs).append(" ms worst ").append(worstUpdMs)
                .append(" ms\n");
        sb.append("pinpoint: ").append(rig.pinpoint == null ? "MISSING"
                : rig.pinpoint.debugLine()).append('\n');
        sb.append(odomEvents);
        return sb.toString();
    }

    private static String pct(double v) {
        return v < 0 ? "n/a" : String.format(Locale.US, "%.0f%%", v);
    }

    private static String join(String[] a) {
        StringBuilder sb = new StringBuilder();
        if (a != null) {
            for (String s : a) {
                if (sb.length() > 0) {
                    sb.append(',');
                }
                sb.append(s);
            }
        }
        return sb.toString();
    }
}
