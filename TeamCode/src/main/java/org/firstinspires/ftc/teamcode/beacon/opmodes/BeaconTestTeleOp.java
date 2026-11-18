package org.firstinspires.ftc.teamcode.beacon.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconHardwareConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconRig;
import org.firstinspires.ftc.teamcode.beacon.Pose2dStamped;
import org.firstinspires.ftc.teamcode.beacon.Velocity2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Bench and field test for Beacon. Push the robot by hand (or drive it, if the four mecanum
 * motors in BeaconHardwareConfig exist) and watch the pose.
 *
 * <p>Buttons (gamepad 1): A = set pose to (0, 0, 0); B = toggle vision-fix blending;
 * X = toggle the detail page.
 *
 * <p>What to look for is in the bring-up procedure (DECISIONS.md, "Odometry tier bring-up").
 */
@TeleOp(name = "Beacon Test", group = "Beacon")
public class BeaconTestTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        List<String> notes = new ArrayList<String>();
        BeaconConfig cfg = Support.loadConfig(notes);
        BeaconHardwareConfig hc = new BeaconHardwareConfig();
        BeaconRig rig = BeaconRig.create(hardwareMap, hc, cfg);
        OptionalDrive drive = new OptionalDrive(hardwareMap, hc);
        try {
            rig.start();
            rig.localizer.setPose(0, 0, 0);

            // ---- INIT: robot still. The Pinpoint calibrates its IMU, then reads READY.
            while (opModeInInit()) {
                rig.localizer.update();
                telemetry.addLine(rig.isPinpointReady()
                        ? "Ready. Press START."
                        : "Keep the robot STILL: Pinpoint IMU calibrating...");
                for (String n : notes) {
                    telemetry.addLine("!! " + n);
                }
                rig.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            rig.localizer.onStart();

            boolean lastA = false, lastB = false, lastX = false, details = false;
            long lastLoop = System.nanoTime();
            double loopMs = 0, updMs = 0, worstUpdMs = 0;
            while (opModeIsActive()) {
                long t0 = System.nanoTime();
                Pose2dStamped p = rig.localizer.update();
                long t1 = System.nanoTime();
                double u = (t1 - t0) / 1e6;
                updMs = 0.95 * updMs + 0.05 * u;
                worstUpdMs = Math.max(worstUpdMs, u);
                loopMs = 0.95 * loopMs + 0.05 * ((t0 - lastLoop) / 1e6);
                lastLoop = t0;

                if (gamepad1.a && !lastA) {
                    rig.localizer.setPose(0, 0, 0);
                }
                if (gamepad1.b && !lastB) {
                    rig.localizer.setFixBlendEnabled(!rig.localizer.isFixBlendEnabled());
                }
                if (gamepad1.x && !lastX) {
                    details = !details;
                }
                lastA = gamepad1.a;
                lastB = gamepad1.b;
                lastX = gamepad1.x;
                drive.drive(gamepad1, 0.4);

                Velocity2d v = rig.localizer.getVelocity();
                telemetry.addData("Pose m", "x %.3f  y %.3f  h %.1f deg", p.x, p.y,
                        Math.toDegrees(p.heading));
                telemetry.addData("Pose in", "x %.2f  y %.2f", p.x / 0.0254, p.y / 0.0254);
                telemetry.addData("Velocity", "fwd %.2f  left %.2f m/s  turn %.1f deg/s",
                        v.vx, v.vy, Math.toDegrees(v.omega));
                telemetry.addData("Loop", "%.1f ms (%.0f Hz), update() %.2f ms, worst %.2f ms",
                        loopMs, loopMs > 0 ? 1000 / loopMs : 0, updMs, worstUpdMs);
                telemetry.addData("Fix blend", "%s  (B toggles, A zeroes, X details)",
                        rig.localizer.isFixBlendEnabled() ? "ON" : "OFF");
                telemetry.addData("Drive", drive.enabled() ? "gamepad 1 (slow)"
                        : drive.disabledReason);
                for (String n : notes) {
                    telemetry.addLine("!! " + n);
                }
                rig.addTelemetry(telemetry);
                if (details) {
                    telemetry.addData("Localizer", rig.localizer.debugLine());
                }
                telemetry.update();
            }
        } finally {
            drive.stop();
            rig.stop();
        }
    }
}
