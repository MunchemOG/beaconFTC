package org.firstinspires.ftc.teamcode.beacon.opmodes;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconHardwareConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconRig;
import org.firstinspires.ftc.teamcode.beacon.CalibrationMath;
import org.firstinspires.ftc.teamcode.beacon.Pose2dStamped;

import java.util.ArrayList;
import java.util.List;

/**
 * Guided calibration of the Pinpoint odometry. Three steps, each printing in plain words what
 * to change in BeaconHardwareConfig. Fix blending is OFF throughout, so you see the Pinpoint's
 * raw odometry. (The two AprilTag steps of the old tier were removed with the tag path.)
 *
 * <p>Gamepad 1: A = start / finish the current measurement; dpad right / left = next /
 * previous step; dpad up / down = change the number of turns (spin step) or the distance
 * (push steps, 0.1 m at a time). Driving with the sticks works if the drive motors are
 * configured; otherwise push or turn the robot by hand.
 *
 * <ol>
 *   <li>SPIN: mark the floor, spin counter-clockwise (seen from above) exactly N turns, stop on
 *       the mark. Gives pod offsets, heading sign and heading scale.</li>
 *   <li>FORWARD: push exactly D metres straight forward along a tape. Gives the X encoder
 *       direction, swapped pods, distance scale.</li>
 *   <li>LEFT: push exactly D metres straight to the robot's left. Same for the Y pod.</li>
 * </ol>
 */
@TeleOp(name = "Beacon Calibration", group = "Beacon")
public class BeaconCalibrationOpMode extends LinearOpMode {

    private static final String[] STEPS = {"1 SPIN", "2 FORWARD", "3 LEFT"};

    // goBILDA pod resolutions, ticks per mm (from GoBildaPinpointDriver)
    private static final double TICKS_4BAR = 19.89436789;
    private static final double TICKS_SWINGARM = 13.26291192;

    @Override
    public void runOpMode() {
        List<String> notes = new ArrayList<String>();
        BeaconConfig cfg = Support.loadConfig(notes);
        BeaconHardwareConfig hc = new BeaconHardwareConfig();
        BeaconRig rig = BeaconRig.create(hardwareMap, hc, cfg);
        OptionalDrive drive = new OptionalDrive(hardwareMap, hc);
        double ticksPerMm = hc.customTicksPerMm > 0 ? hc.customTicksPerMm
                : (hc.pods == GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                ? TICKS_4BAR : TICKS_SWINGARM);
        try {
            rig.start();
            rig.localizer.setFixBlendEnabled(false);
            rig.localizer.setPose(0, 0, 0);
            while (opModeInInit()) {
                rig.localizer.update();
                telemetry.addLine(rig.isPinpointReady() ? "Pinpoint READY. Press START."
                        : "Keep the robot STILL: Pinpoint IMU calibrating...");
                for (String n : notes) {
                    telemetry.addLine("!! " + n);
                }
                rig.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }
            rig.localizer.onStart();

            int step = 0;
            int turns = 5;
            double distM = 1.0;
            boolean measuring = false;
            CalibrationMath.Spin spin = null;
            double sx = 0, sy = 0, sh = 0;
            List<String> result = new ArrayList<String>();
            boolean pa = false, pr = false, pl = false, pu = false, pd = false;

            while (opModeIsActive()) {
                Pose2dStamped p = rig.localizer.update();
                drive.drive(gamepad1, 0.3);

                boolean a = gamepad1.a && !pa, right = gamepad1.dpad_right && !pr;
                boolean left = gamepad1.dpad_left && !pl, up = gamepad1.dpad_up && !pu;
                boolean down = gamepad1.dpad_down && !pd;
                pa = gamepad1.a;
                pr = gamepad1.dpad_right;
                pl = gamepad1.dpad_left;
                pu = gamepad1.dpad_up;
                pd = gamepad1.dpad_down;

                if ((right || left) && !measuring) {
                    step = (step + (right ? 1 : STEPS.length - 1)) % STEPS.length;
                    result = new ArrayList<String>();
                }
                if (up || down) {
                    if (step == 0) {
                        turns = Math.max(1, turns + (up ? 1 : -1));
                    } else if (step == 1 || step == 2) {
                        distM = Math.max(0.3, Math.round((distM + (up ? 0.1 : -0.1)) * 10) / 10.0);
                    }
                }

                if (step == 0) {                                    // ---- SPIN
                    if (measuring) {
                        spin.add(p.x, p.y, p.heading);
                    }
                    if (a) {
                        if (!measuring) {
                            spin = new CalibrationMath.Spin();
                            spin.add(p.x, p.y, p.heading);
                            measuring = true;
                            result = new ArrayList<String>();
                        } else {
                            measuring = false;
                            result = spin.advice(turns, hc.xPodOffsetMm, hc.yPodOffsetMm,
                                    hc.yawScalar);
                        }
                    }
                    telemetry.addLine("SPIN: counter-clockwise " + turns + " turns in place "
                            + "(dpad up/down), stop on your mark. A = "
                            + (measuring ? "FINISH" : "start"));
                    if (measuring) {
                        telemetry.addData("So far", "%.1f deg, wander %.1f mm",
                                Math.toDegrees(spin.totalTurnRad()), spin.maxWanderM() * 1000);
                    }
                } else {                                            // ---- PUSH
                    if (a) {
                        if (!measuring) {
                            sx = p.x;
                            sy = p.y;
                            sh = p.heading;
                            measuring = true;
                            result = new ArrayList<String>();
                        } else {
                            measuring = false;
                            result = CalibrationMath.pushAdvice(
                                    step == 1 ? CalibrationMath.FORWARD : CalibrationMath.LEFT,
                                    distM, sx, sy, sh, p.x, p.y, p.heading, ticksPerMm);
                        }
                    }
                    telemetry.addLine((step == 1 ? "FORWARD" : "LEFT") + ": push exactly "
                            + distM + " m " + (step == 1 ? "straight forward" : "straight to "
                            + "the robot's left") + " (dpad up/down). A = "
                            + (measuring ? "FINISH" : "start"));
                }

                telemetry.addData("Step", "%s   (dpad left/right to change)", STEPS[step]);
                for (String line : result) {
                    telemetry.addLine("> " + line);
                }
                telemetry.addData("Pinpoint pose", "x %.1f mm  y %.1f mm  h %.2f deg",
                        p.x * 1000, p.y * 1000, Math.toDegrees(p.heading));
                telemetry.addData("Config now", "xPod %.1f mm  yPod %.1f mm  %s/%s  ticks/mm %.4f",
                        hc.xPodOffsetMm, hc.yPodOffsetMm, hc.xDirection, hc.yDirection,
                        ticksPerMm);
                telemetry.addData("Drive", drive.enabled() ? "sticks (slow)" : drive.disabledReason);
                for (String n : notes) {
                    telemetry.addLine("!! " + n);
                }
                rig.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            drive.stop();
            rig.stop();
        }
    }
}
