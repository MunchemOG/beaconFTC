package org.firstinspires.ftc.teamcode.beacon.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconHardwareConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconRig;
import org.firstinspires.ftc.teamcode.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.beacon.Pose2dStamped;
import org.firstinspires.ftc.teamcode.beacon.Velocity2d;

import java.util.ArrayList;
import java.util.List;

/**
 * SAMPLE: how an autonomous uses Beacon. Copy this pattern into your own OpMode. It does not
 * move the robot; put your path follower where the comment says.
 *
 * Units: METRES and RADIANS, heading counter-clockwise positive, in the FTC field frame the
 * Limelight's field map uses (origin at the field centre). 1 inch = 0.0254 m.
 */
@Autonomous(name = "Beacon Sample Auto", group = "Beacon")
public class BeaconSampleAuto extends LinearOpMode {

    // Where the robot starts, measured on the field (example values: change them).
    static final double START_X = -1.50;              // metres
    static final double START_Y = -0.90;              // metres
    static final double START_HEADING = Math.PI / 2;  // radians (90 degrees)

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------------ INIT
        // 1. Load beacon.json (tag gains etc.). A broken file falls back to defaults and the
        //    reason is shown on the Driver Station; it never stops the OpMode.
        List<String> notes = new ArrayList<String>();
        BeaconConfig cfg = Support.loadConfig(notes);

        // 2. Build the rig. This finds the Pinpoint and Limelight by the names in
        //    BeaconHardwareConfig, sends the Pinpoint its pod settings and RESETS it, which
        //    calibrates its IMU. The robot must be STILL now: it is, you just placed it.
        //    Missing hardware does not throw; it appears in rig.problems.
        BeaconRig rig = BeaconRig.create(hardwareMap, new BeaconHardwareConfig(), cfg);

        try {
            // 3. Start reading now, in INIT, so the Pinpoint finishes calibrating before
            //    START (Beacon vision can still correct it later).
            rig.start();

            // 4. Tell Beacon where the robot is. If the Pinpoint is still calibrating this is
            //    remembered and applied on its first good reading.
            rig.localizer.setPose(START_X, START_Y, START_HEADING);

            // 5. Wait in INIT, calling update() every loop (that is what reads the Pinpoint).
            //    Do NOT move the robot: rule G304 says it must be motionless after INIT, and
            //    the IMU calibration needs it still anyway. Press START only once the line
            //    says READY; if START comes first, Beacon discards the start pose (the robot
            //    may have moved) and waits for a vision fix.
            while (opModeInInit()) {
                rig.localizer.update();
                telemetry.addLine(rig.isPinpointReady() ? "READY - press START"
                        : "WAIT: Pinpoint calibrating, keep the robot still");
                for (String n : notes) {
                    telemetry.addLine("!! " + n);
                }
                rig.addTelemetry(telemetry);
                telemetry.update();
                idle();
            }

            // ------------------------------------------------------------- START
            // 6. Tell Beacon the robot may move from now on. If it was already tracking, the
            //    start pose is re-applied once more (removes any drift while waiting).
            rig.localizer.onStart();

            // ------------------------------------------------------------- RUN
            while (opModeIsActive()) {
                // 7. ONE update() per loop, before you use the pose. It reads the Pinpoint
                //    once and applies any new Limelight fix. It never blocks or throws.
                Pose2dStamped pose = rig.localizer.update();
                Velocity2d vel = rig.localizer.getVelocity();   // robot frame, m/s, rad/s

                // 8. Check the status before trusting the pose.
                //    TRACKING     - pose is good, drive.
                //    INITIALIZING - no position yet (Pinpoint not ready, or START came too
                //                   early and Beacon waits for a vision fix): do not drive by pose.
                //    LOST         - Pinpoint fault or no data: the last good pose is held and
                //                   the velocity reads 0. Stop or fall back to a timed move.
                //    INVALID      - Pinpoint missing from the configuration.
                if (pose.status == BeaconStatus.TRACKING) {
                    // >>> Your path follower goes here, e.g. Road Runner / Pedro, fed with
                    //     pose.x, pose.y, pose.heading (metres, radians) and vel. <<<
                    telemetry.addData("Drive to target", "from x %.3f y %.3f h %.1f deg",
                            pose.x, pose.y, Math.toDegrees(pose.heading));
                } else {
                    // >>> Stop the drive motors here. <<<
                    telemetry.addData("Holding", BeaconStatus.name(pose.status));
                }
                rig.addTelemetry(telemetry);
                telemetry.update();
            }
        } finally {
            // 9. Always stop the rig (also stops the Limelight thread). It is safe to call
            //    twice; the rig also stops itself when the OpMode ends.
            rig.stop();
        }
    }
}
