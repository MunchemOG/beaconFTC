package org.firstinspires.ftc.teamcode.beacon;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * What is physically on YOUR robot: device names (they must match the Robot Controller
 * configuration exactly) and the Pinpoint's pod measurements. Edit the defaults here, or
 * build one in your OpMode and change fields before calling {@link BeaconRig#create}.
 * {@code BeaconCalibrationOpMode} tells you which of these to change, and to what.
 *
 * <p>How much to trust vision fixes (gains, gates) lives in beacon.json instead; see
 * {@link BeaconConfig}.
 */
public class BeaconHardwareConfig {
    // ------------------------------------------------------------ Pinpoint

    /** Configured name of the goBILDA Pinpoint (an I2C device on the Control Hub). */
    public String pinpointName = "pinpoint";

    /**
     * Pod positions relative to the robot's centre of rotation (the point it spins about),
     * in millimetres, using the Pinpoint's sign convention (GoBildaPinpointDriver.setOffsets):
     * X pod offset = how far to the LEFT of the centre the forward-rolling (X) pod is (right of
     * centre is negative). Y pod offset = how far FORWARD of the centre the sideways-rolling
     * (Y) pod is (behind is negative). Wrong values show up as the pose drawing a circle when
     * the robot spins in place; the calibration OpMode measures them.
     */
    // From PollenBotLimelightTest.step (2026-11-20): X (forward-rolling) pod 98.4 mm left of the
    // wheelbase centre, Y (sideways-rolling) pod 138.9 mm behind it. Check against the robot.
    public double xPodOffsetMm = 98.4;
    public double yPodOffsetMm = -138.9;

    /** goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD, whichever you built. */
    public GoBildaPinpointDriver.GoBildaOdometryPods pods =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    /** Non-goBILDA pods: encoder ticks per millimetre of travel. 0 = use {@link #pods}. */
    public double customTicksPerMm = 0;

    /** The X pod must count UP driving forward, the Y pod UP strafing LEFT. */
    public GoBildaPinpointDriver.EncoderDirection xDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    // REVERSED on this robot: same as ElectroLarp-Biobuzz Constants.localizerConfig.
    public GoBildaPinpointDriver.EncoderDirection yDirection =
            GoBildaPinpointDriver.EncoderDirection.REVERSED;

    /**
     * Pinpoint yaw scalar. 0 = keep the value goBILDA tuned at the factory (recommended).
     * Change it only if the calibration OpMode's spin test says so; goBILDA says a value
     * outside 0.95..1.05 means a bad device.
     */
    public double yawScalar = 0;

    // ------------------------------------------------------------ Limelight

    /**
     * Configured name of the Limelight 3A (a USB device). Optional: Beacon reads its camera
     * stream either way; with it configured Beacon also learns its address and reported
     * latency.
     */
    public String limelightName = "limelight";
    /** Set false to run on the Pinpoint alone (no vision corrections). */
    public boolean useLimelight = true;

    // ---------------------------------------------- optional drive (test OpModes only)

    /**
     * Mecanum drive motor names, used ONLY by the Beacon test and calibration OpModes so you
     * can drive with the gamepad. If any is missing, driving is disabled and you push the
     * robot by hand instead. Beacon itself never touches motors.
     */
    public String frontLeftName = "frontLeft";
    public String frontRightName = "frontRight";
    public String backLeftName = "backLeft";
    public String backRightName = "backRight";
    /**
     * true = left motors REVERSE, right FORWARD. ElectroLarp's directions (false) made the robot
     * drive backwards on the first test (2026-11-20), so this is back to true. If forward is
     * right but strafe or turning is mirrored, tell me which; do not guess.
     */
    public boolean reverseLeftSide = true;
}
