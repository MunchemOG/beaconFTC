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
    public double xPodOffsetMm = 0;
    public double yPodOffsetMm = 0;

    /** goBILDA_4_BAR_POD or goBILDA_SWINGARM_POD, whichever you built. */
    public GoBildaPinpointDriver.GoBildaOdometryPods pods =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    /** Non-goBILDA pods: encoder ticks per millimetre of travel. 0 = use {@link #pods}. */
    public double customTicksPerMm = 0;

    /** The X pod must count UP driving forward, the Y pod UP strafing LEFT. */
    public GoBildaPinpointDriver.EncoderDirection xDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    public GoBildaPinpointDriver.EncoderDirection yDirection =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

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
    /** Usual for mecanum: the left side motors are mounted mirrored. */
    public boolean reverseLeftSide = true;
}
