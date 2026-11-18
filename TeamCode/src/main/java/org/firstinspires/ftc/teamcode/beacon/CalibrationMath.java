package org.firstinspires.ftc.teamcode.beacon;

import java.util.ArrayList;
import java.util.List;

/**
 * The arithmetic behind {@code BeaconCalibrationOpMode}, in plain Java so it is unit-tested.
 * Every method turns measurements into plain-English lines that say what to change in
 * {@link BeaconHardwareConfig} (or in the Limelight web UI), and by how much.
 *
 * <p>All poses here are the Pinpoint's own robot pose (x forward, y left at reset, metres,
 * heading CCW radians), taken with fix blending OFF so vision cannot hide an odometry error.
 */
public final class CalibrationMath {

    private CalibrationMath() {}

    // ================================================================= spin

    /**
     * Spin test. The robot turns in place, counter-clockwise seen from above, a whole number
     * of turns, and stops where it started. Feed every pose with {@link #add}.
     *
     * <h3>Why this finds the pod offsets</h3>
     * The Pinpoint turns pod readings into motion of the tracking point using the offsets it
     * was given. If the forward (X) pod is really {@code L} to the left of the centre but was
     * configured as {@code Lc}, a pure spin at rate w makes the Pinpoint report a forward
     * velocity of {@code w * (Lc - L)}; likewise a strafe (Y) pod really {@code F} forward but
     * configured {@code Fc} gives a sideways velocity {@code w * (F - Fc)}. So during a spin
     * the reported centre moves at a constant velocity in the robot frame, proportional to the
     * turn rate: it draws a circle of radius |offset error|. Expressing each small position
     * step in the robot frame and dividing by the heading step measures that error vector
     * {@code e = (Lc - L, F - Fc)} directly. Pure translation during the spin biases it, so
     * spin in place.
     */
    public static final class Spin {
        private boolean have;
        private double lastX, lastY, lastH;
        private double startX, startY;
        private double sumRx, sumRy, sumDh;
        private double maxDist;
        private int n;

        public void add(double x, double y, double heading) {
            if (!have) {
                have = true;
                startX = x;
                startY = y;
            } else {
                double dh = FrameMath.angleDiff(heading, lastH);
                double mid = lastH + 0.5 * dh;
                double dx = x - lastX, dy = y - lastY;
                double c = Math.cos(mid), s = Math.sin(mid);
                sumRx += c * dx + s * dy;       // step in the robot frame
                sumRy += -s * dx + c * dy;
                sumDh += dh;
                maxDist = Math.max(maxDist, Math.hypot(x - startX, y - startY));
            }
            lastX = x;
            lastY = y;
            lastH = heading;
            n++;
        }

        /** Signed total heading change, radians (positive = counter-clockwise). */
        public double totalTurnRad() {
            return sumDh;
        }

        /** Offset error e = (Lc - L, F - Fc) in metres; zero if under half a turn so far. */
        public double errorX() {
            return Math.abs(sumDh) > Math.PI ? sumRx / sumDh : 0;
        }

        public double errorY() {
            return Math.abs(sumDh) > Math.PI ? sumRy / sumDh : 0;
        }

        /** Largest distance of the reported centre from where it started, metres. */
        public double maxWanderM() {
            return maxDist;
        }

        /** Distance between the start and the latest reported centre, metres. */
        public double endLeakM() {
            return have ? Math.hypot(lastX - startX, lastY - startY) : 0;
        }

        public int samples() {
            return n;
        }

        /**
         * Plain-words result.
         * @param turns          how many full turns the robot really made (counted on the floor)
         * @param xPodOffsetMm   the current BeaconHardwareConfig.xPodOffsetMm
         * @param yPodOffsetMm   the current BeaconHardwareConfig.yPodOffsetMm
         * @param yawScalarNow   the Pinpoint's current yaw scalar (0 if unknown)
         */
        public List<String> advice(int turns, double xPodOffsetMm, double yPodOffsetMm,
                                   double yawScalarNow) {
            List<String> out = new ArrayList<String>();
            double turned = sumDh;
            out.add(String.format("Measured turn: %.1f deg (%.3f turns); you said %d turns.",
                    Math.toDegrees(turned), turned / (2 * Math.PI), turns));
            if (Math.abs(turned) < Math.PI) {
                out.add("Less than half a turn measured: spin the robot, then press A.");
                return out;
            }
            if (turned < 0) {
                out.add("Heading went DOWN while turning counter-clockwise. Either you turned "
                        + "clockwise (redo it counter-clockwise, seen from above) or the "
                        + "Pinpoint is mounted upside down (its heading must increase "
                        + "counter-clockwise). Beacon needs counter-clockwise positive.");
                return out;
            }
            double real = turns * 2 * Math.PI;
            double ratio = real / turned;
            if (turns > 0 && Math.abs(ratio - 1) > 0.005) {
                String scalar = yawScalarNow > 0
                        ? String.format("set yawScalar = %.4f", yawScalarNow * ratio)
                        : String.format("multiply the Pinpoint yaw scalar by %.4f", ratio);
                out.add(String.format("Heading scale is off by %.2f%%. Only if you are sure the "
                        + "robot made exactly %d turns: %s in BeaconHardwareConfig.",
                        (1 / ratio - 1) * 100, turns, scalar));
                if (Math.abs(ratio - 1) > 0.05) {
                    out.add("That is more than 5%: goBILDA says the device may be faulty, or "
                            + "the turn count is wrong.");
                }
            } else {
                out.add("Heading scale OK (within 0.5%).");
            }
            double exMm = errorX() * 1000, eyMm = errorY() * 1000;
            double r = Math.hypot(exMm, eyMm);
            out.add(String.format("While spinning, the centre wandered up to %.1f mm "
                    + "(circle radius %.1f mm).", maxDist * 1000, r));
            if (r > 2.0) {
                out.add(String.format("Set xPodOffsetMm = %.1f (now %.1f) and yPodOffsetMm = %.1f "
                                + "(now %.1f) in BeaconHardwareConfig, then repeat. The circle "
                                + "must shrink; if it grows, the signs are the other way round: "
                                + "use xPodOffsetMm = %.1f, yPodOffsetMm = %.1f instead.",
                        xPodOffsetMm - exMm, xPodOffsetMm, yPodOffsetMm + eyMm, yPodOffsetMm,
                        xPodOffsetMm + exMm, yPodOffsetMm - eyMm));
            } else {
                out.add("Pod offsets OK (circle under 2 mm).");
            }
            return out;
        }
    }

    // ================================================================= push

    /** Which way the robot was pushed in a push test. */
    public static final int FORWARD = 0;
    public static final int LEFT = 1;

    /**
     * Push test: the robot is pushed (or driven) a measured distance straight forward or
     * straight to its left, without turning. Start and end poses are the Pinpoint's.
     *
     * @param direction  {@link #FORWARD} or {@link #LEFT}
     * @param distanceM  the distance you measured on the floor, metres
     * @param ticksPerMm the encoder resolution now in use (for the scale advice)
     */
    public static List<String> pushAdvice(int direction, double distanceM,
                                          double x0, double y0, double h0,
                                          double x1, double y1, double h1,
                                          double ticksPerMm) {
        List<String> out = new ArrayList<String>();
        double dx = x1 - x0, dy = y1 - y0;
        double c = Math.cos(h0), s = Math.sin(h0);
        double fwd = c * dx + s * dy;           // in the robot frame at the start
        double left = -s * dx + c * dy;
        double turn = FrameMath.angleDiff(h1, h0);
        boolean isFwd = direction == FORWARD;
        double along = isFwd ? fwd : left;
        double across = isFwd ? left : fwd;
        String pod = isFwd ? "X (forward)" : "Y (strafe)";
        String dirField = isFwd ? "xDirection" : "yDirection";
        out.add(String.format("Measured: forward %.1f mm, left %.1f mm, turned %.1f deg "
                + "(expected %s %.0f mm).", fwd * 1000, left * 1000, Math.toDegrees(turn),
                isFwd ? "forward" : "left", distanceM * 1000));
        if (Math.abs(along) + Math.abs(across) < 0.2 * distanceM) {
            out.add("Almost no motion measured: check that both pods touch the floor and are "
                    + "plugged into the Pinpoint.");
            return out;
        }
        if (Math.abs(across) > Math.abs(along)) {
            out.add("The motion showed up on the wrong axis: the X and Y pod cables are swapped "
                    + "on the Pinpoint (X pod must be the one rolling forward).");
            return out;
        }
        if (along < 0) {
            out.add("The " + pod + " pod counts backwards: flip " + dirField
                    + " (FORWARD <-> REVERSED) in BeaconHardwareConfig, then repeat.");
            return out;
        }
        double scale = along / distanceM;
        if (Math.abs(scale - 1) > 0.01) {
            out.add(String.format("Distance reads %.1f%% %s. If your floor measurement is "
                            + "right, set customTicksPerMm = %.4f (now %.4f). With goBILDA pods "
                            + "this usually means the push was measured wrong or a pod "
                            + "slipped; repeat first.", Math.abs(scale - 1) * 100,
                    scale > 1 ? "long" : "short", ticksPerMm * scale, ticksPerMm));
        } else {
            out.add(String.format("Distance OK (%.2f%% off).", (scale - 1) * 100));
        }
        double crabDeg = Math.toDegrees(Math.atan2(across, along));
        if (Math.abs(crabDeg) > 2) {
            out.add(String.format("Path was %.1f deg off straight: push straighter along a tape "
                    + "line; if it persists the %s pod is not square to the robot.", crabDeg,
                    pod));
        }
        if (Math.abs(Math.toDegrees(turn)) > 2) {
            out.add("The robot turned during the push; repeat without turning.");
        }
        return out;
    }

    // ============================================================ frame check

    /**
     * PARKED-PATH HELPER (no active OpMode calls it since AprilTags were dropped).
     * Compares a Limelight MegaTag1 pose (independent of our heading) with the localizer's
     * pose, the robot standing at a spot whose field coordinates were given to setPose.
     * Explains a units, frame or camera-mounting mismatch in plain words.
     */
    public static List<String> frameAdvice(double locX, double locY, double locH,
                                           double llX, double llY, double llH) {
        List<String> out = new ArrayList<String>();
        double dx = llX - locX, dy = llY - locY;
        double d = Math.hypot(dx, dy);
        double dh = FrameMath.angleDiff(llH, locH);
        out.add(String.format("Limelight MT1 (%.3f, %.3f, %.1f deg) vs Beacon (%.3f, %.3f, "
                        + "%.1f deg): off by %.3f m, %.1f deg.", llX, llY, Math.toDegrees(llH),
                locX, locY, Math.toDegrees(locH), d, Math.toDegrees(dh)));
        if (d < 0.05 && Math.abs(Math.toDegrees(dh)) < 3) {
            out.add("Frames agree. Units, axes and the Limelight camera setup look right.");
            return out;
        }
        double rl = Math.hypot(locX, locY), rll = Math.hypot(llX, llY);
        if (rl > 0.3 && rll > 0.3) {
            if (Math.abs(rl / rll - 39.37) < 3) {
                out.add("Your setPose numbers are about 39 times the Limelight's: you gave "
                        + "inches. Beacon uses METRES.");
                return out;
            }
            double[][] rots = {{90, 0}, {180, 0}, {270, 0}};
            for (double[] r : rots) {
                double a = Math.toRadians(r[0]);
                double qx = Math.cos(a) * locX - Math.sin(a) * locY;
                double qy = Math.sin(a) * locX + Math.cos(a) * locY;
                if (Math.hypot(qx - llX, qy - llY) < 0.1) {
                    out.add(String.format("The Limelight's field frame is turned %.0f deg from the "
                            + "coordinates you used. Use the Limelight's (FTC field) frame for "
                            + "setPose: origin at the field centre, axes as in its field map.",
                            r[0]));
                    return out;
                }
            }
            if (Math.hypot(locX - llX, -locY - llY) < 0.1) {
                out.add("The Limelight's y axis is mirrored relative to your coordinates (y "
                        + "has the opposite sign). Check which wall is +y in its field map.");
                return out;
            }
            if (Math.hypot(-locX - llX, locY - llY) < 0.1) {
                out.add("The Limelight's x axis is mirrored relative to your coordinates (x "
                        + "has the opposite sign).");
                return out;
            }
        }
        if (Math.abs(Math.abs(Math.toDegrees(dh)) - 180) < 15) {
            out.add("Heading is off by about 180 deg: the camera yaw in the Limelight's "
                    + "robot-space settings is backwards, or setPose's heading is.");
        } else if (Math.abs(Math.toDegrees(dh)) >= 3) {
            out.add(String.format("Heading differs by %.1f deg: check the camera yaw in the "
                    + "Limelight robot-space settings, and the heading you gave setPose.",
                    Math.toDegrees(dh)));
        }
        if (d >= 0.05) {
            // Express the position error in the robot frame: a camera-offset error in the
            // Limelight UI shows up as a constant robot-frame offset.
            double c = Math.cos(locH), s = Math.sin(locH);
            double fwd = c * dx + s * dy, left = -s * dx + c * dy;
            out.add(String.format("Position differs by %.0f mm forward, %.0f mm left (robot "
                    + "frame). If this stays the same when the robot turns, correct the "
                    + "camera position in the Limelight's robot-space settings (it is measured "
                    + "from the robot's centre of rotation) or the spot you gave setPose.",
                    fwd * 1000, left * 1000));
        }
        return out;
    }

    /**
     * PARKED-PATH HELPER (no active OpMode calls it since AprilTags were dropped).
     * Heading-sign check: the robot turned (roughly 90 deg) with tags in view. Compares the
     * Pinpoint's heading change with the Limelight MegaTag1 heading change.
     */
    public static List<String> turnSignAdvice(double pinpointTurnRad, double limelightTurnRad) {
        List<String> out = new ArrayList<String>();
        out.add(String.format("Turn: Pinpoint %.1f deg, Limelight MT1 %.1f deg.",
                Math.toDegrees(pinpointTurnRad), Math.toDegrees(limelightTurnRad)));
        if (Math.abs(pinpointTurnRad) < Math.toRadians(30)) {
            out.add("Turn the robot about 90 deg (tags still in view), then press A.");
            return out;
        }
        if (pinpointTurnRad * limelightTurnRad < 0) {
            out.add("The Limelight heading turns the OPPOSITE way to the Pinpoint: one of them "
                    + "is clockwise-positive. Beacon is counter-clockwise positive; check that "
                    + "the Pinpoint is not upside down and that the Limelight field map is the "
                    + "FTC one. MegaTag2 cannot work until this agrees.");
        } else if (Math.abs(limelightTurnRad / pinpointTurnRad - 1) > 0.1) {
            out.add("Same direction but more than 10% different: let the robot settle with "
                    + "several tags in view and repeat.");
        } else {
            out.add("Heading directions agree.");
        }
        return out;
    }
}
