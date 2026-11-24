package org.firstinspires.ftc.teamcode.beacon.opmodes;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.beacon.BeaconHardwareConfig;

/**
 * Gamepad mecanum driving for the Beacon test OpModes, if (and only if) all four motors named
 * in {@link BeaconHardwareConfig} exist. Otherwise it is disabled, says why, and every method
 * does nothing, so the OpModes work on a robot pushed by hand. Never throws.
 */
final class OptionalDrive {
    private final DcMotor fl, fr, bl, br;
    /** Empty if driving works, otherwise the reason it is off. */
    final String disabledReason;

    OptionalDrive(HardwareMap hw, BeaconHardwareConfig hc) {
        DcMotor a = null, b = null, c = null, d = null;
        String why = "";
        try {
            a = hw.tryGet(DcMotor.class, hc.frontLeftName);
            b = hw.tryGet(DcMotor.class, hc.frontRightName);
            c = hw.tryGet(DcMotor.class, hc.backLeftName);
            d = hw.tryGet(DcMotor.class, hc.backRightName);
            if (a == null || b == null || c == null || d == null) {
                why = "drive motors not all in the configuration ('" + hc.frontLeftName + "', '"
                        + hc.frontRightName + "', '" + hc.backLeftName + "', '"
                        + hc.backRightName + "'): push the robot by hand";
            } else {
                DcMotorSimple.Direction left = hc.reverseLeftSide
                        ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
                DcMotorSimple.Direction right = hc.reverseLeftSide
                        ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
                a.setDirection(left);
                c.setDirection(left);
                b.setDirection(right);
                d.setDirection(right);
                for (DcMotor m : new DcMotor[]{a, b, c, d}) {
                    m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    m.setPower(0);
                }
            }
        } catch (RuntimeException e) {
            why = "drive disabled: " + e;
        }
        if (!why.isEmpty()) {
            a = b = c = d = null;
        }
        fl = a;
        fr = b;
        bl = c;
        br = d;
        disabledReason = why;
    }

    boolean enabled() {
        return fl != null;
    }

    /** Robot-centric mecanum: left stick translates, right stick x turns. Slow on purpose. */
    void drive(Gamepad g, double maxPower) {
        if (fl == null) {
            return;
        }
        double fwd = -g.left_stick_y, strafe = g.left_stick_x, turn = g.right_stick_x;
        double flp = fwd + strafe + turn, frp = fwd - strafe - turn;
        double blp = fwd - strafe + turn, brp = fwd + strafe - turn;
        double max = Math.max(1.0, Math.max(Math.max(Math.abs(flp), Math.abs(frp)),
                Math.max(Math.abs(blp), Math.abs(brp))));
        try {
            fl.setPower(maxPower * flp / max);
            fr.setPower(maxPower * frp / max);
            bl.setPower(maxPower * blp / max);
            br.setPower(maxPower * brp / max);
        } catch (RuntimeException e) {
            // a motor write failing must not stop the localizer test
        }
    }

    void stop() {
        if (fl == null) {
            return;
        }
        try {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        } catch (RuntimeException e) {
            // ignore
        }
    }
}
