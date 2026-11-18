package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.List;
import org.junit.Test;

public class CalibrationMathTest {

    /**
     * Simulates what a Pinpoint reports for a robot spinning about its true centre, from the
     * rigid-body velocity of each pod's contact point (v = w x r) and the Pinpoint's documented
     * reconstruction with the CONFIGURED offsets: forward pod at lateral position L (left +),
     * strafe pod at forward position F; v_cx = podX + w*Lc, v_cy = podY - w*Fc.
     */
    private static CalibrationMath.Spin simulateSpin(double trueL, double trueF, double cfgL,
                                                     double cfgF, double turns, double scale) {
        CalibrationMath.Spin spin = new CalibrationMath.Spin();
        int steps = 4000;
        double dth = turns * 2 * Math.PI / steps;
        double x = 0, y = 0, h = 0, hReported = 0;
        spin.add(x, y, hReported);
        for (int i = 0; i < steps; i++) {
            // contact point velocity (per unit time, w = dth): w z x r
            double podFwd = -dth * trueL;          // forward pod at (0, L): v = (-wL, 0)
            double podLeft = dth * trueF;          // strafe pod at (F, 0): v = (0, wF)
            double vcx = podFwd + dth * cfgL;      // Pinpoint's centre estimate (robot frame)
            double vcy = podLeft - dth * cfgF;
            double mid = h + dth / 2;
            x += Math.cos(mid) * vcx - Math.sin(mid) * vcy;
            y += Math.sin(mid) * vcx + Math.cos(mid) * vcy;
            h += dth;
            hReported = h * scale;
            spin.add(x, y, FrameMath.wrapAngle(hReported));
        }
        return spin;
    }

    private static String join(List<String> l) {
        StringBuilder sb = new StringBuilder();
        for (String s : l) {
            sb.append(s).append('\n');
        }
        return sb.toString();
    }

    @Test
    public void spinRecoversTheTruePodOffsets() {
        // true: X pod 60 mm left, Y pod 110 mm back; configured both as 0
        CalibrationMath.Spin s = simulateSpin(0.060, -0.110, 0, 0, 5, 1.0);
        assertEquals(5 * 2 * Math.PI, s.totalTurnRad(), 1e-9);
        assertEquals("e = Lc - L", -0.060, s.errorX(), 1e-6);
        assertEquals("e = F - Fc", -0.110, s.errorY(), 1e-6);
        assertTrue("the centre draws a circle", s.maxWanderM() > 0.2);
        assertEquals("but returns after whole turns", 0, s.endLeakM(), 1e-6);
        String advice = join(s.advice(5, 0, 0, 0));
        assertTrue(advice, advice.contains("xPodOffsetMm = 60.0"));
        assertTrue(advice, advice.contains("yPodOffsetMm = -110.0"));
        assertTrue(advice, advice.contains("Heading scale OK"));
    }

    @Test
    public void correctOffsetsGiveNoCircle() {
        CalibrationMath.Spin s = simulateSpin(0.060, -0.110, 0.060, -0.110, 3, 1.0);
        assertEquals(0, s.maxWanderM(), 1e-9);
        assertTrue(join(s.advice(3, 60, -110, 0)).contains("Pod offsets OK"));
    }

    @Test
    public void spinFindsAHeadingScaleErrorAndAClockwiseHeading() {
        CalibrationMath.Spin s = simulateSpin(0, 0, 0, 0, 5, 1.02);   // reads 2% long
        String a = join(s.advice(5, 0, 0, 1.0));
        assertTrue(a, a.contains("yawScalar = 0.98"));
        CalibrationMath.Spin cw = simulateSpin(0, 0, 0, 0, 2, -1.0);   // heading clockwise
        String b = join(cw.advice(2, 0, 0, 0));
        assertTrue(b, b.contains("upside down"));
    }

    @Test
    public void pushAdviceFindsDirectionSwapAndScale() {
        double t = 19.89436789;
        // forward push of 1 m that read backwards
        String a = join(CalibrationMath.pushAdvice(CalibrationMath.FORWARD, 1.0,
                0, 0, 0, -1.0, 0, 0, t));
        assertTrue(a, a.contains("flip xDirection"));
        // forward push that showed up as sideways: swapped cables
        String b = join(CalibrationMath.pushAdvice(CalibrationMath.FORWARD, 1.0,
                0, 0, 0, 0.02, 0.98, 0, t));
        assertTrue(b, b.contains("swapped"));
        // 3% long, robot started facing +y in the Pinpoint frame
        String c = join(CalibrationMath.pushAdvice(CalibrationMath.FORWARD, 1.0,
                0.5, 0.5, Math.PI / 2, 0.5, 1.53, Math.PI / 2, t));
        assertTrue(c, c.contains(String.format("customTicksPerMm = %.4f", t * 1.03)));
        // left push read as right
        String d = join(CalibrationMath.pushAdvice(CalibrationMath.LEFT, 1.0,
                0, 0, 0, 0, -1.0, 0, t));
        assertTrue(d, d.contains("flip yDirection"));
        String ok = join(CalibrationMath.pushAdvice(CalibrationMath.LEFT, 1.0,
                0, 0, 0, 0.004, 1.003, 0.001, t));
        assertTrue(ok, ok.contains("Distance OK"));
    }

    @Test
    public void frameAdviceExplainsTheUsualMismatches() {
        assertTrue(join(CalibrationMath.frameAdvice(-1.2, 0.3, Math.PI, -1.21, 0.31, Math.PI))
                .contains("Frames agree"));
        assertTrue(join(CalibrationMath.frameAdvice(-47.2, 11.8, 0, -1.2, 0.3, 0))
                .contains("inches"));
        // Limelight frame turned 90 deg: (x, y) -> (-y, x)
        assertTrue(join(CalibrationMath.frameAdvice(-1.2, 0.3, 0, -0.3, -1.2, 0))
                .contains("turned 90"));
        assertTrue(join(CalibrationMath.frameAdvice(-1.2, 0.3, 0, -1.2, -0.3, 0))
                .contains("y axis is mirrored"));
        assertTrue(join(CalibrationMath.frameAdvice(-1.2, 0.3, 0, -1.2, 0.3, Math.PI - 0.02))
                .contains("180"));
        // camera position wrong in the Limelight UI: 8 cm, robot facing +y
        String cam = join(CalibrationMath.frameAdvice(-1.2, 0.3, Math.PI / 2,
                -1.2, 0.38, Math.PI / 2));
        assertTrue(cam, cam.contains("80 mm forward"));
    }

    @Test
    public void turnSignAdviceCatchesOppositeConventions() {
        assertTrue(join(CalibrationMath.turnSignAdvice(1.57, -1.55)).contains("OPPOSITE"));
        assertTrue(join(CalibrationMath.turnSignAdvice(1.57, 1.55)).contains("agree"));
        assertTrue(join(CalibrationMath.turnSignAdvice(0.1, 0.1)).contains("about 90"));
    }
}
