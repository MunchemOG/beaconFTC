package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import java.util.Random;
import org.junit.Test;

public class FrameMathTest {
    private static final double EPS = 1e-12;

    private static double[] rot(double[] t, double x, double y, double z) {
        double[] out = new double[3];
        FrameMath.quatRotate(new double[]{t[3], t[4], t[5], t[6]}, x, y, z, out);
        return out;
    }

    private static void assertVec(double ex, double ey, double ez, double[] v, double tol) {
        assertEquals("x", ex, v[0], tol);
        assertEquals("y", ey, v[1], tol);
        assertEquals("z", ez, v[2], tol);
    }

    @Test
    public void wrapAngleAndDiffHandlePlusMinusPi() {
        assertEquals(0.0, FrameMath.wrapAngle(0.0), EPS);
        assertEquals(-Math.PI, FrameMath.wrapAngle(Math.PI), EPS);
        assertEquals(-Math.PI + 0.1, FrameMath.wrapAngle(Math.PI + 0.1), 1e-12);
        assertEquals(Math.PI - 0.1, FrameMath.wrapAngle(-Math.PI - 0.1), 1e-12);
        assertEquals(0.5, FrameMath.wrapAngle(0.5 + 6 * Math.PI), 1e-12);
        // shortest way from 3.1 to -3.1 is a small positive step through pi
        assertEquals(2 * Math.PI - 6.2, FrameMath.angleDiff(-3.1, 3.1), 1e-12);
        assertEquals(-(2 * Math.PI - 6.2), FrameMath.angleDiff(3.1, -3.1), 1e-12);
    }

    @Test
    public void quatToYawIgnoresRollAndPitch() {
        double[] q = new double[4];
        for (double yaw = -3.0; yaw <= 3.0; yaw += 0.37) {
            FrameMath.quatFromRpy(0.3, -0.4, yaw, q);
            assertEquals(yaw, FrameMath.yawFromQuat(q[0], q[1], q[2], q[3]), 1e-12);
        }
    }

    @Test
    public void rpyIsRzRyRxAboutFixedAxes() {
        double[] q = new double[4];
        FrameMath.quatFromRpy(0, 0, Math.PI / 2, q);       // yaw 90: x -> y
        assertVec(0, 1, 0, rot(new double[]{0, 0, 0, q[0], q[1], q[2], q[3]}, 1, 0, 0), 1e-12);
        FrameMath.quatFromRpy(0, Math.PI / 2, 0, q);       // pitch 90: x -> -z (nose down)
        assertVec(0, 0, -1, rot(new double[]{0, 0, 0, q[0], q[1], q[2], q[3]}, 1, 0, 0), 1e-12);
        FrameMath.quatFromRpy(Math.PI / 2, 0, 0, q);       // roll 90: y -> z
        assertVec(0, 0, 1, rot(new double[]{0, 0, 0, q[0], q[1], q[2], q[3]}, 0, 1, 0), 1e-12);
        // order: roll first, then pitch. x stays x under roll, then pitch 90 sends it to -z;
        // if the order were pitch-then-roll the y axis would differ.
        FrameMath.quatFromRpy(Math.PI / 2, Math.PI / 2, 0, q);
        double[] t = {0, 0, 0, q[0], q[1], q[2], q[3]};
        assertVec(0, 0, -1, rot(t, 1, 0, 0), 1e-12);
        assertVec(1, 0, 0, rot(t, 0, 1, 0), 1e-12);   // y -(roll)-> z -(pitch about y)-> x
    }

    @Test
    public void quaternionMatrixRoundTrip() {
        Random r = new Random(1);
        double[] q = new double[4];
        double[] m = new double[9];
        double[] q2 = new double[4];
        for (int i = 0; i < 2000; i++) {
            for (int k = 0; k < 4; k++) {
                q[k] = r.nextGaussian();
            }
            assertTrue(FrameMath.quatNormalize(q));
            if (q[3] < 0) {
                for (int k = 0; k < 4; k++) {
                    q[k] = -q[k];
                }
            }
            FrameMath.quatToMatrix(q, m);
            FrameMath.matrixToQuat(m, q2);
            for (int k = 0; k < 4; k++) {
                assertEquals(q[k], q2[k], 1e-12);
            }
        }
    }

    @Test
    public void quatNormalizeRejectsGarbage() {
        assertFalse(FrameMath.quatNormalize(new double[]{0, 0, 0, 0}));
        assertFalse(FrameMath.quatNormalize(new double[]{Double.NaN, 0, 0, 1}));
        assertFalse(FrameMath.quatNormalize(new double[]{Double.POSITIVE_INFINITY, 0, 0, 1}));
    }

    @Test
    public void composeInvertRoundTrip() {
        Random r = new Random(2);
        double[] a = new double[7];
        double[] b = new double[7];
        double[] ab = new double[7];
        double[] back = new double[7];
        double[] tmp = new double[4];
        for (int i = 0; i < 500; i++) {
            FrameMath.quatFromRpy(r.nextGaussian(), r.nextGaussian(), r.nextGaussian(), tmp);
            a[0] = r.nextGaussian(); a[1] = r.nextGaussian(); a[2] = r.nextGaussian();
            a[3] = tmp[0]; a[4] = tmp[1]; a[5] = tmp[2]; a[6] = tmp[3];
            FrameMath.quatFromRpy(r.nextGaussian(), r.nextGaussian(), r.nextGaussian(), tmp);
            b[0] = r.nextGaussian(); b[1] = r.nextGaussian(); b[2] = r.nextGaussian();
            b[3] = tmp[0]; b[4] = tmp[1]; b[5] = tmp[2]; b[6] = tmp[3];
            FrameMath.compose(a, b, ab);
            double[] ai = new double[7];
            FrameMath.invert(a, ai);
            FrameMath.compose(ai, ab, back);     // a^-1 * (a*b) == b
            for (int k = 0; k < 7; k++) {
                assertEquals(b[k], back[k], 1e-12);
            }
            // point mapping agrees with composition
            double[] p1 = new double[3];
            double[] p2 = new double[3];
            double[] pb = new double[3];
            FrameMath.transformPoint(b, 0.3, -0.2, 0.9, pb);
            FrameMath.transformPoint(a, pb[0], pb[1], pb[2], p1);
            FrameMath.transformPoint(ab, 0.3, -0.2, 0.9, p2);
            assertVec(p1[0], p1[1], p1[2], p2, 1e-12);
        }
    }

    @Test
    public void composeIsAliasSafe() {
        double[] a = {1, 2, 3, 0, 0, Math.sin(0.4), Math.cos(0.4)};
        double[] b = {0.5, -1, 2, Math.sin(0.2), 0, 0, Math.cos(0.2)};
        double[] expect = new double[7];
        FrameMath.compose(a, b, expect);
        double[] a2 = a.clone();
        FrameMath.compose(a2, b, a2);
        for (int k = 0; k < 7; k++) {
            assertEquals(expect[k], a2[k], 1e-15);
        }
    }

    @Test
    public void zeroAnglesMeanForwardLevelUprightCamera() {
        double[] t = new double[7];
        FrameMath.robotFromCamera(0.1, 0.02, 0.2, 0, 0, 0, t);
        assertEquals(0.1, t[0], EPS);
        assertEquals(0.02, t[1], EPS);
        assertEquals(0.2, t[2], EPS);
        assertVec(1, 0, 0, rot(t, 0, 0, 1), 1e-12);    // lens axis -> robot forward
        assertVec(0, -1, 0, rot(t, 1, 0, 0), 1e-12);   // image right -> robot right
        assertVec(0, 0, -1, rot(t, 0, 1, 0), 1e-12);   // image down -> robot down
    }

    @Test
    public void mountExampleLookingStraightAhead() {
        BeaconConfig.CameraMount m = new BeaconConfig.CameraMount(0.10, 0, 0.20, 0, 0, 0);
        double[] t = m.toTransform();
        assertVec(1, 0, 0, rot(t, 0, 0, 1), 1e-12);
        // a point 1 m in front of the lens is 1.10 m in front of the centre of rotation
        double[] p = new double[3];
        FrameMath.transformPoint(t, 0, 0, 1, p);
        assertVec(1.10, 0, 0.20, p, 1e-12);
    }

    @Test
    public void mountExamplePitchedDown15Degrees() {
        double pitch = Math.toRadians(15);
        double[] t = new BeaconConfig.CameraMount(0.10, 0, 0.20, 0, pitch, 0).toTransform();
        assertVec(Math.cos(pitch), 0, -Math.sin(pitch), rot(t, 0, 0, 1), 1e-12);
        assertEquals(0.9659, Math.cos(pitch), 1e-4);
        assertEquals(0.2588, Math.sin(pitch), 1e-4);
        // image "up" (optical -y) tilts to point forward-up: pitching down lifts the top away
        assertVec(Math.sin(pitch), 0, Math.cos(pitch), rot(t, 0, -1, 0), 1e-12);
    }

    @Test
    public void yawLeftAndRollLeftEdgeUp() {
        double[] t = new double[7];
        FrameMath.robotFromCamera(0, 0, 0, 0, 0, Math.PI / 2, t);
        assertVec(0, 1, 0, rot(t, 0, 0, 1), 1e-12);    // yaw +90: looks left
        FrameMath.robotFromCamera(0, 0, 0, Math.PI / 2, 0, 0, t);
        assertVec(0, 0, 1, rot(t, -1, 0, 0), 1e-12);   // roll +90: image-left edge goes up
    }

    @Test
    public void robotCameraRoundTrip() {
        double[] tRc = new double[7];
        FrameMath.robotFromCamera(0.12, -0.05, 0.3, 0.1, 0.4, -0.7, tRc);
        double[] tCr = new double[7];
        FrameMath.invert(tRc, tCr);
        double[] id = new double[7];
        FrameMath.compose(tRc, tCr, id);
        double[] ident = FrameMath.identity();
        for (int k = 0; k < 7; k++) {
            assertEquals(ident[k], id[k], 1e-12);
        }
    }

    @Test
    public void arCoreIdentityCameraLooksAlongPlusYInZUpWorld() {
        double[] ar = FrameMath.identity();
        double[] out = new double[7];
        FrameMath.fromArCoreCameraPose(ar, out);
        assertVec(0, 1, 0, rot(out, 0, 0, 1), 1e-12);    // GL camera looks down -z (y-up) = +y (z-up)
        assertVec(1, 0, 0, rot(out, 1, 0, 0), 1e-12);    // image right stays +x
        assertVec(0, 0, -1, rot(out, 0, 1, 0), 1e-12);   // image down is -z (z-up)
    }

    @Test
    public void arCorePositionMapsXMinusZY() {
        double[] ar = {1, 2, 3, 0, 0, 0, 1};
        double[] out = new double[7];
        FrameMath.fromArCoreCameraPose(ar, out);
        assertEquals(1, out[0], 1e-12);
        assertEquals(-3, out[1], 1e-12);
        assertEquals(2, out[2], 1e-12);
    }
}
