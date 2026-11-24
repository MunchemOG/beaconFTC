package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import android.content.Context;

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.junit.Test;

public class BeaconLocalizerTest {

    private static final long T0 = 1000000000L;
    private static final long DT = 10000000L;   // 100 Hz

    /** Off-centre, pitched down and yawed camera: nothing about it is convenient. */
    private static final String MOUNT =
            "{\"x\":0.18,\"y\":-0.07,\"z\":0.22,\"roll\":0.03,\"pitch\":0.26,\"yaw\":0.12}";

    private static BeaconConfig cfg(String mount, double gXY, double gH, double maxJumpM)
            throws BeaconException {
        return BeaconConfig.parse("{\"tier\":\"fake\",\"T_robot_cam\":" + mount
                + ",\"fix\":{\"enabled\":true,\"gainXY\":" + gXY + ",\"gainHeading\":" + gH
                + ",\"maxJumpM\":" + maxJumpM + ",\"maxJumpRad\":0.5,"
                + "\"maxSpeedMps\":10,\"maxOmegaRadPs\":20}}");
    }

    private static BeaconConfig cfg() throws BeaconException {
        return cfg(MOUNT, 1.0, 1.0, 0.5);
    }

    private static BeaconConfig.CameraMount mount(BeaconConfig c) {
        return c.cameraMount;
    }

    /** Settable tag source. */
    private static final class FakeTags implements FieldFixSource {
        volatile FieldFix fix;

        @Override
        public FieldFix latestFix() {
            return fix;
        }
    }

    private static final class FakeClock implements BeaconLocalizer.Clock {
        long now = 5000;

        @Override
        public long nanoTime() {
            return now;
        }
    }

    private static FakePoseSource started(FakePoseSource s) throws BeaconException {
        s.start(null, null);
        return s;
    }

    // ------------------------------------------------------------------ basics

    @Test
    public void statusIsInitializingBeforeAlignmentAndPoseIsLastSetPose() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = started(FakePoseSource.squareLoop(mount(c), 1, 0.5, 1, 100,
                0, 0, 0, T0));
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        assertEquals(BeaconStatus.INITIALIZING, loc.update().status);
        src.advance();
        Pose2dStamped p = loc.update();
        assertEquals("good sample but no alignment yet", BeaconStatus.INITIALIZING, p.status);
        assertEquals(0, p.x, 0);
        assertEquals(0, p.y, 0);
        assertFalse(loc.isAligned());
        loc.setPose(2.0, 1.0, 0.5);
        p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(2.0, p.x, 1e-12);
        assertEquals(1.0, p.y, 1e-12);
        assertEquals(0.5, p.heading, 1e-12);
    }

    @Test
    public void squareLoopReturnsToStartWithinAMillimetre() throws Exception {
        BeaconConfig c = cfg();
        double x0 = 3.0, y0 = -2.0, yaw0 = 0.7;
        FakePoseSource src = started(FakePoseSource.squareLoop(mount(c), 1.2, 0.6, 1.5, 100,
                x0, y0, yaw0, T0));
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(0, 0, 0);      // before any sample: applied when the first one arrives
        int n = 0;
        double maxErr = 0;
        double cr = Math.cos(-yaw0), sr = Math.sin(-yaw0);
        while (src.advance()) {
            Pose2dStamped p = loc.update();
            assertEquals(BeaconStatus.TRACKING, p.status);
            // independent expectation: robot pose in the source world, re-based to (0,0,0)
            Pose3dStamped s = src.latest();
            double[] camToRobot = new double[7];
            FrameMath.invert(mount(c).toTransform(), camToRobot);
            double[] tw = {s.x, s.y, s.z, s.qx, s.qy, s.qz, s.qw};
            double[] robot = new double[7];
            FrameMath.compose(tw, camToRobot, robot);
            double ex = cr * (robot[0] - x0) - sr * (robot[1] - y0);
            double ey = sr * (robot[0] - x0) + cr * (robot[1] - y0);
            double eh = FrameMath.wrapAngle(FrameMath.yawFromQuat(robot[3], robot[4], robot[5],
                    robot[6]) - yaw0);
            maxErr = Math.max(maxErr, Math.hypot(p.x - ex, p.y - ey));
            assertEquals(0, FrameMath.angleDiff(p.heading, eh), 1e-9);
            n++;
        }
        assertTrue(n > 300);
        assertTrue("tracks independent model, max err " + maxErr, maxErr < 1e-9);
        Pose2dStamped last = loc.getPose();
        assertEquals(0, last.x, 1e-3);
        assertEquals(0, last.y, 1e-3);
        assertEquals(0, FrameMath.angleDiff(last.heading, 0), 1e-3);
        assertEquals("far tighter than a millimetre", 0, Math.hypot(last.x, last.y), 1e-9);
        assertEquals(0, loc.getSamplesRejected());
    }

    @Test
    public void pureSpinProducesZeroTranslationDespiteTheLeverArm() throws Exception {
        BeaconConfig c = cfg("{\"x\":0.30,\"y\":0.12,\"z\":0.2,\"roll\":0.05,\"pitch\":0.4,"
                + "\"yaw\":-0.3}", 1, 1, 0.5);
        double x0 = -1.5, y0 = 0.8, yaw0 = 2.0;
        FakePoseSource src = started(FakePoseSource.spinInPlace(mount(c), x0, y0, yaw0,
                4 * Math.PI, 400, T0, DT));
        // sanity: the camera itself sweeps a circle of radius ~0.3 m, so the test has teeth
        double minCx = 1e9, maxCx = -1e9;
        for (int i = 0; i < src.size(); i++) {
            minCx = Math.min(minCx, src.sample(i).x);
            maxCx = Math.max(maxCx, src.sample(i).x);
        }
        assertTrue("camera moves " + (maxCx - minCx), maxCx - minCx > 0.5);

        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(x0, y0, yaw0);   // identity alignment: output == source-world robot pose
        double maxDev = 0;
        double prevH = yaw0;
        while (src.advance()) {
            Pose2dStamped p = loc.update();
            maxDev = Math.max(maxDev, Math.hypot(p.x - x0, p.y - y0));
            double step = Math.abs(FrameMath.angleDiff(p.heading, prevH));
            assertTrue("heading steps smoothly through the wrap, step " + step, step < 0.05);
            prevH = p.heading;
        }
        assertTrue("translation during spin " + maxDev, maxDev < 1e-9);
        assertEquals(0, FrameMath.angleDiff(loc.getPose().heading, yaw0 + 4 * Math.PI), 1e-9);
    }

    @Test
    public void tiltedRobotStillGivesCorrectPlanarPose() throws Exception {
        BeaconConfig c = cfg();
        // robot on a slight slope: roll 0.05, pitch -0.04, standing at (1, 2) facing 1.2
        FakePoseSource src = started(new FakePoseSource());
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        src.publish(FakePoseSource.cameraSample(mount(c), T0, 1, 2, 0.1, 0.05, -0.04, 1.2,
                BeaconStatus.TRACKING));
        loc.setPose(1, 2, 1.2);
        loc.update();
        src.publish(FakePoseSource.cameraSample(mount(c), T0 + DT, 1.5, 2.5, 0.1, 0.05, -0.04,
                1.2, BeaconStatus.TRACKING));
        Pose2dStamped p = loc.update();
        assertEquals(1.5, p.x, 1e-9);
        assertEquals(2.5, p.y, 1e-9);
        assertEquals(1.2, p.heading, 1e-9);
        assertEquals(0.1, loc.getRobotZ(), 1e-9);
    }

    // --------------------------------------------------------------- tag fixes

    /** Straight drive along sw +x at 1 m/s; returns sources; robot yaw0 in the source world. */
    private static FakePoseSource straightLine(BeaconConfig c, int n, double yaw0)
            throws BeaconException {
        List<Pose3dStamped> l = new ArrayList<Pose3dStamped>();
        for (int i = 0; i < n; i++) {
            double d = 0.01 * i;
            l.add(FakePoseSource.cameraSample(mount(c), T0 + i * DT,
                    10 + d * Math.cos(yaw0), -4 + d * Math.sin(yaw0), 0, 0, 0, yaw0,
                    BeaconStatus.TRACKING));
        }
        return started(new FakePoseSource(l));
    }

    @Test
    public void tagFixShiftsAlignmentAndMotionStaysContinuous() throws Exception {
        BeaconConfig c = cfg(MOUNT, 0.5, 0.5, 0.5);
        FakePoseSource src = straightLine(c, 200, 0.4);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 0.4);
        List<Pose2dStamped> out = new ArrayList<Pose2dStamped>();
        for (int i = 0; i < 100; i++) {
            src.advance();
            out.add(loc.update());
        }
        Pose2dStamped before = out.get(99);
        // tag says: 8 cm to the left, 3 cm back, 0.1 rad more CCW than we think
        double ex = -0.03, ey = 0.08, eh = 0.1;
        tags.fix = FieldFix.full(before.x + ex, before.y + ey, before.heading + eh,
                src.latest().tNs);
        Pose2dStamped afterFix = loc.update();       // no new source sample, fix only
        assertEquals(1, loc.getFixesApplied());
        assertEquals("half the position error removed", before.x + 0.5 * ex, afterFix.x, 1e-9);
        assertEquals(before.y + 0.5 * ey, afterFix.y, 1e-9);
        assertEquals("half the heading error removed", before.heading + 0.5 * eh,
                afterFix.heading, 1e-9);

        // Afterwards the source keeps moving 1 cm/step; output steps must stay 1 cm, with no
        // second jump, and head along the corrected heading.
        Pose2dStamped prev = afterFix;
        for (int i = 100; i < 200; i++) {
            src.advance();
            Pose2dStamped p = loc.update();
            assertEquals("step length", 0.01, Math.hypot(p.x - prev.x, p.y - prev.y), 1e-9);
            assertEquals(afterFix.heading, p.heading, 1e-9);
            prev = p;
        }
        double travelled = Math.hypot(prev.x - afterFix.x, prev.y - afterFix.y);
        assertEquals(1.00, travelled, 1e-9);
        // direction of travel follows the corrected heading
        assertEquals(afterFix.heading, Math.atan2(prev.y - afterFix.y, prev.x - afterFix.x), 1e-9);
    }

    @Test
    public void fullGainTagFixSnapsToTheFixAndLaterIdenticalFixChangesNothing() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 100, -2.9);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(5, 5, -2.9);
        for (int i = 0; i < 50; i++) {
            src.advance();
            loc.update();
        }
        long t = src.latest().tNs;
        Pose2dStamped was = loc.getPose();
        double fx = was.x + 0.4, fy = was.y - 0.2;
        tags.fix = FieldFix.full(fx, fy, -2.9 + 0.2, t);
        Pose2dStamped p = loc.update();
        assertEquals(fx, p.x, 1e-9);
        assertEquals(fy, p.y, 1e-9);
        assertEquals(FrameMath.wrapAngle(-2.7), p.heading, 1e-9);
        // a second fix that agrees with the corrected pose leaves it alone
        tags.fix = FieldFix.full(fx, fy, -2.7, t + 1);
        Pose2dStamped q = loc.update();
        assertEquals(p.x, q.x, 1e-9);
        assertEquals(p.y, q.y, 1e-9);
        assertEquals(2, loc.getFixesApplied());
    }

    @Test
    public void repeatedFixesConvergeGeometricallyWithPartialGain() throws Exception {
        BeaconConfig c = cfg(MOUNT, 0.5, 0.5, 5.0);
        FakePoseSource src = straightLine(c, 10, 0.0);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 0);
        src.advance();
        loc.update();
        // truth is 0.4 m away in x, 0.3 rad rotated; the robot is not moving
        double err0x = 0.4;
        for (int k = 1; k <= 5; k++) {
            tags.fix = FieldFix.full(0.4, 0, 0.3, T0 + k);
            Pose2dStamped p = loc.update();
            assertEquals(0.4 * (1 - Math.pow(0.5, k)), p.x, 1e-9);
            assertEquals(0.3 * (1 - Math.pow(0.5, k)), p.heading, 1e-9);
        }
    }

    @Test
    public void tagFixIsComparedAtTheFixTimeNotNow() throws Exception {
        BeaconConfig c = cfg(MOUNT, 1, 1, 0.5);
        FakePoseSource src = straightLine(c, 100, 0.0);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 0);
        List<Pose2dStamped> out = new ArrayList<Pose2dStamped>();
        for (int i = 0; i < 60; i++) {
            src.advance();
            out.add(loc.update());
        }
        // A perfect fix, but of the pose 3.5 samples ago (halfway between samples 55 and 56).
        Pose2dStamped a = out.get(55);
        Pose2dStamped b = out.get(56);
        long tFix = (a.tNs + b.tNs) / 2;
        tags.fix = FieldFix.full((a.x + b.x) / 2, (a.y + b.y) / 2, 0, tFix);
        Pose2dStamped now = out.get(59);
        Pose2dStamped p = loc.update();
        assertEquals("latency does not become a position error", now.x, p.x, 1e-9);
        assertEquals(now.y, p.y, 1e-9);
        assertEquals(1, loc.getFixesApplied());
    }

    @Test
    public void tooFarFixIsRejectedAndPoseUnchanged() throws Exception {
        BeaconConfig c = cfg(MOUNT, 1, 1, 0.5);
        FakePoseSource src = straightLine(c, 50, 0.0);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 0);
        for (int i = 0; i < 30; i++) {
            src.advance();
            loc.update();
        }
        Pose2dStamped before = loc.getPose();
        tags.fix = FieldFix.full(before.x + 2.0, before.y, 0, src.latest().tNs);
        Pose2dStamped p = loc.update();
        assertEquals(before.x, p.x, 1e-12);
        assertEquals(before.y, p.y, 1e-12);
        assertEquals(1, loc.getFixesRejected());
        assertEquals(0, loc.getFixesApplied());
        // a heading-only jump is rejected too
        tags.fix = FieldFix.full(before.x, before.y, 1.0, src.latest().tNs + 1);
        loc.update();
        assertEquals(2, loc.getFixesRejected());
        // and a just-inside fix is accepted
        tags.fix = FieldFix.full(before.x + 0.45, before.y, 0, src.latest().tNs + 2);
        assertEquals(before.x + 0.45, loc.update().x, 1e-9);
    }

    @Test
    public void firstTagFixSetsAlignmentWhenNothingWasSet() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 30, 1.0);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        for (int i = 0; i < 10; i++) {
            src.advance();
            loc.update();
        }
        assertEquals(BeaconStatus.INITIALIZING, loc.getStatus());
        // far away from anything, which would fail the jump test if it applied
        tags.fix = FieldFix.full(-30, 40, 3.0, src.latest().tNs);
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(-30, p.x, 1e-9);
        assertEquals(40, p.y, 1e-9);
        assertEquals(3.0, p.heading, 1e-9);
        assertTrue(loc.isAligned());
    }

    @Test
    public void staleNullAndDisabledFixesAreIgnored() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 400, 0.0);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 0);
        for (int i = 0; i < 300; i++) {
            src.advance();
            loc.update();
        }
        Pose2dStamped before = loc.getPose();
        tags.fix = null;
        loc.update();
        // 2.9 s old: older than the 1 s limit
        tags.fix = FieldFix.full(before.x + 0.1, before.y, 0, T0);
        loc.update();
        assertEquals(before.x, loc.getPose().x, 1e-12);
        assertEquals(1, loc.getFixesRejected());
        // NaN fix and non-tracking fix
        tags.fix = FieldFix.full(Double.NaN, 0, 0, src.latest().tNs);
        loc.update();
        tags.fix = new FieldFix(0, 0, 0, src.latest().tNs + 1, false, false, "empty");
        loc.update();
        assertEquals(before.x, loc.getPose().x, 1e-12);
        assertEquals(3, loc.getFixesRejected());
        // blend switched off: consumed but ignored
        loc.setFixBlendEnabled(false);
        tags.fix = FieldFix.full(before.x + 0.1, before.y, 0, src.latest().tNs + 2);
        loc.update();
        assertEquals(before.x, loc.getPose().x, 1e-12);
        loc.setFixBlendEnabled(true);
        loc.update();    // the old fix must not apply late
        assertEquals(before.x, loc.getPose().x, 1e-12);
        assertEquals(0, loc.getFixesApplied());
    }

    @Test
    public void headingBlendTakesTheShortWayAcrossPlusMinusPi() throws Exception {
        BeaconConfig c = cfg(MOUNT, 1, 1, 0.5);
        FakePoseSource src = straightLine(c, 10, 0.0);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 3.12);
        src.advance();
        loc.update();
        // true heading is -3.13 (= 3.1532...), 0.0232 rad further CCW through the wrap
        tags.fix = FieldFix.full(0, 0, -3.13, src.latest().tNs);
        Pose2dStamped p = loc.update();
        assertEquals(1, loc.getFixesApplied());
        assertEquals(-3.13, p.heading, 1e-9);
        assertTrue(p.heading >= -Math.PI && p.heading < Math.PI);
        // half gain: lands halfway, still the short way round
        BeaconConfig half = cfg(MOUNT, 1, 0.5, 0.5);
        FakePoseSource src2 = straightLine(half, 10, 0.0);
        FakeTags tags2 = new FakeTags();
        BeaconLocalizer loc2 = new BeaconLocalizer(src2, half, tags2, null);
        loc2.setPose(0, 0, 3.12);
        src2.advance();
        loc2.update();
        tags2.fix = FieldFix.full(0, 0, -3.13, src2.latest().tNs);
        double expect = FrameMath.wrapAngle(3.12 + 0.5 * (2 * Math.PI - 3.13 - 3.12));
        assertEquals(expect, loc2.update().heading, 1e-9);
    }

    @Test
    public void tagFixWithinTheRecoveryWindowAfterLossIsAcceptedWholeEvenIfFar() throws Exception {
        BeaconConfig c = cfg(MOUNT, 0.2, 0.2, 0.5);
        FakePoseSource src = straightLine(c, 50, 0.0);
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 0);
        for (int i = 0; i < 20; i++) {
            src.advance();
            loc.update();
        }
        src.publish(new Pose3dStamped(0, 0, 0, 0, 0, 0, 1, T0 + 20 * DT, BeaconStatus.LOST));
        assertEquals(BeaconStatus.LOST, loc.update().status);
        Pose2dStamped held = loc.getPose();
        // tracking resumes
        src.publish(FakePoseSource.cameraSample(mount(c), T0 + 21 * DT, 10.2, -4, 0, 0, 0, 0,
                BeaconStatus.TRACKING));
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
        // fix is 3 m away: would fail the jump test, but it was captured within the 1 s
        // recovery window after tracking resumed (finding 1: the waiver is time-limited)
        tags.fix = FieldFix.full(held.x + 3, held.y, 0, T0 + 21 * DT);
        Pose2dStamped p = loc.update();
        assertEquals(held.x + 3, p.x, 1e-9);
        assertEquals(1, loc.getFixesApplied());
        // and the exemption is used up
        tags.fix = FieldFix.full(held.x + 6, held.y, 0, T0 + 21 * DT + 1);
        loc.update();
        assertEquals(1, loc.getFixesRejected());
    }

    // ----------------------------------------------------------------- velocity

    @Test
    public void velocityIsInTheRobotFrame() throws Exception {
        BeaconConfig c = cfg();
        double yaw = 0.9;
        // robot translates at (vx=0.6 forward, vy=-0.25 left) in its own frame, no rotation
        double vsx = 0.6 * Math.cos(yaw) + 0.25 * Math.sin(yaw);
        double vsy = 0.6 * Math.sin(yaw) - 0.25 * Math.cos(yaw);
        List<Pose3dStamped> l = new ArrayList<Pose3dStamped>();
        for (int i = 0; i < 20; i++) {
            double t = i * 0.01;
            l.add(FakePoseSource.cameraSample(mount(c), T0 + i * DT, 2 + vsx * t, 1 + vsy * t, 0,
                    0, 0, yaw, BeaconStatus.TRACKING));
        }
        FakePoseSource src = started(new FakePoseSource(l));
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(7, 7, 2.0);   // any alignment: robot-frame velocity must not care
        assertEquals(0, loc.getVelocity().vx, 0);
        for (int i = 0; i < 20; i++) {
            src.advance();
            loc.update();
        }
        Velocity2d v = loc.getVelocity();
        assertEquals(0.6, v.vx, 1e-9);
        assertEquals(-0.25, v.vy, 1e-9);
        assertEquals(0, v.omega, 1e-9);
    }

    @Test
    public void angularVelocityIsCorrectAcrossTheWrapAndWithTagFixesPresent() throws Exception {
        BeaconConfig c = cfg();
        // spin at 2 rad/s about the centre starting just below pi in the SOURCE world
        FakePoseSource src = started(FakePoseSource.spinInPlace(mount(c), 1, 1, Math.PI - 0.05,
                3.0, 150, T0, DT));
        FakeTags tags = new FakeTags();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        loc.setPose(0, 0, 0);
        int i = 0;
        while (src.advance()) {
            loc.update();
            if (++i == 75) {   // a big tag correction mid-spin must not show up as velocity
                tags.fix = FieldFix.full(0.2, -0.1, loc.getPose().heading + 0.3,
                        src.latest().tNs);
            }
            if (i > 3) {
                Velocity2d v = loc.getVelocity();
                assertEquals("omega at sample " + i, 2.0, v.omega, 1e-9);
                assertEquals(0, v.vx, 1e-9);
                assertEquals(0, v.vy, 1e-9);
            }
        }
        assertEquals(1, loc.getFixesApplied());
    }

    // -------------------------------------------------------------- bad inputs

    private static void assertHolding(BeaconLocalizer loc, Pose2dStamped good, int status) {
        Pose2dStamped p = loc.getPose();
        assertEquals(good.x, p.x, 0);
        assertEquals(good.y, p.y, 0);
        assertEquals(good.heading, p.heading, 0);
        assertEquals(status, p.status);
        assertTrue(p.isFinite());
    }

    @Test
    public void nanSampleHoldsLastPoseReportsLostAndRecovers() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 20, 0.0);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(0, 0, 0);
        for (int i = 0; i < 10; i++) {
            src.advance();
            loc.update();
        }
        Pose2dStamped good = loc.getPose();
        long t = src.latest().tNs;
        double[] bad = {Double.NaN, Double.POSITIVE_INFINITY};
        for (int k = 0; k < bad.length; k++) {
            src.publish(new Pose3dStamped(bad[k], 0, 0, 0, 0, 0, 1, t + (k + 1),
                    BeaconStatus.TRACKING));
            loc.update();
            assertHolding(loc, good, BeaconStatus.LOST);
        }
        src.publish(new Pose3dStamped(0, 0, 0, 0, 0, 0, Double.NaN, t + 3,
                BeaconStatus.TRACKING));
        loc.update();
        assertHolding(loc, good, BeaconStatus.LOST);
        assertEquals(3, loc.getSamplesRejected());
        // the next good sample resumes tracking, nothing was poisoned
        src.advance();
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertTrue(p.isFinite());
        assertEquals(good.x + 0.01, p.x, 1e-9);
    }

    @Test
    public void nonUnitQuaternionIsRejected() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 5, 0.0);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(0, 0, 0);
        src.advance();
        Pose2dStamped good = loc.update();
        src.publish(new Pose3dStamped(1, 1, 1, 0, 0, 0, 2.0, T0 + DT, BeaconStatus.TRACKING));
        loc.update();
        assertHolding(loc, good, BeaconStatus.LOST);
        src.publish(new Pose3dStamped(1, 1, 1, 0, 0, 0, 0.0, T0 + 2 * DT, BeaconStatus.TRACKING));
        loc.update();
        assertHolding(loc, good, BeaconStatus.LOST);
        // slightly off (float rounding in a source) is tolerated
        src.publish(FakePoseSource.cameraSample(mount(c), T0 + 3 * DT, 10.0, -4.0, 0, 0, 0, 0,
                BeaconStatus.TRACKING));
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
    }

    @Test
    public void backwardsAndRepeatedTimestamps() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 20, 0.0);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(0, 0, 0);
        for (int i = 0; i < 10; i++) {
            src.advance();
            loc.update();
        }
        Pose2dStamped good = loc.getPose();
        long t = src.latest().tNs;
        // repeat of the same timestamp in a different object: ignored quietly, still tracking
        src.publish(FakePoseSource.cameraSample(mount(c), t, 50, 50, 0, 0, 0, 0,
                BeaconStatus.TRACKING));
        Pose2dStamped p = loc.update();
        assertEquals(good.x, p.x, 0);
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(0, loc.getSamplesRejected());
        // going backwards: rejected, LOST, pose held
        src.publish(FakePoseSource.cameraSample(mount(c), t - DT, 50, 50, 0, 0, 0, 0,
                BeaconStatus.TRACKING));
        loc.update();
        assertHolding(loc, good, BeaconStatus.LOST);
        assertEquals(1, loc.getSamplesRejected());
        // the sample after resumes forward time
        src.advance();
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
    }

    @Test
    public void sourceReportedLostAndInitializingAreMappedAndHeld() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 20, 0.0);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(0, 0, 0);
        for (int i = 0; i < 10; i++) {
            src.advance();
            loc.update();
        }
        Pose2dStamped good = loc.getPose();
        long t = src.latest().tNs;
        src.publish(new Pose3dStamped(9, 9, 9, 0, 0, 0, 1, t + DT, BeaconStatus.LOST));
        loc.update();
        assertHolding(loc, good, BeaconStatus.LOST);
        src.publish(new Pose3dStamped(9, 9, 9, 0, 0, 0, 1, t + 2 * DT, BeaconStatus.INITIALIZING));
        loc.update();
        assertHolding(loc, good, BeaconStatus.INITIALIZING);
        src.publish(new Pose3dStamped(9, 9, 9, 0, 0, 0, 1, t + 3 * DT, BeaconStatus.INVALID));
        loc.update();
        assertHolding(loc, good, BeaconStatus.INVALID);
        assertEquals(BeaconStatus.INVALID, loc.getStatus());
        assertFalse(loc.getVelocity() == null);
    }

    @Test
    public void silentSourceBecomesLostAfterTheTimeout() throws Exception {
        BeaconConfig c = cfg();    // sourceTimeoutS default 0.5
        FakePoseSource src = straightLine(c, 20, 0.0);
        FakeClock clock = new FakeClock();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, null, null, clock);
        loc.setPose(0, 0, 0);
        src.advance();
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
        clock.now += 400000000L;
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
        clock.now += 200000000L;     // 0.6 s since the last new sample
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.LOST, p.status);
        assertEquals(0, p.x, 1e-9);
        src.advance();
        assertEquals(BeaconStatus.TRACKING, loc.update().status);
    }

    @Test
    public void misbehavingSourceAndTagSourceCannotThrowIntoTheLoop() throws Exception {
        BeaconConfig c = cfg();
        PoseSource boom = new PoseSource() {
            public void start(Context ctx, BeaconConfig cfg) {}
            public void stop() {}
            public Pose3dStamped latest() { throw new IllegalStateException("camera died"); }
            public boolean isTracking() { return false; }
            public String debugLine() { return "boom"; }
        };
        BeaconLocalizer loc = new BeaconLocalizer(boom, c);
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.LOST, p.status);
        assertTrue(loc.getLastRejectReason().contains("camera died"));
        assertNotNull(loc.debugLine());

        FieldFixSource badTags = new FieldFixSource() {
            public FieldFix latestFix() { throw new RuntimeException("tag thread"); }
        };
        FakePoseSource src = straightLine(c, 10, 0.0);
        BeaconLocalizer loc2 = new BeaconLocalizer(src, c, badTags, null);
        loc2.setPose(0, 0, 0);
        for (int i = 0; i < 10; i++) {
            src.advance();
            // must not throw, and (finding 4) a throwing tag source only loses the fix:
            // the pose keeps TRACKING and no sample is counted as rejected
            assertEquals(BeaconStatus.TRACKING, loc2.update().status);
        }
        assertEquals(0, loc2.getSamplesRejected());
        assertEquals(10, loc2.getFixesRejected());
        assertTrue(loc2.getLastFixRejectReason().contains("tag thread"));
    }

    @Test
    public void nullSampleAndPlaceholderAreIgnored() throws Exception {
        BeaconConfig c = cfg();
        PoseSource nullSrc = new PoseSource() {
            public void start(Context ctx, BeaconConfig cfg) {}
            public void stop() {}
            public Pose3dStamped latest() { return null; }
            public boolean isTracking() { return false; }
            public String debugLine() { return "null"; }
        };
        BeaconLocalizer loc = new BeaconLocalizer(nullSrc, c);
        assertEquals(BeaconStatus.INITIALIZING, loc.update().status);
        FakePoseSource fresh = started(new FakePoseSource());
        BeaconLocalizer loc2 = new BeaconLocalizer(fresh, c);
        assertEquals(BeaconStatus.INITIALIZING, loc2.update().status);
        assertEquals(0, loc2.getSamplesRejected());
    }

    // ---------------------------------------------------------------- misc API

    @Test
    public void setPoseIgnoresNonFiniteAndWrapsHeading() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 3, 0.0);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        src.advance();
        loc.update();
        loc.setPose(1, 2, 3 * Math.PI / 2 * 2 + 0.25);   // 3 pi + 0.25
        Pose2dStamped p = loc.update();
        assertEquals(1, p.x, 1e-12);
        assertEquals(FrameMath.wrapAngle(3 * Math.PI + 0.25), p.heading, 1e-12);
        loc.setPose(Double.NaN, 0, 0);
        loc.setPose(0, Double.POSITIVE_INFINITY, 0);
        loc.setPose(0, 0, Double.NaN);
        assertEquals(1, loc.update().x, 1e-12);
        assertEquals(3, loc.getSamplesRejected());
    }

    @Test
    public void setPoseSdkOverloadConvertsUnits() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 3, 0.0);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        src.advance();
        loc.update();
        loc.setPose(new Pose2D(DistanceUnit.INCH, 12, -24, AngleUnit.DEGREES, 90));
        Pose2dStamped p = loc.update();
        assertEquals(0.3048, p.x, 1e-9);
        assertEquals(-0.6096, p.y, 1e-9);
        assertEquals(Math.PI / 2, p.heading, 1e-9);
    }

    @Test
    public void setPoseMidRunRealignsAndKeepsTracking() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 100, 0.4);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(0, 0, 0.4);
        for (int i = 0; i < 50; i++) {
            src.advance();
            loc.update();
        }
        loc.setPose(-1, -1, 0);
        Pose2dStamped p = loc.update();
        assertEquals(-1, p.x, 1e-9);
        assertEquals(-1, p.y, 1e-9);
        src.advance();
        Pose2dStamped q = loc.update();
        assertEquals(0.01, Math.hypot(q.x - p.x, q.y - p.y), 1e-9);
        assertEquals(0.0, q.heading, 1e-9);
        assertEquals(0.01, q.x - p.x, 1e-9);   // now travelling along field +x
    }

    @Test
    public void getPoseReturnsTheSameObjectWhenNothingChanged() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 5, 0.0);
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        loc.setPose(0, 0, 0);
        src.advance();
        Pose2dStamped a = loc.update();
        Pose2dStamped b = loc.update();
        assertSame(a, b);
    }

    @Test
    public void startAndStopWithNullContextAndFailingSource() throws Exception {
        BeaconConfig c = cfg();
        FakePoseSource src = new FakePoseSource();
        BeaconLocalizer loc = new BeaconLocalizer(src, c);
        assertTrue(loc.start(null));
        loc.stop();
        loc.stop();
        PoseSource failing = new PoseSource() {
            public void start(Context ctx, BeaconConfig cfg) throws BeaconException {
                throw new BeaconException("ARCore not installed");
            }
            public void stop() {}
            public Pose3dStamped latest() { return null; }
            public boolean isTracking() { return false; }
            public String debugLine() { return "failing"; }
        };
        BeaconLocalizer loc2 = new BeaconLocalizer(failing, c);
        assertFalse(loc2.start(null));
        assertEquals("ARCore not installed", loc2.getStartError());
        assertEquals(BeaconStatus.INVALID, loc2.update().status);
        assertTrue(loc2.debugLine().contains("ARCore not installed"));
    }

    @Test
    public void rowsAreLoggedOncePerNewSampleInFieldFrame() throws Exception {
        java.io.File f = java.io.File.createTempFile("beacon_loc", ".csv");
        f.deleteOnExit();
        BeaconConfig c = cfg();
        FakePoseSource src = straightLine(c, 50, 0.0);
        PoseLogWriter w = new PoseLogWriter(f);
        BeaconLocalizer loc = new BeaconLocalizer(src, c, null, w);
        assertTrue(loc.start(null));
        loc.setPose(2, 3, 0);
        for (int i = 0; i < 50; i++) {
            src.advance();
            loc.update();
            loc.update();   // second call has no new sample: no extra row
        }
        loc.stop();
        List<String> lines = java.nio.file.Files.readAllLines(f.toPath(),
                java.nio.charset.Charset.forName("UTF-8"));
        assertEquals(PoseLogWriter.HEADER, lines.get(0));
        assertEquals(51, lines.size());
        String[] first = lines.get(1).split(",");
        assertEquals(String.valueOf(T0), first[0]);
        assertEquals(2.0, Double.parseDouble(first[1]), 1e-9);
        assertEquals(3.0, Double.parseDouble(first[2]), 1e-9);
        assertEquals("2", first[8]);
        String[] last = lines.get(50).split(",");
        assertEquals(2.49, Double.parseDouble(last[1]), 1e-9);
        assertEquals(0, w.getDroppedCount());
    }
}
