package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Review findings 1, 2, 3, 7, 8, 9 (docs/reviews/01-core.md), partial fixes, the
 * motion gate, and the G304 start-pose rule. Uses a robot-pose source so the expected numbers
 * are plain.
 */
public class BeaconLocalizerRobustnessTest {

    private static final long T0 = 1000000000L;
    private static final long DT = 10000000L;     // 100 Hz

    /** A robot-pose source the test drives by hand. */
    static final class RobotSource implements PoseSource {
        volatile Pose3dStamped s;
        int epoch;

        public void start(android.content.Context ctx, BeaconConfig cfg) {}
        public void stop() {}
        public Pose3dStamped latest() { return s; }
        public boolean isTracking() { return s != null && s.status == BeaconStatus.TRACKING; }
        public String debugLine() { return "robot"; }
        public boolean reportsRobotPose() { return true; }

        void put(long t, double x, double y, double h) {
            put(t, x, y, h, BeaconStatus.TRACKING);
        }

        void put(long t, double x, double y, double h, int status) {
            s = new Pose3dStamped(x, y, 0, 0, 0, Math.sin(h / 2), Math.cos(h / 2), t, status,
                    epoch);
        }
    }

    static final class Tags implements FieldFixSource {
        volatile FieldFix fix;
        public FieldFix latestFix() { return fix; }
    }

    static final class Clock implements BeaconLocalizer.Clock {
        long now = 1;
        public long nanoTime() { return now; }
    }

    private static BeaconConfig cfg(String tag) throws BeaconException {
        return BeaconConfig.parse("{\"tier\":\"odometry\",\"fix\":" + tag + "}");
    }

    private static final String GAIN = "{\"gainXY\":0.3,\"gainHeading\":0.2,\"maxJumpM\":0.5,"
            + "\"maxJumpRad\":0.5,\"maxSpeedMps\":10,\"maxOmegaRadPs\":20}";

    /** Robot parked at source (0.2, 0, 0), aligned to field (0.2, 0, 0). */
    private static BeaconLocalizer parked(RobotSource src, Tags tags, Clock clk, BeaconConfig c,
                                          int samples) {
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null, clk);
        loc.start(null);
        for (int i = 0; i < samples; i++) {
            src.put(T0 + i * DT, 0.2, 0, 0);
            clk.now += DT;
            loc.update();
            if (i == 0) {
                loc.setPose(0.2, 0, 0);
            }
        }
        return loc;
    }

    // --------------------------------------------------------------- finding 1

    @Test
    public void recoveryWaiverExpiresAfterTheWindow() throws Exception {
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = parked(src, tags, clk, cfg(GAIN), 10);
        long t = T0 + 10 * DT;
        src.put(t, 0.2, 0, 0, BeaconStatus.LOST);     // one-sample dropout
        clk.now += DT;
        loc.update();
        // tracking resumes and continues for 5 s, parked
        for (int i = 1; i <= 500; i++) {
            src.put(t + i * DT, 0.2, 0, 0);
            clk.now += DT;
            loc.update();
        }
        assertFalse("window closed", loc.isRecovering());
        // the reviewer's outlier: (2.2, 1.0, 1.2 rad), 5 s after the dropout
        tags.fix = FieldFix.full(2.2, 1.0, 1.2, t + 500 * DT);
        Pose2dStamped p = loc.update();
        assertEquals("outlier rejected", 0.2, p.x, 1e-9);
        assertEquals(0.0, p.y, 1e-9);
        assertEquals(1, loc.getFixesRejected());
        assertEquals(0, loc.getFixesApplied());
    }

    // --------------------------------------------------------------- finding 2

    @Test
    public void aWorldResetContinuesFromTheLastPoseInsteadOfJumping() throws Exception {
        // Reviewer's case: setPose(1, 1, pi/2), drive 0.99 m, the source re-inits its world
        // with the origin at the robot. Truth (1.00, 1.99); the old code said (1.00, 1.00).
        RobotSource src = new RobotSource();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg(GAIN), null, null, clk);
        loc.start(null);
        long t = T0;
        src.put(t, 0, 0, 0);
        loc.update();
        loc.setPose(1, 1, Math.PI / 2);
        for (int i = 1; i <= 99; i++) {
            t += DT;
            clk.now += DT;
            src.put(t, 0.01 * i, 0, 0);
            loc.update();
        }
        assertEquals(1.99, loc.getPose().y, 1e-9);
        t += DT;
        clk.now += DT;
        src.put(t, 0, 0, 0, BeaconStatus.LOST);
        loc.update();
        src.epoch = 1;                          // new world, origin at the robot
        t += DT;
        clk.now += DT;
        src.put(t, 0, 0, 0);
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(1.0, p.x, 1e-9);
        assertEquals("continues, does not jump back", 1.99, p.y, 1e-9);
        assertEquals(1, loc.getWorldResets());
        assertTrue(loc.isRecovering());
        t += DT;
        clk.now += DT;
        src.put(t, 0.01, 0, 0);
        assertEquals(2.00, loc.update().y, 1e-9);
    }

    @Test
    public void aWorldResetWithoutDropoutExtrapolatesWithTheLastVelocity() throws Exception {
        RobotSource src = new RobotSource();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg(GAIN), null, null, clk);
        loc.start(null);
        long t = T0;
        src.put(t, 0, 0, 0);
        loc.update();
        loc.setPose(0, 0, 0);
        for (int i = 1; i <= 20; i++) {          // 1 m/s along +x
            t += DT;
            clk.now += DT;
            src.put(t, 0.01 * i, 0, 0);
            loc.update();
        }
        src.epoch = 1;                          // a 5 m jump while tracking
        t += DT;
        clk.now += DT;
        src.put(t, 5.0, 3.0, 1.0);
        Pose2dStamped p = loc.update();
        assertEquals("one more 1 cm step", 0.21, p.x, 1e-9);
        assertEquals(0.0, p.y, 1e-9);
        assertEquals(0.0, p.heading, 1e-9);
    }

    // --------------------------------------------------------------- finding 3

    @Test
    public void oneFutureStampedSampleDoesNotFreezeTheLocalizer() throws Exception {
        RobotSource src = new RobotSource();
        Clock clk = new Clock();
        BeaconLocalizer loc = parked(src, null, clk, cfg(GAIN), 10);
        long t = T0 + 10 * DT;
        src.put(t + 10000000000L, 0.2, 0, 0);    // 10 s in the future
        clk.now += DT;
        loc.update();
        int lost = 0;
        for (int i = 1; i <= 190; i++) {
            src.put(t + i * DT, 0.2 + 0.001 * i, 0, 0);
            clk.now += DT;
            if (loc.update().status != BeaconStatus.TRACKING) {
                lost++;
            }
        }
        assertEquals("time base accepted after sourceTimeoutS", 1, loc.getTimeBaseResets());
        assertTrue("lost only for about the timeout, not 190 samples: " + lost, lost <= 52);
        assertEquals(BeaconStatus.TRACKING, loc.getStatus());
        assertEquals(0.2 + 0.19, loc.getPose().x, 1e-9);
    }

    @Test
    public void aFutureStampedFixIsRejectedAndDoesNotBlockLaterFixes() throws Exception {
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = parked(src, tags, clk, cfg(GAIN), 50);
        long newest = T0 + 49 * DT;
        tags.fix = FieldFix.full(0.3, 0, 0, newest + 10000000000L);
        loc.update();
        assertEquals(1, loc.getFixesRejected());
        assertTrue(loc.getLastFixRejectReason().contains("clock"));
        tags.fix = FieldFix.full(0.3, 0, 0, newest - DT);     // a correct fix, 10 cm off
        Pose2dStamped p = loc.update();
        assertEquals(1, loc.getFixesApplied());
        assertEquals(0.2 + 0.3 * 0.1, p.x, 1e-9);
    }

    @Test
    public void aFixSlightlyAheadOfTheNewestSampleIsComparedWithIt() throws Exception {
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = parked(src, tags, clk, cfg(GAIN), 20);
        long newest = T0 + 19 * DT;
        tags.fix = FieldFix.full(0.3, 0, 0, newest + DT / 2);
        Pose2dStamped p = loc.update();
        assertEquals(1, loc.getFixesApplied());
        assertEquals(0.2 + 0.3 * 0.1, p.x, 1e-9);
    }

    // ------------------------------------------------------- findings 7, 8, 9

    @Test
    public void velocityIsZeroWheneverNotTracking() throws Exception {
        RobotSource src = new RobotSource();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg(GAIN), null, null, clk);
        loc.start(null);
        long t = T0;
        for (int i = 0; i < 10; i++) {
            src.put(t + i * DT, 0.01 * i, 0, 0);
            clk.now += DT;
            loc.update();
            if (i == 0) {
                loc.setPose(0, 0, 0);
            }
        }
        assertEquals(1.0, loc.getVelocity().vx, 1e-9);
        clk.now += 2000000000L;                 // source silent for 2 s
        assertEquals(BeaconStatus.LOST, loc.update().status);
        assertEquals("stale velocity not reported", 0.0, loc.getVelocity().vx, 0);
    }

    @Test
    public void hugeFiniteInputsNeverReachTheOutput() throws Exception {
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = parked(src, tags, clk, cfg(GAIN), 10);
        src.put(T0 + 10 * DT, 1.5e308, 0, 0);
        clk.now += DT;
        Pose2dStamped p = loc.update();
        assertTrue(p.isFinite());
        assertEquals(0.2, p.x, 1e-9);
        assertEquals(BeaconStatus.LOST, p.status);
        tags.fix = FieldFix.full(1e300, 0, 0, T0 + 9 * DT);
        assertTrue(loc.update().isFinite());
        assertEquals(1, loc.getFixesRejected());
        loc.setPose(1e300, 0, 0);
        assertEquals(0.2, loc.update().x, 1e-9);
    }

    @Test
    public void aHeldPoseKeepsTheTimeOfTheSampleItCameFrom() throws Exception {
        RobotSource src = new RobotSource();
        Clock clk = new Clock();
        BeaconLocalizer loc = parked(src, null, clk, cfg(GAIN), 10);
        long good = T0 + 9 * DT;
        assertEquals(good, loc.getPose().tNs);
        for (int i = 10; i < 60; i++) {
            src.put(T0 + i * DT, 9, 9, 0, BeaconStatus.LOST);
            clk.now += DT;
            loc.update();
        }
        assertEquals(BeaconStatus.LOST, loc.getStatus());
        assertEquals("stamped with the last GOOD sample", good, loc.getPose().tNs);
    }

    // ------------------------------------------------------------ partial fixes

    @Test
    public void positionOnlyAndHeadingOnlyFixesTouchOnlyTheirPart() throws Exception {
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = parked(src, tags, clk, cfg("{\"gainXY\":1,\"gainHeading\":1,"
                + "\"maxSpeedMps\":10,\"maxOmegaRadPs\":20}"), 10);
        long t = T0 + 9 * DT;
        tags.fix = new FieldFix(0.3, 0.1, 2.0, t - 2, true, false, "MT2");
        Pose2dStamped p = loc.update();
        assertEquals(0.3, p.x, 1e-9);
        assertEquals(0.1, p.y, 1e-9);
        assertEquals("heading of an MT2 fix is ignored", 0.0, p.heading, 1e-12);
        tags.fix = new FieldFix(9, 9, 0.2, t - 1, false, true, "MT1h");
        p = loc.update();
        assertEquals("position of a heading-only fix is ignored", 0.3, p.x, 1e-9);
        assertEquals(0.1, p.y, 1e-9);
        assertEquals(0.2, p.heading, 1e-9);
        assertEquals(2, loc.getFixesApplied());
    }

    @Test
    public void aPartialFixCannotSetTheAlignmentButAFullOneCan() throws Exception {
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg(GAIN), tags, null, clk);
        loc.start(null);
        src.put(T0, 0, 0, 0);
        loc.update();
        assertEquals(BeaconStatus.INITIALIZING, loc.getStatus());
        tags.fix = new FieldFix(1, 1, 0, T0, true, false, "MT2");
        loc.update();
        assertFalse(loc.isAligned());
        assertTrue(loc.getLastFixRejectReason().contains("partial"));
        src.put(T0 + DT, 0, 0, 0);
        clk.now += DT;
        loc.update();
        tags.fix = FieldFix.full(-1.2, 0.4, 3.0, T0 + DT);
        Pose2dStamped p = loc.update();
        assertTrue(loc.isAligned());
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(-1.2, p.x, 1e-9);
    }

    // ---------------------------------------------------------- motion gate

    @Test
    public void fixesTakenWhileMovingOrTurningFastAreIgnored() throws Exception {
        BeaconConfig c = cfg("{\"gainXY\":1,\"gainHeading\":1,\"maxJumpM\":5,"
                + "\"maxSpeedMps\":0.75,\"maxOmegaRadPs\":0.35}");
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null, clk);
        loc.start(null);
        long t = T0;
        for (int i = 0; i < 40; i++) {          // 1 m/s for 0.2 s, then still for 0.2 s
            double x = i < 20 ? 0.01 * i : 0.19;
            src.put(t + i * DT, x, 0, 0);
            clk.now += DT;
            loc.update();
            if (i == 0) {
                loc.setPose(0, 0, 0);
            }
        }
        tags.fix = FieldFix.full(0.5, 0, 0, t + 10 * DT);     // captured while driving
        loc.update();
        assertTrue(loc.getLastFixRejectReason(), loc.getLastFixRejectReason().contains("moving"));
        tags.fix = FieldFix.full(0.2, 0, 0, t + 35 * DT);     // captured while still
        loc.update();
        assertEquals(1, loc.getFixesApplied());
        // turning at 1 rad/s
        for (int i = 40; i < 60; i++) {
            src.put(t + i * DT, 0.19, 0, 0.01 * (i - 40));
            clk.now += DT;
            loc.update();
        }
        tags.fix = FieldFix.full(0.2, 0, 0.1, t + 55 * DT);
        loc.update();
        assertTrue(loc.getLastFixRejectReason(),
                loc.getLastFixRejectReason().contains("turning"));
    }

    // -------------------------------------------------------- G304 start rule

    @Test
    public void onStartReassertsTheStartPoseAgainstDriftWhileWaitingInInit() throws Exception {
        RobotSource src = new RobotSource();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg(GAIN), null, null, clk);
        loc.start(null);
        src.put(T0, 0, 0, 0);
        loc.update();
        loc.setPose(1.0, -0.9, Math.PI / 2);
        // 30 s of INIT, robot still, source drifting 1 mm/s and 0.001 rad/s
        for (int i = 1; i <= 3000; i++) {
            src.put(T0 + i * DT, 0.00001 * i, 0, 0.00001 * i);
            clk.now += DT;
            loc.update();
        }
        Pose2dStamped drifted = loc.getPose();
        assertTrue(Math.hypot(drifted.x - 1.0, drifted.y + 0.9) > 0.01);
        loc.onStart();
        Pose2dStamped p = loc.update();
        assertEquals(1.0, p.x, 1e-12);
        assertEquals(-0.9, p.y, 1e-12);
        assertEquals(Math.PI / 2, p.heading, 1e-12);
        assertEquals(1, loc.getStartReasserts());
    }

    @Test
    public void startPoseExpiresIfTheSourceOnlyTracksAfterStartAndATagFixRescues()
            throws Exception {
        RobotSource src = new RobotSource();
        Tags tags = new Tags();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg(GAIN), tags, null, clk);
        loc.start(null);
        src.put(T0, 0, 0, 0, BeaconStatus.INITIALIZING);
        loc.update();
        loc.setPose(1.0, -0.9, 0);
        loc.onStart();
        // the robot drives off; the source only starts tracking 0.5 s later
        src.put(T0 + 50 * DT, 0, 0, 0);
        clk.now += 50 * DT;
        Pose2dStamped p = loc.update();
        assertEquals("not aligned to a start pose that is no longer true",
                BeaconStatus.INITIALIZING, p.status);
        assertTrue(loc.isStartPoseExpired());
        src.put(T0 + 51 * DT, 0.01, 0, 0);
        clk.now += DT;
        loc.update();
        tags.fix = FieldFix.full(1.6, -0.9, 0.05, T0 + 51 * DT);   // full MT1 fix: rescue
        p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(1.6, p.x, 1e-9);
        assertFalse(loc.isStartPoseExpired());
    }

    @Test
    public void startPoseBeforeTrackingIsAppliedWhenTrackingBeginsDuringInit() throws Exception {
        RobotSource src = new RobotSource();
        Clock clk = new Clock();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg(GAIN), null, null, clk);
        loc.start(null);
        loc.setPose(1.0, -0.9, 0.3);
        src.put(T0, 5, 5, 1, BeaconStatus.INITIALIZING);
        loc.update();
        src.put(T0 + DT, 5, 5, 1);              // tracking begins, still in INIT
        loc.update();
        loc.onStart();
        Pose2dStamped p = loc.update();
        assertEquals(BeaconStatus.TRACKING, p.status);
        assertEquals(1.0, p.x, 1e-12);
        assertEquals(0.3, p.heading, 1e-12);
    }
}
