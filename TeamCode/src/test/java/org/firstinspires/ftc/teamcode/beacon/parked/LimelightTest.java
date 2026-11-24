package org.firstinspires.ftc.teamcode.beacon.parked;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconException;
import org.firstinspires.ftc.teamcode.beacon.BeaconLocalizer;
import org.firstinspires.ftc.teamcode.beacon.FieldFix;
import org.firstinspires.ftc.teamcode.beacon.OdometryTierTest;
import org.firstinspires.ftc.teamcode.beacon.PinpointPoseSource;
import org.firstinspires.ftc.teamcode.beacon.Pose2dStamped;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** PARKED AprilTag path: LimelightFixFilter rules, LimelightTagSource's worker cycle, the full loop. */
public class LimelightTest {

    private static final long NOW = 50000000000L;
    private static final long FRESH = NOW - 2000000L;      // heading sent 2 ms ago

    private static LimelightTagConfig ll(String json) throws BeaconException {
        return LimelightTagConfig.parse(json);
    }

    private static LimelightTagConfig ll() throws BeaconException {
        return ll("{}");
    }

    /** A good result: 2 tags at 1.5 m, MT2 and MT1 a little apart. */
    private static LimelightFixFilter.Reading good(double ts) {
        LimelightFixFilter.Reading r = new LimelightFixFilter.Reading();
        r.valid = true;
        r.pipelineIndex = 0;
        r.llTimestamp = ts;
        r.stalenessMs = 5;
        r.captureLatencyMs = 20;
        r.targetingLatencyMs = 15;
        r.tagCount = 2;
        r.avgDistM = 1.5;
        r.hasMt2 = true;
        r.mt2X = 1.0;
        r.mt2Y = 0.5;
        r.mt2HeadingRad = 0.3;
        r.hasMt1 = true;
        r.mt1X = 1.02;
        r.mt1Y = 0.49;
        r.mt1HeadingRad = 0.31;
        return r;
    }

    // ================================================================ filter

    @Test
    public void captureTimeSubtractsStalenessAndBothLimelightLatencies() throws Exception {
        LimelightFixFilter f = new LimelightFixFilter(ll());
        FieldFix fix = f.evaluate(good(1), NOW, true, FRESH);
        assertNotNull(f.lastReason(), fix);
        assertEquals(NOW - (5 + 20 + 15) * 1000000L, fix.tNs);
        assertEquals(40, f.lastLatencyMs(), 1e-9);
    }

    @Test
    public void megaTag2GivesPositionOnlyAndMegaTag1GivesTheHeading() throws Exception {
        LimelightFixFilter f = new LimelightFixFilter(ll());
        FieldFix fix = f.evaluate(good(1), NOW, true, FRESH);
        assertEquals("MT2+MT1h", fix.kind);
        assertTrue(fix.hasPosition);
        assertEquals("position from MegaTag2", 1.0, fix.x, 0);
        assertEquals(0.5, fix.y, 0);
        assertTrue(fix.hasHeading);
        assertEquals("heading from MegaTag1, never the MT2 echo", 0.31, fix.heading, 1e-12);
    }

    @Test
    public void oneTagGivesMegaTag2PositionWithoutAnyHeading() throws Exception {
        LimelightFixFilter f = new LimelightFixFilter(ll());
        LimelightFixFilter.Reading r = good(1);
        r.tagCount = 1;
        FieldFix fix = f.evaluate(r, NOW, true, FRESH);
        assertEquals("MT2", fix.kind);
        assertTrue(fix.hasPosition);
        assertFalse(fix.hasHeading);
    }

    @Test
    public void mt1HeadingCanBeSwitchedOff() throws Exception {
        LimelightFixFilter f = new LimelightFixFilter(ll("{\"useMt1Heading\":false}"));
        FieldFix fix = f.evaluate(good(1), NOW, true, FRESH);
        assertEquals("MT2", fix.kind);
        assertFalse(fix.hasHeading);
    }

    @Test
    public void aStaleHeadingFeedDisablesMegaTag2() throws Exception {
        LimelightFixFilter f = new LimelightFixFilter(ll());
        long tCap = NOW - 40000000L;
        long staleSend = tCap - 60000000L;            // 60 ms before capture; limit 50
        FieldFix fix = f.evaluate(good(1), NOW, true, staleSend);
        assertEquals("falls back to MegaTag1 with 2 tags", "MT1", fix.kind);
        assertEquals(1.02, fix.x, 0);
        LimelightFixFilter g = new LimelightFixFilter(ll());
        LimelightFixFilter.Reading one = good(1);
        one.tagCount = 1;
        assertNull(g.evaluate(one, NOW, true, staleSend));
        assertTrue(g.lastReason(), g.lastReason().contains("heading"));
        assertNull(new LimelightFixFilter(ll()).evaluate(one, NOW, true, Long.MIN_VALUE));
    }

    @Test
    public void whenTheLocalizerIsNotTrackingOnlyAFullMegaTag1FixIsProduced() throws Exception {
        LimelightFixFilter f = new LimelightFixFilter(ll());
        FieldFix fix = f.evaluate(good(1), NOW, false, FRESH);
        assertEquals("MT1", fix.kind);
        assertTrue(fix.hasPosition && fix.hasHeading);
        assertEquals(1.02, fix.x, 0);
        LimelightFixFilter.Reading one = good(2);
        one.tagCount = 1;
        assertNull(f.evaluate(one, NOW, false, FRESH));
        // ... even with MT1 heading switched off: the rescue path needs a heading
        FieldFix rescue = new LimelightFixFilter(ll("{\"useMt1Heading\":false}"))
                .evaluate(good(1), NOW, false, FRESH);
        assertTrue(rescue.hasHeading);
    }

    @Test
    public void badResultsAreRejectedWithAReason() throws Exception {
        assertRejected(null, "no result");
        LimelightFixFilter.Reading r = good(1);
        r.stalenessMs = 900;                    // unplugged: the SDK keeps the last result
        assertRejected(r, "old");
        r = good(1);
        r.pipelineIndex = 3;
        assertRejected(r, "pipeline 3");
        r = good(1);
        r.valid = false;
        assertRejected(r, "no tags");
        r = good(1);
        r.mt1X = r.mt1Y = r.mt2X = r.mt2Y = 0;  // SDK returns zeros when there is no solve
        assertRejected(r, "zeros");
        r = good(1);
        r.mt1X = r.mt2X = 2.5;                  // outside a 3.66 m field
        assertRejected(r, "outside");
        r = good(1);
        r.avgDistM = 3.0;
        assertRejected(r, "away");
        r = good(1);
        r.mt1Y = r.mt2Y = Double.NaN;
        assertRejected(r, "finite");
        r = good(1);
        r.captureLatencyMs = -5;
        assertRejected(r, "latency");
    }

    private static void assertRejected(LimelightFixFilter.Reading r, String why)
            throws BeaconException {
        LimelightFixFilter f = new LimelightFixFilter(ll());
        assertNull(f.evaluate(r, NOW, true, FRESH));
        assertTrue("reason '" + f.lastReason() + "' should mention " + why,
                f.lastReason().contains(why));
    }

    @Test
    public void theSameCameraFrameFetchedTwiceIsOneFix() throws Exception {
        LimelightFixFilter f = new LimelightFixFilter(ll());
        FieldFix a = f.evaluate(good(100), NOW, true, FRESH);
        assertNotNull(a);
        LimelightFixFilter.Reading again = good(100);
        again.stalenessMs = 12;                 // the SDK's next poll of the same frame
        assertNull(f.evaluate(again, NOW + 7000000L, true, FRESH));
        FieldFix b = f.evaluate(good(133), NOW + 33000000L, true, FRESH);
        assertNotNull(b);
        assertTrue(b.tNs > a.tNs);
        FieldFix c = f.evaluate(good(166), NOW + 33000000L, true, FRESH);  // same capture time
        assertTrue("capture times strictly increase", c.tNs > b.tNs);
        assertEquals(3, f.producedCount());
    }

    // ============================================================ tag source

    static final class FakeDevice implements LimelightTagSource.Device {
        volatile LimelightFixFilter.Reading next;
        volatile double lastDeg = Double.NaN;
        volatile int sends, switches;
        volatile boolean sendOk = true;
        volatile boolean throwOnRead;

        public LimelightFixFilter.Reading read() {
            if (throwOnRead) {
                throw new IllegalStateException("json");
            }
            return next;
        }
        public boolean sendHeadingDegrees(double deg) {
            lastDeg = deg;
            sends++;
            return sendOk;
        }
        public boolean switchPipeline(int index) {
            switches++;
            return true;
        }
    }

    static final class Clock implements PinpointPoseSource.TimeSource {
        volatile long t = NOW;
        public long nowNs() { return t; }
    }

    @Test
    public void headingIsSentInDegreesOnlyWhileTheLocalizerTracks() throws Exception {
        FakeDevice d = new FakeDevice();
        Clock clk = new Clock();
        LimelightTagSource s = new LimelightTagSource(d, clk, ll());
        final double[] h = {Double.NaN};
        s.setHeadingProvider(new LimelightTagSource.HeadingProvider() {
            public double headingRadiansOrNaN() { return h[0]; }
        });
        s.step(s.generation());
        assertEquals("nothing sent while the heading is untrusted", 0, d.sends);
        h[0] = Math.PI / 2;
        s.step(s.generation());
        assertEquals(1, d.sends);
        assertEquals(90.0, d.lastDeg, 1e-9);
        d.sendOk = false;
        s.step(s.generation());
        assertEquals(1, s.getHeadingFailures());
    }

    @Test
    public void wrongPipelineIsSwitchedButNotMoreThanOncePerSecond() throws Exception {
        FakeDevice d = new FakeDevice();
        Clock clk = new Clock();
        LimelightTagSource s = new LimelightTagSource(d, clk, ll("{\"pipeline\":1}"));
        d.next = good(1);                       // reports pipeline 0
        s.step(s.generation());
        assertEquals(1, d.switches);
        clk.t += 500000000L;
        s.step(s.generation());
        assertEquals(1, d.switches);
        clk.t += 600000000L;
        s.step(s.generation());
        assertEquals(2, d.switches);
        assertNull(s.latestFix());
        assertTrue(s.getReason(), s.getReason().contains("pipeline 0"));
    }

    @Test
    public void aStepFromAStoppedGenerationPublishesNothing() throws Exception {
        FakeDevice d = new FakeDevice();
        Clock clk = new Clock();
        LimelightTagSource s = new LimelightTagSource(d, clk, ll());
        s.setHeadingProvider(new LimelightTagSource.HeadingProvider() {
            public double headingRadiansOrNaN() { return 0.3; }
        });
        d.next = good(1);
        FakeDevice dd = d;
        int old = s.generation();
        s.stop();                               // a new generation begins
        s.step(old);
        assertNull("an old thread must not feed a new run", s.latestFix());
        assertEquals("nor send the Limelight a stale heading", 0, dd.sends);
        s.step(s.generation());
        assertNotNull(s.latestFix());
    }

    @Test
    public void mt2EchoErrorComparesTheReportedYawWithWhatWeSent() throws Exception {
        FakeDevice d = new FakeDevice();
        LimelightTagSource s = new LimelightTagSource(d, new Clock(), ll());
        s.setHeadingProvider(new LimelightTagSource.HeadingProvider() {
            public double headingRadiansOrNaN() { return Math.toRadians(30); }
        });
        LimelightFixFilter.Reading r = good(1);
        r.mt2HeadingRad = Math.toRadians(30.5);
        d.next = r;
        s.step(s.generation());
        assertEquals(0.5, s.getMt2EchoErrorDeg(), 1e-9);
    }

    @Test
    public void threadLifecycleIsIdempotentAndSurvivesADeviceThatThrows() throws Exception {
        FakeDevice d = new FakeDevice();
        LimelightTagSource s = new LimelightTagSource(d, new Clock(), ll("{\"headingHz\":200}"));
        s.stop();                               // before start: fine
        s.setHeadingProvider(new LimelightTagSource.HeadingProvider() {
            public double headingRadiansOrNaN() { return 0.1; }
        });
        d.throwOnRead = true;
        s.start();
        s.start();                              // twice: one thread
        long end = System.currentTimeMillis() + 2000;
        while (!s.getReason().contains("thread error") && System.currentTimeMillis() < end) {
            Thread.sleep(5);
        }
        assertTrue(s.getReason(), s.getReason().contains("thread error"));
        d.throwOnRead = false;
        d.next = good(1);
        while (s.latestFix() == null && System.currentTimeMillis() < end) {
            Thread.sleep(5);
        }
        assertNotNull("thread kept running after the exception", s.latestFix());
        s.stop();
        s.stop();
        assertFalse(s.isRunning());
        Thread.sleep(50);
        int sends = d.sends;
        Thread.sleep(100);
        assertEquals("thread has exited", sends, d.sends);
    }

    // ============================================================= full loop

    @Test
    public void megaTag2CorrectsPositionDriftButNeverHeading() throws Exception {
        BeaconConfig c = BeaconConfig.parse("{\"tier\":\"odometry\",\"fix\":{\"gainXY\":1,"
                + "\"gainHeading\":1}}");
        OdometryTierTest.FakeReader pin = new OdometryTierTest.FakeReader();
        Clock clk = new Clock();
        PinpointPoseSource src = new PinpointPoseSource(pin, clk);
        FakeDevice d = new FakeDevice();
        LimelightTagSource tags = new LimelightTagSource(d, clk, LimelightTagConfig.defaults());
        final BeaconLocalizer loc = new BeaconLocalizer(src, c, tags, null);
        tags.setHeadingProvider(new LimelightTagSource.HeadingProvider() {
            public double headingRadiansOrNaN() { return loc.getHeadingIfTrackingOrNaN(); }
        });
        loc.start(null);
        loc.update();
        loc.setPose(1.0, 0.5, 0.0);
        for (int i = 0; i < 50; i++) {          // robot still; Pinpoint reports a 4 cm drift
            clk.t += 10000000L;
            pin.x = 0.04 * i / 49.0;
            loc.update();
        }
        assertEquals(1.04, loc.getPose().x, 1e-9);
        // One tag (no MT1 heading); MT2 says the robot is still at (1.0, 0.5) and, being an
        // echo, reports a heading 0.2 rad off which must be ignored.
        LimelightFixFilter.Reading r = good(1);
        r.tagCount = 1;
        r.mt2X = 1.0;
        r.mt2Y = 0.5;
        r.mt2HeadingRad = 0.2;
        r.stalenessMs = 0;
        r.captureLatencyMs = 0;
        r.targetingLatencyMs = 0;
        d.next = r;
        tags.step(tags.generation());
        FieldFix fix = tags.latestFix();
        assertNotNull(tags.getReason(), fix);
        assertFalse(fix.hasHeading);
        Pose2dStamped p = loc.update();
        assertEquals(1, loc.getFixesApplied());
        assertEquals("position corrected", 1.0, p.x, 1e-6);
        assertEquals("heading untouched by MegaTag2", 0.0, p.heading, 1e-12);
    }

    // ============================================================= config

    @Test
    public void parsesTheMegaTagSettings() throws Exception {
        LimelightTagConfig c = LimelightTagConfig.parse("{\"pipeline\": 2, \"minTags\": 2, "
                + "\"maxRangeM\": 2.0, \"mt1MinTags\": 3, \"mt1MaxRangeM\": 1.5, "
                + "\"useMt1Heading\": false, \"maxStalenessMs\": 100, \"headingHz\": 50, "
                + "\"headingFreshMs\": 40, \"fieldHalfM\": 1.8}");
        assertEquals(2, c.pipeline);
        assertEquals(2, c.minTags);
        assertEquals(2.0, c.maxRangeM, 0);
        assertEquals(3, c.mt1MinTags);
        assertEquals(1.5, c.mt1MaxRangeM, 0);
        assertFalse(c.useMt1Heading);
        assertEquals(100, c.maxStalenessMs, 0);
        assertEquals(50, c.headingHz, 0);
        assertEquals(40, c.headingFreshMs, 0);
        assertEquals(1.8, c.fieldHalfM, 0);
        String[][] bad = {{"{\"pipeline\": 10}", "limelight.pipeline"},
                {"{\"minTags\": 0}", "limelight.minTags"},
                {"{\"maxStalenessMs\": 0}", "limelight.maxStalenessMs"},
                {"{\"headingHz\": 1000}", "limelight.headingHz"},
                {"{\"fieldHalfM\": 72}", "limelight.fieldHalfM"}};
        for (String[] b : bad) {
            try {
                LimelightTagConfig.parse(b[0]);
                org.junit.Assert.fail(b[0] + " accepted");
            } catch (BeaconException e) {
                assertTrue(e.getMessage(), e.getMessage().contains(b[1]));
            }
        }
    }
}
