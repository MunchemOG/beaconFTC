package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Task J1: time-sized history and sigma-weighted gains on the renamed fix path (I4). */
public class FieldFixTest {

    /** Hub clock the test sets by hand. */
    static final class ManualClock implements PinpointPoseSource.TimeSource {
        long t = 1000000000L;

        @Override
        public long nowNs() {
            return t;
        }
    }

    static final class SettableFixes implements FieldFixSource {
        volatile FieldFix fix;

        @Override
        public FieldFix latestFix() {
            return fix;
        }
    }

    private static final long PINPOINT_PERIOD_NS = 666667L;   // 1.5 kHz

    private final OdometryTierTest.FakeReader pin = new OdometryTierTest.FakeReader();
    private final ManualClock clk = new ManualClock();
    private final SettableFixes fixes = new SettableFixes();

    private BeaconLocalizer still(String json, int samples) throws Exception {
        BeaconConfig c = BeaconConfig.parse(json);
        PinpointPoseSource src = new PinpointPoseSource(pin, clk);
        BeaconLocalizer loc = new BeaconLocalizer(src, c, fixes, null);
        loc.start(null);
        loc.setPose(0, 0, 0);
        for (int i = 0; i < samples; i++) {
            clk.t += PINPOINT_PERIOD_NS;
            loc.update();
        }
        assertEquals(BeaconStatus.TRACKING, loc.getStatus());
        return loc;
    }

    @Test
    public void historyCoversHalfSecondAtPinpointRate() throws Exception {
        // One second of samples at 1.5 kHz: a 128-sample ring would hold only 85 ms.
        BeaconLocalizer loc = still("{}", 1500);
        fixes.fix = FieldFix.full(0.10, 0, 0, clk.t - 500000000L);    // 0.5 s old, 10 cm off
        loc.update();
        assertEquals(loc.getLastFixRejectReason(), 1, loc.getFixesApplied());
        assertEquals(0.3 * 0.10, loc.getPose().x, 1e-9);                // default gainXY 0.3

    }

    @Test
    public void fixOlderThanHistoryIsRejected() throws Exception {
        BeaconLocalizer loc = still("{}", 1500);
        fixes.fix = FieldFix.full(0.10, 0, 0, clk.t - 900000000L);    // historyS is 0.75
        loc.update();
        assertEquals(0, loc.getFixesApplied());
        assertTrue(loc.getLastFixRejectReason(),
                loc.getLastFixRejectReason().contains("older than history"));
    }

    @Test
    public void historyLengthFollowsTheConfig() throws Exception {
        BeaconLocalizer loc = still("{\"fix\":{\"historyS\":0.2}}", 1500);
        fixes.fix = FieldFix.full(0.10, 0, 0, clk.t - 300000000L);
        loc.update();
        assertEquals(0, loc.getFixesApplied());
        assertTrue(loc.getLastFixRejectReason().contains("older than history"));
    }

    private double moveFrom(double sigmaXY) throws Exception {
        pin.x = 0;
        BeaconLocalizer loc = still("{}", 300);
        fixes.fix = FieldFix.full(0.10, 0, 0, clk.t - 50000000L, sigmaXY, Double.NaN);
        loc.update();
        assertEquals(loc.getLastFixRejectReason(), 1, loc.getFixesApplied());
        return loc.getPose().x;
    }

    @Test
    public void sigmaWeightsGain() throws Exception {
        double precise = moveFrom(0.005);
        double vague = moveFrom(0.10);
        double unknown = moveFrom(Double.NaN);
        assertTrue("precise " + precise + " vs vague " + vague, precise > vague);
        // gain = p^2 / (p^2 + s^2) with the default prior p = 0.02 m
        assertEquals(0.10 * 0.0004 / (0.0004 + 0.000025), precise, 1e-9);
        assertEquals(0.10 * 0.0004 / (0.0004 + 0.01), vague, 1e-9);
        assertEquals("no sigma: the configured gain", 0.10 * 0.3, unknown, 1e-9);
    }

    @Test
    public void headingSigmaWeightsHeadingGain() throws Exception {
        BeaconLocalizer loc = still("{}", 300);
        fixes.fix = new FieldFix(0, 0, 0.05, clk.t - 10000000L, false, true, "h", Double.NaN,
                0.02);
        loc.update();
        assertEquals(1, loc.getFixesApplied());
        assertEquals(0.05 * 0.5, loc.getPose().heading, 1e-9);   // p = s = 0.02 rad -> 0.5
    }

    @Test
    public void negativeOrInfiniteSigmaIsRejected() throws Exception {
        BeaconLocalizer loc = still("{}", 300);
        fixes.fix = FieldFix.full(0.1, 0, 0, clk.t - 10000000L, -1, Double.NaN);
        loc.update();
        fixes.fix = FieldFix.full(0.1, 0, 0, clk.t - 5000000L, Double.POSITIVE_INFINITY, 0.1);
        loc.update();
        assertEquals(0, loc.getFixesApplied());
        assertEquals(2, loc.getFixesRejected());
        assertEquals(0, loc.getPose().x, 0);
    }

    @Test
    public void speedGateUsesATimeWindowAtHighSampleRates() throws Exception {
        BeaconLocalizer loc = still("{}", 300);
        // Drive at 2.5 m/s for 0.2 s (default limit 2.0 m/s), sampled at 1.5 kHz.
        for (int i = 0; i < 300; i++) {
            clk.t += PINPOINT_PERIOD_NS;
            pin.x += 2.5 * PINPOINT_PERIOD_NS * 1e-9;
            loc.update();
        }
        fixes.fix = FieldFix.full(pin.x, 0, 0, clk.t - 100000000L);
        loc.update();
        assertEquals(0, loc.getFixesApplied());
        assertTrue(loc.getLastFixRejectReason(), loc.getLastFixRejectReason().contains("moving"));
    }

    @Test
    public void defaultMotionGatesMatchTheCore() throws Exception {
        BeaconConfig d = BeaconConfig.defaults();
        assertEquals(2.0, d.fix.maxSpeedMps, 0);     // the core gates at 2.0 m/s
        assertEquals(1.5, d.fix.maxOmegaRadPs, 0);   // ... and 1.5 rad/s
        assertEquals(0.75, BeaconConfig.parse("{\"fix\":{\"maxSpeedMps\":0.75}}").fix.maxSpeedMps,
                0);
    }

    /** Spins in place at {@code omega} rad/s for 0.2 s, then offers a fix from 0.1 s ago. */
    private BeaconLocalizer spinThenFix(double omega) throws Exception {
        BeaconLocalizer loc = still("{}", 300);
        for (int i = 0; i < 300; i++) {
            clk.t += PINPOINT_PERIOD_NS;
            pin.h += omega * PINPOINT_PERIOD_NS * 1e-9;
            loc.update();
        }
        double hAtFix = pin.h - omega * 0.1;
        fixes.fix = FieldFix.full(0.02, 0, hAtFix, clk.t - 100000000L);
        loc.update();
        return loc;
    }

    @Test
    public void turnAtHardMatchRateIsNoLongerRejected() throws Exception {
        // hard-match-s5 lost 957 vision fixes to "turning 1.07 rad/s" under the old 0.35 limit
        BeaconLocalizer loc = spinThenFix(1.07);
        assertEquals(loc.getLastFixRejectReason(), 1, loc.getFixesApplied());
    }

    @Test
    public void turnAboveTheDefaultIsStillRejected() throws Exception {
        BeaconLocalizer loc = spinThenFix(1.8);
        assertEquals(0, loc.getFixesApplied());
        assertTrue(loc.getLastFixRejectReason(),
                loc.getLastFixRejectReason().contains("turning"));
    }
}
