package org.firstinspires.ftc.teamcode.beacon.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import java.util.Random;

public class LedLatencyMeterTest {

    /**
     * Simulates a camera at {@code fps} whose frames arrive {@code latencyMs} after the middle
     * of their exposure, looking at an LED in one image block, with the LED toggled at random
     * times. The meter must recover the latency.
     */
    private static LedLatencyMeter simulate(double fps, double latencyMs, int ledLevelOn,
                                            long seed) {
        int w = 160, h = 120;
        LedLatencyMeter m = new LedLatencyMeter();
        Random rnd = new Random(seed);
        long periodNs = (long) (1e9 / fps);
        long phase = rnd.nextInt((int) periodNs);
        long t = 1000000000L;
        long nextToggle = t + 200000000L;
        boolean ledOn = false;
        long ledChangedAt = Long.MIN_VALUE;
        GrayFrame f = new GrayFrame();
        f.ensureSize(w, h);
        long captureMid = t + phase;
        for (int k = 0; k < 4000; k++) {
            captureMid += periodNs;
            // Toggle events between frames, at random times.
            while (nextToggle < captureMid) {
                ledOn = !ledOn;
                ledChangedAt = nextToggle;
                m.onLedChange(nextToggle, ledOn);
                nextToggle += 350000000L + rnd.nextInt(300000000);
            }
            boolean litInThisFrame = ledOn;   // exposure middle after the change sees it
            for (int i = 0; i < f.gray.length; i++) {
                f.gray[i] = (byte) (30 + rnd.nextInt(5));
            }
            if (litInThisFrame) {
                for (int y = 40; y < 60; y++) {           // LED near the middle row
                    for (int x = 100; x < 120; x++) {
                        f.gray[y * w + x] = (byte) ledLevelOn;
                    }
                }
            }
            f.arrivalNs = captureMid + (long) (latencyMs * 1e6) + rnd.nextInt(1000000);
            f.seq = k + 1;
            m.onFrame(f);
            if (ledChangedAt == Long.MIN_VALUE) {
                continue;
            }
        }
        return m;
    }

    @Test
    public void recoversLatencyOfASimulatedCamera() {
        LedLatencyMeter m = simulate(90, 23.0, 250, 1);
        assertTrue(m.statusLine(), m.isDone());
        assertTrue(m.statusLine(), m.sampleCount() >= LedLatencyMeter.TARGET_SAMPLES);
        // Expected error: frame period / sqrt(12 N) ~ 0.4 ms, plus 0.5 ms of mean jitter.
        assertEquals(m.statusLine(), 23.5, m.suggestedLatencyMs(), 1.5);
        assertEquals(1000.0 / 90, m.framePeriodMs(), 0.5);
    }

    @Test
    public void recoversLatencyAtThirtyFps() {
        LedLatencyMeter m = simulate(30, 41.0, 250, 2);
        assertTrue(m.statusLine(), m.isDone());
        assertEquals(m.statusLine(), 41.5, m.suggestedLatencyMs(), 3.0);
    }

    @Test
    public void invisibleLedIsReportedNotGuessed() {
        LedLatencyMeter m = simulate(60, 20.0, 33, 3);   // "LED" no brighter than the noise
        assertFalse(m.isDone());
        assertTrue(Double.isNaN(m.suggestedLatencyMs()));
        assertTrue(m.statusLine(), m.statusLine().contains("not visible"));
    }
}
