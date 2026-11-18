package org.firstinspires.ftc.teamcode.beacon.vision;

import java.util.Arrays;
import java.util.Locale;

/**
 * Measures the Limelight's capture-to-Hub latency end to end with a blinking LED: the probe
 * OpMode switches the Control Hub's status LED on and off at random moments (stamping each
 * change on the Hub clock) while the Limelight looks at the LED, and this class finds the first
 * frame that shows each change.
 *
 * <h3>Method</h3>
 * <ol>
 *   <li>Every frame is reduced to a 16 x 12 grid of block brightness means.</li>
 *   <li>Learning (first {@value #LEARN_EVENTS} changes): for each change, the frames that arrive
 *       150-300 ms after it show the settled state. The block with the largest on/off
 *       difference is the LED; if that difference is under {@value #MIN_CONTRAST} gray levels
 *       the LED is reported as not visible and nothing is guessed.</li>
 *   <li>Measuring: for each later change at Hub time {@code tc}, the first frame arriving after
 *       {@code tc} whose LED block crosses the on/off midpoint gives a sample
 *       {@code arrival - tc}.</li>
 * </ol>
 * That first frame is the one whose exposure middle came after the change, so
 * {@code sample = latency + U}, with U uniform in [0, frame period) because the changes are at
 * random times. Hence {@code latency = mean(sample) - period / 2}
 * ({@link #suggestedLatencyMs()}), where the period is the median time between frame
 * arrivals. Its standard error is about {@code period / sqrt(12 N)} plus jitter. It includes
 * the time the LED command takes to reach the LED (one Lynx command, not measured) and
 * refers to the image row the LED is on: keep the LED near the middle row
 * ({@link #ledRowFraction()} near 0.5) so a rolling shutter does not bias it.
 *
 * <p>Thread-safe: {@link #onLedChange} comes from the LED thread, {@link #onFrame} from the
 * frame source's decode thread. Allocation-free after construction.
 */
public final class LedLatencyMeter {
    public static final int TARGET_SAMPLES = 30;
    static final int COLS = 16, ROWS = 12, BLOCKS = COLS * ROWS;
    static final int LEARN_EVENTS = 6;
    static final double MIN_CONTRAST = 20;
    private static final long SETTLE_FROM_NS = 150000000L;
    private static final long SETTLE_TO_NS = 300000000L;
    private static final long DETECT_NS = 400000000L;
    private static final int RING = 256;
    private static final int MAX_EVENTS = 2048;

    // frame ring: arrival time + block means
    private final long[] fT = new long[RING];
    private final float[] fB = new float[RING * BLOCKS];
    private int fHead, fCount;

    // LED changes
    private final long[] eT = new long[MAX_EVENTS];
    private final boolean[] eOn = new boolean[MAX_EVENTS];
    private final int[] eState = new int[MAX_EVENTS];   // 0 new, 1 used
    private int eCount;

    // learning
    private final double[] sumOn = new double[BLOCKS];
    private final double[] sumOff = new double[BLOCKS];
    private final double[] eventMean = new double[BLOCKS];
    private int nOn, nOff, learned;
    private int block = -1;
    private double levelOn, levelOff;
    private boolean notVisible;
    private int ledRow;

    // samples
    private final double[] samples = new double[TARGET_SAMPLES * 4];
    private int nSamples, missed;
    private final double[] scratch = new double[RING];

    public synchronized void reset() {
        fHead = 0;
        fCount = 0;
        eCount = 0;
        Arrays.fill(sumOn, 0);
        Arrays.fill(sumOff, 0);
        nOn = 0;
        nOff = 0;
        learned = 0;
        block = -1;
        notVisible = false;
        nSamples = 0;
        missed = 0;
    }

    /** The LED was switched to {@code on}; {@code tNs} = Hub time the command completed. */
    public synchronized void onLedChange(long tNs, boolean on) {
        if (eCount < MAX_EVENTS) {
            eT[eCount] = tNs;
            eOn[eCount] = on;
            eState[eCount] = 0;
            eCount++;
        }
    }

    /** Feeds one decoded frame (uses its pixels and {@link GrayFrame#arrivalNs}). */
    public synchronized void onFrame(GrayFrame f) {
        if (f.width < COLS * 2 || f.height < ROWS * 2) {
            return;
        }
        int base = fHead * BLOCKS;
        blockMeans(f, base);
        fT[fHead] = f.arrivalNs;
        fHead = (fHead + 1) % RING;
        if (fCount < RING) {
            fCount++;
        }
        process(f.arrivalNs);
    }

    private void blockMeans(GrayFrame f, int base) {
        int bw = f.width / COLS;
        int bh = f.height / ROWS;
        for (int by = 0; by < ROWS; by++) {
            for (int bx = 0; bx < COLS; bx++) {
                long sum = 0;
                int n = 0;
                for (int y = by * bh; y < (by + 1) * bh; y += 2) {
                    int row = y * f.width;
                    for (int x = bx * bw; x < (bx + 1) * bw; x += 2) {
                        sum += f.gray[row + x] & 0xFF;
                        n++;
                    }
                }
                fB[base + by * COLS + bx] = n == 0 ? 0 : (float) sum / n;
            }
        }
    }

    private int ringIndex(int back) {   // back = 0 is the newest frame
        return (fHead - 1 - back + 2 * RING) % RING;
    }

    private void process(long now) {
        for (int e = 0; e < eCount; e++) {
            if (eState[e] != 0) {
                continue;
            }
            if (block < 0 && !notVisible) {
                if (now > eT[e] + SETTLE_TO_NS) {
                    learn(e);
                    eState[e] = 1;
                }
            } else if (block >= 0 && nSamples < samples.length) {
                if (now > eT[e] + DETECT_NS) {
                    measure(e);
                    eState[e] = 1;
                }
            } else {
                eState[e] = 1;
            }
        }
    }

    private void learn(int e) {
        long from = eT[e] + SETTLE_FROM_NS;
        long to = eT[e] + SETTLE_TO_NS;
        if (e + 1 < eCount) {
            to = Math.min(to, eT[e + 1]);
        }
        // Average of the settled frames for this one change, so every change weighs the same.
        Arrays.fill(eventMean, 0);
        int n = 0;
        for (int back = 0; back < fCount; back++) {
            int i = ringIndex(back);
            if (fT[i] >= from && fT[i] <= to) {
                int b0 = i * BLOCKS;
                for (int k = 0; k < BLOCKS; k++) {
                    eventMean[k] += fB[b0 + k];
                }
                n++;
            }
        }
        if (n == 0) {
            return;
        }
        double[] acc = eOn[e] ? sumOn : sumOff;
        for (int k = 0; k < BLOCKS; k++) {
            acc[k] += eventMean[k] / n;
        }
        if (eOn[e]) {
            nOn++;
        } else {
            nOff++;
        }
        learned++;
        if (learned >= LEARN_EVENTS && nOn >= 2 && nOff >= 2) {
            choose();
        }
    }

    private void choose() {
        int best = -1;
        double bestC = 0;
        for (int k = 0; k < BLOCKS; k++) {
            double c = Math.abs(sumOn[k] / nOn - sumOff[k] / nOff);
            if (c > bestC) {
                bestC = c;
                best = k;
            }
        }
        if (best < 0 || bestC < MIN_CONTRAST) {
            notVisible = true;
            return;
        }
        block = best;
        ledRow = best / COLS;
        levelOn = sumOn[best] / nOn;
        levelOff = sumOff[best] / nOff;
    }

    private void measure(int e) {
        double mid = 0.5 * (levelOn + levelOff);
        boolean brighterWhenOn = levelOn > levelOff;
        boolean wantHigh = eOn[e] == brighterWhenOn;
        // oldest-first search for the first frame after the change that shows it
        for (int back = fCount - 1; back >= 0; back--) {
            int i = ringIndex(back);
            if (fT[i] <= eT[e]) {
                continue;
            }
            if (fT[i] > eT[e] + DETECT_NS) {
                break;
            }
            float v = fB[i * BLOCKS + block];
            if (wantHigh ? v > mid : v < mid) {
                samples[nSamples++] = (fT[i] - eT[e]) / 1e6;
                return;
            }
        }
        missed++;
    }

    // ---------------------------------------------------------------- results

    public synchronized int sampleCount() {
        return nSamples;
    }

    public synchronized boolean isDone() {
        return nSamples >= TARGET_SAMPLES;
    }

    /** Median time between frame arrivals in the ring (ms), NaN if too few frames. */
    public synchronized double framePeriodMs() {
        if (fCount < 3) {
            return Double.NaN;
        }
        int n = 0;
        for (int back = 0; back + 1 < fCount; back++) {
            scratch[n++] = (fT[ringIndex(back)] - fT[ringIndex(back + 1)]) / 1e6;
        }
        Arrays.sort(scratch, 0, n);
        return scratch[n / 2];
    }

    /** Mean sample minus half a frame period (ms); NaN until there are samples. */
    public synchronized double suggestedLatencyMs() {
        if (nSamples == 0 || block < 0) {
            return Double.NaN;
        }
        double sum = 0;
        for (int i = 0; i < nSamples; i++) {
            sum += samples[i];
        }
        return sum / nSamples - framePeriodMs() / 2;
    }

    /** Smallest sample (ms): an upper bound on the true latency. */
    public synchronized double minSampleMs() {
        double m = Double.NaN;
        for (int i = 0; i < nSamples; i++) {
            m = Double.isNaN(m) ? samples[i] : Math.min(m, samples[i]);
        }
        return m;
    }

    /** Vertical position of the LED block in the image, 0 = top, 1 = bottom; NaN if unknown. */
    public synchronized double ledRowFraction() {
        return block < 0 ? Double.NaN : (ledRow + 0.5) / ROWS;
    }

    public synchronized String statusLine() {
        if (notVisible) {
            return "LED not visible: hold the Limelight lens a few cm from the Hub's status LED, "
                    + "in the middle of the image, then restart the test";
        }
        if (block < 0) {
            return String.format(Locale.US, "learning which pixels are the LED (%d/%d blinks)",
                    learned, LEARN_EVENTS);
        }
        String row = Math.abs(ledRowFraction() - 0.5) > 0.2
                ? " | LED is far from the middle row: move it to the image centre" : "";
        if (!isDone()) {
            return String.format(Locale.US, "measuring %d/%d (missed %d), LED contrast %.0f%s",
                    nSamples, TARGET_SAMPLES, missed, Math.abs(levelOn - levelOff), row);
        }
        return String.format(Locale.US, "DONE: latency %.1f ms (n=%d, min sample %.1f ms, "
                        + "frame period %.1f ms, missed %d). Put \"latencyMs\": %.1f in the "
                        + "limelight section of beacon.json%s", suggestedLatencyMs(), nSamples,
                minSampleMs(), framePeriodMs(), missed, suggestedLatencyMs(), row);
    }
}
