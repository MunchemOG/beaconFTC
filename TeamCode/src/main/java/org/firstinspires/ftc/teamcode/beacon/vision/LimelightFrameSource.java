package org.firstinspires.ftc.teamcode.beacon.vision;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.PinpointPoseSource;

import java.io.Closeable;
import java.io.IOException;
import java.io.InputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Frames from the Limelight 3A's built-in MJPEG camera stream, read over the Limelight's
 * USB-Ethernet link, decoded to gray on the Hub. Design section 6.2 path (a).
 *
 * <h3>Threads (nothing here runs on the OpMode thread)</h3>
 * <ul>
 *   <li>"BeaconFrameReader": connects (candidates from {@link StreamCandidates}, the last
 *       working one first), reads MJPEG parts, stamps their arrival, and hands each JPEG to the
 *       decoder through a one-slot mailbox. If the decoder has not taken the previous JPEG it
 *       is replaced (newest wins) and counted in {@link #getDroppedBeforeDecode()}: frames are
 *       dropped rather than queued, so latency never builds up. The reader never stops reading,
 *       so the Limelight's send buffer never backs up either.</li>
 *   <li>"BeaconFrameDecode": decodes the newest JPEG, computes the capture time, publishes
 *       the frame (a reference swap under a lock), then calls the {@link FrameSource.Listener}
 *       (the dataset recorder, later the native core).</li>
 * </ul>
 * Unplugging: a dead link shows up as a refused/failed connect or as no data for
 * {@code limelight.readTimeoutMs}; the stream is closed, {@link #isStreaming()} goes false,
 * telemetry says why, and the reader retries every second until the stream comes back.
 *
 * <h3>Capture timestamp: how it is estimated</h3>
 * {@code tNs = arrivalNs - latency}, where
 * <ul>
 *   <li>{@code arrivalNs} ({@link GrayFrame#arrivalNs}) is {@code elapsedRealtimeNanos()}
 *       taken right after the socket read that returned the first byte of the frame's MJPEG
 *       part. Measured. Error: the reader thread's wake-up delay, expected about 0.1-2 ms on
 *       a loaded Control Hub, occasionally more (not measured yet; the probe shows
 *       inter-arrival jitter, which bounds it).</li>
 *   <li>{@code latency} is, in order of preference:
 *     <ol>
 *       <li>{@code limelight.latencyMs} from beacon.json, which the team measures with the
 *           probe OpMode's LED test ({@link LedLatencyMeter}): end to end, from the middle of
 *           the exposure to first-byte arrival, everything included. Expected error of that
 *           measurement: about frame period / sqrt(12 N) from the random phase (under 1 ms
 *           for 30 blinks at 90 fps) plus the Hub LED command delay (a couple of ms, not
 *           measured) plus thread wake-up jitter.</li>
 *       <li>Otherwise, the Limelight's own reported latency of its newest result, capture
 *           latency {@code cl} + pipeline latency {@code tl} (Limelight docs: {@code cl} runs
 *           from the end of the middle row's exposure to the start of the pipeline), smoothed
 *           (EMA 0.2). This is a LOWER BOUND on the true latency: it leaves out half the
 *           exposure time, JPEG encoding and sending the stream, and the newest result may
 *           belong to a different frame than the streamed one. Expect the capture time to
 *           come out late by roughly 5-20 ms (unmeasured guess).</li>
 *       <li>Otherwise (no Limelight in the RC configuration, or its results are stale),
 *           {@code limelight.defaultLatencyMs}: an unmeasured guess, shown as UNMEASURED.</li>
 *     </ol></li>
 * </ul>
 * Capture times are forced strictly increasing (a frame whose estimate would not be later
 * than the previous one is stamped 1 ns after it, and counted). At 2.7 m/s, every ms of
 * timestamp error is 2.7 mm of position error, so until the LED test has been done the
 * frames are fit for recording and bench work, not for corrections at speed.
 */
public final class LimelightFrameSource implements FrameSource {

    /** Opens an HTTP stream; replaceable in tests. */
    public interface StreamOpener {
        OpenStream open(String url, int connectTimeoutMs, int readTimeoutMs) throws IOException;
    }

    /** An open HTTP response body. */
    public static final class OpenStream {
        public final InputStream in;
        public final String contentType;
        public final Closeable closer;

        public OpenStream(InputStream in, String contentType, Closeable closer) {
            this.in = in;
            this.contentType = contentType;
            this.closer = closer;
        }
    }

    /** The Limelight's own latency for its newest result (cl + tl, ms), or NaN if unknown. */
    public interface LatencyHint {
        double limelightLatencyMs();
    }

    /** Plain {@link HttpURLConnection} GET. */
    public static final class HttpOpener implements StreamOpener {
        @Override
        public OpenStream open(String url, int connectTimeoutMs, int readTimeoutMs)
                throws IOException {
            final HttpURLConnection c = (HttpURLConnection) new URL(url).openConnection();
            c.setConnectTimeout(connectTimeoutMs);
            c.setReadTimeout(readTimeoutMs);
            c.setUseCaches(false);
            c.setInstanceFollowRedirects(true);
            c.setRequestProperty("Accept", "multipart/x-mixed-replace, image/jpeg, */*");
            int code = c.getResponseCode();
            if (code != HttpURLConnection.HTTP_OK) {
                c.disconnect();
                throw new IOException("HTTP " + code);
            }
            final InputStream in = c.getInputStream();
            return new OpenStream(in, c.getContentType(), new Closeable() {
                @Override
                public void close() {
                    try {
                        in.close();
                    } catch (IOException ignored) {
                        // closing anyway
                    } catch (RuntimeException ignored) {
                        // closing anyway
                    }
                    c.disconnect();
                }
            });
        }
    }

    static final int MAX_PART_BYTES = 4 << 20;
    private static final long RETRY_SLEEP_MS = 1000;
    private static final long RECONNECT_SLEEP_MS = 100;
    private static final int R_FAILED = 0, R_NOT_MJPEG = 1, R_STREAMED = 2;
    private static final double LATENCY_EMA = 0.2;

    private final List<String> candidates;
    private final StreamOpener opener;
    private final JpegDecoder decoder;
    private final LatencyHint hint;
    private final PinpointPoseSource.TimeSource time;
    private final BeaconConfig.StreamConfig cfg;

    // lifecycle
    private volatile boolean running;
    private volatile int generation;
    private Thread reader;
    private Thread decodeThread;
    private volatile Closeable current;
    private volatile Listener listener;

    // reader -> decoder mailbox (all three swapped only under mailLock)
    private final Object mailLock = new Object();
    private MjpegStreamReader.Part readerPart = new MjpegStreamReader.Part();
    private MjpegStreamReader.Part pendingPart = new MjpegStreamReader.Part();
    private MjpegStreamReader.Part decoderPart = new MjpegStreamReader.Part();
    private boolean pendingFull;

    // decoder -> consumers (swapped only under pubLock)
    private final Object pubLock = new Object();
    private GrayFrame scratch = new GrayFrame();
    private GrayFrame published = new GrayFrame();
    private volatile long latestSeq;
    private long seqCounter;
    private long lastCaptureNs = Long.MIN_VALUE;
    private double latencyEma = Double.NaN;

    // diagnostics: written by one thread each, read by telemetry
    private final Map<String, String> attempts = new LinkedHashMap<String, String>();
    private volatile String workingUrl = "";
    private volatile String lastLoss = "not started";
    private volatile boolean streaming;
    private volatile boolean everStreamed;
    private volatile long reconnects;
    private volatile long droppedBeforeDecode;
    private volatile long decodeFailures;
    private volatile long badParts;
    private volatile long nonMonotonic;
    private volatile long listenerErrors;
    private volatile String lastListenerError = "";
    private volatile String lastDecodeError = "";
    private volatile long framesDecoded;
    private volatile int width, height, jpegBytes;
    private volatile double fps, jitterMs, maxGapMs, transferMs, decodeMsAvg, decodeMsMax;
    private volatile double latencyMs = Double.NaN;
    private volatile int latencySource;
    private volatile double decodeCpuPct = -1;
    private volatile String timestampHeader;

    // reader-thread window stats
    private long winStartNs, prevArrivalNs;
    private int winCount;
    private double winSum, winSumSq, winMax;
    // decoder-thread window stats
    private long dWinStartNs, dWinCpuNs = -1;
    private double dWinMax;

    /**
     * @param candidates URLs in the order to try (see {@link StreamCandidates#build})
     * @param hint       Limelight-reported latency, or null for none
     */
    public LimelightFrameSource(List<String> candidates, StreamOpener opener, JpegDecoder decoder,
                                LatencyHint hint, PinpointPoseSource.TimeSource time,
                                BeaconConfig.StreamConfig cfg) {
        this.candidates = new ArrayList<String>(candidates);
        this.opener = opener;
        this.decoder = decoder;
        this.hint = hint;
        this.time = time;
        this.cfg = cfg;
    }

    /** The Hub's clock, {@code SystemClock.elapsedRealtimeNanos()}. */
    public static PinpointPoseSource.TimeSource hubClock() {
        return new PinpointPoseSource.TimeSource() {
            @Override
            public long nowNs() {
                return SystemClock.elapsedRealtimeNanos();
            }
        };
    }

    public List<String> getCandidates() {
        return new ArrayList<String>(candidates);
    }

    // ------------------------------------------------------------- lifecycle

    @Override
    public synchronized void start() {
        if (running) {
            return;
        }
        running = true;
        final int gen = ++generation;
        lastLoss = "connecting";
        reader = new Thread(new Runnable() {
            @Override
            public void run() {
                readerLoop(gen);
            }
        }, "BeaconFrameReader");
        decodeThread = new Thread(new Runnable() {
            @Override
            public void run() {
                decodeLoop(gen);
            }
        }, "BeaconFrameDecode");
        reader.setDaemon(true);
        decodeThread.setDaemon(true);
        reader.start();
        decodeThread.start();
    }

    /** Stops both threads without waiting for a hung read. Idempotent. */
    @Override
    public synchronized void stop() {
        running = false;
        generation++;
        closeCurrent();
        synchronized (mailLock) {
            mailLock.notifyAll();
        }
        if (reader != null) {
            reader.interrupt();
        }
        if (decodeThread != null) {
            decodeThread.interrupt();
        }
        reader = null;
        decodeThread = null;
        streaming = false;
        lastLoss = "stopped";
    }

    private boolean alive(int gen) {
        return running && generation == gen;
    }

    private void closeCurrent() {
        Closeable c = current;
        current = null;
        if (c != null) {
            try {
                c.close();
            } catch (IOException ignored) {
                // closing anyway
            } catch (RuntimeException ignored) {
                // closing anyway
            }
        }
    }

    // ---------------------------------------------------------------- reader

    private void readerLoop(int gen) {
        String preferred = null;
        while (alive(gen)) {
            List<String> order = new ArrayList<String>();
            if (preferred != null) {
                order.add(preferred);
            }
            for (String c : candidates) {
                if (!order.contains(c)) {
                    order.add(c);
                }
            }
            String worked = null;
            for (int i = 0; i < order.size() && alive(gen) && worked == null; i++) {
                String url = order.get(i);
                int r = tryUrl(url, gen);
                if (r == R_STREAMED) {
                    worked = url;
                } else if (r == R_NOT_MJPEG) {
                    for (String v : StreamCandidates.pathVariants(url)) {
                        if (!alive(gen)) {
                            break;
                        }
                        if (tryUrl(v, gen) == R_STREAMED) {
                            worked = v;
                            break;
                        }
                    }
                }
            }
            if (worked != null) {
                preferred = worked;
            }
            if (!sleep(worked != null ? RECONNECT_SLEEP_MS : RETRY_SLEEP_MS, gen)) {
                return;
            }
        }
    }

    private boolean sleep(long ms, int gen) {
        long end = System.currentTimeMillis() + ms;
        while (alive(gen)) {
            long left = end - System.currentTimeMillis();
            if (left <= 0) {
                return true;
            }
            try {
                Thread.sleep(Math.min(left, 50));
            } catch (InterruptedException e) {
                return false;
            }
        }
        return false;
    }

    private synchronized void attempt(String url, String result) {
        attempts.put(url, result);
    }

    private int tryUrl(String url, int gen) {
        OpenStream s;
        try {
            s = opener.open(url, cfg.connectTimeoutMs, cfg.readTimeoutMs);
        } catch (IOException e) {
            attempt(url, url + ": " + reason(e));
            return R_FAILED;
        } catch (RuntimeException e) {
            attempt(url, url + ": " + reason(e));
            return R_FAILED;
        }
        current = s.closer;
        if (!alive(gen)) {
            closeCurrent();
            return R_FAILED;
        }
        String boundary = MjpegStreamReader.boundaryFromContentType(s.contentType);
        if (boundary == null) {
            closeCurrent();
            attempt(url, url + ": answers, but not MJPEG (" + s.contentType + ")");
            return R_NOT_MJPEG;
        }
        MjpegStreamReader r = new MjpegStreamReader(s.in, boundary, time, MAX_PART_BYTES);
        long frames = 0;
        String why = "stream ended";
        attempt(url, "OK " + url + " (connecting)");
        try {
            while (alive(gen)) {
                if (!r.next(readerPart)) {
                    break;
                }
                frames++;
                if (frames == 1) {
                    if (everStreamed) {
                        reconnects++;
                    }
                    everStreamed = true;
                    workingUrl = url;
                    streaming = true;
                }
                badParts = r.badParts();
                onArrival(readerPart);
                handOff();
            }
        } catch (IOException e) {
            why = reason(e);
        } catch (RuntimeException e) {
            why = "reader error: " + e;
        } finally {
            streaming = false;
            closeCurrent();
        }
        if (!alive(gen)) {
            return R_FAILED;
        }
        lastLoss = frames > 0 ? "lost " + url + " after " + frames + " frames: " + why
                : url + " sent no frames: " + why;
        attempt(url, frames > 0 ? "OK " + url + " then lost after " + frames + " frames: " + why
                : url + ": connected, no frames: " + why);
        return frames > 0 ? R_STREAMED : R_FAILED;
    }

    private static String reason(Exception e) {
        String m = e.getMessage();
        String n = e.getClass().getSimpleName();
        return m == null ? n : n + " " + m;
    }

    /** Arrival statistics, reader thread. */
    private void onArrival(MjpegStreamReader.Part p) {
        long t = p.firstByteNs;
        transferMs = (p.lastByteNs - p.firstByteNs) / 1e6;
        timestampHeader = p.timestampHeader;
        if (winStartNs == 0) {
            winStartNs = t;
        } else if (prevArrivalNs != 0) {
            double dt = (t - prevArrivalNs) / 1e6;
            winCount++;
            winSum += dt;
            winSumSq += dt * dt;
            winMax = Math.max(winMax, dt);
        }
        prevArrivalNs = t;
        double span = (t - winStartNs) / 1e9;
        if (span >= 1.0 && winCount > 0) {
            fps = winCount / span;
            double mean = winSum / winCount;
            jitterMs = Math.sqrt(Math.max(0, winSumSq / winCount - mean * mean));
            maxGapMs = winMax;
            winStartNs = t;
            winCount = 0;
            winSum = 0;
            winSumSq = 0;
            winMax = 0;
        }
    }

    private void handOff() {
        synchronized (mailLock) {
            MjpegStreamReader.Part t = pendingPart;
            pendingPart = readerPart;
            readerPart = t;
            if (pendingFull) {
                droppedBeforeDecode++;
            }
            pendingFull = true;
            mailLock.notifyAll();
        }
    }

    // --------------------------------------------------------------- decoder

    private void decodeLoop(int gen) {
        while (alive(gen)) {
            synchronized (mailLock) {
                while (!pendingFull && alive(gen)) {
                    try {
                        mailLock.wait(100);
                    } catch (InterruptedException e) {
                        return;
                    }
                }
                if (!alive(gen)) {
                    return;
                }
                MjpegStreamReader.Part t = decoderPart;
                decoderPart = pendingPart;
                pendingPart = t;
                pendingFull = false;
            }
            decodeOne(decoderPart, gen);
        }
    }

    private void decodeOne(MjpegStreamReader.Part p, int gen) {
        long t0 = time.nowNs();
        boolean ok;
        try {
            ok = decoder.decode(p.data, p.length, scratch);
            if (!ok) {
                lastDecodeError = "not a decodable JPEG";
            }
        } catch (RuntimeException e) {
            ok = false;
            lastDecodeError = e.toString();
        } catch (OutOfMemoryError e) {
            ok = false;
            lastDecodeError = "out of memory decoding a " + p.length + "-byte JPEG";
        }
        long t1 = time.nowNs();
        if (!ok || scratch.width <= 0 || scratch.height <= 0) {
            decodeFailures++;
            return;
        }
        if (!alive(gen)) {
            return;
        }

        // Capture time = first-byte arrival - latency (see the class Javadoc).
        double lat;
        int src;
        double h = hint == null ? Double.NaN : safeHint();
        if (cfg.latencyMs >= 0) {
            lat = cfg.latencyMs;
            src = GrayFrame.LATENCY_CONFIGURED;
        } else if (!Double.isNaN(h)) {
            latencyEma = Double.isNaN(latencyEma) ? h
                    : latencyEma + LATENCY_EMA * (h - latencyEma);
            lat = latencyEma;
            src = GrayFrame.LATENCY_LIMELIGHT;
        } else {
            lat = cfg.defaultLatencyMs;
            src = GrayFrame.LATENCY_DEFAULT;
        }
        long capture = p.firstByteNs - Math.round(lat * 1e6);
        if (capture <= lastCaptureNs) {
            capture = lastCaptureNs + 1;
            nonMonotonic++;
        }
        lastCaptureNs = capture;

        GrayFrame f = scratch;
        f.seq = ++seqCounter;
        f.tNs = capture;
        f.arrivalNs = p.firstByteNs;
        f.lastByteNs = p.lastByteNs;
        f.latencyMs = lat;
        f.latencySource = src;
        f.jpegBytes = p.length;
        f.decodeNs = t1 - t0;
        synchronized (pubLock) {
            scratch = published;
            published = f;
        }
        latestSeq = f.seq;
        width = f.width;
        height = f.height;
        jpegBytes = f.jpegBytes;
        latencyMs = lat;
        latencySource = src;
        decodeStats(f.decodeNs / 1e6, t1);
        framesDecoded++;

        Listener l = listener;
        if (l != null) {
            try {
                l.onFrame(f);     // f stays untouched until the next decode on this thread
            } catch (RuntimeException e) {
                listenerErrors++;
                lastListenerError = e.toString();
            }
        }
    }

    private double safeHint() {
        try {
            double v = hint.limelightLatencyMs();
            return (v >= 0 && v < 1000) ? v : Double.NaN;
        } catch (RuntimeException e) {
            return Double.NaN;
        }
    }

    private void decodeStats(double ms, long now) {
        decodeMsAvg = Double.isNaN(decodeMsAvg) || framesDecoded == 0 ? ms
                : 0.9 * decodeMsAvg + 0.1 * ms;
        dWinMax = Math.max(dWinMax, ms);
        if (dWinStartNs == 0) {
            dWinStartNs = now;
            dWinCpuNs = CpuTime.threadNs();
            return;
        }
        double span = (now - dWinStartNs) / 1e9;
        if (span >= 1.0) {
            decodeMsMax = dWinMax;
            dWinMax = 0;
            long cpu = CpuTime.threadNs();
            if (cpu >= 0 && dWinCpuNs >= 0) {
                decodeCpuPct = 100.0 * (cpu - dWinCpuNs) / (span * 1e9);
            }
            dWinCpuNs = cpu;
            dWinStartNs = now;
        }
    }

    // ------------------------------------------------------------- consumers

    @Override
    public boolean latest(GrayFrame out) {
        synchronized (pubLock) {
            if (published.seq == 0 || published.seq <= out.seq) {
                return false;
            }
            out.copyFrom(published);
            return true;
        }
    }

    @Override
    public long latestSeq() {
        return latestSeq;
    }

    @Override
    public void setListener(Listener l) {
        listener = l;
    }

    @Override
    public boolean isStreaming() {
        return streaming;
    }

    // ----------------------------------------------------------- diagnostics

    public String getWorkingUrl() {
        return workingUrl;
    }

    /** Every candidate tried and its last result, for telemetry. */
    public synchronized String getAttemptsLine() {
        StringBuilder sb = new StringBuilder();
        for (String v : attempts.values()) {
            if (sb.length() > 0) {
                sb.append("; ");
            }
            sb.append(v);
        }
        return sb.length() == 0 ? "(none yet)" : sb.toString();
    }

    /** Why the stream is not running (or "stopped"). */
    public String getLastLoss() {
        return lastLoss;
    }

    public long getFramesDecoded() {
        return framesDecoded;
    }

    public long getReconnects() {
        return reconnects;
    }

    public long getDroppedBeforeDecode() {
        return droppedBeforeDecode;
    }

    public long getDecodeFailures() {
        return decodeFailures;
    }

    public long getBadParts() {
        return badParts;
    }

    public long getNonMonotonicStamps() {
        return nonMonotonic;
    }

    public long getListenerErrors() {
        return listenerErrors;
    }

    public double getFps() {
        return fps;
    }

    /** Standard deviation of the time between frame arrivals over the last second (ms). */
    public double getJitterMs() {
        return jitterMs;
    }

    /** Longest time between two frame arrivals over the last second (ms). */
    public double getMaxGapMs() {
        return maxGapMs;
    }

    /** First to last byte of the newest frame (ms): the link's transfer time for one frame. */
    public double getTransferMs() {
        return transferMs;
    }

    public double getDecodeMsAvg() {
        return decodeMsAvg;
    }

    public double getDecodeMsMax() {
        return decodeMsMax;
    }

    /** CPU used by the decode thread, percent of one core; -1 if unknown. */
    public double getDecodeCpuPct() {
        return decodeCpuPct;
    }

    public int getWidth() {
        return width;
    }

    public int getHeight() {
        return height;
    }

    public int getJpegBytes() {
        return jpegBytes;
    }

    public double getLatencyMs() {
        return latencyMs;
    }

    public int getLatencySource() {
        return latencySource;
    }

    /** Raw value of a per-frame timestamp header from the server, or null if none is sent. */
    public String getTimestampHeader() {
        return timestampHeader;
    }

    @Override
    public String debugLine() {
        if (!running) {
            return "camera: stopped";
        }
        StringBuilder sb = new StringBuilder(200);
        if (streaming) {
            sb.append("STREAMING ").append(workingUrl)
                    .append(String.format(" | %.1f fps %dx%d jpg %.0fKB dec %.1f/%.1fms "
                                    + "(cpu %s) xfer %.1fms jit %.1fms lat %.1fms (%s)",
                            fps, width, height, jpegBytes / 1024.0, decodeMsAvg, decodeMsMax,
                            decodeCpuPct < 0 ? "n/a" : String.format("%.0f%%", decodeCpuPct),
                            transferMs, jitterMs, latencyMs,
                            GrayFrame.latencySourceName(latencySource)));
        } else {
            sb.append("NO STREAM: ").append(lastLoss).append(" | tried: ")
                    .append(getAttemptsLine());
        }
        if (droppedBeforeDecode > 0 || decodeFailures > 0 || reconnects > 0) {
            sb.append(" | drop ").append(droppedBeforeDecode).append(" fail ")
                    .append(decodeFailures).append(" reconn ").append(reconnects);
        }
        if (decodeFailures > 0) {
            sb.append(" (").append(lastDecodeError).append(')');
        }
        if (listenerErrors > 0) {
            sb.append(" | listener errors ").append(listenerErrors).append(": ")
                    .append(lastListenerError);
        }
        return sb.toString();
    }
}
