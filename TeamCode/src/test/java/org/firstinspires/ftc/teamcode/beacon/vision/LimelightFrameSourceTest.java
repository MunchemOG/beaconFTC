package org.firstinspires.ftc.teamcode.beacon.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.PinpointPoseSource;
import org.junit.After;
import org.junit.Test;

import java.net.ServerSocket;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Runs {@link LimelightFrameSource} against a local fake MJPEG server over real sockets, with
 * the production HTTP opener. Only the JPEG codec (the TestImages fake container instead of BitmapFactory)
 * and the clock (System.nanoTime instead of elapsedRealtimeNanos) are stand-ins.
 */
public class LimelightFrameSourceTest {

    static final PinpointPoseSource.TimeSource NANO = new PinpointPoseSource.TimeSource() {
        @Override
        public long nowNs() {
            return System.nanoTime();
        }
    };

    static final class FixedHint implements LimelightFrameSource.LatencyHint {
        volatile double ms;

        FixedHint(double ms) {
            this.ms = ms;
        }

        @Override
        public double limelightLatencyMs() {
            return ms;
        }
    }

    private final List<FakeMjpegServer> servers = new ArrayList<FakeMjpegServer>();
    private final List<LimelightFrameSource> sources = new ArrayList<LimelightFrameSource>();

    @After
    public void tearDown() {
        for (LimelightFrameSource s : sources) {
            s.stop();
        }
        for (FakeMjpegServer s : servers) {
            s.stop();
        }
    }

    private FakeMjpegServer server(String path, double fps) throws Exception {
        FakeMjpegServer s = new FakeMjpegServer(path, 64, 48, fps);
        s.start();
        servers.add(s);
        return s;
    }

    private static BeaconConfig.StreamConfig cfg(double latencyMs, int readTimeoutMs) {
        return new BeaconConfig.StreamConfig("", 300, readTimeoutMs, latencyMs, 30);
    }

    private LimelightFrameSource source(List<String> urls, JpegDecoder dec,
                                        LimelightFrameSource.LatencyHint hint,
                                        BeaconConfig.StreamConfig c) {
        LimelightFrameSource s = new LimelightFrameSource(urls, new LimelightFrameSource.HttpOpener(),
                dec, hint, NANO, c);
        sources.add(s);
        return s;
    }

    private static boolean waitFrames(LimelightFrameSource s, long atLeast, long timeoutMs)
            throws InterruptedException {
        long end = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < end) {
            if (s.getFramesDecoded() >= atLeast) {
                return true;
            }
            Thread.sleep(5);
        }
        return false;
    }

    private static boolean waitNotStreaming(LimelightFrameSource s, long timeoutMs)
            throws InterruptedException {
        long end = System.currentTimeMillis() + timeoutMs;
        while (System.currentTimeMillis() < end) {
            if (!s.isStreaming()) {
                return true;
            }
            Thread.sleep(5);
        }
        return false;
    }

    private static int closedPort() throws Exception {
        ServerSocket s = new ServerSocket(0);
        int p = s.getLocalPort();
        s.close();
        return p;
    }

    @Test
    public void streamsAndDecodesOffTheCallerThreadAndStampsCaptureTime() throws Exception {
        FakeMjpegServer srv = server("/", 60);
        TestImages.FakeJpegDecoder dec = new TestImages.FakeJpegDecoder();
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")), dec, new FixedHint(Double.NaN),
                cfg(20, 1000));
        assertFalse(s.isStreaming());
        s.start();
        assertTrue("no frames: " + s.debugLine(), waitFrames(s, 5, 5000));
        assertTrue(s.isStreaming());
        assertEquals(srv.url("/"), s.getWorkingUrl());

        GrayFrame f = new GrayFrame();
        assertTrue(s.latest(f));
        assertEquals(64, f.width);
        assertEquals(48, f.height);
        assertEquals(64 * 48, f.gray.length);
        assertTrue(f.seq > 0);
        assertEquals("capture = first byte - configured latency", f.arrivalNs - 20000000L, f.tNs);
        assertEquals(GrayFrame.LATENCY_CONFIGURED, f.latencySource);
        assertTrue(f.jpegBytes > 0);
        long prevSeq = f.seq;
        if (s.latest(f)) {
            assertTrue("latest() only reports a frame newer than the copy", f.seq > prevSeq);
        }
        assertNotEquals(Thread.currentThread().getName(), dec.lastThread);
        assertEquals("BeaconFrameDecode", dec.lastThread);
        assertTrue(s.debugLine(), s.debugLine().contains(srv.url("/")));
    }

    @Test
    public void frameSourceUnpluggedReportsAndRecovers() throws Exception {
        FakeMjpegServer srv = server("/", 60);
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")),
                new TestImages.FakeJpegDecoder(), new FixedHint(Double.NaN), cfg(20, 300));
        s.start();
        assertTrue(waitFrames(s, 3, 5000));

        srv.stop();   // device unplugged: connection dies, port refuses
        assertTrue("still streaming after unplug: " + s.debugLine(), waitNotStreaming(s, 3000));
        long before = s.getFramesDecoded();
        String line = s.debugLine();
        assertTrue(line, line.contains("NO STREAM"));
        assertEquals(before, s.getFramesDecoded());
        GrayFrame f = new GrayFrame();
        s.latest(f);
        long seqAtUnplug = f.seq;

        srv.start();  // plugged back in on the same address
        assertTrue("did not recover: " + s.debugLine(), waitFrames(s, before + 3, 8000));
        assertTrue(s.isStreaming());
        assertTrue(s.getReconnects() >= 1);
        assertTrue(s.latest(f));
        assertTrue(f.seq > seqAtUnplug);
    }

    @Test
    public void stalledStreamIsDetectedByTheReadTimeout() throws Exception {
        FakeMjpegServer srv = server("/", 60);
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")),
                new TestImages.FakeJpegDecoder(), new FixedHint(Double.NaN), cfg(20, 250));
        s.start();
        assertTrue(waitFrames(s, 3, 5000));
        srv.paused = true;   // socket open, no data (camera hung)
        assertTrue(s.debugLine(), waitNotStreaming(s, 3000));
        srv.paused = false;
        long n = s.getFramesDecoded();
        assertTrue(s.debugLine(), waitFrames(s, n + 3, 8000));
    }

    @Test
    public void triesCandidatesInOrderAndReportsWhichWorked() throws Exception {
        FakeMjpegServer srv = server("/stream.mjpg", 60);
        String refused = "http://127.0.0.1:" + closedPort() + "/";
        LimelightFrameSource s = source(Arrays.asList(refused, srv.url("/")),
                new TestImages.FakeJpegDecoder(), new FixedHint(Double.NaN), cfg(20, 1000));
        s.start();
        assertTrue(s.debugLine(), waitFrames(s, 3, 8000));
        // "/" is an HTML page on this server, so the path variants are tried next.
        assertEquals(srv.url("/stream.mjpg"), s.getWorkingUrl());
        String attempts = s.getAttemptsLine();
        assertTrue(attempts, attempts.contains(refused));
        assertTrue(attempts, attempts.contains("not MJPEG"));
        assertTrue(attempts, attempts.contains("OK " + srv.url("/stream.mjpg")));
    }

    @Test
    public void latencyComesFromLimelightResultsWhenNotConfigured() throws Exception {
        FakeMjpegServer srv = server("/", 60);
        FixedHint hint = new FixedHint(25.0);
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")),
                new TestImages.FakeJpegDecoder(), hint, cfg(-1, 1000));
        s.start();
        assertTrue(waitFrames(s, 2, 5000));
        GrayFrame f = new GrayFrame();
        assertTrue(s.latest(f));
        assertEquals(GrayFrame.LATENCY_LIMELIGHT, f.latencySource);
        assertEquals(25.0, f.latencyMs, 1e-9);
        assertEquals(f.arrivalNs - 25000000L, f.tNs);

        hint.ms = Double.NaN;   // results stale or no Limelight in the configuration
        long n = s.getFramesDecoded();
        assertTrue(waitFrames(s, n + 2, 5000));
        assertTrue(s.latest(f));
        assertEquals(GrayFrame.LATENCY_DEFAULT, f.latencySource);
        assertEquals(30.0, f.latencyMs, 1e-9);
        assertTrue(s.debugLine(), s.debugLine().contains("UNMEASURED"));
    }

    @Test
    public void captureTimesStrictlyIncreaseAndLatestNeverBlocks() throws Exception {
        FakeMjpegServer srv = server("/", 100);
        final FixedHint hint = new FixedHint(10);
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")),
                new TestImages.FakeJpegDecoder(), hint, cfg(-1, 1000));
        final List<Long> stamps = new ArrayList<Long>();
        s.setListener(new FrameSource.Listener() {
            @Override
            public void onFrame(GrayFrame f) {
                synchronized (stamps) {
                    stamps.add(f.tNs);
                }
                // Latency estimate jumps around; capture times must still never go backwards.
                hint.ms = (f.seq % 2 == 0) ? 5 : 60;
            }
        });
        s.start();
        GrayFrame f = new GrayFrame();
        long[] took = new long[2000000];
        int n = 0;
        long end = System.currentTimeMillis() + 1500;
        while (System.currentTimeMillis() < end && n < took.length) {
            long t0 = System.nanoTime();
            s.latest(f);
            took[n++] = System.nanoTime() - t0;
        }
        java.util.Arrays.sort(took, 0, n);
        // The lock is only ever held for one frame copy or a reference swap. A single slow
        // sample can be a JVM/OS pause, so judge the 99.9th percentile; report the worst.
        long p999 = took[(int) (n * 0.999)];
        assertTrue("latest() p99.9 " + p999 / 1e6 + " ms (worst " + took[n - 1] / 1e6
                + " ms over " + n + " calls)", p999 < 2000000L);
        synchronized (stamps) {
            assertTrue(stamps.size() > 10);
            for (int i = 1; i < stamps.size(); i++) {
                assertTrue("t_ns not increasing at " + i, stamps.get(i) > stamps.get(i - 1));
            }
        }
    }

    @Test
    public void slowDecoderDropsOldFramesInsteadOfFallingBehind() throws Exception {
        FakeMjpegServer srv = server("/", 200);
        JpegDecoder slow = new JpegDecoder() {
            final TestImages.FakeJpegDecoder real = new TestImages.FakeJpegDecoder();

            @Override
            public boolean decode(byte[] jpeg, int length, GrayFrame out) {
                try {
                    Thread.sleep(40);
                } catch (InterruptedException e) {
                    return false;
                }
                return real.decode(jpeg, length, out);
            }
        };
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")), slow, new FixedHint(1),
                cfg(-1, 1000));
        s.start();
        assertTrue(waitFrames(s, 10, 8000));
        assertTrue("expected drops: " + s.debugLine(), s.getDroppedBeforeDecode() > 0);
        GrayFrame f = new GrayFrame();
        assertTrue(s.latest(f));
        // The decoded frame is at most ~one decode time old, not a growing backlog.
        long ageMs = (System.nanoTime() - f.arrivalNs) / 1000000L;
        assertTrue("frame age " + ageMs + " ms", ageMs < 200);
    }

    @Test
    public void decoderFailuresAndListenerCrashesAreCountedNotFatal() throws Exception {
        FakeMjpegServer srv = server("/", 60);
        JpegDecoder flaky = new JpegDecoder() {
            final TestImages.FakeJpegDecoder real = new TestImages.FakeJpegDecoder();
            int n;

            @Override
            public boolean decode(byte[] jpeg, int length, GrayFrame out) {
                n++;
                if (n % 3 == 0) {
                    throw new RuntimeException("boom");
                }
                if (n % 3 == 1) {
                    return false;
                }
                return real.decode(jpeg, length, out);
            }
        };
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")), flaky, new FixedHint(1),
                cfg(-1, 1000));
        s.setListener(new FrameSource.Listener() {
            @Override
            public void onFrame(GrayFrame f) {
                throw new IllegalStateException("listener bug");
            }
        });
        s.start();
        assertTrue(s.debugLine(), waitFrames(s, 4, 8000));
        assertTrue(s.getDecodeFailures() >= 4);
        assertTrue(s.debugLine(), s.debugLine().contains("listener"));
    }

    @Test
    public void stopIsPromptIdempotentAndSafeBeforeStart() throws Exception {
        FakeMjpegServer srv = server("/", 60);
        LimelightFrameSource s = source(Arrays.asList(srv.url("/")),
                new TestImages.FakeJpegDecoder(), new FixedHint(1), cfg(-1, 1000));
        s.stop();
        s.start();
        assertTrue(waitFrames(s, 2, 5000));
        long t0 = System.nanoTime();
        s.stop();
        s.stop();
        assertTrue("stop took too long", System.nanoTime() - t0 < 500000000L);
        Thread.sleep(150);
        long n = s.getFramesDecoded();
        Thread.sleep(200);
        assertEquals(n, s.getFramesDecoded());
        assertFalse(s.isStreaming());
        assertTrue(s.debugLine(), s.debugLine().contains("stopped"));
    }

    @Test
    public void unreachableEverywhereReportsEveryCandidate() throws Exception {
        String a = "http://127.0.0.1:" + closedPort() + "/";
        String b = "http://127.0.0.1:" + closedPort() + "/";
        LimelightFrameSource s = source(Arrays.asList(a, b), new TestImages.FakeJpegDecoder(),
                new FixedHint(1), cfg(-1, 1000));
        s.start();
        Thread.sleep(400);
        assertFalse(s.isStreaming());
        String att = s.getAttemptsLine();
        assertTrue(att, att.contains(a) && att.contains(b));
        assertTrue(s.debugLine(), s.debugLine().contains("NO STREAM"));
    }
}
