package org.firstinspires.ftc.teamcode.beacon.vision;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconLocalizer;
import org.firstinspires.ftc.teamcode.beacon.BeaconStatus;
import org.firstinspires.ftc.teamcode.beacon.FieldFix;
import org.firstinspires.ftc.teamcode.beacon.Pose3dStamped;
import org.firstinspires.ftc.teamcode.beacon.PoseSource;
import org.junit.After;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.List;

public class VisionFixSourceTest {

    @Rule
    public TemporaryFolder tmp = new TemporaryFolder();

    private final List<VisionFixSource> sources = new ArrayList<VisionFixSource>();

    @After
    public void tearDown() {
        for (VisionFixSource v : sources) {
            v.stop();
        }
    }

    static final String CAMERA = "{\"width\": 64, \"height\": 48, \"fx\": 62.0, \"fy\": 62.0, "
            + "\"cx\": 31.5, \"cy\": 23.5, \"dist\": [0.01, 0, 0, 0, 0], \"mount\": {}, "
            + "\"rollingShutterReadoutS\": 0.02}";

    /** A robot-pose source the test moves by hand (stands in for the Pinpoint). */
    static final class HandSource implements PoseSource {
        volatile Pose3dStamped latest = new Pose3dStamped(0, 0, 0, 0, 0, 0, 1, 0,
                BeaconStatus.INITIALIZING);
        long t = 1000000000L;
        double x;
        int epoch;

        void step(double dx) {
            t += 2000000L;
            x += dx;
            latest = new Pose3dStamped(x, 0, 0, 0, 0, 0, 1, t, BeaconStatus.TRACKING, epoch);
        }

        @Override
        public void start(android.content.Context ctx, BeaconConfig cfg) {
        }

        @Override
        public void stop() {
        }

        @Override
        public Pose3dStamped latest() {
            return latest;
        }

        @Override
        public boolean isTracking() {
            return latest.status == BeaconStatus.TRACKING;
        }

        @Override
        public String debugLine() {
            return "hand";
        }

        @Override
        public boolean reportsRobotPose() {
            return true;
        }
    }

    /** Fake native core: records calls, optionally slow, hands out queued fixes. */
    static final class FakeCore implements BeaconCore {
        final List<long[]> odomT = new ArrayList<long[]>();
        final List<double[]> starts = new ArrayList<double[]>();
        volatile int frames, framesErr;
        volatile long pushOdomSleepMs;
        volatile String createError;
        volatile long[] fixT;
        volatile double[] fixV;
        volatile boolean destroyed;
        volatile String fieldJson;
        volatile BeaconCore.Camera camera;
        volatile int status = BeaconStatus.INITIALIZING;

        @Override
        public boolean create(String fieldJson, BeaconCore.Camera cam,
                              BeaconConfig.CameraMount mount, String paramsJson, String[] err) {
            if (createError != null) {
                err[0] = createError;
                return false;
            }
            this.fieldJson = fieldJson;
            this.camera = cam;
            return true;
        }

        @Override
        public void setStartPose(long t, double x, double y, double h) {
            synchronized (starts) {
                starts.add(new double[] {t, x, y, h});
            }
        }

        @Override
        public void pushOdom(long t, double x, double y, double h) {
            if (pushOdomSleepMs > 0) {
                try {
                    Thread.sleep(pushOdomSleepMs);
                } catch (InterruptedException e) {
                    return;
                }
            }
            synchronized (odomT) {
                odomT.add(new long[] {t});
            }
        }

        @Override
        public int pushFrame(long t, byte[] gray, int w, int h, int stride) {
            if (camera != null && (w != camera.width || h != camera.height)) {
                framesErr++;
                return BeaconCore.ERR;
            }
            frames++;
            return BeaconCore.OK;
        }

        @Override
        public synchronized boolean latestFix(long[] t, double[] v) {
            if (fixT == null) {
                return false;
            }
            t[0] = fixT[0];
            System.arraycopy(fixV, 0, v, 0, 7);
            fixT = null;
            return true;
        }

        synchronized void giveFix(long t, double x, double y, double h, double sxy, double sh,
                                  int kind) {
            fixV = new double[] {x, y, h, sxy, sh, 100, kind};
            fixT = new long[] {t};
        }

        @Override
        public int status() {
            return status;
        }

        @Override
        public String debugLine() {
            return "fake core ok";
        }

        @Override
        public void destroy() {
            destroyed = true;
        }
    }

    static final class FakeFrames implements FrameSource {
        volatile Listener listener;

        @Override
        public void start() {
        }

        @Override
        public void stop() {
        }

        @Override
        public boolean latest(GrayFrame out) {
            return false;
        }

        @Override
        public long latestSeq() {
            return 0;
        }

        @Override
        public void setListener(Listener l) {
            listener = l;
        }

        @Override
        public boolean isStreaming() {
            return true;
        }

        @Override
        public String debugLine() {
            return "fake frames";
        }
    }

    private BeaconConfig cfg(boolean withCamera, boolean withField) throws Exception {
        File dir = tmp.newFolder();
        File cam = new File(dir, "camera.json");
        File field = new File(dir, "field.json");
        if (withCamera) {
            Files.write(cam.toPath(), CAMERA.getBytes(Charset.forName("UTF-8")));
        }
        if (withField) {
            Files.write(field.toPath(), "{\"schema\": 1}".getBytes(Charset.forName("UTF-8")));
        }
        return BeaconConfig.parse("{\"vision\": {\"cameraFile\": \"" + cam.getPath()
                + "\", \"fieldFile\": \"" + field.getPath() + "\"}}");
    }

    private VisionFixSource source(FrameSource frames, PoseSource odom, BeaconConfig c,
                                   BeaconCore core) {
        VisionFixSource v = new VisionFixSource(frames, odom, c, core);
        sources.add(v);
        return v;
    }

    private static boolean waitFor(VisionFixSourceTest.Cond c, long ms) throws Exception {
        long end = System.currentTimeMillis() + ms;
        while (System.currentTimeMillis() < end) {
            if (c.ok()) {
                return true;
            }
            Thread.sleep(2);
        }
        return c.ok();
    }

    interface Cond {
        boolean ok();
    }

    @Test
    public void missingNativeLibFallsBackToPinpoint() throws Exception {
        // On the desktop JVM libbeacon_jni is not on the library path: exactly the case of a
        // robot whose APK lacks the library (or a Hub whose ABI it was not built for).
        assertFalse(NativeBeacon.isLoaded());
        assertNotNull(NativeBeacon.loadError());
        HandSource pin = new HandSource();
        VisionFixSource v = new VisionFixSource(new FakeFrames(), pin, cfg(true, true));
        sources.add(v);
        assertFalse(v.isNativeLoaded());
        BeaconLocalizer loc = new BeaconLocalizer(pin, BeaconConfig.parse("{}"), v, null);
        v.attachLocalizer(loc);
        loc.start(null);
        v.start();
        loc.setPose(0.5, 0.5, 0);
        for (int i = 0; i < 100; i++) {
            pin.step(0.001);
            loc.update();
        }
        assertEquals(BeaconStatus.TRACKING, loc.getStatus());   // pure Pinpoint
        // aligned at the first sample, then 99 more 1 mm steps
        assertEquals(0.5 + 0.099, loc.getPose().x, 1e-9);
        assertNull(v.latestFix());
        assertEquals(0, loc.getFixesApplied());
        assertTrue(v.debugLine(), v.debugLine().startsWith("vision: native library not loaded"));
        assertEquals(BeaconStatus.INVALID, v.status());
    }

    @Test
    public void odomFedWithoutBlockingUpdate() throws Exception {
        HandSource pin = new HandSource();
        FakeCore core = new FakeCore();
        core.pushOdomSleepMs = 20;     // a pathologically slow core
        VisionFixSource v = source(new FakeFrames(), pin, cfg(true, true), core);
        BeaconLocalizer loc = new BeaconLocalizer(pin, BeaconConfig.parse("{}"), v, null);
        v.attachLocalizer(loc);
        loc.start(null);
        v.start();
        loc.setPose(0, 0, 0);
        long worst = 0;
        for (int i = 0; i < 300; i++) {
            pin.step(0.0005);
            long t0 = System.nanoTime();
            loc.update();
            worst = Math.max(worst, System.nanoTime() - t0);
            Thread.sleep(1);
        }
        assertTrue("update() took " + worst / 1e6 + " ms", worst < 5000000L);
        synchronized (core.odomT) {
            assertTrue("odometry reached the core: " + core.odomT.size(), core.odomT.size() > 5);
            for (int i = 1; i < core.odomT.size(); i++) {
                assertTrue(core.odomT.get(i)[0] > core.odomT.get(i - 1)[0]);
            }
        }
    }

    @Test
    public void startPoseFollowsSetPoseAndWorldResets() throws Exception {
        HandSource pin = new HandSource();
        final FakeCore core = new FakeCore();
        VisionFixSource v = source(new FakeFrames(), pin, cfg(true, true), core);
        BeaconLocalizer loc = new BeaconLocalizer(pin, BeaconConfig.parse("{}"), v, null);
        v.attachLocalizer(loc);
        loc.start(null);
        v.start();
        loc.setPose(1.0, -0.5, 0.25);
        pin.step(0);
        loc.update();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                synchronized (core.starts) {
                    return core.starts.size() == 1;
                }
            }
        }, 2000));
        synchronized (core.starts) {
            double[] s = core.starts.get(0);
            assertEquals(1.0, s[1], 1e-12);
            assertEquals(-0.5, s[2], 1e-12);
            assertEquals(0.25, s[3], 1e-12);
            assertEquals((double) pin.t, s[0], 0);
        }
        loc.setPose(0, 0, 0);          // the team re-zeroes: the core must hear about it
        pin.step(0);
        loc.update();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                synchronized (core.starts) {
                    return core.starts.size() == 2;
                }
            }
        }, 2000));
        pin.epoch++;                   // Pinpoint rebooted: odometry world restarted
        pin.x = 0;
        pin.step(0);
        loc.update();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                synchronized (core.starts) {
                    return core.starts.size() == 3;
                }
            }
        }, 2000));
    }

    @Test
    public void fixesFromTheCoreReachTheLocalizerWithTheirSigma() throws Exception {
        HandSource pin = new HandSource();
        FakeCore core = new FakeCore();
        VisionFixSource v = source(new FakeFrames(), pin, cfg(true, true), core);
        BeaconLocalizer loc = new BeaconLocalizer(pin, BeaconConfig.parse("{}"), v, null);
        v.attachLocalizer(loc);
        loc.start(null);
        v.start();
        loc.setPose(0, 0, 0);
        for (int i = 0; i < 50; i++) {
            pin.step(0);
            loc.update();
        }
        core.giveFix(pin.t - 20000000L, 0.05, 0, 0, 0.02, 0.01, BeaconCore.FIX_EDGES);
        final VisionFixSource vv = v;
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                return vv.latestFix() != null;
            }
        }, 2000));
        FieldFix f = v.latestFix();
        assertEquals(0.02, f.sigmaXY, 0);
        assertEquals(0.01, f.sigmaHeading, 0);
        assertEquals("edges", f.kind);
        pin.step(0);
        loc.update();
        assertEquals(loc.getLastFixRejectReason(), 1, loc.getFixesApplied());
        assertEquals(0.05 * 0.5, loc.getPose().x, 1e-9);   // p = s = 0.02 m -> gain 0.5
    }

    @Test
    public void badFixesFromTheCoreAreDropped() throws Exception {
        HandSource pin = new HandSource();
        FakeCore core = new FakeCore();
        final VisionFixSource v = source(new FakeFrames(), pin, cfg(true, true), core);
        v.start();
        core.giveFix(5, Double.NaN, 0, 0, 0.01, 0.01, BeaconCore.FIX_EDGES);
        Thread.sleep(100);
        core.giveFix(6, 0, 0, 0, 0.01, 0.01, BeaconCore.FIX_EDGES | BeaconCore.FIX_NO_ODOM);
        Thread.sleep(100);
        assertNull(v.latestFix());
        assertTrue(v.getFixesDropped() >= 2);
    }

    @Test
    public void framesArePushedFromTheDecodeThreadAndMismatchesCounted() throws Exception {
        HandSource pin = new HandSource();
        FakeCore core = new FakeCore();
        FakeFrames frames = new FakeFrames();
        final VisionFixSource v = source(frames, pin, cfg(true, true), core);
        v.start();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                return v.isCoreRunning();
            }
        }, 2000));
        GrayFrame f = new GrayFrame();
        f.ensureSize(64, 48);
        f.tNs = 123;
        frames.listener.onFrame(f);
        f.ensureSize(32, 24);
        f.tNs = 124;
        frames.listener.onFrame(f);
        assertEquals(1, core.frames);
        assertEquals(1, v.getFramesRejected());
        assertTrue(v.debugLine(), v.debugLine().contains("32x24"));
        assertEquals(64, core.camera.width);
        assertEquals(0.02, core.camera.rollingShutterS, 0);
        assertEquals(0.01, core.camera.dist[0], 0);
        assertEquals("{\"schema\": 1}", core.fieldJson);
    }

    @Test
    public void missingCalibrationOrFieldDisablesVisionWithAReason() throws Exception {
        FakeCore core = new FakeCore();
        final VisionFixSource v = source(new FakeFrames(), new HandSource(), cfg(false, true),
                core);
        v.start();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                return v.debugLine().contains("camera");
            }
        }, 2000));
        assertFalse(v.isCoreRunning());

        FakeCore core2 = new FakeCore();
        final VisionFixSource v2 = source(new FakeFrames(), new HandSource(), cfg(true, false),
                core2);
        v2.start();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                return v2.debugLine().contains("field");
            }
        }, 2000));
    }

    @Test
    public void coreCreateFailureIsReportedNotThrown() throws Exception {
        FakeCore core = new FakeCore();
        core.createError = "field model: hive missing";
        final VisionFixSource v = source(new FakeFrames(), new HandSource(), cfg(true, true),
                core);
        v.start();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                return v.debugLine().contains("hive missing");
            }
        }, 2000));
        assertFalse(v.isCoreRunning());
        assertNull(v.latestFix());
    }

    @Test
    public void stopDestroysTheCoreOnceAndIsIdempotent() throws Exception {
        FakeCore core = new FakeCore();
        FakeFrames frames = new FakeFrames();
        final VisionFixSource v = source(frames, new HandSource(), cfg(true, true), core);
        v.stop();
        v.start();
        assertTrue(waitFor(new Cond() {
            public boolean ok() {
                return v.isCoreRunning();
            }
        }, 2000));
        v.stop();
        v.stop();
        assertTrue(core.destroyed);
        assertNull(frames.listener);
        GrayFrame f = new GrayFrame();
        f.ensureSize(64, 48);
        assertEquals(0, core.frames);
        assertTrue(v.debugLine(), v.debugLine().contains("stopped"));
    }
}
