package org.firstinspires.ftc.teamcode.beacon.vision;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.MiniJson;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.List;
import java.util.Map;

public class DatasetRecorderTest {
    private static final Charset UTF8 = Charset.forName("UTF-8");

    @Rule
    public TemporaryFolder tmp = new TemporaryFolder();

    private static final BeaconConfig.CameraMount MOUNT =
            new BeaconConfig.CameraMount(0.1, 0.0, 0.3, 0.0, 0.1, 0.0);

    private static GrayFrame frame(int w, int h, int n, long tNs) {
        GrayFrame f = new GrayFrame();
        f.ensureSize(w, h);
        byte[] g = TestImages.grayPattern(w, h, n);
        System.arraycopy(g, 0, f.gray, 0, g.length);
        f.seq = n + 1;
        f.tNs = tNs;
        f.arrivalNs = tNs + 20000000L;
        f.lastByteNs = f.arrivalNs + 1000000L;
        f.latencyMs = 20;
        f.latencySource = GrayFrame.LATENCY_CONFIGURED;
        f.jpegBytes = 1234;
        f.decodeNs = 3000000L;
        return f;
    }

    private static List<String> lines(File f) throws Exception {
        return Files.readAllLines(f.toPath(), UTF8);
    }

    private static void assertIncreasing(List<String> rows, String what) {
        long prev = Long.MIN_VALUE;
        for (int i = 1; i < rows.size(); i++) {
            String[] c = rows.get(i).split(",");
            long t = Long.parseLong(what.equals("frames") ? c[1] : c[0]);
            assertTrue(what + " t_ns not strictly increasing at row " + i, t > prev);
            prev = t;
        }
    }

    @Test
    @SuppressWarnings("unchecked")
    public void recorderWritesDatasetLayout() throws Exception {
        File field = tmp.newFile("field.json");
        Files.write(field.toPath(), "{\"schema\": 1}".getBytes(UTF8));
        File run = new File(tmp.getRoot(), "runs/20261117_120000");
        DatasetRecorder r = new DatasetRecorder(run, 8, 4096, 0, MOUNT, field);
        assertTrue(r.getError(), r.start());

        r.writeStart(5000, 0.5, -1.0, 1.57);
        long t = 1000000000L;
        for (int i = 0; i < 20; i++) {
            while (i - r.getFramesWritten() >= 8) {
                Thread.sleep(1);   // test only: wait for a free slot so nothing is dropped
            }
            assertTrue(r.offerFrame(frame(64, 48, i, t + i * 33000000L)));
        }
        for (int i = 0; i < 500; i++) {
            assertTrue(r.offerOdom(t + i * 2000000L, i * 0.001, -i * 0.002, 0.001 * i));
        }
        r.addMark(t + 999, 1.2, 0.6, 0.0);
        r.addMark(t + 1999, -1.2, 0.6, 3.14);
        r.writeText("probe_summary.txt", "hello");
        r.close();
        assertNull(r.getError());
        assertEquals(20, r.getFramesWritten());
        assertEquals(0, r.getFramesDropped());
        assertEquals(500, r.getOdomWritten());

        // camera.json (I2): size from the frames, mount from the config, nominal intrinsics.
        Map<String, Object> cam = (Map<String, Object>) MiniJson.parse(
                new String(Files.readAllBytes(new File(run, "camera.json").toPath()), UTF8));
        assertEquals(64.0, (Double) cam.get("width"), 0);
        assertEquals(48.0, (Double) cam.get("height"), 0);
        assertEquals("exactly the I2 keys", new java.util.HashSet<String>(java.util.Arrays.asList(
                "width", "height", "fx", "fy", "cx", "cy", "dist", "mount",
                "rollingShutterReadoutS")), cam.keySet());
        // Same nominal convention as tools/sim/camera.py Camera.from_hfov (54.5 deg HFOV).
        double fx = 32.0 / Math.tan(Math.toRadians(54.5 / 2));
        assertEquals(fx, (Double) cam.get("fx"), 1e-9);
        assertEquals(fx, (Double) cam.get("fy"), 1e-9);
        assertEquals(31.5, (Double) cam.get("cx"), 0);
        assertEquals(23.5, (Double) cam.get("cy"), 0);
        assertTrue(new File(run, "camera_notes.txt").isFile());
        Map<String, Object> mount = (Map<String, Object>) cam.get("mount");
        assertEquals(0.3, (Double) mount.get("z"), 0);
        assertEquals(0.1, (Double) mount.get("pitch"), 0);
        assertEquals(5, ((List<Object>) cam.get("dist")).size());

        assertEquals("{\"schema\": 1}",
                new String(Files.readAllBytes(new File(run, "field.json").toPath()), UTF8));

        List<String> frames = lines(new File(run, "frames.csv"));
        assertEquals("idx,t_ns", frames.get(0));
        assertEquals(21, frames.size());
        assertEquals("0," + t, frames.get(1));
        assertIncreasing(frames, "frames");
        for (int i = 0; i < 20; i++) {
            File png = new File(run, String.format("frames/%06d.png", i));
            assertTrue(png.getPath(), png.isFile());
            int[] dims = new int[2];
            byte[] g = TestImages.decodePng(Files.readAllBytes(png.toPath()), dims);
            assertEquals(64, dims[0]);
            assertArrayEquals("frame " + i, TestImages.grayPattern(64, 48, i), g);
        }

        List<String> odom = lines(new File(run, "odom.csv"));
        assertEquals("t_ns,x,y,heading", odom.get(0));
        assertEquals(501, odom.size());
        assertIncreasing(odom, "odom");

        List<String> start = lines(new File(run, "start.csv"));
        assertEquals("t_ns,x,y,heading", start.get(0));
        assertEquals("5000,0.5,-1.0,1.57", start.get(1));

        List<String> marks = lines(new File(run, "marks.csv"));
        assertEquals("t_ns,x,y,heading", marks.get(0));
        assertEquals(3, marks.size());
        assertIncreasing(marks, "marks");

        List<String> meta = lines(new File(run, "frames_meta.csv"));
        assertTrue(meta.get(0).startsWith("idx,t_ns,arrival_ns"));
        assertEquals(21, meta.size());
        assertEquals("hello", new String(Files.readAllBytes(
                new File(run, "probe_summary.txt").toPath()), UTF8));
        assertFalse(new File(run, "truth.csv").exists());   // recorded runs have no truth
    }

    @Test
    public void recorderNeverBlocksLoop() throws Exception {
        File run = new File(tmp.getRoot(), "busy");
        DatasetRecorder r = new DatasetRecorder(run, 2, 64, 0, MOUNT, null);
        assertTrue(r.start());
        GrayFrame f = frame(640, 480, 1, 1);
        int accepted = 0;
        long worst = 0;
        for (int i = 0; i < 200; i++) {
            f.tNs = 1000 + i * 1000000L;
            long t0 = System.nanoTime();
            if (r.offerFrame(f)) {
                accepted++;
            }
            worst = Math.max(worst, System.nanoTime() - t0);
        }
        int odomAccepted = 0;
        for (int i = 0; i < 100000; i++) {
            long t0 = System.nanoTime();
            if (r.offerOdom(1 + i, 0, 0, 0)) {
                odomAccepted++;
            }
            worst = Math.max(worst, System.nanoTime() - t0);
        }
        assertTrue("an offer took " + worst / 1e6 + " ms", worst < 20000000L);
        assertTrue("full queue must drop", accepted < 200);
        assertEquals(200 - accepted, r.getFramesDropped());
        assertEquals(100000 - odomAccepted, r.getOdomDropped());
        r.close();
        assertEquals(accepted, r.getFramesWritten());
        assertEquals(odomAccepted, r.getOdomWritten());
        assertEquals(accepted + 1, lines(new File(run, "frames.csv")).size());
    }

    @Test
    public void framesWithNonIncreasingTimeOrOverTheRateAreNotRecorded() throws Exception {
        File run = new File(tmp.getRoot(), "rate");
        DatasetRecorder r = new DatasetRecorder(run, 64, 64, 10, MOUNT, null);  // 10 Hz cap
        assertTrue(r.start());
        long t = 1000000000L;
        for (int i = 0; i < 50; i++) {   // 100 Hz input for 0.5 s
            r.offerFrame(frame(32, 24, i, t + i * 10000000L));
        }
        assertFalse(r.offerFrame(frame(32, 24, 99, t)));    // earlier than the last accepted
        r.close();
        assertEquals(5, r.getFramesWritten());
        assertEquals(0, r.getFramesDropped());
        assertEquals(1, r.getFramesOutOfOrder());
        assertEquals(45, r.getFramesSkippedForRate());
    }

    @Test
    public void unwritableDirectoryFailsCleanly() throws Exception {
        File notADir = tmp.newFile("plainfile");
        DatasetRecorder r = new DatasetRecorder(new File(notADir, "run"), 4, 64, 0, MOUNT, null);
        assertFalse(r.start());
        assertNotNull(r.getError());
        assertFalse(r.offerFrame(frame(8, 8, 0, 1)));
        assertFalse(r.offerOdom(1, 0, 0, 0));
        r.addMark(1, 0, 0, 0);
        r.close();
        r.close();
    }
}
