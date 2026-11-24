package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.io.File;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.List;
import org.junit.Test;

public class PoseLogWriterTest {

    private static File tmp() throws Exception {
        File f = File.createTempFile("beacon_log", ".csv");
        f.deleteOnExit();
        return f;
    }

    @Test
    public void writesHeaderAndRowsInOrder() throws Exception {
        File f = tmp();
        PoseLogWriter w = new PoseLogWriter(f, 64);
        assertTrue(w.start());
        for (int i = 0; i < 1000; i++) {
            while (!w.offer(1000 + i, i * 0.5, -i, 0.25, 0, 0, 0.6, 0.8, 2)) {
                Thread.yield();   // test only: a 64-row ring is smaller than the burst
            }
        }
        w.close();
        List<String> lines = Files.readAllLines(f.toPath(), Charset.forName("UTF-8"));
        assertEquals("t_ns,x,y,z,qx,qy,qz,qw,status", lines.get(0));
        assertEquals(1001, lines.size());
        String[] r = lines.get(500).split(",");
        assertEquals(9, r.length);
        assertEquals("1499", r[0]);
        assertEquals(249.5, Double.parseDouble(r[1]), 0);
        assertEquals(-499, Double.parseDouble(r[2]), 0);
        assertEquals(0.8, Double.parseDouble(r[7]), 0);
        assertEquals("2", r[8]);
        assertEquals(1000, w.getWrittenCount());
        assertNull(w.getError());
    }

    @Test
    public void fullRingDropsRowsAndCountsThemWithoutBlocking() throws Exception {
        File f = tmp();
        PoseLogWriter w = new PoseLogWriter(f, 16);
        assertTrue(w.start());
        int offered = 200000;
        int accepted = 0;
        long t0 = System.nanoTime();
        for (int i = 0; i < offered; i++) {
            if (w.offer(i + 1, 0, 0, 0, 0, 0, 0, 1, 2)) {
                accepted++;
            }
        }
        long ms = (System.nanoTime() - t0) / 1000000;
        w.close();
        assertTrue("offer burst took " + ms + " ms", ms < 2000);
        assertEquals(offered, accepted + w.getDroppedCount());
        assertTrue("a 16-row ring must overflow in a tight burst", w.getDroppedCount() > 0);
        assertEquals(accepted, w.getWrittenCount());
        List<String> lines = Files.readAllLines(f.toPath(), Charset.forName("UTF-8"));
        assertEquals(accepted + 1, lines.size());
        // surviving rows are still in increasing time order (oldest kept, newest dropped)
        long prev = 0;
        for (int i = 1; i < lines.size(); i++) {
            long t = Long.parseLong(lines.get(i).split(",")[0]);
            assertTrue(t > prev);
            prev = t;
        }
    }

    @Test
    public void unwritablePathFailsStartGracefully() throws Exception {
        File dir = File.createTempFile("beacon_dir", "");
        assertTrue(dir.delete());
        assertTrue(dir.mkdir());
        dir.deleteOnExit();
        PoseLogWriter w = new PoseLogWriter(dir, 64);    // a directory cannot be opened as a file
        assertFalse(w.start());
        assertNotNull(w.getError());
        assertFalse(w.offer(1, 0, 0, 0, 0, 0, 0, 1, 2));
        w.close();   // safe
    }

    @Test
    public void createsMissingDirectoryAndCloseIsIdempotent() throws Exception {
        File dir = new File(System.getProperty("java.io.tmpdir"), "beacon_t_" + System.nanoTime());
        File f = new File(new File(dir, "sub"), "a.csv");
        PoseLogWriter w = new PoseLogWriter(f);
        assertTrue(w.start());
        assertTrue(w.offer(5, 1, 2, 3, 0, 0, 0, 1, 1));
        w.close();
        w.close();
        assertTrue(f.isFile());
        assertEquals(2, Files.readAllLines(f.toPath(), Charset.forName("UTF-8")).size());
        f.delete();
        new File(dir, "sub").delete();
        dir.delete();
    }

    @Test
    public void forConfigIsNullWhenLoggingDisabled() throws Exception {
        assertNull(PoseLogWriter.forConfig(BeaconConfig.defaults(), "x"));
        BeaconConfig c = BeaconConfig.parse("{\"log\":{\"enabled\":true,\"dir\":\"/tmp/zz\"}}");
        PoseLogWriter w = PoseLogWriter.forConfig(c, "loc");
        assertNotNull(w);
        assertTrue(w.getFile().getName().startsWith("loc_"));
        assertEquals("/tmp/zz", w.getFile().getParent());
    }

    @Test
    public void restartAppendsInsteadOfTruncatingAndWritesTheHeaderOnce() throws Exception {
        // review finding 11: start() after close() used to truncate the file
        File f = tmp();
        assertTrue(f.delete());
        PoseLogWriter w = new PoseLogWriter(f, 64);
        assertTrue(w.start());
        assertTrue(w.offer(1, 1, 0, 0, 0, 0, 0, 1, 2));
        w.close();
        assertTrue("restart allowed once the old thread has exited", w.start());
        assertTrue(w.offer(2, 2, 0, 0, 0, 0, 0, 1, 2));
        w.close();
        List<String> lines = Files.readAllLines(f.toPath(), Charset.forName("UTF-8"));
        assertEquals(PoseLogWriter.HEADER, lines.get(0));
        assertEquals(3, lines.size());
        assertTrue(lines.get(1).startsWith("1,"));
        assertTrue(lines.get(2).startsWith("2,"));
    }
}
