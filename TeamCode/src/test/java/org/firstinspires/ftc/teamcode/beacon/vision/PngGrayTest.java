package org.firstinspires.ftc.teamcode.beacon.vision;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;

import org.junit.Test;

import java.io.ByteArrayOutputStream;

public class PngGrayTest {

    @Test
    public void roundTripsExactlyThroughAStandardDecoder() throws Exception {
        int w = 157, h = 93;   // odd sizes on purpose
        byte[] g = new byte[w * h];
        java.util.Random rnd = new java.util.Random(7);
        for (int i = 0; i < g.length; i++) {
            g[i] = (byte) (i % 7 == 0 ? rnd.nextInt(256) : (i * 31) & 0xFF);
        }
        PngGray png = new PngGray();
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        png.write(out, g, w, h);
        png.write(new ByteArrayOutputStream(), g, w, h);   // reuse must not corrupt state
        out.reset();
        png.write(out, g, w, h);
        int[] dims = new int[2];
        byte[] back = TestImages.decodePng(out.toByteArray(), dims);
        assertEquals(w, dims[0]);
        assertEquals(h, dims[1]);
        assertArrayEquals(g, back);
    }
}
