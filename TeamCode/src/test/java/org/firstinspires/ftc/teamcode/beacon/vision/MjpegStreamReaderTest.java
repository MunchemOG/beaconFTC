package org.firstinspires.ftc.teamcode.beacon.vision;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import org.firstinspires.ftc.teamcode.beacon.PinpointPoseSource;
import org.junit.Test;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.Charset;

public class MjpegStreamReaderTest {
    private static final Charset ASCII = Charset.forName("US-ASCII");

    /** Clock that advances 1 ms every time it is read, so stamps are distinguishable. */
    static final class TickClock implements PinpointPoseSource.TimeSource {
        long t = 1000000000L;

        @Override
        public long nowNs() {
            t += 1000000L;
            return t;
        }
    }

    private static byte[] fakeJpeg(int n, int len) {
        byte[] b = new byte[len];
        b[0] = (byte) 0xFF;
        b[1] = (byte) 0xD8;
        for (int i = 2; i < len - 2; i++) {
            b[i] = (byte) (n + i);
            if (b[i] == (byte) 0xFF) {
                b[i] = 0x11;   // keep the body free of accidental markers
            }
        }
        b[len - 2] = (byte) 0xFF;
        b[len - 1] = (byte) 0xD9;
        return b;
    }

    private static byte[] stream(String boundary, boolean withLength, byte[]... parts)
            throws IOException {
        ByteArrayOutputStream out = new ByteArrayOutputStream();
        for (byte[] p : parts) {
            StringBuilder h = new StringBuilder();
            h.append("--").append(boundary).append("\r\nContent-Type: image/jpeg\r\n");
            if (withLength) {
                h.append("Content-Length: ").append(p.length).append("\r\n");
            }
            h.append("X-Timestamp: 12.5\r\n\r\n");
            out.write(h.toString().getBytes(ASCII));
            out.write(p);
            out.write("\r\n".getBytes(ASCII));
        }
        return out.toByteArray();
    }

    @Test
    public void boundaryParsedFromContentType() {
        assertEquals("abc", MjpegStreamReader.boundaryFromContentType(
                "multipart/x-mixed-replace; boundary=abc"));
        assertEquals("abc", MjpegStreamReader.boundaryFromContentType(
                "multipart/x-mixed-replace;boundary=--abc"));
        assertEquals("a b", MjpegStreamReader.boundaryFromContentType(
                "Multipart/X-Mixed-Replace; charset=x; boundary=\"a b\""));
        assertEquals("", MjpegStreamReader.boundaryFromContentType("multipart/x-mixed-replace"));
        assertNull(MjpegStreamReader.boundaryFromContentType("text/html"));
        assertNull(MjpegStreamReader.boundaryFromContentType(null));
    }

    @Test
    public void readsPartsWithContentLength() throws Exception {
        byte[] a = fakeJpeg(1, 500);
        byte[] b = fakeJpeg(2, 70000);   // bigger than the initial part buffer
        MjpegStreamReader r = new MjpegStreamReader(
                new ByteArrayInputStream(stream("bnd", true, a, b)), "bnd", new TickClock(),
                1 << 20);
        MjpegStreamReader.Part p = new MjpegStreamReader.Part();
        assertTrue(r.next(p));
        assertEquals(500, p.length);
        assertArrayEquals(a, java.util.Arrays.copyOf(p.data, p.length));
        assertEquals("12.5", p.timestampHeader);
        assertTrue(p.firstByteNs > 0 && p.lastByteNs >= p.firstByteNs);
        assertTrue(r.next(p));
        assertEquals(70000, p.length);
        assertArrayEquals(b, java.util.Arrays.copyOf(p.data, p.length));
        assertFalse(r.next(p));
    }

    @Test
    public void readsPartsWithoutContentLengthByScanningForEndOfImage() throws Exception {
        byte[] a = fakeJpeg(3, 1234);
        byte[] b = fakeJpeg(4, 999);
        MjpegStreamReader r = new MjpegStreamReader(
                new ByteArrayInputStream(stream("x", false, a, b)), "x", new TickClock(),
                1 << 20);
        MjpegStreamReader.Part p = new MjpegStreamReader.Part();
        assertTrue(r.next(p));
        assertArrayEquals(a, java.util.Arrays.copyOf(p.data, p.length));
        assertTrue(r.next(p));
        assertArrayEquals(b, java.util.Arrays.copyOf(p.data, p.length));
        assertFalse(r.next(p));
    }

    @Test
    public void skipsAPartThatIsNotAJpegAndCountsIt() throws Exception {
        byte[] bad = "hello world, not a jpeg".getBytes(ASCII);
        byte[] good = fakeJpeg(5, 300);
        MjpegStreamReader r = new MjpegStreamReader(
                new ByteArrayInputStream(stream("x", true, bad, good)), "x", new TickClock(),
                1 << 20);
        MjpegStreamReader.Part p = new MjpegStreamReader.Part();
        assertTrue(r.next(p));
        assertArrayEquals(good, java.util.Arrays.copyOf(p.data, p.length));
        assertEquals(1, r.badParts());
    }

    @Test
    public void oversizedPartIsAFormatErrorNotAnOutOfMemory() throws Exception {
        byte[] big = fakeJpeg(6, 5000);
        MjpegStreamReader r = new MjpegStreamReader(
                new ByteArrayInputStream(stream("x", true, big)), "x", new TickClock(), 4096);
        try {
            r.next(new MjpegStreamReader.Part());
            fail("expected an IOException for a part over the size limit");
        } catch (IOException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("4096"));
        }
    }

    @Test
    public void garbageWithoutBoundaryEndsInAnErrorAfterALimit() throws Exception {
        byte[] junk = new byte[3 << 20];
        java.util.Arrays.fill(junk, (byte) 'a');
        MjpegStreamReader r = new MjpegStreamReader(new ByteArrayInputStream(junk), "x",
                new TickClock(), 1 << 20);
        try {
            r.next(new MjpegStreamReader.Part());
            fail("expected an IOException");
        } catch (IOException e) {
            assertTrue(e.getMessage(), e.getMessage().contains("boundary"));
        }
    }

    @Test
    public void firstByteStampIsTheReadThatDeliveredTheBoundary() throws Exception {
        // Each read() delivers one part; the clock ticks once per fill, so the second part's
        // stamp must be later than the first part's last byte.
        final byte[] all = stream("x", true, fakeJpeg(1, 100), fakeJpeg(2, 100));
        final int half = all.length / 2;
        InputStream in = new InputStream() {
            int pos;

            @Override
            public int read() {
                return pos < all.length ? all[pos++] & 0xFF : -1;
            }

            @Override
            public int read(byte[] b, int off, int len) {
                if (pos >= all.length) {
                    return -1;
                }
                int end = pos < half ? half : all.length;
                int n = Math.min(len, end - pos);
                System.arraycopy(all, pos, b, off, n);
                pos += n;
                return n;
            }
        };
        MjpegStreamReader r = new MjpegStreamReader(in, "x", new TickClock(), 1 << 20);
        MjpegStreamReader.Part p = new MjpegStreamReader.Part();
        assertTrue(r.next(p));
        long firstLast = p.lastByteNs;
        assertTrue(r.next(p));
        assertTrue(p.firstByteNs > firstLast);
    }
}
