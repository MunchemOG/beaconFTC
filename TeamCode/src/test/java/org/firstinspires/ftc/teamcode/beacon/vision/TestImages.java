package org.firstinspires.ftc.teamcode.beacon.vision;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.zip.CRC32;
import java.util.zip.DataFormatException;
import java.util.zip.Inflater;

/**
 * Image helpers for JVM tests. The Android unit-test classpath has no javax.imageio, so:
 * <ul>
 *   <li>"JPEG" parts in tests use a trivial container that starts with the JPEG SOI marker
 *       (FF D8) and ends with EOI (FF D9) around a width/height header and raw gray pixels.
 *       It exercises the MJPEG framing and decode-thread plumbing; real JPEG decoding is
 *       {@link AndroidJpegDecoder}'s job on the robot.</li>
 *   <li>{@link #decodePng} is an independent PNG reader (all five filter types, CRC checks)
 *       used to verify {@link PngGray}'s files.</li>
 * </ul>
 */
final class TestImages {
    private TestImages() {}

    /** A gray pattern whose top-left block holds {@code n}; never contains the byte 0xFF. */
    static byte[] grayPattern(int w, int h, int n) {
        byte[] g = new byte[w * h];
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                int v = ((x / 8 + y / 8) % 2 == 0) ? 40 : 200;
                if (x < 32 && y < 32) {
                    v = n % 250;
                }
                g[y * w + x] = (byte) v;
            }
        }
        return g;
    }

    /** FF D8, width (2 bytes), height (2 bytes), pixels, FF D9. */
    static byte[] fakeJpeg(byte[] gray, int w, int h) {
        byte[] b = new byte[6 + w * h + 2];
        b[0] = (byte) 0xFF;
        b[1] = (byte) 0xD8;
        b[2] = (byte) (w >> 8);
        b[3] = (byte) w;
        b[4] = (byte) (h >> 8);
        b[5] = (byte) h;
        System.arraycopy(gray, 0, b, 6, w * h);
        b[b.length - 2] = (byte) 0xFF;
        b[b.length - 1] = (byte) 0xD9;
        return b;
    }

    static byte[] fakeJpegPattern(int w, int h, int n) {
        return fakeJpeg(grayPattern(w, h, n), w, h);
    }

    /** Decoder for {@link #fakeJpeg}; records which thread called it. */
    static final class FakeJpegDecoder implements JpegDecoder {
        volatile long calls;
        volatile String lastThread = "";

        @Override
        public boolean decode(byte[] jpeg, int length, GrayFrame out) {
            calls++;
            lastThread = Thread.currentThread().getName();
            if (length < 8 || (jpeg[0] & 0xFF) != 0xFF || (jpeg[1] & 0xFF) != 0xD8) {
                return false;
            }
            int w = ((jpeg[2] & 0xFF) << 8) | (jpeg[3] & 0xFF);
            int h = ((jpeg[4] & 0xFF) << 8) | (jpeg[5] & 0xFF);
            if (w <= 0 || h <= 0 || length != 6 + w * h + 2) {
                return false;
            }
            out.ensureSize(w, h);
            System.arraycopy(jpeg, 6, out.gray, 0, w * h);
            return true;
        }
    }

    // ------------------------------------------------------------------- PNG

    private static int u32(byte[] b, int off) {
        return ((b[off] & 0xFF) << 24) | ((b[off + 1] & 0xFF) << 16) | ((b[off + 2] & 0xFF) << 8)
                | (b[off + 3] & 0xFF);
    }

    /** Decodes an 8-bit grayscale, non-interlaced PNG; dims gets {width, height}. */
    static byte[] decodePng(byte[] png, int[] dims) throws IOException {
        byte[] sig = {(byte) 0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
        for (int i = 0; i < 8; i++) {
            if (png[i] != sig[i]) {
                throw new IOException("bad PNG signature");
            }
        }
        int pos = 8;
        int w = -1, h = -1;
        boolean sawEnd = false;
        ByteArrayOutputStream idat = new ByteArrayOutputStream();
        while (pos < png.length) {
            int len = u32(png, pos);
            String type = new String(png, pos + 4, 4, "US-ASCII");
            CRC32 crc = new CRC32();
            crc.update(png, pos + 4, 4 + len);
            if ((int) crc.getValue() != u32(png, pos + 8 + len)) {
                throw new IOException("bad CRC in " + type);
            }
            if (type.equals("IHDR")) {
                w = u32(png, pos + 8);
                h = u32(png, pos + 12);
                if (png[pos + 16] != 8 || png[pos + 17] != 0 || png[pos + 20] != 0) {
                    throw new IOException("not 8-bit gray non-interlaced");
                }
            } else if (type.equals("IDAT")) {
                idat.write(png, pos + 8, len);
            } else if (type.equals("IEND")) {
                sawEnd = true;
            }
            pos += 12 + len;
        }
        if (!sawEnd || w <= 0) {
            throw new IOException("incomplete PNG");
        }
        Inflater inf = new Inflater();
        inf.setInput(idat.toByteArray());
        byte[] raw = new byte[(w + 1) * h];
        try {
            int got = 0;
            while (got < raw.length && !inf.finished()) {
                int n = inf.inflate(raw, got, raw.length - got);
                if (n == 0 && inf.needsInput()) {
                    break;
                }
                got += n;
            }
            if (got != raw.length) {
                throw new IOException("short image data: " + got + " of " + raw.length);
            }
        } catch (DataFormatException e) {
            throw new IOException(e);
        }
        byte[] out = new byte[w * h];
        for (int y = 0; y < h; y++) {
            int ft = raw[y * (w + 1)];
            for (int x = 0; x < w; x++) {
                int v = raw[y * (w + 1) + 1 + x] & 0xFF;
                int a = x > 0 ? out[y * w + x - 1] & 0xFF : 0;
                int b = y > 0 ? out[(y - 1) * w + x] & 0xFF : 0;
                int c = (x > 0 && y > 0) ? out[(y - 1) * w + x - 1] & 0xFF : 0;
                int p;
                switch (ft) {
                    case 0: p = 0; break;
                    case 1: p = a; break;
                    case 2: p = b; break;
                    case 3: p = (a + b) / 2; break;
                    case 4: {
                        int pa = Math.abs(b - c), pb = Math.abs(a - c), pc = Math.abs(a + b - 2 * c);
                        p = (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c);
                        break;
                    }
                    default: throw new IOException("bad filter type " + ft);
                }
                out[y * w + x] = (byte) (v + p);
            }
        }
        dims[0] = w;
        dims[1] = h;
        return out;
    }
}
