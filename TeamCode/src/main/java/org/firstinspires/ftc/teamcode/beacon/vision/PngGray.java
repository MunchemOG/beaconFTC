package org.firstinspires.ftc.teamcode.beacon.vision;

import java.io.IOException;
import java.io.OutputStream;
import java.util.zip.CRC32;
import java.util.zip.Deflater;

/**
 * Minimal PNG writer for 8-bit grayscale images (colour type 0), lossless. Uses only
 * java.util.zip, so it behaves the same on the Hub and on a laptop. One instance reuses its
 * buffers; not thread-safe (the recorder's writer thread owns one).
 *
 * <p>Each row uses the PNG "Sub" filter (difference to the pixel on the left), which
 * compresses camera images noticeably better than no filter for little cost; deflate runs at
 * the fastest level because Hub CPU time is scarce and SD space is not.
 */
public final class PngGray {
    private static final byte[] SIGNATURE = {(byte) 0x89, 'P', 'N', 'G', '\r', '\n', 0x1A, '\n'};
    private static final int CHUNK = 64 * 1024;

    private final Deflater deflater = new Deflater(Deflater.BEST_SPEED);
    private final CRC32 crc = new CRC32();
    private byte[] row = new byte[0];
    private final byte[] out = new byte[CHUNK];
    private final byte[] header = new byte[13];

    public void write(OutputStream os, byte[] gray, int width, int height) throws IOException {
        if (width <= 0 || height <= 0 || gray.length < width * height) {
            throw new IllegalArgumentException("bad image size " + width + "x" + height);
        }
        os.write(SIGNATURE);
        putInt(header, 0, width);
        putInt(header, 4, height);
        header[8] = 8;    // bit depth
        header[9] = 0;    // colour type: grayscale
        header[10] = 0;   // deflate
        header[11] = 0;   // adaptive filtering
        header[12] = 0;   // no interlace
        chunk(os, "IHDR", header, 13);

        if (row.length < width + 1) {
            row = new byte[width + 1];
        }
        deflater.reset();
        for (int y = 0; y < height; y++) {
            int base = y * width;
            row[0] = 1;   // filter type Sub
            row[1] = gray[base];
            for (int x = 1; x < width; x++) {
                row[x + 1] = (byte) (gray[base + x] - gray[base + x - 1]);
            }
            deflater.setInput(row, 0, width + 1);
            while (!deflater.needsInput()) {
                drain(os, false);
            }
        }
        deflater.finish();
        while (!deflater.finished()) {
            drain(os, true);
        }
        chunk(os, "IEND", out, 0);
    }

    private void drain(OutputStream os, boolean finishing) throws IOException {
        int n = deflater.deflate(out, 0, out.length);
        if (n > 0) {
            chunk(os, "IDAT", out, n);
        }
    }

    private void chunk(OutputStream os, String type, byte[] data, int len) throws IOException {
        byte[] t = {(byte) type.charAt(0), (byte) type.charAt(1), (byte) type.charAt(2),
                (byte) type.charAt(3)};
        byte[] lenBytes = new byte[4];
        putInt(lenBytes, 0, len);
        os.write(lenBytes);
        os.write(t);
        os.write(data, 0, len);
        crc.reset();
        crc.update(t);
        crc.update(data, 0, len);
        byte[] c = new byte[4];
        putInt(c, 0, (int) crc.getValue());
        os.write(c);
    }

    private static void putInt(byte[] b, int off, int v) {
        b[off] = (byte) (v >>> 24);
        b[off + 1] = (byte) (v >>> 16);
        b[off + 2] = (byte) (v >>> 8);
        b[off + 3] = (byte) v;
    }
}
