package org.firstinspires.ftc.teamcode.beacon.vision;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

/**
 * JPEG to 8-bit gray with Android's {@link BitmapFactory} (always present on the Control Hub;
 * no extra native library). The bitmap and the pixel buffer are reused between frames, so a
 * steady stream of same-sized frames allocates nothing here.
 *
 * <p>Gray = (77 R + 150 G + 29 B + 128) / 256, i.e. BT.601 luma. If the Limelight streams
 * a colour image this throws away the chroma; for a grayscale stream R = G = B and the value
 * is the original sample.
 *
 * <p>Cost (EXPECTED, not measured; the probe OpMode shows the real number): decoding a
 * 640x480 JPEG on one Cortex-A53 core at 1.5 GHz is roughly 5-10 ms, plus about 2 ms for the
 * gray conversion. Only the decode thread calls this.
 */
public final class AndroidJpegDecoder implements JpegDecoder {
    private final BitmapFactory.Options opts = new BitmapFactory.Options();
    private Bitmap reuse;
    private int[] argb = new int[0];

    public AndroidJpegDecoder() {
        opts.inPreferredConfig = Bitmap.Config.ARGB_8888;
        opts.inMutable = true;     // required for inBitmap reuse
        opts.inSampleSize = 1;
    }

    @Override
    public boolean decode(byte[] jpeg, int length, GrayFrame out) {
        Bitmap b = null;
        opts.inBitmap = reuse;
        try {
            b = BitmapFactory.decodeByteArray(jpeg, 0, length, opts);
        } catch (IllegalArgumentException e) {
            // The reusable bitmap does not fit this frame (size changed): decode fresh.
            opts.inBitmap = null;
            reuse = null;
            b = BitmapFactory.decodeByteArray(jpeg, 0, length, opts);
        }
        if (b == null) {
            return false;
        }
        reuse = b;
        int w = b.getWidth();
        int h = b.getHeight();
        int n = w * h;
        if (argb.length < n) {
            argb = new int[n];
        }
        b.getPixels(argb, 0, w, 0, 0, w, h);
        out.ensureSize(w, h);
        byte[] g = out.gray;
        for (int i = 0; i < n; i++) {
            int c = argb[i];
            int r = (c >> 16) & 0xFF;
            int gg = (c >> 8) & 0xFF;
            int bb = c & 0xFF;
            g[i] = (byte) ((77 * r + 150 * gg + 29 * bb + 128) >> 8);
        }
        return true;
    }
}
