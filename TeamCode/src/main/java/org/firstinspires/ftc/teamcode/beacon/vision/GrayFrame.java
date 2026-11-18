package org.firstinspires.ftc.teamcode.beacon.vision;

/**
 * One 8-bit grayscale camera frame plus its timing. A reusable container: a consumer keeps
 * one instance and lets {@link FrameSource#latest(GrayFrame)} overwrite it, so steady-state
 * frame handling allocates nothing.
 *
 * <h3>Times (all {@code SystemClock.elapsedRealtimeNanos()}, the Hub's monotonic clock)</h3>
 * <ul>
 *   <li>{@link #arrivalNs}: when the first byte of this frame's MJPEG part (its boundary line)
 *       was returned by a socket read on the Hub. Measured.</li>
 *   <li>{@link #tNs}: the CAPTURE time, defined as the middle of the exposure of the image's
 *       middle row. Estimated as {@code arrivalNs - latencyMs}; {@link #latencySource} says
 *       where {@code latencyMs} came from. See {@code LimelightFrameSource} for the error
 *       budget.</li>
 * </ul>
 */
public final class GrayFrame {
    /** latencyMs was measured by the team (beacon.json {@code limelight.latencyMs}). */
    public static final int LATENCY_CONFIGURED = 1;
    /** latencyMs is the Limelight's own reported capture + pipeline latency (cl + tl). */
    public static final int LATENCY_LIMELIGHT = 2;
    /** latencyMs is {@code limelight.defaultLatencyMs}: an unmeasured guess. */
    public static final int LATENCY_DEFAULT = 3;

    public int width;
    public int height;
    /** Row-major pixels, stride = width, length = width * height (may be longer). */
    public byte[] gray = new byte[0];
    /** 1, 2, 3, ... per source; 0 = no frame yet. */
    public long seq;
    /** Estimated capture time (see class Javadoc). */
    public long tNs;
    /** First byte of this frame reached the Hub. */
    public long arrivalNs;
    /** Last byte of this frame reached the Hub. */
    public long lastByteNs;
    /** What was subtracted from arrivalNs to get tNs (ms). */
    public double latencyMs;
    /** One of the LATENCY_ constants; 0 if not set. */
    public int latencySource;
    /** Size of the JPEG this frame was decoded from (bytes). */
    public int jpegBytes;
    /** Time the JPEG decode took (ns, wall time on the decode thread). */
    public long decodeNs;

    /** Makes room for a w x h image (reallocates only when it grows). */
    public void ensureSize(int w, int h) {
        width = w;
        height = h;
        int n = w * h;
        if (gray.length < n) {
            gray = new byte[n];
        }
    }

    /** Copies everything (pixels and timing) from {@code o}. */
    public void copyFrom(GrayFrame o) {
        ensureSize(o.width, o.height);
        System.arraycopy(o.gray, 0, gray, 0, o.width * o.height);
        copyInfoFrom(o);
    }

    /** Copies the timing and size fields only (not the pixels). */
    public void copyInfoFrom(GrayFrame o) {
        width = o.width;
        height = o.height;
        seq = o.seq;
        tNs = o.tNs;
        arrivalNs = o.arrivalNs;
        lastByteNs = o.lastByteNs;
        latencyMs = o.latencyMs;
        latencySource = o.latencySource;
        jpegBytes = o.jpegBytes;
        decodeNs = o.decodeNs;
    }

    public static String latencySourceName(int s) {
        switch (s) {
            case LATENCY_CONFIGURED: return "measured (beacon.json)";
            case LATENCY_LIMELIGHT: return "Limelight cl+tl";
            case LATENCY_DEFAULT: return "UNMEASURED default";
            default: return "none";
        }
    }
}
