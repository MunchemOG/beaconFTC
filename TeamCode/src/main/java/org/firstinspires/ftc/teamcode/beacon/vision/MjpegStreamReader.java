package org.firstinspires.ftc.teamcode.beacon.vision;

import org.firstinspires.ftc.teamcode.beacon.PinpointPoseSource;

import java.io.IOException;
import java.io.InputStream;
import java.util.Locale;

/**
 * Splits an HTTP {@code multipart/x-mixed-replace} body (an MJPEG stream) into JPEG parts and
 * records when each part reached us. Pure Java; used on the frame source's reader thread.
 *
 * <p>Each part is {@code --boundary CRLF headers CRLF CRLF jpeg-bytes}. With a
 * {@code Content-Length} header exactly that many bytes are read; without one the body is read
 * up to the JPEG end-of-image marker (FF D9). A part that does not start with the JPEG
 * start-of-image marker (FF D8) is skipped and counted ({@link #badParts()}).
 *
 * <h3>Timing</h3>
 * {@link Part#firstByteNs} is the clock reading taken right after the socket read that
 * delivered the first byte of the part's boundary line; {@link Part#lastByteNs} the same for
 * the part's last byte. The reader thread does nothing but read and hand off, so it is almost
 * always blocked in {@code read()} when a frame starts to arrive, and the stamp is the arrival
 * time plus the thread's wake-up delay.
 *
 * <p>Limits keep a garbage stream from costing memory: a part larger than {@code maxPartBytes},
 * or more than 2 MiB without a boundary, ends the stream with an {@link IOException}.
 */
public final class MjpegStreamReader {

    /** One JPEG part. Reused: {@link #data} grows as needed and is never shrunk. */
    public static final class Part {
        public byte[] data = new byte[64 * 1024];
        public int length;
        public long firstByteNs;
        public long lastByteNs;
        /** Value of an "X-Timestamp"-style header if the server sends one, else null. */
        public String timestampHeader;
    }

    private static final int MAX_LINE = 1024;
    private static final int MAX_SKIP = 2 << 20;
    private static final int MAX_HEADERS = 32;

    private final InputStream in;
    private final String boundary;
    private final PinpointPoseSource.TimeSource time;
    private final int maxPartBytes;
    private final byte[] buf = new byte[16 * 1024];
    private int pos;
    private int lim;
    private long fillNs;
    private final StringBuilder line = new StringBuilder(128);
    private int lineBytes;
    private long lineStartNs;
    private long badParts;

    /**
     * @param boundary from {@link #boundaryFromContentType}; "" accepts any line starting
     *                 with "--"
     */
    public MjpegStreamReader(InputStream in, String boundary, PinpointPoseSource.TimeSource time,
                             int maxPartBytes) {
        this.in = in;
        this.boundary = stripDashes(boundary == null ? "" : boundary);
        this.time = time;
        this.maxPartBytes = maxPartBytes;
    }

    /**
     * The boundary named in a Content-Type header, without leading dashes or quotes; "" if the
     * type is multipart but names none; null if it is not multipart at all.
     */
    public static String boundaryFromContentType(String contentType) {
        if (contentType == null) {
            return null;
        }
        String lower = contentType.toLowerCase(Locale.US).trim();
        if (!lower.startsWith("multipart/")) {
            return null;
        }
        int i = lower.indexOf("boundary=");
        if (i < 0) {
            return "";
        }
        String b = contentType.trim().substring(i + "boundary=".length()).trim();
        int semi = b.indexOf(';');
        if (semi >= 0) {
            b = b.substring(0, semi).trim();
        }
        if (b.length() >= 2 && b.startsWith("\"") && b.endsWith("\"")) {
            b = b.substring(1, b.length() - 1);
        }
        return stripDashes(b);
    }

    private static String stripDashes(String s) {
        int i = 0;
        while (i < s.length() && s.charAt(i) == '-') {
            i++;
        }
        return s.substring(i);
    }

    /** Parts skipped because they were not JPEG images. */
    public long badParts() {
        return badParts;
    }

    /**
     * Reads the next JPEG part into {@code p}.
     * @return false at the end of the stream
     * @throws IOException on a read error, a read timeout, or a malformed stream
     */
    public boolean next(Part p) throws IOException {
        while (true) {
            // 1. Boundary line.
            int skipped = 0;
            long start;
            while (true) {
                if (!readLine()) {
                    return false;
                }
                if (isBoundary()) {
                    start = lineStartNs;
                    break;
                }
                skipped += lineBytes;
                if (skipped > MAX_SKIP) {
                    throw new IOException("no MJPEG boundary \"--" + boundary + "\" in "
                            + MAX_SKIP + " bytes");
                }
            }
            // 2. Part headers.
            int contentLength = -1;
            String ts = null;
            int headers = 0;
            while (true) {
                if (!readLine()) {
                    return false;
                }
                if (line.length() == 0) {
                    break;
                }
                if (++headers > MAX_HEADERS) {
                    throw new IOException("more than " + MAX_HEADERS + " headers in one part");
                }
                int colon = line.indexOf(":");
                if (colon <= 0) {
                    continue;
                }
                String name = line.substring(0, colon).trim().toLowerCase(Locale.US);
                String value = line.substring(colon + 1).trim();
                if (name.equals("content-length")) {
                    contentLength = parseLength(value);
                } else if (name.contains("timestamp")) {
                    ts = value;
                }
            }
            // 3. Body.
            boolean ok;
            if (contentLength >= 0) {
                if (contentLength > maxPartBytes) {
                    throw new IOException("MJPEG part of " + contentLength + " bytes is over the "
                            + "limit of " + maxPartBytes + " bytes");
                }
                if (!readBody(p, contentLength)) {
                    return false;
                }
                ok = p.length >= 2 && (p.data[0] & 0xFF) == 0xFF && (p.data[1] & 0xFF) == 0xD8;
            } else {
                int r = readUntilEndOfImage(p);
                if (r < 0) {
                    return false;
                }
                ok = r == 1;
            }
            p.firstByteNs = start;
            p.lastByteNs = fillNs;
            p.timestampHeader = ts;
            if (ok) {
                return true;
            }
            badParts++;
        }
    }

    private static int parseLength(String v) throws IOException {
        try {
            int n = Integer.parseInt(v);
            if (n < 0) {
                throw new IOException("negative Content-Length " + v);
            }
            return n;
        } catch (NumberFormatException e) {
            throw new IOException("bad Content-Length \"" + v + "\"");
        }
    }

    private boolean isBoundary() {
        if (line.length() < 2 || line.charAt(0) != '-' || line.charAt(1) != '-') {
            return false;
        }
        int i = 0;
        while (i < line.length() && line.charAt(i) == '-') {
            i++;
        }
        if (boundary.length() == 0) {
            return true;
        }
        return line.length() - i >= boundary.length()
                && line.substring(i, i + boundary.length()).equals(boundary);
    }

    private boolean fill() throws IOException {
        int n = in.read(buf, 0, buf.length);
        fillNs = time.nowNs();
        if (n <= 0) {
            pos = 0;
            lim = 0;
            return false;
        }
        pos = 0;
        lim = n;
        return true;
    }

    /** Reads one line (CR/LF stripped, kept to MAX_LINE chars). False at end of stream. */
    private boolean readLine() throws IOException {
        line.setLength(0);
        lineBytes = 0;
        if (pos >= lim && !fill()) {
            return false;
        }
        lineStartNs = fillNs;
        while (true) {
            if (pos >= lim && !fill()) {
                return lineBytes > 0;
            }
            byte b = buf[pos++];
            lineBytes++;
            if (b == '\n') {
                return true;
            }
            if (b != '\r' && line.length() < MAX_LINE) {
                line.append((char) (b & 0xFF));
            }
            if (lineBytes > MAX_SKIP) {
                return true;   // caller's skip limit fires
            }
        }
    }

    private void ensureCapacity(Part p, int n) {
        if (p.data.length < n) {
            int size = Math.max(n, Math.min(maxPartBytes, p.data.length * 2));
            byte[] d = new byte[size];
            System.arraycopy(p.data, 0, d, 0, p.length);
            p.data = d;
        }
    }

    /** Exactly n body bytes into p. False if the stream ended first. */
    private boolean readBody(Part p, int n) throws IOException {
        p.length = 0;
        ensureCapacity(p, n);
        int have = Math.min(n, lim - pos);
        System.arraycopy(buf, pos, p.data, 0, have);
        pos += have;
        p.length = have;
        while (p.length < n) {
            int r = in.read(p.data, p.length, n - p.length);
            fillNs = time.nowNs();
            if (r <= 0) {
                return false;
            }
            p.length += r;
        }
        return true;
    }

    /**
     * Body without a Content-Length: up to and including FF D9.
     * @return 1 = JPEG read, 0 = not a JPEG (skipped), -1 = end of stream
     */
    private int readUntilEndOfImage(Part p) throws IOException {
        p.length = 0;
        int prev = -1;
        while (true) {
            if (pos >= lim && !fill()) {
                return -1;
            }
            int b = buf[pos++] & 0xFF;
            if (p.length == 0 && b != 0xFF) {
                return 0;
            }
            if (p.length == 1 && b != 0xD8) {
                return 0;
            }
            if (p.length >= maxPartBytes) {
                throw new IOException("MJPEG part without Content-Length is over the limit of "
                        + maxPartBytes + " bytes");
            }
            ensureCapacity(p, p.length + 1);
            p.data[p.length++] = (byte) b;
            if (prev == 0xFF && b == 0xD9 && p.length > 2) {
                return 1;
            }
            prev = b;
        }
    }
}
