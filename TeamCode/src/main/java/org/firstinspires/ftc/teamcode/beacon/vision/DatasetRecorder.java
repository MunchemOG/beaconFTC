package org.firstinspires.ftc.teamcode.beacon.vision;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;

import java.io.BufferedOutputStream;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.LockSupport;

/**
 * Records a run in the plan's dataset format (interface I2) on its own writer thread:
 * <pre>
 *   camera.json        width, height, UNCALIBRATED intrinsics from the datasheet HFOV
 *                      (see camera_notes.txt), mount = beacon.json T_robot_cam
 *   field.json         copy of the season field model (if the file exists on the Hub)
 *   frames.csv         idx,t_ns           (estimated capture time) + frames/&lt;idx:06d&gt;.png
 *   frames_meta.csv    how each t_ns was obtained (arrival, latency used, JPEG size, ...)
 *   odom.csv           t_ns,x,y,heading   (raw Pinpoint world, robot centre)
 *   start.csv          t_ns,x,y,heading   (start pose the team typed, field frame)
 *   marks.csv          t_ns,x,y,heading   (tape-mark checks, field frame)
 * </pre>
 *
 * <h3>Threads and back-pressure</h3>
 * {@link #offerFrame} (one producer thread: the frame source's decode thread) copies the
 * frame into a free preallocated slot and returns; if every slot is still waiting for the
 * writer, the frame is DROPPED and counted ({@link #getFramesDropped()}). Frames arriving faster
 * than {@code maxHz} are skipped by design and counted separately. {@link #offerOdom} (one
 * producer: the OpMode thread) puts a row in a preallocated ring, dropping and counting when
 * full. Neither ever blocks, allocates in steady state, or touches a file. The writer thread
 * encodes PNGs and writes everything; {@link #close()} drains what is queued (bounded wait).
 * Rows and frames whose time does not increase are not recorded (counted), so every
 * {@code t_ns} column is strictly increasing.
 */
public final class DatasetRecorder {
    /** Limelight 3A horizontal field of view from its datasheet (deg); nominal intrinsics only. */
    public static final double NOMINAL_HFOV_DEG = 54.5;
    private static final long POLL_NS = 3000000L;
    private static final long FLUSH_NS = 1000000000L;
    private static final int EV_START = 1, EV_MARK = 2, EV_TEXT = 3;

    private final File dir;
    private final GrayFrame[] slots;
    private final long minPeriodNs;
    private final BeaconConfig.CameraMount mount;
    private final File fieldFile;

    private final int odomCap;
    private final long[] oT;
    private final double[] oX, oY, oH;

    private volatile long fHead, fTail, oHead, oTail;
    private volatile boolean running;
    private volatile boolean failed;
    private volatile String error;
    private Thread thread;

    // producer-only state
    private long lastFrameTNs = Long.MIN_VALUE;
    private long lastOdomTNs = Long.MIN_VALUE;

    private final List<Object[]> events = new ArrayList<Object[]>();

    private final AtomicLong framesDropped = new AtomicLong();
    private volatile long framesWritten, framesSkippedRate, framesOutOfOrder;
    private final AtomicLong odomDropped = new AtomicLong();
    private volatile long odomWritten, odomOutOfOrder, marksWritten;
    private volatile long lastPngMs;
    private volatile int width, height;

    /**
     * @param runDir       directory to create (must not be a file)
     * @param frameSlots   frames that may wait for the writer (at least 1)
     * @param odomCapacity odometry rows that may wait for the writer (at least 16)
     * @param maxFrameHz   most frames per second recorded; 0 = no limit
     * @param mount        T_robot_cam for camera.json
     * @param fieldFile    copied to field.json if it exists; may be null
     */
    public DatasetRecorder(File runDir, int frameSlots, int odomCapacity, double maxFrameHz,
                           BeaconConfig.CameraMount mount, File fieldFile) {
        this.dir = runDir;
        this.slots = new GrayFrame[Math.max(1, frameSlots)];
        for (int i = 0; i < slots.length; i++) {
            slots[i] = new GrayFrame();
        }
        this.minPeriodNs = maxFrameHz > 0 ? Math.round(1e9 / maxFrameHz) : 0;
        this.mount = mount;
        this.fieldFile = fieldFile;
        this.odomCap = Math.max(16, odomCapacity);
        this.oT = new long[odomCap];
        this.oX = new double[odomCap];
        this.oY = new double[odomCap];
        this.oH = new double[odomCap];
    }

    public File getDir() {
        return dir;
    }

    // ------------------------------------------------------------- lifecycle

    /**
     * Creates the run directory and files and starts the writer thread. Call from INIT or on a
     * button press, not every loop. False (with {@link #getError()}) if the files cannot be
     * created; the recorder is then inert.
     */
    public synchronized boolean start() {
        if (running) {
            return true;
        }
        final Files files = new Files();
        try {
            File frames = new File(dir, "frames");
            if (!frames.isDirectory() && !frames.mkdirs()) {
                throw new IOException("cannot create directory " + frames.getPath());
            }
            files.frames = open("frames.csv", "idx,t_ns");
            files.meta = open("frames_meta.csv", "idx,t_ns,arrival_ns,last_byte_ns,latency_ms,"
                    + "latency_source,jpeg_bytes,decode_ms,width,height");
            files.odom = open("odom.csv", "t_ns,x,y,heading");
            files.start = open("start.csv", "t_ns,x,y,heading");
            files.marks = open("marks.csv", "t_ns,x,y,heading");
            if (fieldFile != null && fieldFile.isFile()) {
                copy(fieldFile, new File(dir, "field.json"));
            }
            writeCameraJson();
        } catch (IOException e) {
            error = "cannot start recording in " + dir.getPath() + ": " + e.getMessage();
            files.closeAll();
            return false;
        }
        error = null;
        failed = false;
        running = true;
        thread = new Thread(new Runnable() {
            @Override
            public void run() {
                writerLoop(files);
            }
        }, "BeaconRecorder");
        thread.setDaemon(true);
        thread.start();
        return true;
    }

    /** Writes everything queued, closes the files and stops the thread (waits up to 5 s). */
    public void close() {
        Thread t;
        synchronized (this) {
            if (!running) {
                return;
            }
            running = false;
            t = thread;
        }
        if (t != null) {
            LockSupport.unpark(t);
            try {
                t.join(5000);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    public boolean isRunning() {
        return running;
    }

    // ------------------------------------------------------------- producers

    /** Queues a copy of {@code f}. One producer thread only. Never blocks. */
    public boolean offerFrame(GrayFrame f) {
        if (!running || failed || f == null || f.width <= 0 || f.height <= 0) {
            return false;
        }
        if (f.tNs <= lastFrameTNs) {
            framesOutOfOrder++;
            return false;
        }
        if (minPeriodNs > 0 && lastFrameTNs != Long.MIN_VALUE
                && f.tNs - lastFrameTNs < minPeriodNs) {
            framesSkippedRate++;
            return false;
        }
        long h = fHead;
        if (h - fTail >= slots.length) {
            framesDropped.incrementAndGet();
            return false;
        }
        slots[(int) (h % slots.length)].copyFrom(f);
        lastFrameTNs = f.tNs;
        fHead = h + 1;    // volatile write publishes the slot to the writer
        return true;
    }

    /** Queues one Pinpoint row (raw odometry world). One producer thread only. Never blocks. */
    public boolean offerOdom(long tNs, double x, double y, double heading) {
        if (!running || failed) {
            return false;
        }
        if (tNs <= lastOdomTNs) {
            odomOutOfOrder++;
            return false;
        }
        long h = oHead;
        if (h - oTail >= odomCap) {
            odomDropped.incrementAndGet();
            return false;
        }
        int i = (int) (h % odomCap);
        oT[i] = tNs;
        oX[i] = x;
        oY[i] = y;
        oH[i] = heading;
        lastOdomTNs = tNs;
        oHead = h + 1;
        return true;
    }

    /** Start pose the team typed (field frame). Any thread; rare. */
    public void writeStart(long tNs, double x, double y, double heading) {
        event(EV_START, tNs, x, y, heading, null);
    }

    /** A tape-mark check (field frame). Any thread; rare. */
    public void addMark(long tNs, double x, double y, double heading) {
        event(EV_MARK, tNs, x, y, heading, null);
    }

    /** Writes a small text file into the run directory (on the writer thread). */
    public void writeText(String name, String content) {
        event(EV_TEXT, 0, 0, 0, 0, new String[] {name, content});
    }

    private void event(int type, long t, double x, double y, double h, Object extra) {
        if (!running) {
            return;
        }
        synchronized (events) {
            events.add(new Object[] {type, t, x, y, h, extra});
        }
    }

    // -------------------------------------------------------------- counters

    public long getFramesWritten() {
        return framesWritten;
    }

    /** Frames lost because the writer was behind (or failed). */
    public long getFramesDropped() {
        return framesDropped.get();
    }

    /** Frames left out on purpose to stay under maxHz. */
    public long getFramesSkippedForRate() {
        return framesSkippedRate;
    }

    public long getFramesOutOfOrder() {
        return framesOutOfOrder;
    }

    public long getOdomWritten() {
        return odomWritten;
    }

    public long getOdomDropped() {
        return odomDropped.get();
    }

    public long getMarksWritten() {
        return marksWritten;
    }

    /** Time to encode and write the last PNG (ms). */
    public long getLastPngMs() {
        return lastPngMs;
    }

    /** Null if healthy. */
    public String getError() {
        return error;
    }

    public String debugLine() {
        return String.format(Locale.US, "rec %s: frames %d (drop %d, rate-skip %d) odom %d "
                        + "(drop %d) marks %d png %dms%s", running ? "ON" : "off",
                framesWritten, getFramesDropped(), framesSkippedRate, odomWritten,
                getOdomDropped(), marksWritten, lastPngMs, error == null ? "" : " ERR " + error);
    }

    // ---------------------------------------------------------- writer thread

    private static final class Files {
        Writer frames, meta, odom, start, marks;

        void flushAll() throws IOException {
            frames.flush();
            meta.flush();
            odom.flush();
            start.flush();
            marks.flush();
        }

        void closeAll() {
            Writer[] all = {frames, meta, odom, start, marks};
            for (Writer w : all) {
                if (w != null) {
                    try {
                        w.close();
                    } catch (IOException ignored) {
                        // closing anyway
                    }
                }
            }
        }
    }

    private Writer open(String name, String header) throws IOException {
        Writer w = new BufferedWriter(new OutputStreamWriter(
                new FileOutputStream(new File(dir, name)), "UTF-8"), 1 << 16);
        w.write(header);
        w.write('\n');
        return w;
    }

    private void writerLoop(Files f) {
        PngGray png = new PngGray();
        StringBuilder sb = new StringBuilder(256);
        long idx = 0;
        long lastFlush = System.nanoTime();
        boolean cameraWritten = false;
        try {
            while (true) {
                boolean idle = true;
                // frames
                while (fTail < fHead) {
                    GrayFrame g = slots[(int) (fTail % slots.length)];
                    if (!cameraWritten || g.width != width || g.height != height) {
                        width = g.width;
                        height = g.height;
                        writeCameraJson();
                        cameraWritten = true;
                    }
                    writeFrame(png, g, idx, f, sb);
                    idx++;
                    fTail = fTail + 1;      // slot free again
                    framesWritten = idx;
                    idle = false;
                }
                // odometry
                while (oTail < oHead) {
                    int i = (int) (oTail % odomCap);
                    sb.setLength(0);
                    sb.append(oT[i]).append(',').append(oX[i]).append(',').append(oY[i])
                            .append(',').append(oH[i]).append('\n');
                    f.odom.append(sb);
                    odomWritten++;
                    oTail = oTail + 1;
                    idle = false;
                }
                // start / marks / text
                Object[][] evs = takeEvents();
                for (Object[] e : evs) {
                    writeEvent(e, f, sb);
                    idle = false;
                }
                long now = System.nanoTime();
                if (now - lastFlush > FLUSH_NS) {
                    f.flushAll();
                    lastFlush = now;
                }
                if (idle) {
                    if (!running) {
                        break;
                    }
                    LockSupport.parkNanos(POLL_NS);
                }
            }
            if (width > 0) {
                writeCameraJson();
            }
            f.flushAll();
        } catch (IOException e) {
            error = "recording failed: " + e.getMessage();
            failed = true;
        } catch (RuntimeException e) {
            error = "recording failed: " + e;
            failed = true;
        } finally {
            f.closeAll();
            running = false;
        }
    }

    private Object[][] takeEvents() {
        synchronized (events) {
            if (events.isEmpty()) {
                return new Object[0][];
            }
            Object[][] out = events.toArray(new Object[events.size()][]);
            events.clear();
            return out;
        }
    }

    private void writeEvent(Object[] e, Files f, StringBuilder sb) throws IOException {
        int type = (Integer) e[0];
        if (type == EV_TEXT) {
            String[] nc = (String[]) e[5];
            Writer w = new OutputStreamWriter(new FileOutputStream(new File(dir, nc[0])),
                    "UTF-8");
            try {
                w.write(nc[1]);
            } finally {
                w.close();
            }
            return;
        }
        sb.setLength(0);
        sb.append((long) (Long) e[1]).append(',').append((double) (Double) e[2]).append(',')
                .append((double) (Double) e[3]).append(',').append((double) (Double) e[4])
                .append('\n');
        if (type == EV_START) {
            f.start.append(sb);
        } else {
            f.marks.append(sb);
            marksWritten++;
        }
    }

    private void writeFrame(PngGray png, GrayFrame g, long idx, Files f, StringBuilder sb)
            throws IOException {
        long t0 = System.nanoTime();
        File out = new File(dir, "frames/" + pad6(idx) + ".png");
        OutputStream os = new BufferedOutputStream(new FileOutputStream(out), 1 << 16);
        try {
            png.write(os, g.gray, g.width, g.height);
        } finally {
            os.close();
        }
        lastPngMs = (System.nanoTime() - t0) / 1000000L;
        sb.setLength(0);
        sb.append(idx).append(',').append(g.tNs).append('\n');
        f.frames.append(sb);
        sb.setLength(0);
        sb.append(idx).append(',').append(g.tNs).append(',').append(g.arrivalNs).append(',')
                .append(g.lastByteNs).append(',').append(g.latencyMs).append(',')
                .append(g.latencySource).append(',').append(g.jpegBytes).append(',')
                .append(g.decodeNs / 1e6).append(',').append(g.width).append(',')
                .append(g.height).append('\n');
        f.meta.append(sb);
    }

    private static String pad6(long v) {
        String s = Long.toString(v);
        StringBuilder sb = new StringBuilder(8);
        for (int i = s.length(); i < 6; i++) {
            sb.append('0');
        }
        return sb.append(s).toString();
    }

    /**
     * camera.json with exactly the I2 keys: the frame size seen so far and UNCALIBRATED
     * intrinsics from the datasheet HFOV, the same convention as tools/sim/camera.py
     * {@code Camera.from_hfov} and the Mac recorder (square pixels, principal point at
     * ((w-1)/2, (h-1)/2), zero distortion, rollingShutterReadoutS 0). camera_notes.txt says
     * so. A checkerboard calibration (tools/limelight/ll_calibrate.py on a "_checker" run)
     * replaces the intrinsics.
     */
    private void writeCameraJson() throws IOException {
        int w = width;
        int h = height;
        double fx = w > 0 ? (w / 2.0) / Math.tan(Math.toRadians(NOMINAL_HFOV_DEG / 2)) : 0;
        StringBuilder sb = new StringBuilder(512);
        sb.append("{\n  \"width\": ").append(w).append(",\n  \"height\": ").append(h)
                .append(",\n  \"fx\": ").append(fx).append(",\n  \"fy\": ").append(fx)
                .append(",\n  \"cx\": ").append((w - 1) / 2.0).append(",\n  \"cy\": ")
                .append((h - 1) / 2.0)
                .append(",\n  \"dist\": [0.0, 0.0, 0.0, 0.0, 0.0],\n  \"mount\": {\"x\": ")
                .append(mount.x).append(", \"y\": ").append(mount.y).append(", \"z\": ")
                .append(mount.z).append(", \"roll\": ").append(mount.roll)
                .append(", \"pitch\": ").append(mount.pitch).append(", \"yaw\": ")
                .append(mount.yaw).append("},\n  \"rollingShutterReadoutS\": 0.0\n}\n");
        writeAtomically("camera.json", sb.toString());
        writeAtomically("camera_notes.txt", "camera.json intrinsics are UNCALIBRATED: fx = fy "
                + "from the Limelight 3A datasheet HFOV " + NOMINAL_HFOV_DEG + " deg, principal "
                + "point at the image centre, no distortion. rollingShutterReadoutS 0 is a "
                + "placeholder, not a measurement. mount is beacon.json T_robot_cam. Replace the "
                + "intrinsics with a checkerboard calibration (ll_calibrate.py).\n");
    }

    private void writeAtomically(String name, String text) throws IOException {
        File tmp = new File(dir, name + ".tmp");
        Writer out = new OutputStreamWriter(new FileOutputStream(tmp), "UTF-8");
        try {
            out.write(text);
        } finally {
            out.close();
        }
        File dst = new File(dir, name);
        if (dst.exists() && !dst.delete()) {
            throw new IOException("cannot replace " + dst.getPath());
        }
        if (!tmp.renameTo(dst)) {
            throw new IOException("cannot write " + dst.getPath());
        }
    }

    private static void copy(File from, File to) throws IOException {
        InputStream in = new FileInputStream(from);
        try {
            OutputStream out = new FileOutputStream(to);
            try {
                byte[] buf = new byte[8192];
                int n;
                while ((n = in.read(buf)) > 0) {
                    out.write(buf, 0, n);
                }
            } finally {
                out.close();
            }
        } finally {
            in.close();
        }
    }
}
