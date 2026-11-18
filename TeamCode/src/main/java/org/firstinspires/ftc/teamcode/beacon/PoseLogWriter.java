package org.firstinspires.ftc.teamcode.beacon;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.concurrent.atomic.AtomicLong;
import java.util.concurrent.locks.LockSupport;

/**
 * Writes pose rows in the project CSV format
 * {@code t_ns,x,y,z,qx,qy,qz,qw,status} on its own thread.
 *
 * <h3>Threading and back-pressure policy</h3>
 * {@link #offer} is called from the OpMode loop (or a pose-source thread). It copies the row
 * into a preallocated ring buffer and returns: no file I/O, no locks, no allocation, never
 * blocks. A background thread drains the ring, formats and writes. <b>If the writer falls
 * behind</b> (ring full, slow SD card), the NEWEST row is dropped and {@link #getDroppedCount}
 * is incremented; the loop is never delayed. If the file becomes unwritable the writer stops
 * writing, remembers the message ({@link #getError}), and counts every later row as dropped.
 * Check {@link #getDroppedCount()} after a run; any non-zero value means the log has gaps.
 *
 * <p>Single producer: only ONE thread may call {@code offer} on a given writer. Give each
 * producer its own writer (and file).
 *
 * <p>Row content is the caller's choice. {@link BeaconLocalizer} logs the robot pose in the
 * field frame (yaw-only quaternion); a pose source logs its camera pose in its own world.
 * Doubles are written with {@code Double.toString} (may use E notation; Python and Excel
 * read it).
 */
public final class PoseLogWriter {
    public static final String HEADER = "t_ns,x,y,z,qx,qy,qz,qw,status";
    private static final int FIELDS = 7;
    private static final long POLL_NS = 5000000L;      // 5 ms idle poll, no busy wait
    private static final long FLUSH_NS = 1000000000L;  // flush at least once a second

    private final File file;
    private final int capacity;
    private final long[] tNs;
    private final double[] vals;
    private final int[] status;

    private volatile long head;   // written only by the producer
    private volatile long tail;   // written only by the writer thread
    private volatile boolean running;
    private volatile String error;
    private final AtomicLong droppedFull = new AtomicLong();
    private final AtomicLong droppedFailed = new AtomicLong();
    private volatile long written;

    private Thread thread;

    public PoseLogWriter(File file) {
        this(file, 4096);
    }

    /** @param capacity ring size in rows (at 100 Hz, 4096 rows is about 40 s of slack). */
    public PoseLogWriter(File file, int capacity) {
        if (capacity < 16) {
            throw new IllegalArgumentException("capacity must be at least 16");
        }
        this.file = file;
        this.capacity = capacity;
        this.tNs = new long[capacity];
        this.vals = new double[capacity * FIELDS];
        this.status = new int[capacity];
    }

    /**
     * A writer for {@code <log.dir>/<prefix>_<yyyyMMdd_HHmmss>.csv}, or null if logging is
     * disabled in the config. The wall clock is used only to name the file.
     */
    public static PoseLogWriter forConfig(BeaconConfig cfg, String prefix) {
        if (!cfg.log.enabled) {
            return null;
        }
        String stamp = new java.text.SimpleDateFormat("yyyyMMdd_HHmmss", java.util.Locale.US)
                .format(new java.util.Date());
        return new PoseLogWriter(new File(cfg.log.dir, prefix + "_" + stamp + ".csv"));
    }

    public File getFile() {
        return file;
    }

    /**
     * Creates the directory and file, writes the header and starts the writer thread. Call it
     * from init, not from the loop.
     * @return false if the file could not be opened (see {@link #getError()}); the writer is
     *         then inert and {@code offer} just returns false.
     */
    public synchronized boolean start() {
        if (running) {
            return true;
        }
        if (thread != null && thread.isAlive()) {
            // close() timed out waiting for the old drain thread; a second consumer on the
            // same ring would corrupt it, so refuse rather than restart.
            error = "previous writer thread is still draining; not restarted";
            return false;
        }
        thread = null;
        BufferedWriter w = null;
        try {
            File dir = file.getAbsoluteFile().getParentFile();
            if (dir != null && !dir.isDirectory() && !dir.mkdirs()) {
                throw new IOException("cannot create directory " + dir.getPath());
            }
            // Append, so a restart never truncates rows already written; header only once.
            boolean fresh = !file.exists() || file.length() == 0;
            w = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(file, true),
                    "UTF-8"), 1 << 16);
            if (fresh) {
                w.write(HEADER);
                w.write('\n');
            }
            w.flush();
        } catch (IOException e) {
            error = "cannot open " + file.getPath() + ": " + e.getMessage();
            closeQuietly(w);
            return false;
        }
        error = null;
        running = true;
        final BufferedWriter out = w;
        thread = new Thread(new Runnable() {
            @Override
            public void run() {
                drainLoop(out);
            }
        }, "BeaconLogWriter");
        thread.setDaemon(true);
        thread.start();
        return true;
    }

    /**
     * Hands one row to the writer thread. Non-blocking.
     * @return true if queued, false if dropped (ring full, writer failed) or not started.
     */
    public boolean offer(long t, double x, double y, double z,
                         double qx, double qy, double qz, double qw, int st) {
        if (!running) {
            return false;
        }
        long h = head;
        if (h - tail >= capacity) {
            droppedFull.incrementAndGet();
            return false;
        }
        int i = (int) (h % capacity);
        int b = i * FIELDS;
        tNs[i] = t;
        vals[b] = x;
        vals[b + 1] = y;
        vals[b + 2] = z;
        vals[b + 3] = qx;
        vals[b + 4] = qy;
        vals[b + 5] = qz;
        vals[b + 6] = qw;
        status[i] = st;
        head = h + 1;   // volatile write publishes the row to the writer thread
        return true;
    }

    /** Rows lost because the ring was full or the file failed. */
    public long getDroppedCount() {
        return droppedFull.get() + droppedFailed.get();
    }

    /** Rows written to the file so far (flushed or not). */
    public long getWrittenCount() {
        return written;
    }

    /** Null if healthy. */
    public String getError() {
        return error;
    }

    public boolean isRunning() {
        return running;
    }

    /**
     * Writes everything queued, flushes, closes the file and stops the thread. Waits at most
     * about a second. Safe to call twice. Not for use inside the loop.
     */
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
                t.join(1500);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
        synchronized (this) {
            if (thread == t && (t == null || !t.isAlive())) {
                thread = null;   // fully stopped; start() may run again
            }
        }
    }

    // ------------------------------------------------------------ writer thread

    private void drainLoop(BufferedWriter out) {
        StringBuilder sb = new StringBuilder(160);
        boolean failed = false;
        long lastFlush = System.nanoTime();
        while (true) {
            long t = tail;
            long h = head;
            if (t == h) {
                if (!running) {
                    break;
                }
                LockSupport.parkNanos(POLL_NS);
            } else {
                while (t < h) {
                    if (failed) {
                        droppedFailed.incrementAndGet();
                    } else {
                        int i = (int) (t % capacity);
                        int b = i * FIELDS;
                        sb.setLength(0);
                        sb.append(tNs[i]);
                        for (int k = 0; k < FIELDS; k++) {
                            sb.append(',').append(vals[b + k]);
                        }
                        sb.append(',').append(status[i]).append('\n');
                        try {
                            out.append(sb);
                            written++;
                        } catch (IOException e) {
                            error = "write failed: " + e.getMessage();
                            failed = true;
                            droppedFailed.incrementAndGet();
                        }
                    }
                    t++;
                }
                tail = t;
            }
            long now = System.nanoTime();
            if (!failed && now - lastFlush > FLUSH_NS) {
                try {
                    out.flush();
                } catch (IOException e) {
                    error = "flush failed: " + e.getMessage();
                    failed = true;
                }
                lastFlush = now;
            }
        }
        if (!failed) {
            try {
                out.flush();
            } catch (IOException e) {
                error = "flush failed: " + e.getMessage();
            }
        }
        closeQuietly(out);
    }

    private static void closeQuietly(BufferedWriter w) {
        if (w != null) {
            try {
                w.close();
            } catch (IOException ignored) {
                // nothing useful to do
            }
        }
    }
}
