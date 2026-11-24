package org.firstinspires.ftc.teamcode.beacon;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Desktop fusion replay (plan task J4): runs the REAL {@link BeaconLocalizer} over a dataset
 * (interface I2) and a fix file from the native core's replay bench, and writes what team code
 * would have read.
 *
 * <pre>
 *   inputs   &lt;run&gt;/odom.csv    t_ns,x,y,heading   (Pinpoint world, robot centre)
 *            &lt;run&gt;/start.csv   t_ns,x,y,heading   (field frame; first row used)
 *            fixes.csv           t_ns,x,y,heading,sigma_xy,sigma_heading,inliers,kind
 *                                (field frame, t_ns = capture time; C1 beacon_replay output)
 *   output   fused.csv           t_ns,x,y,heading   (field frame, one row per odom sample
 *                                while the localizer is TRACKING)
 * </pre>
 *
 * <h3>Replay rules</h3>
 * <ul>
 *   <li>Each odom row is one Pinpoint sample = one {@code update()}. The localizer's clock is the
 *       replay time (the current row's t_ns), so its timeouts behave as on the robot.</li>
 *   <li>The start pose is declared before the first sample, so it aligns the odometry world at
 *       the first odom row (datasets start with the robot still at the start pose), then
 *       {@code onStart()} is called.</li>
 *   <li>A fix becomes visible to the localizer {@code fixDelayMs} after its capture time (the
 *       time the native core needs to publish it; default 50 ms). Fixes are delivered in file
 *       order; a later fix replaces an unread earlier one, exactly like the robot's
 *       latest-value hand-off.</li>
 *   <li>sigma_xy / sigma_heading empty or "nan" = unknown (the configured gains apply).</li>
 * </ul>
 * Run from the command line (Gradle forwards these -P properties to the test JVM):
 * {@code ./gradlew :TeamCode:testDebugUnitTest --offline --tests '*ReplayFusionTest*'
 * -Pbeacon.replay.run=DIR [-Pbeacon.replay.fixes=FILE] [-Pbeacon.replay.out=FILE]
 * [-Pbeacon.replay.fixDelayMs=50] [-Pbeacon.replay.config=beacon.json]}. Without
 * {@code fixes} it uses {@code DIR/fixes.csv} if present, else no fixes; output defaults to
 * {@code DIR/fused.csv}.
 */
public final class ReplayFusion {

    /** What a replay did, for the caller and the console. */
    public static final class Result {
        public int odomRows;
        public int fusedRows;
        public int fixesRead;
        public long fixesApplied;
        public long fixesRejected;
        public String lastFixReject = "";

        @Override
        public String toString() {
            return "odom rows " + odomRows + ", fused rows " + fusedRows + ", fixes read "
                    + fixesRead + ", applied " + fixesApplied + ", rejected " + fixesRejected
                    + (lastFixReject.isEmpty() ? "" : " (last: " + lastFixReject + ")");
        }
    }

    private ReplayFusion() {}

    /** Replays and writes {@code out}. {@code fixes} may be null (odometry only). */
    public static Result run(File runDir, File fixes, File out, BeaconConfig cfg, double fixDelayMs)
            throws IOException, BeaconException {
        List<Row> odom = readPoses(new File(runDir, "odom.csv"));
        List<Row> start = readPoses(new File(runDir, "start.csv"));
        if (odom.isEmpty()) {
            throw new IOException("odom.csv has no rows");
        }
        if (start.isEmpty()) {
            throw new IOException("start.csv has no rows");
        }
        List<FieldFix> fixList = fixes == null ? new ArrayList<FieldFix>() : readFixes(fixes);

        final long[] now = {odom.get(0).t};
        ReplaySource src = new ReplaySource(odom);
        QueuedFixes fq = new QueuedFixes(fixList, (long) (fixDelayMs * 1e6));
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg, fq, null, new BeaconLocalizer.Clock() {
            @Override
            public long nanoTime() {
                return now[0];
            }
        });
        loc.start(null);
        Row s0 = start.get(0);
        loc.setPose(s0.x, s0.y, s0.h);

        Result r = new Result();
        r.odomRows = odom.size();
        r.fixesRead = fixList.size();
        Writer w = new BufferedWriter(new OutputStreamWriter(new FileOutputStream(out), "UTF-8"));
        try {
            w.write("t_ns,x,y,heading\n");
            boolean started = false;
            StringBuilder sb = new StringBuilder(96);
            for (int i = 0; i < odom.size(); i++) {
                long t = odom.get(i).t;
                now[0] = t;
                fq.advanceTo(t);
                Pose2dStamped p = loc.update();
                if (!started) {
                    loc.onStart();
                    started = true;
                }
                if (p.status == BeaconStatus.TRACKING) {
                    sb.setLength(0);
                    sb.append(t).append(',').append(p.x).append(',').append(p.y).append(',')
                            .append(p.heading).append('\n');
                    w.write(sb.toString());
                    r.fusedRows++;
                }
            }
        } finally {
            w.close();
        }
        r.fixesApplied = loc.getFixesApplied();
        r.fixesRejected = loc.getFixesRejected();
        r.lastFixReject = loc.getLastFixRejectReason();
        return r;
    }

    // ---------------------------------------------------------------- sources

    /** One odom row per poll, as a TRACKING robot pose (yaw-only quaternion). */
    static final class ReplaySource implements PoseSource {
        private final List<Row> rows;
        private int next;
        private Pose3dStamped latest = new Pose3dStamped(0, 0, 0, 0, 0, 0, 1, 0,
                BeaconStatus.INITIALIZING);

        ReplaySource(List<Row> rows) {
            this.rows = rows;
        }

        @Override
        public void start(android.content.Context ctx, BeaconConfig cfg) {
        }

        @Override
        public void stop() {
        }

        @Override
        public void poll() {
            if (next < rows.size()) {
                Row r = rows.get(next++);
                latest = new Pose3dStamped(r.x, r.y, 0, 0, 0, Math.sin(0.5 * r.h),
                        Math.cos(0.5 * r.h), r.t, BeaconStatus.TRACKING);
            }
        }

        @Override
        public Pose3dStamped latest() {
            return latest;
        }

        @Override
        public boolean isTracking() {
            return latest.status == BeaconStatus.TRACKING;
        }

        @Override
        public String debugLine() {
            return "replay " + next + "/" + rows.size();
        }

        @Override
        public boolean reportsRobotPose() {
            return true;
        }
    }

    /** Releases each fix fixDelay after its capture time; latest released wins. */
    static final class QueuedFixes implements FieldFixSource {
        private final List<FieldFix> fixes;
        private final long delayNs;
        private int next;
        private FieldFix current;

        QueuedFixes(List<FieldFix> fixes, long delayNs) {
            this.fixes = fixes;
            this.delayNs = delayNs;
        }

        void advanceTo(long t) {
            while (next < fixes.size() && fixes.get(next).tNs + delayNs <= t) {
                current = fixes.get(next++);
            }
        }

        @Override
        public FieldFix latestFix() {
            return current;
        }
    }

    // -------------------------------------------------------------------- csv

    static List<FieldFix> readFixes(File f) throws IOException {
        List<FieldFix> out = new ArrayList<FieldFix>();
        BufferedReader in = new BufferedReader(new InputStreamReader(new FileInputStream(f),
                "UTF-8"));
        try {
            String header = in.readLine();
            if (header == null) {
                return out;
            }
            Map<String, Integer> col = columns(header);
            for (String need : new String[] {"t_ns", "x", "y", "heading"}) {
                if (!col.containsKey(need)) {
                    throw new IOException(f.getPath() + ": no column " + need);
                }
            }
            String line;
            int n = 1;
            while ((line = in.readLine()) != null) {
                n++;
                if (line.trim().isEmpty()) {
                    continue;
                }
                String[] c = line.split(",", -1);
                long t = Long.parseLong(c[col.get("t_ns")].trim());
                double x = num(c, col, "x", f, n);
                double y = num(c, col, "y", f, n);
                double h = num(c, col, "heading", f, n);
                double sxy = optional(c, col, "sigma_xy");
                double sh = optional(c, col, "sigma_heading");
                String kind = col.containsKey("kind") ? "kind" + c[col.get("kind")].trim() : "fix";
                out.add(new FieldFix(x, y, h, t, true, true, kind, sxy, sh));
            }
        } finally {
            in.close();
        }
        return out;
    }

    private static double optional(String[] c, Map<String, Integer> col, String name) {
        Integer i = col.get(name);
        if (i == null || i >= c.length) {
            return Double.NaN;
        }
        String v = c[i].trim();
        if (v.isEmpty() || v.equalsIgnoreCase("nan")) {
            return Double.NaN;
        }
        return Double.parseDouble(v);
    }

    private static double num(String[] c, Map<String, Integer> col, String name, File f, int line)
            throws IOException {
        try {
            return Double.parseDouble(c[col.get(name)].trim());
        } catch (RuntimeException e) {
            throw new IOException(f.getPath() + " line " + line + ": bad " + name);
        }
    }

    private static Map<String, Integer> columns(String header) {
        Map<String, Integer> m = new HashMap<String, Integer>();
        String[] h = header.split(",");
        for (int i = 0; i < h.length; i++) {
            m.put(h[i].trim(), i);
        }
        return m;
    }

    /** One t_ns,x,y,heading row. */
    static final class Row {
        final long t;
        final double x, y, h;

        Row(long t, double x, double y, double h) {
            this.t = t;
            this.x = x;
            this.y = y;
            this.h = h;
        }
    }

    /** Reads t_ns,x,y,heading (any column order, extra columns ignored); t_ns as a long. */
    static List<Row> readPoses(File f) throws IOException {
        List<Row> out = new ArrayList<Row>();
        BufferedReader in = new BufferedReader(new InputStreamReader(new FileInputStream(f),
                "UTF-8"));
        try {
            String header = in.readLine();
            if (header == null) {
                return out;
            }
            Map<String, Integer> col = columns(header);
            for (String need : new String[] {"t_ns", "x", "y", "heading"}) {
                if (!col.containsKey(need)) {
                    throw new IOException(f.getPath() + ": no column " + need);
                }
            }
            String line;
            int n = 1;
            while ((line = in.readLine()) != null) {
                n++;
                if (line.trim().isEmpty()) {
                    continue;
                }
                String[] c = line.split(",", -1);
                long t;
                try {
                    t = Long.parseLong(c[col.get("t_ns")].trim());
                } catch (RuntimeException e) {
                    throw new IOException(f.getPath() + " line " + n + ": bad t_ns");
                }
                out.add(new Row(t, num(c, col, "x", f, n), num(c, col, "y", f, n),
                        num(c, col, "heading", f, n)));
            }
        } finally {
            in.close();
        }
        return out;
    }
}
