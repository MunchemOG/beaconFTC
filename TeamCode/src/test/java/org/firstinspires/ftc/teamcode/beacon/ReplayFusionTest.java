package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Assume;
import org.junit.Rule;
import org.junit.Test;
import org.junit.rules.TemporaryFolder;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.OutputStreamWriter;
import java.io.Writer;
import java.util.List;

/** Task J4: the real localizer replayed over datasets (synthetic here; any I2 run by property). */
public class ReplayFusionTest {

    @Rule
    public TemporaryFolder tmp = new TemporaryFolder();

    private static final long T0 = 1000000000L;
    private static final long DT = 5000000L;            // odom at 200 Hz
    private static final double R = 1.0, V = 0.3;       // circle radius (m), speed (m/s)
    private static final double STILL_S = 1.0, RUN_S = 30.0;

    /** Truth at sample i: x, y, heading (field frame). Still for 1 s, then a CCW circle. */
    private static double[] truth(int i) {
        double t = i * DT * 1e-9;
        double th = t < STILL_S ? 0 : (V / R) * (t - STILL_S);
        return new double[] {R * Math.cos(th), R * Math.sin(th), th + Math.PI / 2};
    }

    private static int samples() {
        return (int) ((STILL_S + RUN_S) / (DT * 1e-9));
    }

    /**
     * Writes odom.csv (Pinpoint-like: own world rotated 0.7 rad and shifted, 1 % distance scale
     * error, 0.01 rad/s heading drift), start.csv (truth at t0) and, if fixEvery > 0, fixes.csv
     * with the exact truth every fixEvery samples.
     */
    private static void writeDataset(File dir, int fixEvery) throws IOException {
        int n = samples();
        Writer odom = open(new File(dir, "odom.csv"), "t_ns,x,y,heading");
        Writer fixes = fixEvery > 0 ? open(new File(dir, "fixes.csv"),
                "t_ns,x,y,heading,sigma_xy,sigma_heading,inliers,kind") : null;
        double ox = 0.3, oy = -0.2, oh = 0.7 + Math.PI / 2;     // odom world start pose
        double[] prev = truth(0);
        for (int i = 0; i < n; i++) {
            double[] tr = truth(i);
            if (i > 0) {
                // truth step in the robot frame of the previous pose
                double dxw = tr[0] - prev[0], dyw = tr[1] - prev[1];
                double c = Math.cos(prev[2]), s = Math.sin(prev[2]);
                double fwd = (c * dxw + s * dyw) * 1.01;
                double left = (-s * dxw + c * dyw) * 1.01;
                double dh = FrameMath.angleDiff(tr[2], prev[2]) + 0.01 * DT * 1e-9;
                double oc = Math.cos(oh), os = Math.sin(oh);
                ox += oc * fwd - os * left;
                oy += os * fwd + oc * left;
                oh += dh;
            }
            prev = tr;
            long t = T0 + i * DT;
            odom.write(t + "," + ox + "," + oy + "," + FrameMath.wrapAngle(oh) + "\n");
            if (fixes != null && i > 0 && i % fixEvery == 0) {
                fixes.write(t + "," + tr[0] + "," + tr[1] + "," + FrameMath.wrapAngle(tr[2])
                        + ",0.002,0.002,120,3\n");
            }
        }
        odom.close();
        if (fixes != null) {
            fixes.close();
        }
        Writer start = open(new File(dir, "start.csv"), "t_ns,x,y,heading");
        double[] s0 = truth(0);
        start.write(T0 + "," + s0[0] + "," + s0[1] + "," + s0[2] + "\n");
        start.close();
    }

    private static Writer open(File f, String header) throws IOException {
        Writer w = new OutputStreamWriter(new FileOutputStream(f), "UTF-8");
        w.write(header + "\n");
        return w;
    }

    @Test
    public void replayWithNoFixesEqualsOdometry() throws Exception {
        File run = tmp.newFolder("nofix");
        writeDataset(run, 0);
        File out = new File(run, "fused.csv");
        ReplayFusion.Result r = ReplayFusion.run(run, null, out, BeaconConfig.parse("{}"), 50);
        assertEquals(samples(), r.odomRows);
        assertEquals(samples(), r.fusedRows);
        assertEquals(0, r.fixesApplied);

        // Expected: odometry mapped by the one rigid alignment that puts odom row 0 on start.
        List<ReplayFusion.Row> odom = ReplayFusion.readPoses(new File(run, "odom.csv"));
        List<ReplayFusion.Row> fused = ReplayFusion.readPoses(out);
        double[] s0 = truth(0);
        ReplayFusion.Row o0 = odom.get(0);
        double a = s0[2] - o0.h;
        double c = Math.cos(a), s = Math.sin(a);
        for (int i = 0; i < odom.size(); i++) {
            ReplayFusion.Row o = odom.get(i);
            ReplayFusion.Row f = fused.get(i);
            assertEquals(o.t, f.t);
            double ex = s0[0] + c * (o.x - o0.x) - s * (o.y - o0.y);
            double ey = s0[1] + s * (o.x - o0.x) + c * (o.y - o0.y);
            assertEquals("x at " + i, ex, f.x, 1e-9);
            assertEquals("y at " + i, ey, f.y, 1e-9);
            assertEquals("h at " + i, 0, FrameMath.angleDiff(o.h + a, f.h), 1e-9);
        }
        // ...and that odometry really drifts, so the next test means something.
        double[] tEnd = truth(odom.size() - 1);
        ReplayFusion.Row last = fused.get(fused.size() - 1);
        assertTrue(Math.hypot(last.x - tEnd[0], last.y - tEnd[1]) > 0.05);
    }

    @Test
    public void replayWithExactFixesTracksTruth() throws Exception {
        File run = tmp.newFolder("fix");
        writeDataset(run, 20);    // a fix every 100 ms, published 50 ms after capture
        File out = new File(run, "fused.csv");
        ReplayFusion.Result r = ReplayFusion.run(run, new File(run, "fixes.csv"), out,
                BeaconConfig.parse("{}"), 50);
        assertTrue(r.toString(), r.fixesApplied > 0.95 * r.fixesRead);
        List<ReplayFusion.Row> fused = ReplayFusion.readPoses(out);
        assertEquals(samples(), fused.size());
        double worst = 0, worstH = 0;
        for (int i = 0; i < fused.size(); i++) {
            double[] tr = truth(i);
            ReplayFusion.Row f = fused.get(i);
            worst = Math.max(worst, Math.hypot(f.x - tr[0], f.y - tr[1]));
            worstH = Math.max(worstH, Math.abs(FrameMath.angleDiff(f.h, tr[2])));
        }
        assertTrue("worst position error " + worst * 1000 + " mm; " + r, worst < 0.005);
        assertTrue("worst heading error " + Math.toDegrees(worstH) + " deg", worstH
                < Math.toRadians(0.5));
    }

    /** Replays a real or synthetic run given with -Pbeacon.replay.run=DIR; skipped otherwise. */
    @Test
    public void replaysTheRunNamedOnTheCommandLine() throws Exception {
        String dir = System.getProperty("beacon.replay.run");
        Assume.assumeTrue("set -Pbeacon.replay.run=DIR to replay a dataset",
                dir != null && !dir.isEmpty());
        File run = new File(dir);
        String fx = System.getProperty("beacon.replay.fixes");
        File fixes = fx != null && !fx.isEmpty() ? new File(fx) : new File(run, "fixes.csv");
        if (!fixes.isFile()) {
            fixes = null;
        }
        String o = System.getProperty("beacon.replay.out");
        File out = o != null && !o.isEmpty() ? new File(o) : new File(run, "fused.csv");
        String cfgPath = System.getProperty("beacon.replay.config");
        BeaconConfig cfg = cfgPath != null && !cfgPath.isEmpty()
                ? BeaconConfig.load(new File(cfgPath)) : BeaconConfig.parse("{}");
        String d = System.getProperty("beacon.replay.fixDelayMs");
        double delay = d != null && !d.isEmpty() ? Double.parseDouble(d) : 50;
        ReplayFusion.Result r = ReplayFusion.run(run, fixes, out, cfg, delay);
        System.out.println("ReplayFusion " + run + " -> " + out + ": " + r);
        assertTrue(r.toString(), r.fusedRows > 0);
    }
}
