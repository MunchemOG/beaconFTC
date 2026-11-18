package org.firstinspires.ftc.teamcode.beacon;

import static org.junit.Assert.assertTrue;

import java.io.File;
import org.junit.Test;

/** Spec acceptance: update() averages under 0.5 ms. */
public class BeaconLocalizerTimingTest {

    private static final class Tags implements FieldFixSource {
        volatile FieldFix fix;

        @Override
        public FieldFix latestFix() {
            return fix;
        }
    }

    @Test
    public void updateAveragesUnderHalfAMillisecond() throws Exception {
        BeaconConfig cfg = BeaconConfig.parse("{\"tier\":\"fake\",\"T_robot_cam\":"
                + "{\"x\":0.18,\"y\":-0.07,\"z\":0.22,\"pitch\":0.26,\"yaw\":0.12},"
                + "\"fix\":{\"maxJumpM\":10,\"maxSpeedMps\":10,\"maxOmegaRadPs\":20}}");
        // 20 laps of a 1 m square at 100 Hz (about 20000 samples)
        java.util.List<Pose3dStamped> all = new java.util.ArrayList<Pose3dStamped>();
        long t0 = 1000000000L;
        for (int lap = 0; lap < 20; lap++) {
            FakePoseSource one = FakePoseSource.squareLoop(cfg.cameraMount, 1.0, 0.5, 2.0, 100,
                    0, 0, 0, t0);
            for (int i = 0; i < one.size(); i++) {
                all.add(one.sample(i));
            }
            t0 = all.get(all.size() - 1).tNs + 10000000L;
        }
        FakePoseSource src = new FakePoseSource(all);
        src.start(null, cfg);
        File f = File.createTempFile("beacon_timing", ".csv");
        f.deleteOnExit();
        PoseLogWriter log = new PoseLogWriter(f);
        Tags tags = new Tags();
        BeaconLocalizer loc = new BeaconLocalizer(src, cfg, tags, log);
        loc.start(null);
        loc.setPose(0, 0, 0);

        final int warmup = 3000;
        final int measured = 12000;
        long total = 0;
        long worst = 0;
        int n = 0;
        int steps = 0;
        double sink = 0;
        while (n < warmup + measured && src.advance()) {
            if (steps % 25 == 0) {      // a tag fix every 0.25 s, small error
                Pose2dStamped p = loc.getPose();
                tags.fix = FieldFix.full(p.x + 0.002, p.y, p.heading, src.latest().tNs);
            }
            steps++;
            long a = System.nanoTime();
            Pose2dStamped p = loc.update();
            long b = System.nanoTime();
            sink += p.x + loc.getVelocity().vx;
            if (n >= warmup) {
                total += b - a;
                worst = Math.max(worst, b - a);
            }
            n++;
        }
        loc.stop();
        double meanUs = total / 1000.0 / (n - warmup);
        System.out.println("BEACON_TIMING update() mean = " + meanUs + " us over " + (n - warmup)
                + " calls, worst = " + worst / 1000.0 + " us, log dropped = "
                + log.getDroppedCount() + " (sink " + (sink != 0) + ")");
        assertTrue("measured " + (n - warmup) + " calls", n - warmup >= 3000);
        assertTrue("mean update() " + meanUs + " us", meanUs < 500.0);
    }
}
