package org.firstinspires.ftc.teamcode.beacon;

import android.content.Context;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;

/**
 * A {@link PoseSource} that replays a list of poses, for unit tests and for bench work with
 * no camera. It never touches the Android {@code Context} (tests pass null) and meets the
 * frame contract only to the extent the samples it is given do.
 *
 * <p>Time is driven by the caller, so tests are deterministic: {@link #advance()} publishes the
 * next sample, {@link #advanceTo(long)} publishes every sample up to a time, and
 * {@link #publish(Pose3dStamped)} injects an arbitrary sample (NaN, backwards time) to test
 * the localizer's defences. {@link #latest()} is a volatile read.
 *
 * <p>Factories: {@link #fromCsv(File)} (rows {@code t_ns,x,y,z,qx,qy,qz,qw,status} as written
 * by a pose source's log), {@link #squareLoop} and {@link #spinInPlace} (synthetic robot
 * paths converted to camera poses through a mount, so lever-arm handling is exercised).
 */
public final class FakePoseSource implements PoseSource {

    private static final Pose3dStamped NONE =
            new Pose3dStamped(0, 0, 0, 0, 0, 0, 1, 0, BeaconStatus.INVALID);

    private final List<Pose3dStamped> samples;
    private int next;
    private volatile Pose3dStamped latest = NONE;
    private volatile boolean started;

    public FakePoseSource(List<Pose3dStamped> samples) {
        this.samples = new ArrayList<Pose3dStamped>(samples);
    }

    /** An empty source; feed it with {@link #publish}. */
    public FakePoseSource() {
        this(new ArrayList<Pose3dStamped>());
    }

    // ------------------------------------------------------------ PoseSource

    @Override
    public void start(Context ctx, BeaconConfig cfg) throws BeaconException {
        started = true;   // ctx and cfg deliberately unused: may be null
    }

    @Override
    public void stop() {
        started = false;
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
        return "fake " + (started ? "running" : "stopped") + " sample " + next + "/"
                + samples.size();
    }

    // -------------------------------------------------------- test controls

    /** Publishes the next sample of the list. @return false if there are none left. */
    public boolean advance() {
        if (next >= samples.size()) {
            return false;
        }
        latest = samples.get(next++);
        return true;
    }

    /**
     * Publishes the newest sample whose time is &lt;= tNs (skipping earlier ones, like a slow
     * reader would).
     * @return number of samples consumed
     */
    public int advanceTo(long tNs) {
        int n = 0;
        while (next < samples.size() && samples.get(next).tNs <= tNs) {
            latest = samples.get(next++);
            n++;
        }
        return n;
    }

    /** Publishes an arbitrary sample immediately. */
    public void publish(Pose3dStamped s) {
        latest = s;
    }

    public int size() {
        return samples.size();
    }

    public int position() {
        return next;
    }

    public boolean hasMore() {
        return next < samples.size();
    }

    public Pose3dStamped sample(int i) {
        return samples.get(i);
    }

    // -------------------------------------------------------------- factories

    /** Reads a pose CSV (header optional). @throws BeaconException on a bad row. */
    public static FakePoseSource fromCsv(File f) throws BeaconException {
        List<Pose3dStamped> list = new ArrayList<Pose3dStamped>();
        BufferedReader r = null;
        try {
            r = new BufferedReader(new InputStreamReader(new FileInputStream(f), "UTF-8"));
            String line;
            int n = 0;
            while ((line = r.readLine()) != null) {
                n++;
                line = line.trim();
                if (line.isEmpty() || line.startsWith("#") || line.startsWith("t_ns")) {
                    continue;
                }
                list.add(parseRow(line, n));
            }
        } catch (IOException e) {
            throw new BeaconException("Cannot read " + f.getPath() + ": " + e.getMessage(), e);
        } finally {
            if (r != null) {
                try {
                    r.close();
                } catch (IOException ignored) {
                    // nothing useful to do
                }
            }
        }
        return new FakePoseSource(list);
    }

    private static Pose3dStamped parseRow(String line, int lineNo) throws BeaconException {
        String[] p = line.split(",");
        if (p.length < 9) {
            throw new BeaconException("CSV line " + lineNo + ": expected 9 columns, got "
                    + p.length);
        }
        try {
            return new Pose3dStamped(
                    Double.parseDouble(p[1]), Double.parseDouble(p[2]), Double.parseDouble(p[3]),
                    Double.parseDouble(p[4]), Double.parseDouble(p[5]), Double.parseDouble(p[6]),
                    Double.parseDouble(p[7]),
                    Long.parseLong(p[0].trim()), Integer.parseInt(p[8].trim()));
        } catch (NumberFormatException e) {
            throw new BeaconException("CSV line " + lineNo + ": bad number (" + e.getMessage()
                    + ")");
        }
    }

    /**
     * Camera pose (per the frame contract) of a robot at (rx, ry, rz) with the given roll,
     * pitch, yaw in the source world, for a camera mounted as {@code mount}.
     */
    public static Pose3dStamped cameraSample(BeaconConfig.CameraMount mount, long tNs,
                                             double rx, double ry, double rz,
                                             double roll, double pitch, double yaw,
                                             int status) {
        double[] tWorldRobot = new double[7];
        tWorldRobot[0] = rx;
        tWorldRobot[1] = ry;
        tWorldRobot[2] = rz;
        double[] q = new double[4];
        FrameMath.quatFromRpy(roll, pitch, yaw, q);
        tWorldRobot[3] = q[0];
        tWorldRobot[4] = q[1];
        tWorldRobot[5] = q[2];
        tWorldRobot[6] = q[3];
        double[] tWorldCam = new double[7];
        FrameMath.compose(tWorldRobot, mount.toTransform(), tWorldCam);
        return new Pose3dStamped(tWorldCam[0], tWorldCam[1], tWorldCam[2],
                tWorldCam[3], tWorldCam[4], tWorldCam[5], tWorldCam[6], tNs, status);
    }

    /**
     * A robot driving a square of side {@code side} metres: forward, turn left 90 degrees in
     * place, four times, ending exactly where it started and facing the start heading plus 2
     * pi. Starts at (x0, y0) heading yaw0 in the SOURCE world. First sample is at
     * {@code t0Ns}; samples are {@code 1/hz} apart; every sample has status TRACKING.
     */
    public static FakePoseSource squareLoop(BeaconConfig.CameraMount mount,
                                            double side, double speed, double turnRate,
                                            double hz, double x0, double y0, double yaw0,
                                            long t0Ns) {
        double dt = 1.0 / hz;
        long dtNs = Math.round(1e9 * dt);
        List<Pose3dStamped> list = new ArrayList<Pose3dStamped>();
        double x = x0, y = y0, yaw = yaw0;
        long t = t0Ns;
        list.add(cameraSample(mount, t, x, y, 0, 0, 0, yaw, BeaconStatus.TRACKING));
        int driveSteps = Math.max(1, (int) Math.round(side / speed / dt));
        int turnSteps = Math.max(1, (int) Math.round((Math.PI / 2) / turnRate / dt));
        double stepLen = side / driveSteps;
        double stepTurn = (Math.PI / 2) / turnSteps;
        for (int leg = 0; leg < 4; leg++) {
            for (int i = 0; i < driveSteps; i++) {
                x += stepLen * Math.cos(yaw);
                y += stepLen * Math.sin(yaw);
                t += dtNs;
                list.add(cameraSample(mount, t, x, y, 0, 0, 0, yaw, BeaconStatus.TRACKING));
            }
            for (int i = 0; i < turnSteps; i++) {
                yaw += stepTurn;
                t += dtNs;
                list.add(cameraSample(mount, t, x, y, 0, 0, 0, yaw, BeaconStatus.TRACKING));
            }
        }
        return new FakePoseSource(list);
    }

    /** A robot spinning in place about (x0, y0) through {@code totalRad}. */
    public static FakePoseSource spinInPlace(BeaconConfig.CameraMount mount,
                                             double x0, double y0, double yaw0,
                                             double totalRad, int steps, long t0Ns,
                                             long dtNs) {
        List<Pose3dStamped> list = new ArrayList<Pose3dStamped>();
        for (int i = 0; i <= steps; i++) {
            double yaw = yaw0 + totalRad * i / steps;
            list.add(cameraSample(mount, t0Ns + i * dtNs, x0, y0, 0, 0, 0, yaw,
                    BeaconStatus.TRACKING));
        }
        return new FakePoseSource(list);
    }
}
