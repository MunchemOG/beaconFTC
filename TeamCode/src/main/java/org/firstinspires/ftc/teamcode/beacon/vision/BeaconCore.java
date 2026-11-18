package org.firstinspires.ftc.teamcode.beacon.vision;

import org.firstinspires.ftc.teamcode.beacon.BeaconConfig;
import org.firstinspires.ftc.teamcode.beacon.BeaconException;
import org.firstinspires.ftc.teamcode.beacon.MiniJson;

import java.util.List;
import java.util.Map;

/**
 * Beacon's vision core as Java sees it: the native C API (plan interface I3,
 * beacon-native/include/beacon/beacon.h) one call for one call. The robot implementation is
 * {@link NativeBeacon}; tests use a fake. Units, frames and threading are those of beacon.h:
 * field frame, metres, radians, {@code elapsedRealtimeNanos}; every call except
 * {@link #create} and {@link #destroy} may come from any thread and none waits for vision work.
 */
public interface BeaconCore {
    int OK = 0;
    int DROPPED = 1;
    int ERR = -1;

    int FIX_EDGES = 1;
    int FIX_POINTS = 2;
    int FIX_SEAMS = 4;
    /** Bench-only fix made without odometry; never accepted on a robot. */
    int FIX_NO_ODOM = 8;

    /** Pinhole camera with OpenCV distortion (k1, k2, p1, p2, k3), pixels. */
    final class Camera {
        public final int width, height;
        public final double fx, fy, cx, cy;
        public final double[] dist;
        public final double rollingShutterS;

        public Camera(int width, int height, double fx, double fy, double cx, double cy,
                      double[] dist, double rollingShutterS) {
            this.width = width;
            this.height = height;
            this.fx = fx;
            this.fy = fy;
            this.cx = cx;
            this.cy = cy;
            this.dist = dist;
            this.rollingShutterS = rollingShutterS;
        }

        /** Parses an I2 camera.json (its "mount" is ignored). */
        @SuppressWarnings("unchecked")
        public static Camera parse(String json) throws BeaconException {
            Object o = MiniJson.parse(json);
            if (!(o instanceof Map)) {
                throw new BeaconException("camera.json must be a JSON object");
            }
            Map<String, Object> m = (Map<String, Object>) o;
            int w = (int) num(m, "width", 1, 10000);
            int h = (int) num(m, "height", 1, 10000);
            double fx = num(m, "fx", 1, 1e5);
            double fy = num(m, "fy", 1, 1e5);
            double cx = num(m, "cx", -1e4, 1e4);
            double cy = num(m, "cy", -1e4, 1e4);
            Object dv = m.get("dist");
            if (!(dv instanceof List) || ((List<Object>) dv).size() != 5) {
                throw new BeaconException("camera.json dist must be [k1, k2, p1, p2, k3]");
            }
            double[] dist = new double[5];
            for (int i = 0; i < 5; i++) {
                Object v = ((List<Object>) dv).get(i);
                if (!(v instanceof Double) || !finite((Double) v)) {
                    throw new BeaconException("camera.json dist[" + i + "] must be a number");
                }
                dist[i] = (Double) v;
            }
            double rs = m.containsKey("rollingShutterReadoutS")
                    ? num(m, "rollingShutterReadoutS", 0, 1) : 0;
            return new Camera(w, h, fx, fy, cx, cy, dist, rs);
        }

        private static double num(Map<String, Object> m, String k, double lo, double hi)
                throws BeaconException {
            Object v = m.get(k);
            if (!(v instanceof Double) || !finite((Double) v)) {
                throw new BeaconException("camera.json " + k + " must be a number");
            }
            double d = (Double) v;
            if (d < lo || d > hi) {
                throw new BeaconException("camera.json " + k + " = " + d + " is outside [" + lo
                        + ", " + hi + "]");
            }
            return d;
        }

        private static boolean finite(double v) {
            return !Double.isNaN(v) && !Double.isInfinite(v);
        }
    }

    /**
     * beacon_create. {@code fieldJson} is the field model file CONTENTS.
     * @return false (and a message in err[0]) if the core could not be created
     */
    boolean create(String fieldJson, Camera camera, BeaconConfig.CameraMount mount,
                   String paramsJson, String[] err);

    /** beacon_set_start_pose: field-frame robot pose at time t. */
    void setStartPose(long tNs, double x, double y, double heading);

    /** beacon_push_odom: one Pinpoint sample in the odometry world. */
    void pushOdom(long tNs, double x, double y, double heading);

    /** beacon_push_frame: OK, DROPPED (core busy) or ERR (bad size). Copies; never blocks. */
    int pushFrame(long captureTNs, byte[] gray, int width, int height, int stride);

    /**
     * beacon_latest_fix. Fills t[0] and v = {x, y, heading, sigmaXY, sigmaHeading, inliers,
     * kind}. @return true if this fix is new since the last call
     */
    boolean latestFix(long[] t, double[] v);

    /** beacon_status: a BeaconStatus code. */
    int status();

    /** beacon_debug_line. */
    String debugLine();

    /** beacon_destroy. Called once, after every other call has returned. */
    void destroy();
}
