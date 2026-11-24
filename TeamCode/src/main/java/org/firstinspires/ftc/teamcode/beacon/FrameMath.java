package org.firstinspires.ftc.teamcode.beacon;

/**
 * Small hand-written quaternion and rigid-transform library shared by the localizer and every
 * pose source. Plain doubles and arrays; callers pass the output array, and every method is
 * safe when the output aliases an input. Nothing allocates except {@link #robotFromCamera} and
 * {@link #identity} (configuration-time helpers).
 *
 * <h3>Conventions</h3>
 * <ul>
 *   <li>Quaternion: {@code double[4] = {x, y, z, w}}, Hamilton product, unit length. Rotates
 *       vectors as {@code v' = q v q^-1}.</li>
 *   <li>Rotation matrix: {@code double[9]}, row-major.</li>
 *   <li>Rigid transform ("pose"): {@code double[7] = {x, y, z, qx, qy, qz, qw}}. A transform
 *       named {@code T_a_b} is the pose of frame b expressed in frame a; it maps a point given
 *       in b to a: {@code p_a = R_a_b * p_b + t_a_b}. Chain with
 *       {@code T_a_c = compose(T_a_b, T_b_c)}.</li>
 *   <li>Angles in radians. Yaw is counter-clockwise positive seen from above (+z up).</li>
 * </ul>
 *
 * <h3>Frames used by Beacon</h3>
 * <ul>
 *   <li>Robot: +x forward, +y left, +z up, origin at the centre of rotation.</li>
 *   <li>Camera optical: +x right in the image, +y down in the image, +z forward out of the
 *       lens (OpenCV convention).</li>
 *   <li>Source world: z-up, gravity-aligned, arbitrary origin and yaw.</li>
 * </ul>
 */
public final class FrameMath {

    private static final double TWO_PI = 2.0 * Math.PI;

    /** Rotation taking optical-frame vectors to robot-frame vectors when the camera looks
     *  straight forward and upright (the zero of roll/pitch/yaw). Row-major.
     *  optical x (right) -> robot -y; optical y (down) -> robot -z; optical z (fwd) -> robot +x. */
    private static final double[] M_ROBOT_FROM_OPTICAL_FORWARD = {
            0, 0, 1,
            -1, 0, 0,
            0, -1, 0};
    private static final double[] Q_ROBOT_FROM_OPTICAL_FORWARD = new double[4];

    /** ARCore world (y up) to z-up world: rotation about x by +90 degrees, (x,y,z)->(x,-z,y). */
    private static final double S = Math.sqrt(0.5);
    private static final double[] Q_ZUP_FROM_YUP = {S, 0, 0, S};
    /** ARCore/OpenGL camera body (x right, y up, z back) from optical: diag(1,-1,-1) =
     *  rotation about x by 180 degrees. */
    private static final double[] Q_GL_FROM_OPTICAL = {1, 0, 0, 0};
    private static final double[] T_ZUP_FROM_YUP = {0, 0, 0, S, 0, 0, S};
    private static final double[] T_GL_FROM_OPTICAL = {0, 0, 0, 1, 0, 0, 0};

    static {
        matrixToQuat(M_ROBOT_FROM_OPTICAL_FORWARD, Q_ROBOT_FROM_OPTICAL_FORWARD);
    }

    private FrameMath() {}

    // ------------------------------------------------------------------ angles

    /** Wraps an angle to [-pi, pi). */
    public static double wrapAngle(double a) {
        if (a >= -Math.PI && a < Math.PI) {
            return a;
        }
        return a - TWO_PI * Math.floor((a + Math.PI) / TWO_PI);
    }

    /** Shortest signed angular difference {@code a - b} in [-pi, pi). */
    public static double angleDiff(double a, double b) {
        return wrapAngle(a - b);
    }

    public static double degToRad(double deg) {
        return Math.toRadians(deg);
    }

    public static double inchesToMetres(double in) {
        return in * 0.0254;
    }

    // -------------------------------------------------------------- quaternions

    /** out = a * b (apply b first, then a). */
    public static void quatMultiply(double[] a, double[] b, double[] out) {
        double ax = a[0], ay = a[1], az = a[2], aw = a[3];
        double bx = b[0], by = b[1], bz = b[2], bw = b[3];
        out[0] = aw * bx + ax * bw + ay * bz - az * by;
        out[1] = aw * by - ax * bz + ay * bw + az * bx;
        out[2] = aw * bz + ax * by - ay * bx + az * bw;
        out[3] = aw * bw - ax * bx - ay * by - az * bz;
    }

    public static void quatConjugate(double[] q, double[] out) {
        out[0] = -q[0];
        out[1] = -q[1];
        out[2] = -q[2];
        out[3] = q[3];
    }

    /**
     * Normalizes in place.
     * @return false (and leaves q untouched) if the quaternion is non-finite or has near-zero
     *         length.
     */
    public static boolean quatNormalize(double[] q) {
        double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
        if (!(n2 > 1e-12) || Double.isInfinite(n2)) {
            return false;
        }
        double inv = 1.0 / Math.sqrt(n2);
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
        return true;
    }

    /** Quaternion for rotation of {@code angle} radians about the unit axis (ax, ay, az). */
    public static void quatFromAxisAngle(double ax, double ay, double az, double angle,
                                         double[] out) {
        double h = 0.5 * angle;
        double s = Math.sin(h);
        out[0] = ax * s;
        out[1] = ay * s;
        out[2] = az * s;
        out[3] = Math.cos(h);
    }

    /**
     * Quaternion for roll, pitch, yaw about FIXED axes, applied roll first, then pitch, then
     * yaw: {@code R = Rz(yaw) * Ry(pitch) * Rx(roll)}. Same thing as intrinsic yaw, then
     * pitch, then roll (aerospace Z-Y'-X''). Axes are x forward, y left, z up, so positive
     * pitch tips the nose DOWN and positive yaw turns LEFT.
     */
    public static void quatFromRpy(double roll, double pitch, double yaw, double[] out) {
        double cr = Math.cos(0.5 * roll), sr = Math.sin(0.5 * roll);
        double cp = Math.cos(0.5 * pitch), sp = Math.sin(0.5 * pitch);
        double cy = Math.cos(0.5 * yaw), sy = Math.sin(0.5 * yaw);
        out[0] = sr * cp * cy - cr * sp * sy;
        out[1] = cr * sp * cy + sr * cp * sy;
        out[2] = cr * cp * sy - sr * sp * cy;
        out[3] = cr * cp * cy + sr * sp * sy;
    }

    /** Rotates the vector (x, y, z) by unit quaternion q into out[0..2]. */
    public static void quatRotate(double[] q, double x, double y, double z, double[] out) {
        double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
        // t = 2 * cross(q.xyz, v); v' = v + w t + cross(q.xyz, t)
        double tx = 2.0 * (qy * z - qz * y);
        double ty = 2.0 * (qz * x - qx * z);
        double tz = 2.0 * (qx * y - qy * x);
        out[0] = x + qw * tx + (qy * tz - qz * ty);
        out[1] = y + qw * ty + (qz * tx - qx * tz);
        out[2] = z + qw * tz + (qx * ty - qy * tx);
    }

    /** Row-major 3x3 rotation matrix of unit quaternion q into m[0..8]. */
    public static void quatToMatrix(double[] q, double[] m) {
        quatToMatrix(q[0], q[1], q[2], q[3], m);
    }

    public static void quatToMatrix(double x, double y, double z, double w, double[] m) {
        m[0] = 1 - 2 * (y * y + z * z);
        m[1] = 2 * (x * y - w * z);
        m[2] = 2 * (x * z + w * y);
        m[3] = 2 * (x * y + w * z);
        m[4] = 1 - 2 * (x * x + z * z);
        m[5] = 2 * (y * z - w * x);
        m[6] = 2 * (x * z - w * y);
        m[7] = 2 * (y * z + w * x);
        m[8] = 1 - 2 * (x * x + y * y);
    }

    /** Converts a row-major rotation matrix to a unit quaternion (w >= 0). */
    public static void matrixToQuat(double[] m, double[] out) {
        double tr = m[0] + m[4] + m[8];
        double x, y, z, w;
        if (tr > 0) {
            double s = Math.sqrt(tr + 1.0) * 2;
            w = 0.25 * s;
            x = (m[7] - m[5]) / s;
            y = (m[2] - m[6]) / s;
            z = (m[3] - m[1]) / s;
        } else if (m[0] > m[4] && m[0] > m[8]) {
            double s = Math.sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
            w = (m[7] - m[5]) / s;
            x = 0.25 * s;
            y = (m[1] + m[3]) / s;
            z = (m[2] + m[6]) / s;
        } else if (m[4] > m[8]) {
            double s = Math.sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
            w = (m[2] - m[6]) / s;
            x = (m[1] + m[3]) / s;
            y = 0.25 * s;
            z = (m[5] + m[7]) / s;
        } else {
            double s = Math.sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
            w = (m[3] - m[1]) / s;
            x = (m[2] + m[6]) / s;
            y = (m[5] + m[7]) / s;
            z = 0.25 * s;
        }
        if (w < 0) {
            x = -x;
            y = -y;
            z = -z;
            w = -w;
        }
        out[0] = x;
        out[1] = y;
        out[2] = z;
        out[3] = w;
        quatNormalize(out);
    }

    /**
     * Heading (yaw) of a body whose orientation is the given unit quaternion: the angle of the
     * body's +x axis projected onto the horizontal plane, CCW positive, in (-pi, pi]. Works
     * with roll and pitch present, unlike Euler extraction, and is undefined (returns 0) only
     * if the +x axis is exactly vertical.
     */
    public static double yawFromQuat(double qx, double qy, double qz, double qw) {
        double r10 = 2.0 * (qx * qy + qw * qz);
        double r00 = 1.0 - 2.0 * (qy * qy + qz * qz);
        return Math.atan2(r10, r00);
    }

    // ---------------------------------------------------- rigid transforms (7)

    /** out = a * b, i.e. T_a_c from T_a_b and T_b_c. Alias-safe, no allocation. */
    public static void compose(double[] a, double[] b, double[] out) {
        double ax = a[0], ay = a[1], az = a[2];
        double qx = a[3], qy = a[4], qz = a[5], qw = a[6];
        double bx = b[0], by = b[1], bz = b[2];
        double rx = b[3], ry = b[4], rz = b[5], rw = b[6];
        // rotate b's translation by qa: t = 2 q x v; v' = v + w t + q x t
        double tx = 2.0 * (qy * bz - qz * by);
        double ty = 2.0 * (qz * bx - qx * bz);
        double tz = 2.0 * (qx * by - qy * bx);
        out[0] = ax + bx + qw * tx + (qy * tz - qz * ty);
        out[1] = ay + by + qw * ty + (qz * tx - qx * tz);
        out[2] = az + bz + qw * tz + (qx * ty - qy * tx);
        out[3] = qw * rx + qx * rw + qy * rz - qz * ry;
        out[4] = qw * ry - qx * rz + qy * rw + qz * rx;
        out[5] = qw * rz + qx * ry - qy * rx + qz * rw;
        out[6] = qw * rw - qx * rx - qy * ry - qz * rz;
    }

    /** out = a^-1, i.e. T_b_a from T_a_b. Alias-safe, no allocation. */
    public static void invert(double[] a, double[] out) {
        double px = a[0], py = a[1], pz = a[2];
        double qx = -a[3], qy = -a[4], qz = -a[5], qw = a[6];
        double tx = 2.0 * (qy * pz - qz * py);
        double ty = 2.0 * (qz * px - qx * pz);
        double tz = 2.0 * (qx * py - qy * px);
        out[0] = -(px + qw * tx + (qy * tz - qz * ty));
        out[1] = -(py + qw * ty + (qz * tx - qx * tz));
        out[2] = -(pz + qw * tz + (qx * ty - qy * tx));
        out[3] = qx;
        out[4] = qy;
        out[5] = qz;
        out[6] = qw;
    }

    /** Maps point (x, y, z) given in frame b to frame a, where t = T_a_b. No allocation. */
    public static void transformPoint(double[] t, double x, double y, double z, double[] out) {
        double qx = t[3], qy = t[4], qz = t[5], qw = t[6];
        double tx = 2.0 * (qy * z - qz * y);
        double ty = 2.0 * (qz * x - qx * z);
        double tz = 2.0 * (qx * y - qy * x);
        double ox = t[0], oy = t[1], oz = t[2];
        out[0] = x + qw * tx + (qy * tz - qz * ty) + ox;
        out[1] = y + qw * ty + (qz * tx - qx * tz) + oy;
        out[2] = z + qw * tz + (qx * ty - qy * tx) + oz;
    }

    /** Identity transform. */
    public static double[] identity() {
        return new double[]{0, 0, 0, 0, 0, 0, 1};
    }

    // -------------------------------------------------- camera mount (T_robot_cam)

    /**
     * Builds T_robot_cam, the pose of the camera OPTICAL frame in the ROBOT frame, from the
     * numbers in beacon.json. The angles are defined against a "camera looking straight
     * forward, upright" baseline (see {@link BeaconConfig}):
     * {@code R_robot_cam = Rz(yaw) * Ry(pitch) * Rx(roll) * R_forward}, where
     * {@code R_forward} turns optical axes into robot axes for a camera looking along robot +x
     * with the image upright. All zeros therefore means "looking straight ahead, level".
     * Allocates two small arrays; called once at configuration time, never per sample.
     */
    public static void robotFromCamera(double x, double y, double z,
                                       double roll, double pitch, double yaw, double[] out7) {
        double[] qa = new double[4];
        quatFromRpy(roll, pitch, yaw, qa);
        double[] q = new double[4];
        quatMultiply(qa, Q_ROBOT_FROM_OPTICAL_FORWARD, q);
        out7[0] = x;
        out7[1] = y;
        out7[2] = z;
        out7[3] = q[0];
        out7[4] = q[1];
        out7[5] = q[2];
        out7[6] = q[3];
    }

    // ------------------------------------------------ helpers for pose sources

    /**
     * Converts an ARCore camera pose (world: y-up, camera body: OpenGL x right, y up, z
     * back) into the Beacon frame contract (world: z-up, camera body: optical x right, y
     * down, z forward). Result = C * T_ar * B, with C a world-side rotation of +90 degrees
     * about x and B a body-side rotation of 180 degrees about x. Alias-safe.
     *
     * @param arPose {x, y, z, qx, qy, qz, qw} from {@code Camera.getPose()}
     */
    public static void fromArCoreCameraPose(double[] arPose, double[] out7) {
        compose(T_ZUP_FROM_YUP, arPose, out7);
        compose(out7, T_GL_FROM_OPTICAL, out7);
    }

    /**
     * Applies a fixed rotation on the WORLD side of a pose: out = R * pose (rotates both the
     * position and the orientation about the world origin). Use it to re-express a pose in a
     * differently oriented world. Alias-safe.
     */
    public static void rotateWorld(double[] worldRotQuat, double[] pose7, double[] out7) {
        double qx = worldRotQuat[0], qy = worldRotQuat[1], qz = worldRotQuat[2];
        double qw = worldRotQuat[3];
        double bx = pose7[0], by = pose7[1], bz = pose7[2];
        double rx = pose7[3], ry = pose7[4], rz = pose7[5], rw = pose7[6];
        double tx = 2.0 * (qy * bz - qz * by);
        double ty = 2.0 * (qz * bx - qx * bz);
        double tz = 2.0 * (qx * by - qy * bx);
        out7[0] = bx + qw * tx + (qy * tz - qz * ty);
        out7[1] = by + qw * ty + (qz * tx - qx * tz);
        out7[2] = bz + qw * tz + (qx * ty - qy * tx);
        out7[3] = qw * rx + qx * rw + qy * rz - qz * ry;
        out7[4] = qw * ry - qx * rz + qy * rw + qz * rx;
        out7[5] = qw * rz + qx * ry - qy * rx + qz * rw;
        out7[6] = qw * rw - qx * rx - qy * ry - qz * rz;
    }

    /**
     * Applies a fixed rotation on the BODY side of a pose: out = pose * R. Use it to change
     * which body axes the pose describes (for example IMU axes to camera axes). Alias-safe.
     */
    public static void rotateBody(double[] pose7, double[] bodyRotQuat, double[] out7) {
        double qx = pose7[3], qy = pose7[4], qz = pose7[5], qw = pose7[6];
        double rx = bodyRotQuat[0], ry = bodyRotQuat[1], rz = bodyRotQuat[2];
        double rw = bodyRotQuat[3];
        out7[0] = pose7[0];
        out7[1] = pose7[1];
        out7[2] = pose7[2];
        out7[3] = qw * rx + qx * rw + qy * rz - qz * ry;
        out7[4] = qw * ry - qx * rz + qy * rw + qz * rx;
        out7[5] = qw * rz + qx * ry - qy * rx + qz * rw;
        out7[6] = qw * rw - qx * rx - qy * ry - qz * rz;
    }

    /** Planar rotation of (x, y) by angle into out[0..1]. */
    public static void rotate2d(double x, double y, double angle, double[] out) {
        double c = Math.cos(angle), s = Math.sin(angle);
        double nx = c * x - s * y;
        double ny = s * x + c * y;
        out[0] = nx;
        out[1] = ny;
    }
}
