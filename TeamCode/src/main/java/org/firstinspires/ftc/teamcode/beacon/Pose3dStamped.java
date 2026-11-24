package org.firstinspires.ftc.teamcode.beacon;

/**
 * Immutable 3-D pose emitted by a {@link PoseSource}: position in metres, orientation as a unit
 * quaternion (Hamilton convention, x y z w), {@code tNs} on the Android monotonic clock,
 * {@code status} a {@link BeaconStatus}. What the pose means (which frame, which body) is the
 * frame contract documented on {@link PoseSource}.
 *
 * <p>{@code worldEpoch} numbers the source's world frame. A source increments it whenever its
 * world frame stops being continuous with the previous one (the odometry computer rebooted and
 * restarted at 0,0,0; a visual source re-initialized; a sudden implausible jump). The
 * localizer then continues its output from where it was instead of applying the old field
 * alignment to an unrelated world, and opens a short window in which the next field fix is
 * trusted fully. Sources that never reset their world can leave it at 0.
 */
public final class Pose3dStamped {
    public final double x;
    public final double y;
    public final double z;
    public final double qx;
    public final double qy;
    public final double qz;
    public final double qw;
    public final long tNs;
    public final int status;
    public final int worldEpoch;

    public Pose3dStamped(double x, double y, double z,
                         double qx, double qy, double qz, double qw,
                         long tNs, int status) {
        this(x, y, z, qx, qy, qz, qw, tNs, status, 0);
    }

    public Pose3dStamped(double x, double y, double z,
                         double qx, double qy, double qz, double qw,
                         long tNs, int status, int worldEpoch) {
        this.x = x;
        this.y = y;
        this.z = z;
        this.qx = qx;
        this.qy = qy;
        this.qz = qz;
        this.qw = qw;
        this.tNs = tNs;
        this.status = status;
        this.worldEpoch = worldEpoch;
    }

    /** True if all seven pose numbers are finite. */
    public boolean isFinite() {
        return fin(x) && fin(y) && fin(z) && fin(qx) && fin(qy) && fin(qz) && fin(qw);
    }

    private static boolean fin(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    @Override
    public String toString() {
        return "Pose3d(" + x + ", " + y + ", " + z + " | " + qx + ", " + qy + ", " + qz + ", "
                + qw + " | t=" + tNs + ", " + BeaconStatus.name(status) + ", epoch "
                + worldEpoch + ")";
    }
}
