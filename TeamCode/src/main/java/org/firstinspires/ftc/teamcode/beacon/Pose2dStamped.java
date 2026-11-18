package org.firstinspires.ftc.teamcode.beacon;

/**
 * Immutable planar pose: metres and radians, heading counter-clockwise positive, wrapped to
 * [-pi, pi) when produced by {@link BeaconLocalizer}. {@code tNs} is on the Android monotonic
 * clock ({@code SystemClock.elapsedRealtimeNanos()}). {@code status} is a {@link BeaconStatus}.
 */
public final class Pose2dStamped {
    public final double x;
    public final double y;
    public final double heading;
    public final long tNs;
    public final int status;

    public Pose2dStamped(double x, double y, double heading, long tNs, int status) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tNs = tNs;
        this.status = status;
    }

    /** True if x, y and heading are all finite numbers. */
    public boolean isFinite() {
        return !Double.isNaN(x) && !Double.isInfinite(x)
                && !Double.isNaN(y) && !Double.isInfinite(y)
                && !Double.isNaN(heading) && !Double.isInfinite(heading);
    }

    @Override
    public String toString() {
        return "Pose2d(" + x + ", " + y + ", " + heading + " rad, t=" + tNs + ", "
                + BeaconStatus.name(status) + ")";
    }
}
