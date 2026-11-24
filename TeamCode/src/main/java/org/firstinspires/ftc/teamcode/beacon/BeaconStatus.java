package org.firstinspires.ftc.teamcode.beacon;

/**
 * Status codes carried by {@link Pose2dStamped} and {@link Pose3dStamped}, and written to the
 * last column of the pose CSV.
 */
public final class BeaconStatus {
    /** No usable pose (never started, stopped, or the source reported garbage). */
    public static final int INVALID = 0;
    /** Source is warming up, or the field alignment has not been set yet. */
    public static final int INITIALIZING = 1;
    /** Pose is valid. */
    public static final int TRACKING = 2;
    /** Tracking was lost or is relocalizing; the last good pose is being held. */
    public static final int LOST = 3;

    private BeaconStatus() {}

    /** Short name for telemetry. */
    public static String name(int status) {
        switch (status) {
            case INVALID: return "INVALID";
            case INITIALIZING: return "INITIALIZING";
            case TRACKING: return "TRACKING";
            case LOST: return "LOST";
            default: return "STATUS_" + status;
        }
    }
}
