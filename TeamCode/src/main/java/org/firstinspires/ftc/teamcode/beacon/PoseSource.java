package org.firstinspires.ftc.teamcode.beacon;

import android.content.Context;

/**
 * A producer of timestamped poses (goBILDA Pinpoint odometry, a camera VIO, or a fake for
 * tests).
 *
 * <h3>Frame contract (every implementation must meet this)</h3>
 * Each {@link Pose3dStamped} returned by {@link #latest()} is a pose in a <b>z-up,
 * gravity-aligned world frame</b> with arbitrary origin and yaw (wherever the source started;
 * the localizer learns the field alignment from {@code setPose} or a field fix). Position in
 * metres, unit Hamilton quaternion (x y z w), timestamps on
 * {@code SystemClock.elapsedRealtimeNanos()}, strictly increasing.
 * <p>Which BODY the pose describes depends on {@link #reportsRobotPose()}:
 * <ul>
 *   <li>{@code true} (odometry sources such as {@link PinpointPoseSource}): the robot itself,
 *       +x forward, +y left, +z up, origin at the centre of rotation. {@code T_robot_cam} in
 *       beacon.json is ignored for such a source.</li>
 *   <li>{@code false} (camera sources): the camera <b>optical frame</b> (x right in the
 *       image, y down, z out of the lens). The localizer converts it to the robot with the
 *       fixed {@code T_robot_cam} from beacon.json (see {@link BeaconConfig}).</li>
 * </ul>
 * If the world frame of a source jumps or restarts, it must increment
 * {@link Pose3dStamped#worldEpoch}.
 *
 * <h3>Status</h3>
 * Publish {@link BeaconStatus#TRACKING} only for poses that can be trusted. While
 * initializing or lost, publish INITIALIZING or LOST (the numeric pose may then be anything;
 * the localizer ignores it). A source must never throw from {@link #latest()} and should not
 * publish NaN; if it does, the localizer holds the last good pose and reports LOST.
 *
 * <h3>Threading and cost</h3>
 * {@link BeaconLocalizer#update()} calls {@link #poll()} once, then {@link #latest()} once,
 * on the OpMode thread. A source that has to read a device (the Pinpoint's single I2C bulk
 * read) does it in {@code poll()}, so there is exactly one device read per loop. Sources with
 * their own threads leave {@code poll()} empty and publish through a volatile reference.
 * {@code latest()} is a plain volatile read: no I/O, no locks, never blocks.
 */
public interface PoseSource {
    /**
     * Starts producing poses. May do slow work but must not block indefinitely. {@code ctx}
     * may be null; a source that has no use for it must not touch it.
     *
     * @throws BeaconException if the source cannot start; the message is shown to the student.
     */
    void start(Context ctx, BeaconConfig cfg) throws BeaconException;

    /** Stops producing poses and releases resources. Safe to call twice, or before start. */
    void stop();

    /**
     * Reads the device, if this source needs to, and publishes a new sample. Called once per
     * {@link BeaconLocalizer#update()} before {@link #latest()}. Must not throw and must
     * return within a few milliseconds. Default: nothing to do.
     */
    default void poll() {
    }

    /** Newest pose, in the frame contract above. Non-blocking; never null after start. */
    Pose3dStamped latest();

    /** True if the most recent pose has status TRACKING. */
    boolean isTracking();

    /** One short line for telemetry. */
    String debugLine();

    /**
     * True if {@link #latest()} is the pose of the ROBOT (centre of rotation), false if it is
     * the camera optical frame. Default false (camera).
     */
    default boolean reportsRobotPose() {
        return false;
    }
}
