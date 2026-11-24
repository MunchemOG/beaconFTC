package org.firstinspires.ftc.teamcode.beacon;

/**
 * A source of absolute fixes in the FIELD frame. {@link BeaconLocalizer} depends on this
 * interface, not on any camera classes, so it can be tested on a laptop. The robot
 * implementation is {@code vision.VisionFixSource} (Beacon's native vision core). (Plan
 * interface I4: this was {@code TagFixSource}.)
 *
 * <p>Contract:
 * <ul>
 *   <li>{@link #latestFix()} never blocks and may be called every loop (a volatile read).</li>
 *   <li>The fix is the ROBOT's centre-of-rotation pose in the field frame (metres, radians,
 *       CCW positive). Only the parts flagged present ({@link FieldFix#hasPosition},
 *       {@link FieldFix#hasHeading}) are used. The source does its own quality gating; the
 *       localizer adds the jump test and the robot speed / rotation-rate gate at the fix time.</li>
 *   <li>{@code tNs} is the capture time of the image, on the same clock as the pose source
 *       ({@code SystemClock.elapsedRealtimeNanos()}).</li>
 *   <li>Freshness: a fix is new only if its {@code tNs} is strictly greater than the last fix
 *       the localizer consumed. Keep returning the same object until a new fix arrives.</li>
 *   <li>Return {@code null} if there has never been a fix. Throwing is tolerated (the localizer
 *       counts it as a rejected fix) but is a bug.</li>
 * </ul>
 */
public interface FieldFixSource {
    FieldFix latestFix();
}
