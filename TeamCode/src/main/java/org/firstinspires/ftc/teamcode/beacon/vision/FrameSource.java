package org.firstinspires.ftc.teamcode.beacon.vision;

/**
 * A camera that delivers grayscale frames with capture timestamps, on its own threads.
 * The robot implementation is {@link LimelightFrameSource}.
 *
 * <p>Contract:
 * <ul>
 *   <li>{@link #start()} returns at once; connecting and decoding happen on background
 *       threads. {@link #stop()} is idempotent, safe before start, and returns promptly even if
 *       the device is hung.</li>
 *   <li>{@link #latest(GrayFrame)} never blocks for longer than one frame copy and never
 *       throws. It copies the newest frame into the caller's container only if it is newer
 *       than what the container holds ({@code out.seq}).</li>
 *   <li>Frame capture times ({@link GrayFrame#tNs}) are strictly increasing.</li>
 *   <li>A missing or unplugged device is a state ({@link #isStreaming()} false, explained in
 *       {@link #debugLine()}), never an exception; the source keeps retrying until stopped.</li>
 * </ul>
 */
public interface FrameSource {

    /**
     * Called on the source's decode thread for every decoded frame, in order. It must be quick
     * (copy what you need; never block or do file I/O) and must not keep a reference to
     * {@code frame}, which is reused. An exception thrown here is counted and ignored.
     */
    interface Listener {
        void onFrame(GrayFrame frame);
    }

    void start();

    void stop();

    /** Copies the newest frame into {@code out} if newer than {@code out.seq}; true if it did. */
    boolean latest(GrayFrame out);

    /** Sequence number of the newest decoded frame (0 = none yet). A volatile read. */
    long latestSeq();

    /** Sets (or clears, with null) the per-frame listener. */
    void setListener(Listener listener);

    /** True while frames are arriving. */
    boolean isStreaming();

    /** One line for telemetry. */
    String debugLine();
}
