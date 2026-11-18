package org.firstinspires.ftc.teamcode.beacon.parked;

import org.firstinspires.ftc.teamcode.beacon.FieldFix;
import org.firstinspires.ftc.teamcode.beacon.FieldFixSource;
import org.firstinspires.ftc.teamcode.beacon.FrameMath;
import org.firstinspires.ftc.teamcode.beacon.PinpointPoseSource;

/**
 * PARKED (2026-11-17): AprilTag code from the old odometry tier. Beacon uses no AprilTags; this
 * class is not wired into BeaconRig and is kept compiling and tested only so it can be revived.
 *
 * Decides whether one Limelight 3A result becomes a {@link FieldFix}, and which parts of it.
 * Plain Java (no SDK types) so every rule is unit-tested; {@link LimelightTagSource} feeds it
 * from the SDK's {@code LLResult} on a background thread.
 *
 * <h3>MegaTag2 vs MegaTag1, and the heading loop</h3>
 * MegaTag2 ({@code getBotpose_MT2}) solves the robot POSITION from the tags given the robot
 * heading we send it ({@code updateRobotOrientation}). Its heading output is our own heading
 * echoed back, so it carries no heading information, and using it to correct heading would
 * be a loop correcting itself. Its position is good (robust at range, no pose ambiguity) as
 * long as the heading we sent was right at the moment the image was processed: a heading
 * error of e radians at range d moves the MegaTag2 position by about d * e (2 m and 1 degree
 * give 3.5 cm). So:
 * <ul>
 *   <li>MegaTag2 corrects POSITION only, and only if our heading was sent successfully within
 *       {@code headingFreshMs} of the capture time, and the localizer is TRACKING (otherwise
 *       the heading we would send is not real). Turning fast makes the sent heading wrong for
 *       the image, which is why the localizer also ignores fixes taken while turning faster
 *       than {@code tag.maxOmegaRadPs}.</li>
 *   <li>MegaTag1 ({@code getBotpose}) is a full independent solve from the tags alone. It is
 *       the only independent HEADING reference, so it corrects heading (slowly, through
 *       {@code tag.gainHeading}) when at least {@code mt1MinTags} tags are in view within
 *       {@code mt1MaxRangeM}; with one small tag its heading is too noisy to use.</li>
 *   <li>When the localizer is not TRACKING (no trusted heading to send), only a MegaTag1 fix
 *       with both position and heading is produced: that is the rescue path that can set the
 *       alignment from scratch.</li>
 * </ul>
 *
 * <h3>Timing</h3>
 * From the SDK source (LLResult): {@code getStaleness()} is the age in ms of the moment the
 * Control Hub parsed the result ({@code System.currentTimeMillis()} at parse), and
 * {@code getCaptureLatency()} plus {@code getTargetingLatency()} is, in ms, how long before
 * publishing the Limelight captured the image. So the capture time on our clock is
 * {@code now - (staleness + captureLatency + targetingLatency)} ms. What remains unmeasured is
 * the HTTP transfer and the wait for the SDK's 10 ms poll, so the stamp is late by at most
 * about one poll interval; the speed gate keeps that small.
 *
 * <h3>Other filters</h3>
 * The result must be valid, from the configured pipeline, younger than
 * {@code maxStalenessMs} (an unplugged Limelight leaves its last result cached forever),
 * with a botpose that is not all zeros (the SDK returns zeros when there is no solve), finite,
 * and inside the field ({@code |x|, |y| <= fieldHalfM}). The SDK re-fetches the same camera
 * frame several times (it polls at 100 Hz); repeats are recognized by the Limelight's own
 * timestamp and pose and are not turned into extra fixes.
 *
 * <p>Single-threaded: one instance belongs to one thread.
 */
public final class LimelightFixFilter {

    /** One Limelight result reduced to plain numbers. Filled by the SDK adapter. */
    public static final class Reading {
        public boolean valid;
        public int pipelineIndex;
        /** Limelight-local timestamp of the result (identifies the frame). */
        public double llTimestamp;
        /** Age of the Control Hub's copy when read, ms ({@code LLResult.getStaleness}). */
        public double stalenessMs;
        public double captureLatencyMs;
        public double targetingLatencyMs;
        public int tagCount;
        public double avgDistM;
        public boolean hasMt2;
        public double mt2X, mt2Y, mt2HeadingRad;
        public boolean hasMt1;
        public double mt1X, mt1Y, mt1HeadingRad;
    }

    private final LimelightTagConfig cfg;

    private boolean haveLast;
    private double lastLlTs, lastMt1X, lastMt1Y, lastMt2X, lastMt2Y;
    private long lastFixTNs = Long.MIN_VALUE;

    private String lastReason = "no result yet";
    private long produced, rejected;
    private double lastLatencyMs;

    public LimelightFixFilter(LimelightTagConfig cfg) {
        this.cfg = cfg;
    }

    /**
     * @param r                 the newest result, or null if there is none
     * @param nowNs             now, on the pose source clock (elapsedRealtimeNanos)
     * @param localizerTracking true if the localizer is TRACKING (its heading is being sent)
     * @param lastHeadingOkNs   when the last heading send succeeded (same clock), or
     *                          Long.MIN_VALUE if never
     * @return a new fix, or null (then {@link #lastReason()} says why)
     */
    public FieldFix evaluate(Reading r, long nowNs, boolean localizerTracking,
                           long lastHeadingOkNs) {
        if (r == null) {
            lastReason = "no result from the Limelight yet";
            return null;
        }
        if (r.stalenessMs < 0 || r.stalenessMs > cfg.maxStalenessMs
                || Double.isNaN(r.stalenessMs)) {
            lastReason = String.format("result is %.0f ms old: Limelight unplugged or stopped?",
                    r.stalenessMs);
            return null;
        }
        if (r.pipelineIndex != cfg.pipeline) {
            lastReason = "Limelight is on pipeline " + r.pipelineIndex + ", expected "
                    + cfg.pipeline;
            return null;
        }
        if (!r.valid) {
            lastReason = "no tags in view";
            return null;
        }
        if (haveLast && r.llTimestamp == lastLlTs && r.mt1X == lastMt1X && r.mt1Y == lastMt1Y
                && r.mt2X == lastMt2X && r.mt2Y == lastMt2Y) {
            return null;    // same camera frame fetched again; not a new measurement
        }
        haveLast = true;
        lastLlTs = r.llTimestamp;
        lastMt1X = r.mt1X;
        lastMt1Y = r.mt1Y;
        lastMt2X = r.mt2X;
        lastMt2Y = r.mt2Y;

        double lat = r.stalenessMs + r.captureLatencyMs + r.targetingLatencyMs;
        if (!fin(lat) || r.captureLatencyMs < 0 || r.targetingLatencyMs < 0
                || r.captureLatencyMs > 1000 || r.targetingLatencyMs > 1000) {
            return reject("latency numbers are not sane");
        }
        lastLatencyMs = lat;
        long tCap = nowNs - (long) (lat * 1e6);

        String why1 = mt1Problem(r);
        boolean mt1Ok = why1 == null;
        String why2 = mt2Problem(r, tCap, localizerTracking, lastHeadingOkNs);
        boolean mt2Ok = why2 == null;

        boolean hasPos, hasHead;
        double x, y, h;
        String kind;
        if (!localizerTracking) {
            if (!mt1Ok) {
                return reject("localizer not tracking and MegaTag1 unusable: " + why1);
            }
            hasPos = true;
            hasHead = true;
            x = r.mt1X;
            y = r.mt1Y;
            h = r.mt1HeadingRad;
            kind = "MT1";
        } else {
            boolean useHead = mt1Ok && cfg.useMt1Heading;
            if (mt2Ok) {
                hasPos = true;
                x = r.mt2X;
                y = r.mt2Y;
                kind = useHead ? "MT2+MT1h" : "MT2";
            } else if (mt1Ok) {
                hasPos = true;
                x = r.mt1X;
                y = r.mt1Y;
                kind = useHead ? "MT1" : "MT1pos";
            } else {
                return reject("MegaTag2: " + why2 + "; MegaTag1: " + why1);
            }
            hasHead = useHead;
            h = useHead ? r.mt1HeadingRad : 0;
        }
        if (tCap <= lastFixTNs) {
            tCap = lastFixTNs + 1;     // contract: strictly increasing capture times
        }
        lastFixTNs = tCap;
        produced++;
        lastReason = "ok (" + kind + ")";
        return new FieldFix(x, y, FrameMath.wrapAngle(h), tCap, hasPos, hasHead, kind);
    }

    /** Why MegaTag1 cannot be used, or null if it can. */
    private String mt1Problem(Reading r) {
        if (!r.hasMt1) {
            return "no MT1 pose";
        }
        String p = poseProblem(r.mt1X, r.mt1Y, r.mt1HeadingRad);
        if (p != null) {
            return p;
        }
        if (r.tagCount < cfg.mt1MinTags) {
            return r.tagCount + " tag(s), need " + cfg.mt1MinTags;
        }
        if (!(r.avgDistM <= cfg.mt1MaxRangeM)) {
            return String.format("tags %.2f m away, max %.2f", r.avgDistM, cfg.mt1MaxRangeM);
        }
        return null;
    }

    /** Why MegaTag2 cannot be used, or null if it can. */
    private String mt2Problem(Reading r, long tCap, boolean tracking, long lastHeadingOkNs) {
        if (!tracking) {
            return "localizer not tracking, so no heading was sent";
        }
        if (!r.hasMt2) {
            return "no MT2 pose";
        }
        if (lastHeadingOkNs == Long.MIN_VALUE
                || lastHeadingOkNs < tCap - (long) (cfg.headingFreshMs * 1e6)) {
            return "our heading did not reach the Limelight in time";
        }
        String p = poseProblem(r.mt2X, r.mt2Y, r.mt2HeadingRad);
        if (p != null) {
            return p;
        }
        if (r.tagCount < cfg.minTags) {
            return r.tagCount + " tag(s), need " + cfg.minTags;
        }
        if (!(r.avgDistM <= cfg.maxRangeM)) {
            return String.format("tags %.2f m away, max %.2f", r.avgDistM, cfg.maxRangeM);
        }
        return null;
    }

    private String poseProblem(double x, double y, double h) {
        if (!fin(x) || !fin(y) || !fin(h)) {
            return "pose not finite";
        }
        if (Math.abs(x) < 1e-9 && Math.abs(y) < 1e-9) {
            return "no solve (pose is all zeros)";
        }
        if (Math.abs(x) > cfg.fieldHalfM || Math.abs(y) > cfg.fieldHalfM) {
            return String.format("pose (%.2f, %.2f) is outside the field", x, y);
        }
        return null;
    }

    private FieldFix reject(String why) {
        rejected++;
        lastReason = why;
        return null;
    }

    public String lastReason() {
        return lastReason;
    }

    public long producedCount() {
        return produced;
    }

    public long rejectedCount() {
        return rejected;
    }

    /** Total latency of the last new result, ms (staleness + capture + targeting). */
    public double lastLatencyMs() {
        return lastLatencyMs;
    }

    private static boolean fin(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }
}
