package org.firstinspires.ftc.teamcode.beacon;

/**
 * One absolute fix of the ROBOT's centre of rotation in the FIELD frame (from Beacon's vision
 * core). Metres and radians, heading counter-clockwise positive. Immutable. (Plan interface I4:
 * this was {@code TagFix}.)
 *
 * <p>A fix may carry only part of a pose: {@code hasPosition} says whether x, y are a real
 * measurement, {@code hasHeading} whether {@code heading} is. The localizer corrects only the
 * parts that are present.
 *
 * <p>{@code tNs} is the image CAPTURE time on the Hub clock
 * ({@code SystemClock.elapsedRealtimeNanos()}), never the time the result was read.
 *
 * <p>{@code sigmaXY} (metres, 1-sigma per axis) and {@code sigmaHeading} (radians) are the fix's
 * own uncertainty. When finite, the localizer derives its blending gain from them (a precise fix
 * moves the pose more than a vague one); NaN means "unknown: use the configured fixed gain".
 */
public final class FieldFix {
    public final double x;
    public final double y;
    public final double heading;
    public final long tNs;
    public final boolean hasPosition;
    public final boolean hasHeading;
    /** Short label for telemetry, for example "edges+points". Never null. */
    public final String kind;
    public final double sigmaXY;
    public final double sigmaHeading;

    public FieldFix(double x, double y, double heading, long tNs, boolean hasPosition,
                    boolean hasHeading, String kind, double sigmaXY, double sigmaHeading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.tNs = tNs;
        this.hasPosition = hasPosition;
        this.hasHeading = hasHeading;
        this.kind = kind == null ? "" : kind;
        this.sigmaXY = sigmaXY;
        this.sigmaHeading = sigmaHeading;
    }

    /** A fix without uncertainty (the configured gains apply). */
    public FieldFix(double x, double y, double heading, long tNs, boolean hasPosition,
                    boolean hasHeading, String kind) {
        this(x, y, heading, tNs, hasPosition, hasHeading, kind, Double.NaN, Double.NaN);
    }

    /** A fix with both position and heading, without uncertainty. */
    public static FieldFix full(double x, double y, double heading, long tNs) {
        return new FieldFix(x, y, heading, tNs, true, true, "full");
    }

    /** A fix with both position and heading and their uncertainty. */
    public static FieldFix full(double x, double y, double heading, long tNs, double sigmaXY,
                                double sigmaHeading) {
        return new FieldFix(x, y, heading, tNs, true, true, "full", sigmaXY, sigmaHeading);
    }

    /**
     * True if the parts this fix claims to carry are finite numbers. A sigma may be NaN
     * (unknown) but not infinite or negative.
     */
    public boolean isFinite() {
        boolean ok = true;
        if (hasPosition) {
            ok = fin(x) && fin(y) && sigmaOk(sigmaXY);
        }
        if (hasHeading) {
            ok = ok && fin(heading) && sigmaOk(sigmaHeading);
        }
        return ok;
    }

    private static boolean sigmaOk(double s) {
        return Double.isNaN(s) || (!Double.isInfinite(s) && s >= 0);
    }

    private static boolean fin(double v) {
        return !Double.isNaN(v) && !Double.isInfinite(v);
    }

    @Override
    public String toString() {
        return "FieldFix(" + kind + " " + (hasPosition ? x + ", " + y : "-") + ", "
                + (hasHeading ? heading + " rad" : "-") + ", sigma " + sigmaXY + " m / "
                + sigmaHeading + " rad, t=" + tNs + ")";
    }
}
