package org.firstinspires.ftc.teamcode.beacon.parked;

import org.firstinspires.ftc.teamcode.beacon.BeaconException;
import org.firstinspires.ftc.teamcode.beacon.MiniJson;

import java.util.Map;

/**
 * PARKED: AprilTag (MegaTag) filtering settings of the old Limelight tag path. Beacon uses no
 * AprilTags; this code is kept compiling and tested only so it can be revived. Nothing in the
 * active build reads it, and beacon.json no longer configures it (its keys produce a warning).
 */
public final class LimelightTagConfig {
    public final int pipeline;
    public final int minTags;
    public final double maxRangeM;
    public final int mt1MinTags;
    public final double mt1MaxRangeM;
    public final boolean useMt1Heading;
    public final double maxStalenessMs;
    public final double headingHz;
    public final double headingFreshMs;
    public final double fieldHalfM;

    public LimelightTagConfig(int pipeline, int minTags, double maxRangeM, int mt1MinTags,
                              double mt1MaxRangeM, boolean useMt1Heading, double maxStalenessMs,
                              double headingHz, double headingFreshMs, double fieldHalfM) {
        this.pipeline = pipeline;
        this.minTags = minTags;
        this.maxRangeM = maxRangeM;
        this.mt1MinTags = mt1MinTags;
        this.mt1MaxRangeM = mt1MaxRangeM;
        this.useMt1Heading = useMt1Heading;
        this.maxStalenessMs = maxStalenessMs;
        this.headingHz = headingHz;
        this.headingFreshMs = headingFreshMs;
        this.fieldHalfM = fieldHalfM;
    }

    public static LimelightTagConfig defaults() {
        return new LimelightTagConfig(0, 1, 2.5, 2, 2.0, true, 150, 100, 50, 1.83);
    }

    /** Parses one JSON object with the keys above (absent = default), validating ranges. */
    @SuppressWarnings("unchecked")
    public static LimelightTagConfig parse(String json) throws BeaconException {
        Object o = MiniJson.parse(json);
        if (!(o instanceof Map)) {
            throw new BeaconException("limelight tag settings must be a JSON object");
        }
        Map<String, Object> m = (Map<String, Object>) o;
        LimelightTagConfig d = defaults();
        LimelightTagConfig c = new LimelightTagConfig(
                (int) num(m, "pipeline", d.pipeline, 0, 9),
                (int) num(m, "minTags", d.minTags, 1, 32),
                num(m, "maxRangeM", d.maxRangeM, 0.1, 10),
                (int) num(m, "mt1MinTags", d.mt1MinTags, 1, 32),
                num(m, "mt1MaxRangeM", d.mt1MaxRangeM, 0.1, 10),
                bool(m, "useMt1Heading", d.useMt1Heading),
                num(m, "maxStalenessMs", d.maxStalenessMs, 10, 2000),
                num(m, "headingHz", d.headingHz, 1, 250),
                num(m, "headingFreshMs", d.headingFreshMs, 5, 1000),
                num(m, "fieldHalfM", d.fieldHalfM, 0.5, 5));
        return c;
    }

    private static double num(Map<String, Object> m, String k, double def, double lo, double hi)
            throws BeaconException {
        if (!m.containsKey(k)) {
            return def;
        }
        Object v = m.get(k);
        if (!(v instanceof Double)) {
            throw new BeaconException("limelight." + k + " must be a number");
        }
        double x = (Double) v;
        if (Double.isNaN(x) || x < lo || x > hi) {
            throw new BeaconException("limelight." + k + " = " + x + " is outside [" + lo + ", "
                    + hi + "]");
        }
        return x;
    }

    private static boolean bool(Map<String, Object> m, String k, boolean def)
            throws BeaconException {
        if (!m.containsKey(k)) {
            return def;
        }
        Object v = m.get(k);
        if (!(v instanceof Boolean)) {
            throw new BeaconException("limelight." + k + " must be true or false");
        }
        return (Boolean) v;
    }
}
