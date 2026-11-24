package org.firstinspires.ftc.teamcode.beacon;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Immutable Beacon configuration, loaded from {@code /sdcard/FIRST/beacon/beacon.json}.
 * Every key is optional; a missing file means "all defaults". {@code //} comments are allowed.
 *
 * <pre>
 * {
 *   "tier": "odometry",          // "odometry" (Pinpoint + Beacon vision) or "fake" (tests)
 *   "T_robot_cam": {"x": 0.15, "y": 0, "z": 0.30, "roll": 0, "pitch": 0.1, "yaw": 0},
 *   "fix": {
 *     "enabled": true,           // blend vision fixes into the pose
 *     "gainXY": 0.3,             // fraction of the position error removed per fix, 0..1,
 *     "gainHeading": 0.2,        //   used when a fix carries no sigma (and same for heading)
 *     "priorSigmaXYM": 0.02,     // how uncertain our own pose is assumed to be between fixes;
 *     "priorSigmaHeadingRad": 0.02, // with a fix sigma s the gain is p^2 / (p^2 + s^2)
 *     "maxJumpM": 0.5,           // reject fixes further than this from our pose (metres)
 *     "maxJumpRad": 0.5,         // ... or this far off in heading (radians)
 *     "recoveryWindowS": 1.0,    // after a tracking loss, trust fixes fully this long
 *     "maxSpeedMps": 2.0,        // ignore fixes taken while the robot moved faster (m/s)
 *     "maxOmegaRadPs": 1.5,      // ... or turned faster (rad/s). Same as the core's own gate;
 *                                //   fixes are capture-time stamped, so motion is no longer a
 *                                //   timing error (the old 0.75 / 0.35 were AprilTag-era)
 *     "historyS": 0.75           // pose history kept for capture-time lookups (seconds)
 *   },
 *   "limelight": {               // the camera stream (see vision.LimelightFrameSource)
 *     "streamUrl": "",           // "" = try the built-in candidates
 *     "connectTimeoutMs": 400, "readTimeoutMs": 1000,
 *     "latencyMs": -1,           // measured capture-to-Hub latency; -1 = estimate
 *     "defaultLatencyMs": 30     // used when nothing better is known (a guess)
 *   },
 *   "record": { "dir": "/sdcard/FIRST/beacon/runs", "maxHz": 30, "frameQueue": 8,
 *               "fieldFile": "/sdcard/FIRST/beacon/field.json", "checkerboardHz": 3 },
 *   "probe":  { "start": [x, y, heading], "marks": [[x, y, heading], ...] },
 *   "vision": { "enabled": true,                       // run Beacon's native vision core
 *               "fieldFile": "/sdcard/FIRST/beacon/field.json",    // season field model (I1)
 *               "cameraFile": "/sdcard/FIRST/beacon/camera.json",  // calibrated intrinsics (I2)
 *               "paramsFile": "",                       // optional core tuning JSON
 *               "odomPollMs": 2 },                      // how often Pinpoint samples are fed
 *   "log": { "enabled": false, "dir": "/sdcard/FIRST/beacon/logs" },
 *   "sourceTimeoutS": 0.5        // no new sample this long means LOST
 * }
 * </pre>
 *
 * <p>The old {@code "tag"} section was renamed {@code "fix"} (same keys); a file that still
 * has {@code "tag"} is rejected with that hint. The AprilTag (MegaTag) keys that used to live
 * in {@code "limelight"} belong to the parked AprilTag code and only produce a warning.
 *
 * <p>Robot hardware (device names, pod offsets, encoder directions) is NOT in this file; it is
 * in {@link BeaconHardwareConfig}, which you edit in Java.
 *
 * <h3>T_robot_cam</h3>
 * The pose of the Limelight's optical frame (x right in the image, y down, z out of the lens)
 * in the robot frame (+x forward, +y left, +z up, origin at the centre of rotation), metres
 * and RADIANS: {@code R_robot_cam = Rz(yaw) * Ry(pitch) * Rx(roll) * R_baseline}, where all
 * zeros means "looking straight forward, upright"; yaw &gt; 0 looks left, pitch &gt; 0 looks
 * down, roll &gt; 0 raises the image's left edge. Pitch must be within +/- pi/2. Beacon's
 * vision core and every recorded camera.json use it; the Pinpoint pose itself does not.
 */
public final class BeaconConfig {

    public static final String DEFAULT_PATH = "/sdcard/FIRST/beacon/beacon.json";
    public static final String DEFAULT_LOG_DIR = "/sdcard/FIRST/beacon/logs";
    public static final String DEFAULT_RUNS_DIR = "/sdcard/FIRST/beacon/runs";
    public static final String DEFAULT_FIELD_FILE = "/sdcard/FIRST/beacon/field.json";
    /** goBILDA Pinpoint odometry (dead wheels + IMU) corrected by Beacon's vision fixes. */
    public static final String TIER_ODOMETRY = "odometry";
    /** Replayed or synthetic poses, for tests and bench work without hardware. */
    public static final String TIER_FAKE = "fake";

    /** Largest believable camera offset from the centre of rotation (metres). */
    public static final double MAX_LEVER_ARM_M = 1.0;

    // ---------------------------------------------------------------- nested

    /** Camera optical frame in the robot frame. See the class Javadoc. */
    public static final class CameraMount {
        public final double x, y, z;
        public final double roll, pitch, yaw;

        public CameraMount(double x, double y, double z, double roll, double pitch, double yaw) {
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }

        /** T_robot_cam as {x, y, z, qx, qy, qz, qw} (see {@link FrameMath}). */
        public double[] toTransform() {
            double[] t = new double[7];
            FrameMath.robotFromCamera(x, y, z, roll, pitch, yaw, t);
            return t;
        }
    }

    /** How the localizer treats field fixes (section "fix", formerly "tag"). */
    public static final class FixConfig {
        public final boolean enabled;
        /** Gain for fixes without a sigma, 0..1. */
        public final double gainXY;
        public final double gainHeading;
        public final double maxJumpM;
        public final double maxJumpRad;
        /** After tracking resumes (or the source world resets) the jump test is waived and
         *  gain 1 is used, but only for fixes captured within this many seconds. */
        public final double recoveryWindowS;
        /** Fixes captured while the robot moved faster than this are ignored (m/s). */
        public final double maxSpeedMps;
        /** Fixes captured while the robot turned faster than this are ignored (rad/s). */
        public final double maxOmegaRadPs;
        /** Pose history kept for looking up the pose at a fix's capture time (s). */
        public final double historyS;
        /** Assumed 1-sigma error of our own pose when a fix arrives (m); see the class doc. */
        public final double priorSigmaXYM;
        /** Same for heading (rad). */
        public final double priorSigmaHeadingRad;

        public FixConfig(boolean enabled, double gainXY, double gainHeading, double maxJumpM,
                         double maxJumpRad, double recoveryWindowS, double maxSpeedMps,
                         double maxOmegaRadPs, double historyS, double priorSigmaXYM,
                         double priorSigmaHeadingRad) {
            this.enabled = enabled;
            this.gainXY = gainXY;
            this.gainHeading = gainHeading;
            this.maxJumpM = maxJumpM;
            this.maxJumpRad = maxJumpRad;
            this.recoveryWindowS = recoveryWindowS;
            this.maxSpeedMps = maxSpeedMps;
            this.maxOmegaRadPs = maxOmegaRadPs;
            this.historyS = historyS;
            this.priorSigmaXYM = priorSigmaXYM;
            this.priorSigmaHeadingRad = priorSigmaHeadingRad;
        }

        /** Same as the 11-argument constructor with the default history and priors. */
        public FixConfig(boolean enabled, double gainXY, double gainHeading, double maxJumpM,
                         double maxJumpRad, double recoveryWindowS, double maxSpeedMps,
                         double maxOmegaRadPs) {
            this(enabled, gainXY, gainHeading, maxJumpM, maxJumpRad, recoveryWindowS,
                    maxSpeedMps, maxOmegaRadPs, 0.75, 0.02, 0.02);
        }

        /** Position gain for a fix whose 1-sigma error is {@code sigma} (NaN = fixed gain). */
        public double gainXYFor(double sigma) {
            return gainFor(sigma, priorSigmaXYM, gainXY);
        }

        public double gainHeadingFor(double sigma) {
            return gainFor(sigma, priorSigmaHeadingRad, gainHeading);
        }

        private static double gainFor(double sigma, double prior, double fixed) {
            if (Double.isNaN(sigma)) {
                return fixed;
            }
            double p2 = prior * prior;
            return p2 / (p2 + sigma * sigma);
        }
    }

    public static final class LogConfig {
        public final boolean enabled;
        public final String dir;

        public LogConfig(boolean enabled, String dir) {
            this.enabled = enabled;
            this.dir = dir;
        }
    }

    /**
     * The Limelight's camera stream (keys in the {@code "limelight"} section). See
     * {@code vision.LimelightFrameSource} for how the URL is found and how capture times are
     * estimated.
     */
    public static final class StreamConfig {
        /** MJPEG stream URL to try first; "" = only the built-in candidates. */
        public final String streamUrl;
        /** TCP connect timeout per candidate (ms). */
        public final int connectTimeoutMs;
        /** No stream data for this long means the stream is gone (ms). */
        public final int readTimeoutMs;
        /**
         * Capture-to-Hub latency in ms: time from the middle of a frame's exposure to the
         * first byte of that frame reaching the Hub. Measure it with the probe OpMode's LED
         * test and enter it here. -1 = estimate it from the Limelight's own reported latency.
         */
        public final double latencyMs;
        /** Used when latencyMs is -1 and the Limelight reports nothing (ms; a guess). */
        public final double defaultLatencyMs;

        public StreamConfig(String streamUrl, int connectTimeoutMs, int readTimeoutMs,
                            double latencyMs, double defaultLatencyMs) {
            this.streamUrl = streamUrl == null ? "" : streamUrl.trim();
            this.connectTimeoutMs = connectTimeoutMs;
            this.readTimeoutMs = readTimeoutMs;
            this.latencyMs = latencyMs;
            this.defaultLatencyMs = defaultLatencyMs;
        }
    }

    /** Dataset recording (probe OpMode). Layout: plan interface I2. */
    public static final class RecordConfig {
        /** Runs go to {@code <dir>/<yyyyMMdd_HHmmss>/}. */
        public final String dir;
        /** Most frames per second written (PNG encoding costs CPU). */
        public final double maxHz;
        /** Frames that may wait for the writer; more are dropped and counted. */
        public final int frameQueue;
        /** Season field model copied into each run as field.json (skipped if absent). */
        public final String fieldFile;
        /** Frame rate for the checkerboard (intrinsics) capture mode. */
        public final double checkerboardHz;

        public RecordConfig(String dir, double maxHz, int frameQueue, String fieldFile,
                            double checkerboardHz) {
            this.dir = dir;
            this.maxHz = maxHz;
            this.frameQueue = frameQueue;
            this.fieldFile = fieldFile;
            this.checkerboardHz = checkerboardHz;
        }
    }

    /** Beacon's native vision core (vision.VisionFixSource). */
    public static final class VisionConfig {
        public final boolean enabled;
        public final String fieldFile;
        /** Camera intrinsics in the I2 camera.json format, from a checkerboard calibration.
         *  Its "mount" is ignored: T_robot_cam in beacon.json is the one mount. */
        public final String cameraFile;
        /** Optional params JSON for the core; "" = none. */
        public final String paramsFile;
        /** Period of the thread that feeds Pinpoint samples to the core (ms). */
        public final double odomPollMs;

        public VisionConfig(boolean enabled, String fieldFile, String cameraFile,
                            String paramsFile, double odomPollMs) {
            this.enabled = enabled;
            this.fieldFile = fieldFile;
            this.cameraFile = cameraFile;
            this.paramsFile = paramsFile == null ? "" : paramsFile;
            this.odomPollMs = odomPollMs;
        }
    }

    /** Probe OpMode inputs, field frame, metres and radians. */
    public static final class ProbeConfig {
        /** Start pose {x, y, heading} the team places the robot at, or null if not given. */
        public final double[] start;
        /** Tape-mark poses {x, y, heading} for ground-truth checks. Never null. */
        public final List<double[]> marks;

        public ProbeConfig(double[] start, List<double[]> marks) {
            this.start = start;
            this.marks = marks == null ? Collections.<double[]>emptyList()
                    : Collections.unmodifiableList(new ArrayList<double[]>(marks));
        }
    }

    // ---------------------------------------------------------------- fields

    public final String tier;
    public final CameraMount cameraMount;
    public final FixConfig fix;
    public final LogConfig log;
    public final StreamConfig stream;
    public final RecordConfig record;
    public final ProbeConfig probe;
    public final VisionConfig vision;
    /** If no new pose arrives for this long, the localizer reports LOST. Seconds. */
    public final double sourceTimeoutS;
    /** Non-fatal problems found while loading. Show them in telemetry. Never null. */
    public final List<String> warnings;

    /**
     * Validating constructor.
     * @throws BeaconException naming the first offending field.
     */
    public BeaconConfig(String tier, CameraMount cameraMount, FixConfig fix, LogConfig log,
                        StreamConfig stream, RecordConfig record, ProbeConfig probe,
                        VisionConfig vision, double sourceTimeoutS, List<String> warnings)
            throws BeaconException {
        this.tier = tier;
        this.cameraMount = cameraMount;
        this.fix = fix;
        this.log = log;
        this.stream = stream;
        this.record = record;
        this.probe = probe;
        this.vision = vision;
        this.sourceTimeoutS = sourceTimeoutS;
        List<String> w = new ArrayList<String>();
        if (warnings != null) {
            w.addAll(warnings);
        }
        validate(w);
        this.warnings = Collections.unmodifiableList(w);
    }

    private void validate(List<String> w) throws BeaconException {
        if ("arcore".equals(tier) || "vins".equals(tier)) {
            throw new BeaconException("tier \"" + tier + "\" is parked and not built into this "
                    + "app. Use \"odometry\" (Pinpoint + Beacon vision).");
        }
        if (!TIER_ODOMETRY.equals(tier) && !TIER_FAKE.equals(tier)) {
            throw new BeaconException("tier must be \"odometry\" or \"fake\", not \"" + tier
                    + "\"");
        }
        CameraMount m = cameraMount;
        requireFinite(m.x, "T_robot_cam.x");
        requireFinite(m.y, "T_robot_cam.y");
        requireFinite(m.z, "T_robot_cam.z");
        double lever = Math.sqrt(m.x * m.x + m.y * m.y + m.z * m.z);
        if (lever > MAX_LEVER_ARM_M) {
            throw new BeaconException("T_robot_cam position is " + lever + " m from the centre "
                    + "of rotation (limit " + MAX_LEVER_ARM_M + " m). Units are metres.");
        }
        requireAngle(m.roll, "T_robot_cam.roll", 2 * Math.PI, w);
        requireAngle(m.pitch, "T_robot_cam.pitch", Math.PI / 2, w);
        requireAngle(m.yaw, "T_robot_cam.yaw", 2 * Math.PI, w);

        requireRange(fix.gainXY, 0, 1, "fix.gainXY");
        requireRange(fix.gainHeading, 0, 1, "fix.gainHeading");
        requireRange(fix.maxJumpM, 1e-6, 10, "fix.maxJumpM");
        requireRange(fix.maxJumpRad, 1e-6, Math.PI, "fix.maxJumpRad");
        requireRange(fix.recoveryWindowS, 0, 10, "fix.recoveryWindowS");
        requireRange(fix.maxSpeedMps, 1e-3, 10, "fix.maxSpeedMps");
        requireRange(fix.maxOmegaRadPs, 1e-3, 20, "fix.maxOmegaRadPs");
        requireRange(fix.historyS, 0.1, 5, "fix.historyS");
        requireRange(fix.priorSigmaXYM, 1e-4, 1, "fix.priorSigmaXYM");
        requireRange(fix.priorSigmaHeadingRad, 1e-4, 1, "fix.priorSigmaHeadingRad");

        requireRange(sourceTimeoutS, 0.01, 60, "sourceTimeoutS");
        if (log.dir == null || log.dir.trim().isEmpty()) {
            throw new BeaconException("log.dir must not be empty");
        }
        validateVision();
    }

    private void validateVision() throws BeaconException {
        StreamConfig s = stream;
        if (s.streamUrl.length() > 0 && s.streamUrl.contains("://")
                && !s.streamUrl.startsWith("http://")) {
            throw new BeaconException("limelight.streamUrl must be an http:// URL (or a bare "
                    + "host like 172.29.0.1), not \"" + s.streamUrl + "\"");
        }
        requireRange(s.connectTimeoutMs, 50, 10000, "limelight.connectTimeoutMs");
        requireRange(s.readTimeoutMs, 100, 10000, "limelight.readTimeoutMs");
        if (s.latencyMs != -1) {
            requireRange(s.latencyMs, 0, 500, "limelight.latencyMs (or -1 = estimate)");
        }
        requireRange(s.defaultLatencyMs, 0, 500, "limelight.defaultLatencyMs");

        RecordConfig r = record;
        if (r.dir == null || r.dir.trim().isEmpty()) {
            throw new BeaconException("record.dir must not be empty");
        }
        requireRange(r.maxHz, 0.1, 200, "record.maxHz");
        requireRange(r.frameQueue, 2, 256, "record.frameQueue");
        requireRange(r.checkerboardHz, 0.1, 30, "record.checkerboardHz");
        if (r.fieldFile == null) {
            throw new BeaconException("record.fieldFile must be a string");
        }

        if (probe.start != null) {
            requirePose(probe.start, "probe.start");
        }
        for (int i = 0; i < probe.marks.size(); i++) {
            requirePose(probe.marks.get(i), "probe.marks[" + i + "]");
        }
        if (vision.fieldFile == null || vision.fieldFile.trim().isEmpty()) {
            throw new BeaconException("vision.fieldFile must not be empty");
        }
        if (vision.cameraFile == null || vision.cameraFile.trim().isEmpty()) {
            throw new BeaconException("vision.cameraFile must not be empty");
        }
        requireRange(vision.odomPollMs, 0.5, 20, "vision.odomPollMs");
    }

    /** {x, y, heading}: on the field (|x|, |y| at most 2 m) and heading within +/- 2 pi. */
    private static void requirePose(double[] p, String name) throws BeaconException {
        if (p.length != 3) {
            throw new BeaconException(name + " must be [x, y, heading] (metres, radians)");
        }
        requireRange(p[0], -2.0, 2.0, name + " x (metres from the field centre)");
        requireRange(p[1], -2.0, 2.0, name + " y (metres from the field centre)");
        requireRange(p[2], -2 * Math.PI, 2 * Math.PI, name + " heading (radians)");
    }

    private static void requireFinite(double v, String name) throws BeaconException {
        if (Double.isNaN(v) || Double.isInfinite(v)) {
            throw new BeaconException(name + " must be a finite number");
        }
    }

    private static void requireRange(double v, double lo, double hi, String name)
            throws BeaconException {
        requireFinite(v, name);
        if (v < lo || v > hi) {
            throw new BeaconException(name + " = " + v + " is outside [" + lo + ", " + hi + "]");
        }
    }

    private static void requireAngle(double v, String name, double limit, List<String> w)
            throws BeaconException {
        requireFinite(v, name);
        if (Math.abs(v) > limit + 1e-9) {
            throw new BeaconException(name + " = " + v + " is beyond +/- " + limit
                    + " rad. Angles are radians; did you enter degrees?");
        }
        if (Math.abs(v) >= 2 && v == Math.rint(v)) {
            w.add(name + " = " + v + " is a whole number of radians (" + Math.toDegrees(v)
                    + " degrees). Angles are radians; did you mean degrees?");
        }
    }

    // ------------------------------------------------------------- factories

    private static CameraMount defaultMount() {
        return new CameraMount(0, 0, 0, 0, 0, 0);
    }

    private static FixConfig defaultFix() {
        return new FixConfig(true, 0.3, 0.2, 0.5, 0.5, 1.0, 2.0, 1.5, 0.75, 0.02, 0.02);
    }

    private static StreamConfig defaultStream() {
        return new StreamConfig("", 400, 1000, -1, 30);
    }

    public static final String DEFAULT_CAMERA_FILE = "/sdcard/FIRST/beacon/camera.json";

    private static VisionConfig defaultVision() {
        return new VisionConfig(true, DEFAULT_FIELD_FILE, DEFAULT_CAMERA_FILE, "", 2);
    }

    private static RecordConfig defaultRecord() {
        return new RecordConfig(DEFAULT_RUNS_DIR, 30, 8, DEFAULT_FIELD_FILE, 3);
    }

    /** Built-in defaults: odometry tier, fixes on, logging off. */
    public static BeaconConfig defaults() {
        return defaultsWithWarning(null);
    }

    private static BeaconConfig defaultsWithWarning(String warning) {
        List<String> w = new ArrayList<String>();
        if (warning != null) {
            w.add(warning);
        }
        try {
            return new BeaconConfig(TIER_ODOMETRY, defaultMount(), defaultFix(),
                    new LogConfig(false, DEFAULT_LOG_DIR), defaultStream(), defaultRecord(),
                    new ProbeConfig(null, null), defaultVision(), 0.5, w);
        } catch (BeaconException e) {
            throw new IllegalStateException("built-in defaults are invalid", e);
        }
    }

    /**
     * Loads the config file. A missing file is NOT an error: it returns the defaults with a
     * warning in {@link #warnings}. A file that exists but is unreadable, malformed or invalid
     * throws {@link BeaconException}.
     */
    public static BeaconConfig load(File file) throws BeaconException {
        if (file == null || !file.isFile()) {
            return defaultsWithWarning("No beacon.json at "
                    + (file == null ? "(null)" : file.getPath()) + " - using defaults.");
        }
        return parse(readFile(file));
    }

    /** {@link #load(File)} on {@link #DEFAULT_PATH}. */
    public static BeaconConfig loadDefault() throws BeaconException {
        return load(new File(DEFAULT_PATH));
    }

    private static String readFile(File f) throws BeaconException {
        InputStream in = null;
        try {
            in = new FileInputStream(f);
            ByteArrayOutputStream out = new ByteArrayOutputStream();
            byte[] buf = new byte[4096];
            int n;
            while ((n = in.read(buf)) > 0) {
                out.write(buf, 0, n);
                if (out.size() > 1 << 20) {
                    throw new BeaconException(f.getPath() + " is too large to be a config file");
                }
            }
            return new String(out.toByteArray(), "UTF-8");
        } catch (IOException e) {
            throw new BeaconException("Cannot read " + f.getPath() + ": " + e.getMessage(), e);
        } finally {
            if (in != null) {
                try {
                    in.close();
                } catch (IOException ignored) {
                    // nothing useful to do
                }
            }
        }
    }

    private static final Set<String> TOP_KEYS = new HashSet<String>(Arrays.asList(
            "tier", "T_robot_cam", "fix", "limelight", "log", "record", "probe", "vision",
            "sourceTimeoutS"));
    private static final Set<String> VISION_KEYS = new HashSet<String>(Arrays.asList(
            "enabled", "fieldFile", "cameraFile", "paramsFile", "odomPollMs"));
    private static final Set<String> MOUNT_KEYS = new HashSet<String>(Arrays.asList(
            "x", "y", "z", "roll", "pitch", "yaw"));
    private static final Set<String> FIX_KEYS = new HashSet<String>(Arrays.asList(
            "enabled", "gainXY", "gainHeading", "maxJumpM", "maxJumpRad", "recoveryWindowS",
            "maxSpeedMps", "maxOmegaRadPs", "historyS", "priorSigmaXYM",
            "priorSigmaHeadingRad"));
    private static final Set<String> LL_KEYS = new HashSet<String>(Arrays.asList(
            "streamUrl", "connectTimeoutMs", "readTimeoutMs", "latencyMs", "defaultLatencyMs"));
    /** AprilTag (MegaTag) keys of the parked tag path; accepted with a warning. */
    private static final Set<String> LL_PARKED_KEYS = new HashSet<String>(Arrays.asList(
            "pipeline", "minTags", "maxRangeM", "mt1MinTags", "mt1MaxRangeM", "useMt1Heading",
            "maxStalenessMs", "headingHz", "headingFreshMs", "fieldHalfM"));
    private static final Set<String> LOG_KEYS = new HashSet<String>(Arrays.asList(
            "enabled", "dir"));
    private static final Set<String> RECORD_KEYS = new HashSet<String>(Arrays.asList(
            "dir", "maxHz", "frameQueue", "fieldFile", "checkerboardHz"));
    private static final Set<String> PROBE_KEYS = new HashSet<String>(Arrays.asList(
            "start", "marks"));

    /**
     * Parses and validates JSON text. Absent keys take their defaults. An unknown TOP-LEVEL
     * key is an error (a misspelled section would otherwise be silently ignored, e.g.
     * "limelite"); an unknown key inside a section is a warning.
     */
    @SuppressWarnings("unchecked")
    public static BeaconConfig parse(String json) throws BeaconException {
        if (json != null && json.length() > 0 && json.charAt(0) == '﻿') {
            json = json.substring(1);   // UTF-8 byte order mark written by some editors
        }
        Object root = MiniJson.parse(json);
        if (!(root instanceof Map)) {
            throw new BeaconException("beacon.json must contain a JSON object { ... }");
        }
        Map<String, Object> top = (Map<String, Object>) root;
        if (top.containsKey("tag")) {
            throw new BeaconException("Section \"tag\" was renamed to \"fix\" (same keys). "
                    + "Rename it in beacon.json.");
        }
        for (String k : top.keySet()) {
            if (!TOP_KEYS.contains(k)) {
                throw new BeaconException("Unknown section \"" + k + "\" in beacon.json. Known: "
                        + "tier, T_robot_cam, fix, limelight, log, record, probe, vision, "
                        + "sourceTimeoutS.");
            }
        }
        List<String> warnings = new ArrayList<String>();

        String tier = getString(top, "tier", TIER_ODOMETRY, "tier");

        Map<String, Object> mj = getObject(top, "T_robot_cam", "T_robot_cam");
        warnUnknown(mj, MOUNT_KEYS, "T_robot_cam.", warnings);
        CameraMount mount = new CameraMount(
                getNumber(mj, "x", 0, "T_robot_cam.x"),
                getNumber(mj, "y", 0, "T_robot_cam.y"),
                getNumber(mj, "z", 0, "T_robot_cam.z"),
                getNumber(mj, "roll", 0, "T_robot_cam.roll"),
                getNumber(mj, "pitch", 0, "T_robot_cam.pitch"),
                getNumber(mj, "yaw", 0, "T_robot_cam.yaw"));

        Map<String, Object> fj = getObject(top, "fix", "fix");
        warnUnknown(fj, FIX_KEYS, "fix.", warnings);
        FixConfig d = defaultFix();
        FixConfig fix = new FixConfig(
                getBoolean(fj, "enabled", d.enabled, "fix.enabled"),
                getNumber(fj, "gainXY", d.gainXY, "fix.gainXY"),
                getNumber(fj, "gainHeading", d.gainHeading, "fix.gainHeading"),
                getNumber(fj, "maxJumpM", d.maxJumpM, "fix.maxJumpM"),
                getNumber(fj, "maxJumpRad", d.maxJumpRad, "fix.maxJumpRad"),
                getNumber(fj, "recoveryWindowS", d.recoveryWindowS, "fix.recoveryWindowS"),
                getNumber(fj, "maxSpeedMps", d.maxSpeedMps, "fix.maxSpeedMps"),
                getNumber(fj, "maxOmegaRadPs", d.maxOmegaRadPs, "fix.maxOmegaRadPs"),
                getNumber(fj, "historyS", d.historyS, "fix.historyS"),
                getNumber(fj, "priorSigmaXYM", d.priorSigmaXYM, "fix.priorSigmaXYM"),
                getNumber(fj, "priorSigmaHeadingRad", d.priorSigmaHeadingRad,
                        "fix.priorSigmaHeadingRad"));

        Map<String, Object> lj = getObject(top, "limelight", "limelight");
        if (lj != null) {
            for (String k : lj.keySet()) {
                if (LL_PARKED_KEYS.contains(k)) {
                    warnings.add("limelight." + k + " is an AprilTag (MegaTag) setting; "
                            + "AprilTags are not used by Beacon, so it is ignored.");
                } else if (!LL_KEYS.contains(k)) {
                    warnings.add("Unknown key \"limelight." + k + "\" ignored (typo?).");
                }
            }
        }
        StreamConfig sd = defaultStream();
        StreamConfig stream = new StreamConfig(
                getString(lj, "streamUrl", sd.streamUrl, "limelight.streamUrl"),
                getInt(lj, "connectTimeoutMs", sd.connectTimeoutMs, "limelight.connectTimeoutMs"),
                getInt(lj, "readTimeoutMs", sd.readTimeoutMs, "limelight.readTimeoutMs"),
                getNumber(lj, "latencyMs", sd.latencyMs, "limelight.latencyMs"),
                getNumber(lj, "defaultLatencyMs", sd.defaultLatencyMs,
                        "limelight.defaultLatencyMs"));

        Map<String, Object> logj = getObject(top, "log", "log");
        warnUnknown(logj, LOG_KEYS, "log.", warnings);
        LogConfig log = new LogConfig(
                getBoolean(logj, "enabled", false, "log.enabled"),
                getString(logj, "dir", DEFAULT_LOG_DIR, "log.dir"));

        Map<String, Object> rj = getObject(top, "record", "record");
        warnUnknown(rj, RECORD_KEYS, "record.", warnings);
        RecordConfig rd = defaultRecord();
        RecordConfig record = new RecordConfig(
                getString(rj, "dir", rd.dir, "record.dir"),
                getNumber(rj, "maxHz", rd.maxHz, "record.maxHz"),
                getInt(rj, "frameQueue", rd.frameQueue, "record.frameQueue"),
                getString(rj, "fieldFile", rd.fieldFile, "record.fieldFile"),
                getNumber(rj, "checkerboardHz", rd.checkerboardHz, "record.checkerboardHz"));

        Map<String, Object> pj = getObject(top, "probe", "probe");
        warnUnknown(pj, PROBE_KEYS, "probe.", warnings);
        double[] start = null;
        if (pj != null && pj.containsKey("start")) {
            start = getNumberArray(pj.get("start"), "probe.start");
        }
        List<double[]> marks = new ArrayList<double[]>();
        if (pj != null && pj.containsKey("marks")) {
            Object mv = pj.get("marks");
            if (!(mv instanceof List)) {
                throw new BeaconException("probe.marks must be a list of [x, y, heading]");
            }
            List<Object> ml = (List<Object>) mv;
            for (int i = 0; i < ml.size(); i++) {
                marks.add(getNumberArray(ml.get(i), "probe.marks[" + i + "]"));
            }
        }
        ProbeConfig probe = new ProbeConfig(start, marks);

        Map<String, Object> vj = getObject(top, "vision", "vision");
        warnUnknown(vj, VISION_KEYS, "vision.", warnings);
        VisionConfig vd = defaultVision();
        VisionConfig vision = new VisionConfig(
                getBoolean(vj, "enabled", vd.enabled, "vision.enabled"),
                getString(vj, "fieldFile", vd.fieldFile, "vision.fieldFile"),
                getString(vj, "cameraFile", vd.cameraFile, "vision.cameraFile"),
                getString(vj, "paramsFile", vd.paramsFile, "vision.paramsFile"),
                getNumber(vj, "odomPollMs", vd.odomPollMs, "vision.odomPollMs"));

        double timeout = getNumber(top, "sourceTimeoutS", 0.5, "sourceTimeoutS");
        return new BeaconConfig(tier, mount, fix, log, stream, record, probe, vision, timeout,
                warnings);
    }

    // ------------------------------------------------------- typed accessors

    private static void warnUnknown(Map<String, Object> m, Set<String> known, String prefix,
                                    List<String> warnings) {
        if (m == null) {
            return;
        }
        for (String k : m.keySet()) {
            if (!known.contains(k)) {
                warnings.add("Unknown key \"" + prefix + k + "\" ignored (typo?).");
            }
        }
    }

    @SuppressWarnings("unchecked")
    private static Map<String, Object> getObject(Map<String, Object> m, String key, String path)
            throws BeaconException {
        if (m == null || !m.containsKey(key)) {
            return null;
        }
        Object v = m.get(key);
        if (!(v instanceof Map)) {
            throw new BeaconException(path + " must be a JSON object { ... }");
        }
        return (Map<String, Object>) v;
    }

    private static double getNumber(Map<String, Object> m, String key, double def, String path)
            throws BeaconException {
        if (m == null || !m.containsKey(key)) {
            return def;
        }
        Object v = m.get(key);
        if (!(v instanceof Double)) {
            throw new BeaconException(path + " must be a number");
        }
        return ((Double) v).doubleValue();
    }

    private static int getInt(Map<String, Object> m, String key, int def, String path)
            throws BeaconException {
        double v = getNumber(m, key, def, path);
        if (v != Math.rint(v) || Math.abs(v) > 1e6) {
            throw new BeaconException(path + " must be a whole number");
        }
        return (int) v;
    }

    private static boolean getBoolean(Map<String, Object> m, String key, boolean def, String path)
            throws BeaconException {
        if (m == null || !m.containsKey(key)) {
            return def;
        }
        Object v = m.get(key);
        if (!(v instanceof Boolean)) {
            throw new BeaconException(path + " must be true or false");
        }
        return ((Boolean) v).booleanValue();
    }

    private static String getString(Map<String, Object> m, String key, String def, String path)
            throws BeaconException {
        if (m == null || !m.containsKey(key)) {
            return def;
        }
        Object v = m.get(key);
        if (!(v instanceof String)) {
            throw new BeaconException(path + " must be a string");
        }
        return (String) v;
    }

    @SuppressWarnings("unchecked")
    private static double[] getNumberArray(Object v, String path) throws BeaconException {
        if (!(v instanceof List)) {
            throw new BeaconException(path + " must be a list of numbers like [x, y, heading]");
        }
        List<Object> l = (List<Object>) v;
        double[] out = new double[l.size()];
        for (int i = 0; i < out.length; i++) {
            Object o = l.get(i);
            if (!(o instanceof Double)) {
                throw new BeaconException(path + " must contain only numbers");
            }
            out[i] = ((Double) o).doubleValue();
        }
        return out;
    }
}
