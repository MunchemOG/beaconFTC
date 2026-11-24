# 02 — Robot Controller app integration: the Java layer every tier plugs into

Goal: the FTC-side package that owns frames, timing, config, logging, tag fusion, and the
`Localizer` adapters, independent of which VIO produces poses. Build and test this with a fake
pose source before any camera code exists.

## Hardware assumptions

- Phone (Android 10+, ARCore-certified for Tier 1) as ROBOT CONTROLLER via USB-OTG → REV
  Expansion Hub #1; Expansion Hub #2 on RS-485 (address changed in REV Hardware Client).
  Wiring spec in `07-RULES.md` / `WIRING.md`.
- Phone mounted rigidly, rear camera unobstructed, near the robot's centre of rotation. Measured
  `T_robot_cam` (camera position/orientation in robot frame) goes in `astro.json`.

## Package: `org.firstinspires.ftc.teamcode.astro`

### `Pose2dStamped`
`double x, y, heading; long tNs;` plus `int status` (0 = invalid, 1 = initializing,
2 = tracking, 3 = lost/relocalizing). Immutable.

### `Pose3dStamped`
`double x, y, z; double qx, qy, qz, qw; long tNs; int status;` Immutable. What pose sources emit.

### `PoseSource` (interface)
```
void start(Context ctx, AstroConfig cfg) throws AstroException;
void stop();
Pose3dStamped latest();            // non-blocking, may return the same object twice
boolean isTracking();
String debugLine();                // one line for telemetry
```
Pose is the **camera** pose in the source's own world frame (gravity-aligned if the source knows
gravity). Conversion to robot/field frames happens in `AstroLocalizer`, not in sources.

### `FakePoseSource`
Replays a CSV log (format in `00`) or synthesizes a path. Used by unit tests and by the bench
OpMode when no camera is available.

### `AstroConfig` (loaded from `/sdcard/FIRST/astro/astro.json`)
```
tier: "arcore" | "vins" | "fake"
T_robot_cam: { x, y, z, roll, pitch, yaw }     // metres, radians, camera in robot frame
tag: { enabled, gainXY, gainHeading, maxJumpM, minDecisionMargin }
log: { enabled, dir }
vins: { configPath }                            // Tier 2 only
```
Missing file → defaults + a loud telemetry warning. Validate on load; throw on nonsense (e.g.
quaternion not normalized, gain outside [0,1]).

### `AstroLocalizer`
Responsibilities, in order, every `update()`:
1. Pull `latest()` from the pose source; skip if same `tNs` as last time.
2. Convert camera pose (source world frame) → robot pose (source world frame) using `T_robot_cam`.
3. Apply the **field alignment** `T_field_sourceworld`, set by `setPose(fieldPose)` at auto start
   or by the first confident tag fix. Until set, status is `initializing` and `getPose()` returns
   the last set pose.
4. Project to 2-D: x, y from the gravity-aligned horizontal plane; heading = yaw. Keep z for
   diagnostics only (a non-zero z drift is the first sign of a bad calibration).
5. If `TagCorrector` has a fresh, valid field pose (see below), blend: apply `gainXY` and
   `gainHeading` to the *alignment transform*, not to the output pose, so the correction persists
   and VIO continues smoothly from the corrected frame. Reject tag fixes that would jump more than
   `maxJumpM` unless status is `initializing` or `lost`.
6. Write the log row.

Public API: `setPose(Pose2d)`, `update()`, `getPose()`, `getVelocity()` (finite difference over
the last 3 samples, robot frame), `getStatus()`, `debugLine()`.

### `TagCorrector`
Wraps the SDK's `AprilTagProcessor` + `AprilTagGameDatabase`. Produces `Pose2dStamped` in the
field frame from `detection.robotPose` (SDK ≥ 10 provides robot pose given camera placement; pass
the same `T_robot_cam`). Only accept detections with `decisionMargin ≥ minDecisionMargin` and
range ≤ 2.5 m. **Camera contention:** Tier 1 (ARCore) and `VisionPortal` cannot share the rear
camera. Resolution options, pick in this order:
  a. Use ARCore's own `Frame.acquireCameraImage()` and run the SDK's AprilTag detector on that
     image (the SDK's `AprilTagProcessor` can be driven manually via its `processFrame`-style
     API or by using the underlying `apriltag` JNI with the ARCore intrinsics).
  b. A cheap UVC webcam on the Expansion Hub USB? **Not legal** — R707 only allows USB vision
     devices on the robot control system; with a phone RC the phone's USB is taken by the hub
     cable unless a USB hub is used (allowed per R707 B). Treat as fallback, verify at inspection.
  c. Tier 2 owns the camera outright and hands every Nth frame to the tag detector — trivial.

### Adapters
- `adapters/RoadRunnerLocalizerAdapter`: implements Road Runner 1.0 `Localizer`
  (`update()` returning `Twist2dDual<Time>` from successive poses; or the simpler pattern of
  overriding `MecanumDrive.updatePoseEstimate()`). Match whichever RR version the repo pins.
- `adapters/PedroLocalizerAdapter`: implements Pedro Pathing's `Localizer` interface
  (`getPose`, `getVelocity`, `setPose`, `update`, `setStartPose`, `resetIMU` no-op).
Both adapters are thin: ≤ 60 lines, no logic beyond unit/frame conversion.

### OpModes
- `AstroTestTeleOp`: drive by hand, telemetry shows pose (m and in), status, source debug line,
  tag status, loop Hz. Buttons: Y = `setPose(0,0,0)`, X = toggle logging, B = toggle tag blend.
- `AstroCalibrationOpMode`: guided sequence (push 1 m forward, 1 m left, spin 360) that prints
  the implied scale/sign/lever-arm errors so `astro.json` can be corrected without math.

## Tests (JUnit, run on the JVM, no Android device)

- `AstroLocalizerTest`: with `FakePoseSource` on a synthetic square, verify (a) closed-loop
  error < 1 mm, (b) lever-arm correction removes translation during pure rotation, (c) a tag fix
  shifts the alignment and later VIO deltas remain continuous, (d) `maxJumpM` rejection.
- `FrameMathTest`: quaternion → yaw, gravity-aligned projection, `T_robot_cam` round trips.
- `AstroConfigTest`: parsing, defaults, validation failures.

## Acceptance

- Build passes with the FTC SDK; `AstroTestTeleOp` runs with `tier: "fake"` and shows a moving
  pose from a replayed EuRoC log.
- Unit tests green. Loop overhead of `update()` < 0.5 ms (measure with `System.nanoTime`).
