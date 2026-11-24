# 03 — Tier 1: `ArCorePoseSource` (Google ARCore inside the Robot Controller app)

Goal: a working on-robot VIO pose in 1–2 weekends with no SLAM code, to validate the phone-RC
setup, mounting, frames, tag fusion and the test protocol before Tier 2.

## Requirements

- Phone on Google's ARCore supported-devices list (Galaxy S22 is). "Google Play Services for AR"
  installed and up to date. Phone in airplane mode with Wi-Fi on is fine for ARCore.
- RC app gets `CAMERA` permission; add `com.google.ar.core` dependency in `TeamCode/build.gradle`
  and `<meta-data android:name="com.google.ar.core" android:value="required" />` to the manifest.
  Note the RC app's `minSdk`/`targetSdk` constraints; ARCore needs API 24+.

## Design

`arcore/ArCorePoseSource implements PoseSource`:

- `start()`: check `ArCoreApk.getInstance().checkAvailability`, request install if needed (fail
  loudly in telemetry if not available — never block the OpMode). Create `Session` with
  `Config`: `UpdateMode.LATEST_CAMERA_IMAGE`, `PlaneFindingMode.DISABLED`,
  `LightEstimationMode.DISABLED`, `DepthMode.DISABLED`, `FocusMode.FIXED`, `InstantPlacement`
  off. Pick the camera config with the highest frame rate at ≤ 640×480 for the CPU image (ARCore
  exposes `getSupportedCameraConfigs`; filter by `getFpsRange` and `getImageSize`).
- Run `session.update()` on a dedicated `HandlerThread` at camera rate. ARCore requires a
  GL context for `update()` in older versions; create a 1×1 offscreen EGL context on that thread
  and call `session.setCameraTextureName`. (If the installed ARCore version supports
  `update()` without a texture, prefer that; document which.)
- From each `Frame`: `camera.getPose()` (world frame is gravity-aligned, y-up, origin at session
  start), `camera.getTrackingState()`, `camera.getTrackingFailureReason()`, frame timestamp
  (`frame.getTimestamp()` is on the same monotonic clock as SensorEvents).
- Publish `Pose3dStamped` with status mapped: `TRACKING` → 2, `PAUSED` → 1 or 3 depending on
  failure reason, `STOPPED` → 0.
- Convert ARCore's y-up to the project's z-up at publish time (`x, y, z` → `x, -z, y`, rotate the
  quaternion accordingly) so `AstroLocalizer` sees the same convention as Tier 2.
- Optional: `frame.acquireCameraImage()` every Nth frame → hand to `TagCorrector` (see 02).
- `stop()`: `session.pause()`, `session.close()`, quit the thread. Must be safe to call twice and
  from `OpMode.stop()`.
- Lifecycle: the RC app is a foreground Activity; ARCore `Session` must `pause()` when the RC app
  backgrounds. Hook the RC's `OpModeManagerNotifier` or the Activity lifecycle via
  `AppUtil.getInstance().getRootActivity()`.

## Known problems to solve

1. **Camera contention with `VisionPortal`.** Decide and document the AprilTag route (02).
2. **Initialization.** ARCore needs motion and texture; a stationary robot at auto start may sit in
   `PAUSED/INSUFFICIENT_FEATURES`. Solution: `AstroCalibrationOpMode` checks tracking before
   `start`; in auto, the first 0.5 s can be a small wiggle, or rely on a tag fix to set the
   alignment while ARCore is still initializing.
3. **ARCore re-localization jumps.** ARCore silently corrects drift when it recognizes a place;
   poses can jump several cm. Detect jumps (> 5 cm between consecutive frames at < 1 m/s) and treat
   them like a tag correction (adjust alignment) rather than as motion.
4. **Thermal.** Log `/sys/class/thermal` temps every 5 s in the debug CSV; expect throttling after
   ~5–10 min.
5. **Screen.** ARCore does not need the screen on, but the RC app does to stay foreground.
   Keep screen on at minimum brightness via `WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON`.

## Tests

- Bench: phone on a sled, push 1.000 m along a tape, 10 reps each axis → scale error, sign.
- Robot: full `06-TESTING.md` protocol with `tier: "arcore"`.
- Log every run; `tools/plot_log.py` overlays VIO path vs. tag fixes.

## Acceptance

- `AstroTestTeleOp` shows tracking within 3 s of start on a lit field; pose updates ≥ 30 Hz.
- Closed 2 × 2 m square return error ≤ 5 cm, heading ≤ 2°; 3 m line ≤ 1 %.
- Spin-in-place leak ≤ 1 cm after 5 turns (validates `T_robot_cam`).
- Full autonomous × 10: endpoint spread ≤ 3 cm with tags, ≤ 5 cm without.
