# ASTRO-FTC — Phone-based visual-inertial localization for FTC robots

Master spec. Read this first; every other spec in this folder assumes it.

## One-paragraph goal

Rebuild, as open source, a localization system where an Android phone **is** the FTC Robot
Controller (R701 option B: phone → USB → REV Expansion Hub, plus one more Expansion Hub) and runs
visual-inertial odometry (VIO) on its own rear camera and IMU inside the FTC Robot Controller app.
The VIO pose is exposed to team code through a `Localizer` interface that drops into Road Runner
or Pedro Pathing, with AprilTag absolute corrections. Target: dead-wheel-class accuracy
(≤ 1 % of distance travelled, 3–5 cm over an autonomous) at ≥ 30 Hz, with no moving parts.

## Why this shape

- The REV Control Hub (RK3328, 4×A53 @ 1.5 GHz, 1–2 GB) cannot run VIO. A 2022-class phone
  (Snapdragon 8 Gen 1, 8 GB) runs it comfortably and, critically, has a camera and IMU on one clock.
- The Limelight 3A (A72 @ 1.5 GHz, no IMU, rolling shutter) cannot do VIO either; it can do
  floor-flow odometry (separate project, `floorflow`). This project is the "full" version.
- FTC 2026-27 rules: a smartphone RC is legal and unsupported; no approved-phone list exists
  any more. See `07-RULES.md`.

## Deliverables (what "done" looks like)

| # | Artifact | Spec |
| --- | --- | --- |
| 1 | `desktop-harness/` — macOS-native OpenVINS (ROS-free) driver on EuRoC with a Rerun viewer, plus ATE and speed scripts | `01-DESKTOP-HARNESS.md` |
| 2 | `TeamCode/.../astro/` — Java package in an FTC Robot Controller project: `PoseSource` interface, `AstroLocalizer`, adapters, test OpModes | `02-RC-APP-INTEGRATION.md` |
| 3 | `ArCorePoseSource` — Tier 1 pose source using Google ARCore inside the RC app | `03-TIER1-ARCORE.md` |
| 4 | `VinsPoseSource` + `astro-native/` — Tier 2 pose source: VINS-Fusion core built with the NDK, fed by Camera2 + SensorManager | `04-TIER2-VINS-NDK.md` |
| 5 | `calibration/` — macOS-native calibration tools (OpenCV intrinsics, Allan variance, OpenVINS online cam-IMU) and per-phone YAML | `05-CALIBRATION.md` |
| 6 | `TESTING.md` + logged results | `06-TESTING.md` |
| 7 | `RULES.md` + wiring diagram for inspection | `07-RULES.md` |
| 8 | `README.md`, MIT license, release checklist | `08-RELEASE.md` |

Build order: 1 → 2 → 3 → 6 (on Tier 1) → 5 → 4 → 6 (on Tier 2) → 8. Tier 1 ships before Tier 2
starts; a team can stop at Tier 1 and have something useful.

## Repository layout

```
astro-ftc/
  README.md  LICENSE  RULES.md  TESTING.md  WIRING.md
  desktop-harness/        # macOS-native: OpenVINS driver + Rerun viewer + eval scripts
  calibration/            # calibration tools, targets, per-phone yaml
  astro-native/           # C++ VIO core (Tier 2), CMake, NDK build
  ftc/                    # FtcRobotController fork or TeamCode-only module
    TeamCode/src/main/java/org/firstinspires/ftc/teamcode/astro/
      PoseSource.java
      Pose2dStamped.java
      AstroLocalizer.java
      arcore/ArCorePoseSource.java
      vins/VinsPoseSource.java
      vins/CameraImuFeeder.java
      adapters/RoadRunnerLocalizerAdapter.java
      adapters/PedroLocalizerAdapter.java
      tags/TagCorrector.java
      opmodes/AstroTestTeleOp.java
      opmodes/AstroCalibrationOpMode.java
    TeamCode/src/main/cpp/   # JNI shim for Tier 2
  tools/                  # log parsers, plotters, replay-on-phone harness
```

## Conventions (apply everywhere)

- **Units:** metres and radians internally. Helpers for inches/degrees only at the API edge.
- **Field frame:** FTC standard field coordinate system (origin field centre, +X toward red wall is
  NOT assumed; use the SDK's `AprilTagGameDatabase` field frame as the single source of truth so
  tag corrections and VIO agree). Heading CCW-positive.
- **Robot frame:** +x forward, +y left, +z up, origin at the robot's centre of rotation.
- **Phone/camera frame:** whatever the source reports; every `PoseSource` converts to robot frame
  using a fixed `T_robot_cam` transform loaded from `astro.json` (see 02).
- **Timestamps:** nanoseconds on the Android monotonic clock (`SystemClock.elapsedRealtimeNanos`).
  Camera frames use `CaptureResult.SENSOR_TIMESTAMP`; IMU uses `SensorEvent.timestamp`. Never
  `System.currentTimeMillis()`.
- **Threading:** pose sources run on their own threads and publish to a lock-free latest-value
  holder; OpMode loops only read. Never block the OpMode loop on a camera frame.
- **Logging:** every pose source writes a CSV (`t_ns, x, y, z, qx, qy, qz, qw, status`) to
  `/sdcard/FIRST/astro/logs/` when enabled; the evaluation tools read that format.
- **Config:** one `astro.json` in `/sdcard/FIRST/astro/` with camera extrinsics, lever arm, tag
  gains, tier selection. No magic numbers in code.
- **Language:** Java for all FTC-side code (teams read Java; Kotlin is allowed only in adapters if
  the target library is Kotlin). C++17 for the native core. Python 3 for tools.
- **Dependencies:** FTC SDK ≥ 11.x; ARCore SDK (Tier 1 only); OpenCV (bundled with the FTC SDK);
  Eigen/Ceres (Tier 2 native). No ROS on the phone.
- **Licensing:** MIT for everything we write. VINS-Fusion is GPL-3 — keep it in `astro-native/`
  as a clearly separated component with its own license file; if GPL contamination is a problem for
  the release, OpenVINS (GPL-3 too) and Basalt (BSD-3) are the alternatives — Basalt is the
  BSD-licensed fallback. Decide in `04` before writing the JNI layer.

## Non-goals

- Dense mapping, semantic anything, multi-robot.
- Running on a Control Hub or Limelight.
- Replacing AprilTags; we fuse with them.
- Supporting iOS.

## Definitions used across specs

- **ATE:** absolute trajectory error (RMSE of position after SE(3) alignment) — the benchmark metric.
- **Drift:** end-point error / distance travelled on a closed or measured path.
- **Pose source:** a component that produces timestamped 6-DoF poses in its own frame.
- **Localizer:** the thing team code calls; combines a pose source, the frame alignment, lever
  arm, and tag corrections into a field-frame 2-D pose.
