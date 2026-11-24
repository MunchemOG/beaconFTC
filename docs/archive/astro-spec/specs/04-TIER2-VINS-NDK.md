# 04 — Tier 2: `VinsPoseSource` — open-source VIO core on the phone via the NDK

Goal: the actual "ASTRO" rebuild. Our own camera + IMU capture feeding an open-source
visual-inertial estimator compiled for arm64, running inside the RC app. Same `PoseSource`
interface as Tier 1 so nothing above it changes.

## Decision 0 — which estimator (do this first, record in `DECISIONS.md`)

| Option | Licence | ROS-free core? | Mono + IMU? | Notes |
| --- | --- | --- | --- | --- |
| **OpenVINS (RPNG)** | GPL-3 | Yes (`-DENABLE_ROS=OFF`, v2.0+) | Yes | MSCKF filter: lowest CPU, fast feature extraction, online cam-IMU calibration, builds on macOS (01) |
| VINS-Fusion (HKUST) | GPL-3 | Partial (needs patching) | Yes | Closest to the demo; Ceres makes it heavier on a phone |
| Basalt (TUM) | BSD-3 | Yes | Stereo-oriented | Not suited to a single phone camera; macOS unverified |

**Default: OpenVINS.** It is the choice that serves "faster and more accurate": filter-based
update is cheaper per frame than sliding-window optimisation, and it models camera-IMU time
offset and rolling-shutter-free pose output cleanly. Keep the GPL inside
`astro-native/third_party/` and decide the repo licence in `08`. Do not write a custom estimator.
The rest of this spec says "the core"; where a VINS-Fusion key is named (e.g. `rolling_shutter`),
use the OpenVINS equivalent from the pinned config and note it in `DECISIONS.md`.

## Native build — `astro-native/`

- CMake project, `ANDROID_ABI=arm64-v8a`, `ANDROID_PLATFORM=android-26`, C++17, `-O3 -march=armv8-a`.
- Dependencies built from source for Android (pin versions): Eigen (header-only), Ceres only if OpenVINS needs it in the pinned version (with `MINIGLOG`, no `SUITESPARSE`, `EIGENSPARSE=ON`), OpenCV (use the `.so` bundled in the FTC SDK
  if ABI-compatible; otherwise build a minimal OpenCV 4.x with `core imgproc features2d video
  calib3d`), Boost-free (patch any boost usage).
- Use OpenVINS `ov_core`, `ov_init`, `ov_msckf` (ROS-free); delete nothing, link the three libs. For VINS-Fusion fallback keep `estimator/`, `feature_tracker/`, `initial/`, `factor/`, `utility/` and delete `loop_fusion` for v1 (OpenVINS has no loop closure in its core, which is fine: absolute correction comes from AprilTags in 02/06). Provide a C API:
  ```
  astro_handle astro_create(const char* config_yaml);
  void astro_push_imu(astro_handle, int64_t t_ns, double ax, ay, az, gx, gy, gz);
  void astro_push_image(astro_handle, int64_t t_ns, const uint8_t* gray, int w, int h, int stride);
  int  astro_get_pose(astro_handle, astro_pose* out);   // latest, non-blocking; returns status
  void astro_destroy(astro_handle);
  ```
- Threading inside the core: image push enqueues; the estimator thread consumes. Back-pressure:
  if the queue exceeds 2 frames, drop the oldest (log a counter). Never block the camera callback.
- JNI shim in `TeamCode/src/main/cpp/astro_jni.cpp`, loaded by `vins/VinsNative.java`.

## Capture — `vins/CameraImuFeeder.java`

- **Camera:** Camera2, rear camera, `YUV_420_888`, 640×480 (or the nearest 4:3 size), target
  30 fps (60 if the sensor supports it at that size), **manual exposure/ISO** (`CONTROL_AE_MODE_OFF`,
  exposure ≤ 4 ms, ISO as needed) and **fixed focus** (`CONTROL_AF_MODE_OFF`, `LENS_FOCUS_DISTANCE`
  = hyperfocal or ∞). OIS off if the device allows (`LENS_OPTICAL_STABILIZATION_MODE_OFF`) — OIS
  silently breaks the camera-IMU extrinsic. Use `SENSOR_TIMESTAMP` from `CaptureResult`; verify
  `SENSOR_INFO_TIMESTAMP_SOURCE == REALTIME` (same clock as IMU) and record the result.
- Pass the Y plane directly to `astro_push_image` (no colour conversion).
- **IMU:** `SensorManager` with `TYPE_ACCELEROMETER_UNCALIBRATED` and `TYPE_GYROSCOPE_UNCALIBRATED`
  (raw, let the estimator estimate bias), `SENSOR_DELAY_FASTEST`, request batch latency 0. Log the
  achieved rate; need ≥ 200 Hz. Push on the sensor thread.
- **Rolling shutter:** read `SENSOR_INFO_ROLLING_SHUTTER_SKEW` / `SENSOR_ROLLING_SHUTTER_SKEW`
  and set the estimator's rolling-shutter readout time (VINS-Fusion `rolling_shutter: 1`,
  `rolling_shutter_tr`).
- Record a replayable dataset (`images/*.pgm` + `imu.csv` + `cam_timestamps.csv`) when logging is
  enabled; this is what calibration (05) and the on-phone replay test consume.

## `vins/VinsPoseSource implements PoseSource`

- `start()`: load `astro.json → vins.configPath` (YAML from 05), `astro_create`, start feeder.
- `latest()`: `astro_get_pose` → `Pose3dStamped` (z-up, gravity-aligned world frame at init).
- Status mapping: estimator not initialized → 1; tracking → 2; feature loss / divergence flag → 3.
- `stop()`: stop feeder first, then destroy core.

## On-phone replay test (before any robot test)

`opmodes/AstroReplayOpMode` or a plain Android instrumentation test: feed EuRoC MH_01 images+IMU
(copied to `/sdcard/FIRST/astro/euroc/`) through `CameraImuFeeder`'s push path with the EuRoC
YAML, write the log, run `desktop-harness/eval/ate.py` on it. **The phone build must reproduce the
desktop ATE within 20 %.** This isolates porting bugs from calibration bugs.

## Performance budget (Snapdragon 8 Gen 1 class)

| Stage | Budget |
| --- | --- |
| Feature tracking (150 pts, KLT, 640×480) | ≤ 6 ms / frame |
| Sliding-window optimization (window 10, Ceres) | ≤ 25 ms / keyframe, on its own thread |
| End-to-end pose latency (exposure mid-point → `latest()`) | ≤ 50 ms |
| CPU | ≤ 2 big cores sustained; log thermal |

## Acceptance

- Replay test passes (ATE within 20 % of desktop).
- `06-TESTING.md` protocol on the robot with `tier: "vins"` meets the same numbers as Tier 1 or
  better; spin-in-place leak ≤ 1 cm; no divergence over a 10-minute drive.
- Loop closure / pose graph (`loop_fusion`) added only after the above pass, as a separate PR.
