# 05 — Calibration: where the centimetres come from

Tier 2 accuracy is set by calibration, not code. Tier 1 (ARCore) is self-calibrated by Google;
only the robot-side transform applies there.

## A. Robot-side (both tiers): `T_robot_cam` and lever arm

- Measure in CAD: camera lens centre relative to the robot's centre of rotation, and the camera's
  orientation (typically pitched down 10–25° for a forward-looking phone). Enter in `astro.json`.
- Verify empirically with `AstroCalibrationOpMode`:
  1. Spin 360° in place: translation leak → lever-arm error. Fix x/y offsets until ≤ 1 cm.
  2. Push 1 m forward / 1 m left: sign and scale.
  3. Pitch check: z drift while driving flat → pitch angle error.

## B. Phone-side (Tier 2): macOS-native calibration

No Linux needed. Three stages, each runnable on a Mac:

Target: an AprilGrid or ChArUco board printed at known scale (verify printed size with a ruler and
put the real numbers in the target yaml).

1. **Camera intrinsics** (`calibration/tools/calib_intrinsics.py`, OpenCV `aruco` +
   `calibrateCamera`/`fisheye.calibrate`): record 60 s of the board with slow, full-image coverage
   (corners, edges, tilts); model `pinhole-radtan` (or equidistant if wide-angle). Reprojection
   error ≤ 0.3 px. Reject frames with motion blur.
2. **IMU noise** (`calibration/tools/allan.py`, `allantools`): phone stationary on a table, ≥ 2 h
   at full rate; extract gyro/accel noise density and random walk. Inflate 3–5× for the estimator
   config (standard practice).
3. **Camera–IMU extrinsics + time offset**: use **OpenVINS online calibration** as the primary
   method (`calib_cam_extrinsics: true`, `calib_cam_timeoffset: true`, initial guess from the
   phone's Camera2 `LENS_POSE_*` and `SENSOR_ORIENTATION`), run on a 90 s recording that rotates
   the phone through all 3 axes in front of the board. Read the converged values from the output
   log and freeze them into the config (turn the flags off for competition runs unless tests in
   `06` show online calibration helps). Expect |timeshift| of a few ms on phones; record it.
   **Optional cross-check:** Kalibr (`ethz-asl/kalibr`) via Docker with `--platform linux/amd64`.
   It is CLI-only, so it works under emulation but is slow. Use it once to validate step 3, not
   as part of the daily workflow.
4. Write `calibration/phones/<model>.yaml` in OpenVINS format (`estimator_config.yaml`,
   `kalibr_imucam_chain.yaml`, `kalibr_imu_chain.yaml`; the format is Kalibr-style, so the files
   stay Kalibr-compatible). Mirror the EuRoC layout from `01`.

Record with the same `CameraImuFeeder` settings used at runtime (exposure, focus, OIS off,
resolution). A calibration at auto-exposure/auto-focus is invalid.

## C. Field alignment

The VIO world frame starts wherever the phone initialized. `AstroLocalizer.setPose()` at auto
start (robot placed on a known tile mark) or the first AprilTag fix sets `T_field_sourceworld`.
Document the auto-start placement jig in `WIRING.md` (a printed corner template beats eyeballing).

## Deliverables

- `calibration/README.md` with the above as a checklist.
- `calibration/targets/aprilgrid_6x6.yaml`, `calibration/phones/galaxy_s22/` (example: the three OpenVINS yaml files),
  `calibration/imu/<model>_allan.yaml`.
- A `tools/check_calibration.py` that loads a yaml and sanity-checks ranges (focal lengths vs
  resolution, |timeshift| < 50 ms, noise params positive, quaternion normalized).
