# 01 — Desktop harness (macOS-native): reproduce the demo and get a benchmark

Goal: the thing in the student's videos — an open-source VIO running on EuRoC with a live view of
tracked features, sparse map points and the trajectory — plus a script that prints ATE against
ground truth. This is the yardstick for every phone build. **No Linux, no ROS, no VM.** Everything
here runs natively on a Mac (Apple Silicon or Intel).

## Decisions made (record in `DECISIONS.md`)

- **Estimator: OpenVINS, ROS-free build** (`cmake -DENABLE_ROS=OFF`). Why: it supports a
  monocular camera + IMU (which is what a phone is), it is an MSCKF filter (no heavy nonlinear
  optimisation, so lower CPU per frame than VINS-Fusion), feature extraction runs fast, and its
  core is a plain C++ library we can later cross-compile with the NDK (04). Licence GPL-3.
- **Not Basalt:** its VIO pipeline is built around stereo cameras and a phone has one; macOS
  support was not verified. **Not VINS-Fusion as primary:** ROS-coupled, Ceres-heavy; keep as an
  optional second opinion only if the OpenVINS build works first.
- **Viewer: Rerun** (`pip install rerun-sdk`) instead of RViz. It runs natively on macOS, shows
  the feature image, 3D points coloured by Z, keyframe poses and the trajectory with a timeline.
  The ROS-free OpenVINS build has no built-in visualiser, so we write our own driver (below).
- **Evaluation: `evo`** (pure Python, works on macOS).

## Environment

- macOS 13+; Xcode Command Line Tools; Homebrew.
- `brew install cmake eigen boost opencv ceres-solver pkg-config python@3.11` (only what the
  OpenVINS ROS-free build asks for; check its current README and pin versions in `VERSIONS.md`).
- Python venv: `pip install evo rerun-sdk numpy opencv-python pyyaml allantools`.
- Apple Silicon note: build everything arm64-native; do not use Rosetta/x86 Homebrew.

## Data

EuRoC MAV dataset, **ASL format** (no ROS bags needed): `MH_01_easy`, `MH_03_medium`,
`V1_01_easy`, `V1_02_medium` from ETH's download page (zip per sequence).
Ground truth: `mav0/state_groundtruth_estimate0/data.csv`.
Use **cam0 only + imu0** (monocular-inertial) so the test matches the phone.

## Tasks

1. `desktop-harness/setup.sh`: brew deps, clone `rpng/open_vins` at a pinned commit, build with
   `-DENABLE_ROS=OFF`, build our driver. Must run clean on a fresh Mac; if the ROS-free build
   fails on macOS, write the exact error and the smallest patch into `DECISIONS.md` first.
2. `desktop-harness/data/get_euroc.sh <sequence>`: downloads + unzips a sequence.
3. `desktop-harness/driver/euroc_driver.cpp`: reads an ASL sequence (cam0 images + imu0 CSV) in
   timestamp order, feeds `ov_msckf::VioManager` (`feed_measurement_imu`,
   `feed_measurement_camera`), and after each frame writes: (a) one trajectory row in the
   project log format (`00-PROJECT.md`, Logging); (b) tracked features for that frame (id, u, v,
   track length) to `features.csv`; (c) current SLAM/MSCKF 3D points to `points.csv`.
   Throttle option `--realtime` to play at sensor speed, default = as fast as possible.
4. `desktop-harness/viz/view.py <run_dir>`: Rerun script that replays the run: image panel with
   features (**green = long-tracked, blue = newly detected**, matching the demo), 3D view with
   points as small spheres coloured by Z, the pose trail, and ground truth in grey. Entity names
   mirror the demo: `astro/image/cam0`, `astro/points`, `astro/pose_imu`, `astro/backend_points`.
5. `desktop-harness/eval/ate.py <est.csv> <gt.csv>`: `evo_ape` with SE(3) alignment (add scale
   for mono only if scale is unobservable; with IMU it is observable, so report **unscaled** as
   the headline number). Prints ATE RMSE / mean / max in metres, saves a top-down PNG.
6. `desktop-harness/eval/speed.py`: reports per-frame processing time (mean, p95, max) and
   effective fps from the driver's timing log. **Speed is a first-class result, not a footnote.**
7. Record a screen capture of MH_01 and V1_01 runs; put ATE + speed numbers in `RESULTS.md`.

## Acceptance

- `./run.sh MH_01_easy` builds (if needed), runs OpenVINS, and opens the Rerun view within
  ~60 s of a clean setup.
- ATE on MH_01_easy and V1_01_easy is reported; target ≤ 0.10 m, and we record how far below
  that OpenVINS gets with the tuned config (below). If far worse than published OpenVINS
  numbers, the config is wrong, not the data.
- Mean per-frame time on the Mac < 10 ms at 20 Hz EuRoC input (the phone will be several times
  slower; this gives headroom).
- The trajectory CSV is in the project log format so `ate.py` evaluates phone logs later.

## Tuning pass (do after the baseline works; each change measured, keep what helps)

Config lives in `config/euroc/estimator_config.yaml`; sweep one knob at a time, log ATE + ms:
- `num_pts` (tracked features per frame): try 100 / 150 / 200 — fewer = faster, more = steadier.
- `max_clones` (sliding-window size) 8 → 11, `max_slam` (persistent landmarks) 0 → 25: more
  landmarks improve accuracy, cost time.
- `use_klt: true`, `grid_x/grid_y` 5×3 to spread features across the image, `min_px_dist` 15–20.
- `fast_threshold`, `downsample_cameras` (on/off) for speed.
- `zupt_*` (zero-velocity update): enable — FTC robots stop often (at scoring, at start) and
  zero-velocity updates remove drift while stationary.
- Online calibration flags (`calib_cam_extrinsics`, `calib_cam_timeoffset`, `calib_cam_intrinsics`):
  test on/off; these matter much more on a phone than on EuRoC.
Write the best config and the table of runs in `RESULTS.md`.

## Notes for the implementer

- The estimator output rate is the camera rate (20 Hz on EuRoC). The OpenVINS state also supports
  IMU-rate "fast propagation" of the latest pose; use it in 04 for 100+ Hz pose reads between frames.
- Keep EuRoC YAMLs (intrinsics, extrinsics, IMU noise) in `config/euroc/`; phone configs in `05`
  mirror their structure exactly.
