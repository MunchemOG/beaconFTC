# Beacon (ORB-SLAM-style field localization) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> In this repo the orchestrator runs the workstreams below as parallel agents under
> `orchestration/CHARTER.md` (file ownership, append-only shared logs, no git by agents, one
> Gradle user at a time, evidence-style final report).

**Goal:** Keep a goBILDA Pinpoint pose within 5 cm after a 2:30 match (Stage 1) and 2-3 cm over
15-20 min (Stage 2) while driving, using monocular ORB-SLAM-style vision on Limelight 3A frames
tied only to BIOBUZZ field geometry, running in the RC app on a Control Hub.

**Architecture:** Pinpoint stays the continuous pose in `BeaconLocalizer`. A C++ core
(`beacon-native/`) takes Limelight frames + Pinpoint odometry and publishes absolute field-frame
fixes (capture-time stamped, with uncertainty) that the localizer blends at capture time. All
landmarks come from a per-season field model: hive and perimeter edges, ORB points constrained
to the floor and hive surfaces, optional tile seams. Development runs on a desktop bench (synthetic
renders + replayed logs) because no hardware is available for 3-4 days.

**Tech Stack:** Java 8 (TeamCode, FTC SDK 12.0), C++17 (CMake; macOS arm64 + NDK 27 arm64-v8a),
OpenCV, Eigen, g2o or Ceres (builder's choice, BSD), Python 3.11 + numpy for tools/sim/eval.

**Ambition:** the most accurate SLAM-based localizer ever built for FTC.

**Spec:** `docs/superpowers/specs/2026-11-17-beacon-orbslam-design.md` (approved 2026-11-17).

## Global Constraints

- No AprilTags anywhere. No venue features in the map. No map persists between runs.
- Field frame = FTC field frame defined by the perimeter walls: origin field centre, +z up,
  heading CCW. Metres, radians internally.
- Timestamps: `SystemClock.elapsedRealtimeNanos()` (Hub) for everything on the robot; datasets
  store `t_ns` int64 on that clock.
- `update()` < 0.5 ms; no allocation in per-loop hot paths; never block or throw on the OpMode
  thread; reject non-finite numbers at entry.
- Licensing: Beacon is MIT (LICENSE file at repo root and in beacon-native/). ASTRO (the reference) is unlicensed = all rights reserved: replicate and beat its behaviour from the videos, never copy its code. Deps only BSD/MIT/Apache (pinned, their notices kept in third_party/); no GPL, no ORB-SLAM source or vocabulary.
- Name: Beacon everywhere (code, entities, files, UI text). Never "astro".
- Build changes only in `TeamCode/build.gradle`; never edit `FtcRobotController/`,
  `build.common.gradle`, `build.dependencies.gradle`.
- Control Hub only; Limelight code within stock firmware; Limelight dedicated to Beacon.
- Design speeds 2.7 m/s, 7 rad/s. Stage 1: < 5 cm at t = 150 s while driving. Stage 2:
  ≤ 2 cm (bench) / 2-3 cm (field) over 20 min, heading ≈ 1°.
- Pin every dependency in `VERSIONS.md`; log deviations in `DECISIONS.md` (append via heredoc).

## Review Focus

1. **Clutter in view** (another robot or game pieces in front of the hive, venue through the
   clear walls): no fix is published that moves the pose toward the clutter; expect fewer fixes,
   not wrong ones. Test: Task C4 `clutter_does_not_bias`.
2. **Long stretches with no hive / wall edge in view** (robot facing a corner, occluded): Beacon
   reports vision-degraded, publishes nothing, pose is pure Pinpoint, recovers on first good view.
   Test: Task C5 `blind_stretch_then_recover`.
3. **Fast motion** (spins at 7 rad/s, 2.7 m/s straights, collisions with slip): fixes are gated,
   none are applied with a stale or blurred frame. Test: Task C4 `fast_spin_no_bad_fix`.
4. **Wrong start pose** typed by the team (e.g. 10 cm / 5° off): refined from walls+hive during
   INIT, or flagged; never silently kept. Test: Task C5 `start_pose_refined`.
5. **Native library missing or crashing / Limelight unplugged**: OpMode runs on pure Pinpoint with a
   telemetry line; no crash. Test: Task J2 `missingNativeLibFallsBackToPinpoint`, J3
   `frameSourceUnpluggedReportsAndRecovers`.

---

## Shared interfaces (fixed here so workstreams run in parallel)

### I1. Field model JSON (`beacon-field/<season>.json`)

```json
{
  "schema": 1, "season": "BIOBUZZ 2026-27", "source": "<drawing name + revision + URL>",
  "toleranceM": 0.0254,
  "floor": { "halfSizeM": 1.8288, "tilePitchM": 0.6096, "gridOriginM": [0.0, 0.0] },
  "walls": [ { "name": "red", "p0": [x, y], "p1": [x, y], "heightM": h, "railHeightM": h,
               "posts": [[x, y], ...] } ],
  "hive":  { "nominalPose": [x, y, yaw],
             "faces": [ { "name": "...", "vertices": [[x,y,z], ...], "textured": true } ],
             "edges": [ [[x,y,z],[x,y,z]], ... ] }
}
```
Coordinates in the field frame; hive faces/edges given at `nominalPose` = (0,0,0) local frame.
All numbers from official drawings (`toleranceM` from the Competition Manual).

### I2. Dataset directory (synthetic and recorded share one format)

```
<run>/camera.json   {"width","height","fx","fy","cx","cy","dist":[k1,k2,p1,p2,k3],
                      "mount":{"x","y","z","roll","pitch","yaw"},   // BeaconConfig T_robot_cam convention
                      "rollingShutterReadoutS": r}
<run>/field.json    copy of I1
<run>/frames.csv    idx,t_ns            (capture time)   + frames/<idx:06d>.png (8-bit gray)
<run>/odom.csv      t_ns,x,y,heading    (Pinpoint world, robot frame, ≥ 200 Hz)
<run>/start.csv     t_ns,x,y,heading    (start pose the team typed, field frame)
<run>/truth.csv     t_ns,x,y,heading    (field frame; synthetic only)
<run>/marks.csv     t_ns,x,y,heading    (field-frame tape-mark checks; recorded only, optional)
```

### I3. Native C API (`beacon-native/include/beacon/beacon.h`)

```c
typedef struct beacon_ctx beacon_ctx;
typedef struct { int width, height; double fx, fy, cx, cy, dist[5]; double rs_readout_s; } beacon_camera;
typedef struct { double x, y, z, roll, pitch, yaw; } beacon_mount;      /* T_robot_cam */
typedef struct { int64_t t_ns; double x, y, heading; } beacon_pose2;    /* robot, metres/rad */
typedef struct { int64_t t_ns; double x, y, heading, sigma_xy, sigma_heading;
                 int inliers; int kind; } beacon_fix;                   /* FIELD frame */
enum { BEACON_OK = 0, BEACON_DROPPED = 1, BEACON_ERR = -1 };
enum { BEACON_FIX_EDGES = 1, BEACON_FIX_POINTS = 2, BEACON_FIX_SEAMS = 4 };  /* kind bitmask */

beacon_ctx* beacon_create(const char* field_json, const beacon_camera*, const beacon_mount*,
                          const char* params_json /* nullable */, char* err, int err_len);
void beacon_set_start_pose(beacon_ctx*, const beacon_pose2* field_pose);
void beacon_push_odom(beacon_ctx*, const beacon_pose2* odom_world_pose);  /* any thread, O(1) */
int  beacon_push_frame(beacon_ctx*, int64_t capture_t_ns, const uint8_t* gray,
                       int width, int height, int stride);              /* copies; never blocks */
int  beacon_latest_fix(beacon_ctx*, beacon_fix* out);   /* 1 = new fix since last call, 0 = none */
int  beacon_status(beacon_ctx*);                        /* BeaconStatus codes */
void beacon_debug_line(beacon_ctx*, char* buf, int len);
void beacon_destroy(beacon_ctx*);
```
The core keeps its own odom-world → field alignment from the start pose and its fixes.

### I4. Java fix path (rename of the tag path)

`TagFix` → `FieldFix` (adds `sigmaXY`, `sigmaHeading`; NaN = use config gains),
`TagFixSource` → `FieldFixSource`, config section `"tag"` → `"fix"` (same keys). Localizer
history sized by time: `fix.historyS` (default 0.75 s).

---

## Workstream D: Docs (orchestrator, done before agents start)

### Task D1: Supersede old docs

**Files:** Modify `orchestration/CHARTER.md`, `beacon-spec/CLAUDE.md`, `RULES.md` (Q&A list);
append `DECISIONS.md`.

- [ ] Charter "What the product has to be" names the design doc + this plan as the source of truth;
      phone-camera language replaced.
- [ ] `beacon-spec/CLAUDE.md`: banner that specs 01, 03, 04, 05 and the phone-RC parts of 00/02
      are superseded by the design doc; conventions in 00 still apply.
- [ ] `RULES.md`: add Q&A question "Limelight 3A frames sent to the Control Hub over USB during a
      match: is that a 'continuous video stream' under R704 D?" and note Beacon uses no AprilTags.
- [ ] `DECISIONS.md` entry `2026-11-17 orchestrator: Beacon is ORB-SLAM-style field
      localization (supersedes phone VIO and AprilTag fixes)`.

---

## Workstream F: Field model, simulator, evaluation (Python) — owns `beacon-field/`, `tools/sim/`, `tools/eval/`

### Task F1: BIOBUZZ field model + schema validator

**Files:** Create `beacon-field/biobuzz-2026.json`, `beacon-field/README.md`,
`tools/sim/fieldmodel.py`, test `tools/tests/test_fieldmodel.py`.

**Interfaces:** Produces I1 file and `load_field(path) -> Field` (dataclass with `walls`,
`hive_faces`, `hive_edges`, `floor`), `Field.validate() -> list[str]` (empty = ok).

- [ ] Find the official BIOBUZZ field drawings / field setup guide (FIRST resources) and the
      Competition Manual field tolerance. If the hive geometry cannot be found, stop and report;
      do not invent it. Record source + revision in `"source"`.
- [ ] Tests: `test_perimeter_closed` (walls form a closed square, inner half-size 1.8288 ± 0.03 m),
      `test_hive_faces_planar` (each face's vertices within 1 mm of a plane),
      `test_validate_rejects_missing_hive`, `test_tile_grid_covers_floor` (6×6 tiles).
- [ ] Commit-ready: `tools/.venv/bin/python -m pytest tools/tests/test_fieldmodel.py -v` passes.

### Task F2: Synthetic dataset generator

**Files:** Create `tools/sim/render.py`, `tools/sim/trajectories.py`, `tools/sim/odomsim.py`,
`tools/sim/make_dataset.py`; test `tools/tests/test_sim.py`.

**Interfaces:** CLI `make_dataset.py --field F --traj {match,session,spin,blind,clutter}
--duration S --seed N --cam-height M --cam-pitch DEG --out DIR` → I2 directory.

- [ ] Renderer: exact projection of textured planar faces (floor tiles with seams and foam texture,
      hive faces, wall rails/posts, clear panels showing a random venue backdrop), lens
      distortion, optional rolling shutter and motion blur, clutter boxes (robots, game pieces).
      Defaults (unconfirmed 3A values, override via camera.json): 640×480, HFOV 54.5°, 30 Hz.
- [ ] Odometry sim (Pinpoint-like, defaults): per-axis scale error ±0.5 %, heading random walk
      0.5°/min, slip bursts of 1-3 cm on collisions, 200 Hz.
- [ ] Trajectories: `match` 150 s auto+teleop-like at up to 2.7 m/s and 7 rad/s; `session` 20 min;
      `spin`, `blind` (10 s facing a corner with walls occluded), `clutter`.
- [ ] Tests: `test_projection_matches_pinhole` (a known 3-D point lands at the analytic pixel
      ±0.1 px), `test_odom_drift_statistics` (seeded drift within configured bounds),
      `test_dataset_layout` (all I2 files present, monotonic `t_ns`).

### Task F2b: Hard odometry profile (added after F2: pure odometry passed the Stage 1 gate)

F2 finding: with nominal drift, odometry alone scores max 4.68 cm over a match (passes Stage 1), so
the gate does not test vision. Add `make_dataset.py --odom-profile {nominal,hard}`. `hard`:
per-axis scale error ±1.5 %, heading random walk 2° SD after 60 s, 4-8 collisions per match each
with 2-5 cm slip and up to 2° heading jump, start error 5 cm / 3° (overridable). Test
`test_hard_profile_fails_odometry_only` (odometry-only fused estimate fails stage1 on 5 seeds).
**All C4/C5 accuracy gates run on `hard` runs.**

### Task F3: Evaluation

**Files:** Create `tools/eval/eval_run.py`; test `tools/tests/test_eval.py`.

**Interfaces:** `eval_run.py --run DIR --est fused.csv [--gate stage1|stage2]` → prints max and
RMS position error, error at t = 150 s, max heading error; exit 0 if gate passes. Gates:
stage1 = position error < 0.05 m at every sample up to t = 150 s; stage2 = ≤ 0.02 m and ≤ 1°
over the whole run. Same script accepts `marks.csv` instead of `truth.csv`.

- [ ] Tests: `test_perfect_estimate_passes`, `test_5cm_offset_fails_stage1`,
      `test_interpolates_truth_at_est_times`.

### Task F4: Mac-side Limelight tools (team has the Limelight now, no Hub until next week) — top priority in F

Limelight docs (2026-11-17): plug the 3A into a laptop by USB-C (USB powers it, 4 W max), web UI
`http://limelight.local:5801`, built-in MJPEG stream (port 5800 per Limelight's own
port-forwarding example), OV5647 **rolling shutter**, 640×480 @ up to 90 fps, FOV H 54.5° V 42°.
Not verified on macOS by us; the tool must report clearly when the device is not reachable.

**Files:** Create `tools/limelight/ll_view.py`, `tools/limelight/ll_record.py`,
`tools/limelight/ll_calibrate.py`, `tools/limelight/passthrough_snapscript.py`,
`tools/limelight/README.md`; test `tools/tests/test_limelight_tools.py` (against a local fake MJPEG
server, no device).

**Interfaces:** `ll_record.py --url http://limelight.local:5800 --out DIR` writes an I2 directory
with `frames.csv` + `frames/` only (Mac monotonic clock; no `odom.csv`); `ll_calibrate.py --run DIR
--board 9x6 --square-mm 25` writes `camera.json` intrinsics; `ll_view.py` shows the live stream with
measured fps, frame size, JPEG size and inter-frame jitter.

- [ ] `passthrough_snapscript.py`: a SnapScript `runPipeline` that returns the unmodified frame
      (no crosshair/overlays) so the stream carries a clean image; README explains loading it in
      the web UI and setting resolution/fps/exposure.
- [ ] README: step-by-step for the team on macOS (plug in, check `limelight.local:5801`, then the
      stream; what to do if `limelight.local` does not resolve).
- [ ] Tests: `test_record_from_fake_mjpeg` (I2 layout, monotonic t_ns), `test_reports_unreachable`
      (clear message, exit code 2), `test_calibrate_on_rendered_board` (F2-rendered checkerboard →
      fx within 1 %).

---

## Workstream C: Native core (C++) — owns `beacon-native/`

Datasets come from F2; until F1/F2 land, use a hand-made perimeter-only field and a minimal
generator inside `beacon-native/bench/testdata/` (delete when F2 exists).

### Task C1: Library skeleton, C API, replay bench

**Files:** Create `beacon-native/CMakeLists.txt`, `include/beacon/beacon.h` (I3),
`src/api.cpp`, `src/field_model.{h,cpp}`, `bench/beacon_replay.cpp`, `tests/` (GoogleTest or
Catch2, pinned), `README.md`, `LICENSE` (MIT).

**Interfaces:** Produces I3; `beacon_replay --run DIR --out fixes.csv` (columns
`t_ns,x,y,heading,sigma_xy,sigma_heading,inliers,kind`) replaying odom and frames in time order.

- [ ] Tests: `api_rejects_bad_field_json` (create returns NULL + message), `push_frame_never_blocks`
      (1000 pushes while core is busy return BEACON_DROPPED, < 1 ms each),
      `nonfinite_odom_ignored`, `no_fix_without_frames`.
- [ ] Odometry optional: with no `odom.csv` (Mac handheld/cart recordings from F4) the core uses a
      constant-velocity prediction and publishes `kind` without odometry; documented as bench-only.
      `beacon_live --url http://limelight.local:5800 --start x,y,heading` (after C2) runs the core on
      the live Mac stream and draws projected field edges over the image.
- [ ] Builds on macOS arm64 (`cmake -S beacon-native -B beacon-native/build && cmake --build ...`)
      and cross-builds for Android arm64-v8a with NDK 27 toolchain (`libbeacon.so`, report size).

### Task C2: Pose prediction + model-edge tracking

**Files:** `src/predict.{h,cpp}` (odom interpolation at capture time, alignment), `src/edges.{h,cpp}`
(project wall rails, posts, wall-floor line, hive edges; search along normals; robust planar
pose solve).

- [ ] Tests: `edge_fix_recovers_offset` (synthetic `match` run with odometry offset 10 cm / 3°: edge
      fixes converge to < 2 cm), `predict_interpolates_at_capture_time`.

### Task C3: Surface-constrained ORB map

**Files:** `src/features.{h,cpp}` (ORB, OpenCV), `src/map.{h,cpp}` (keyframes, map points created by
ray-surface intersection on floor/hive faces only, culling of points inconsistent with their
surface over ≥ 3 keyframes).

- [ ] Tests: `points_only_on_field_surfaces` (on `clutter` run, ≥ 99 % of surviving points within
      2 cm of floor or hive faces), `venue_through_wall_rejected`.

### Task C4: Sliding-window optimization + gating

**Files:** `src/window.{h,cpp}` (planar keyframe poses, odometry factors, reprojection, surface
priors, edge factors, optional seam factors), `src/gate.{h,cpp}`.

- [ ] Tests: `clutter_does_not_bias`, `fast_spin_no_bad_fix` (no fix while |ω| above gate; no fix
      with error > 3 cm on `spin`), **`stage1_match_gate`** (`eval_run.py --gate stage1` passes on 5
      seeded `match` runs via the replay → fusion path in J4).

### Task C5: Relocalization, start-pose refinement, seams decision

**Files:** `src/reloc.{h,cpp}` (BoW over keyframes with vocabulary trained on synthetic + practice
field images; model-based search around predicted pose), `src/seams.{h,cpp}`.

- [ ] Tests: `blind_stretch_then_recover`, `start_pose_refined` (10 cm / 5° wrong start →
      < 2 cm within 3 s of stillness), **`stage2_session_gate`** on 3 seeded `session` runs.
- [ ] Measure seams on vs off over 10 runs; record the result in `DECISIONS.md`; remove seams if
      they do not help.

---

### Task C6: Explore mode + Rerun viewer (team's reference videos; do right after C1)

Design §6.4a. Reference: `~/Downloads/robotics-slam-demo-1.mp4`, `~/Downloads/slam-test-3.mp4`
(read-only; view frames with ffmpeg). The Mac experience must look and behave like them.

**Files:** `src/init_mono.{h,cpp}` (monocular two-view initialization), `src/loop.{h,cpp}` (BoW place
recognition, loop closing, pose-graph optimization), `bench/viewer_rerun.{h,cpp}` (Rerun C++ SDK,
pinned, desktop only), extend `bench/beacon_live.cpp` and `bench/beacon_replay.cpp` with `--view`
and `--mode explore|field`; add `beacon_replay --euroc DIR` (cam0 + timestamps).

**Viewer entities — 1:1 with the reference videos' RViz displays** (team: "1:1 of the setup and
capabilities"; the reference shows ASTRO names, Beacon uses its own), each toggleable, same default on/off as the
videos:

| Reference display | Beacon entity | Default |
| --- | --- | --- |
| Grid | `beacon/world/grid` | on |
| TF | `beacon/world/tf` (camera + robot frames) | off |
| `/astro/points` | `beacon/points` (currently tracked map points, 3-D) | on |
| `/astro/backend_pose_imu_viz` | `beacon/backend_pose_imu_viz` (optimized trajectory) | on |
| `/astro/image/cam0` | `beacon/image/cam0` (+ green tracked / blue new keypoints) | on |
| `/astro/loop_closures` | `beacon/loop_closures` | on |
| `/astro/pose_imu` | `beacon/pose_imu` (Pinpoint/odometry pose; empty without odometry) | off |
| `astro_kfs` | `beacon/kfs` (keyframe poses) | on |
| `frontend_map` | `beacon/frontend_map` (frontend local map) | off |
| `astro_pose_graph` | `beacon/pose_graph` | on |
| `astro_lc_database_map` | `beacon/lc_database_map` (all points in the loop-closure database) | off |
| `backend_points` | `beacon/backend_points` (points refined by BA) | on |

**Layout (Rerun blueprint, matches the videos):** entity/visibility panel top-left, camera image
bottom-left, 3-D orbit view on the right with the grid, fps shown. Live at ≥ 11-15 fps on the Mac.

- [ ] Explore mode runs field-free: any features, mono init, keyframes, local BA, loop closing.
- [ ] Explore mode **uses Pinpoint odometry when `odom.csv` / `beacon_push_odom` data is present**
      (the reference system used the Pinpoint as its "IMU"): metric scale from the first keyframe
      (no mono init), odometry factors in local BA and the pose graph, Pinpoint carries the pose
      through blurred/lost frames. Robot + Pinpoint + explore = the reference videos 1:1.
      Test: `explore_with_odom_is_metric` (synthetic run, map scale error < 1 %, trajectory ATE
      without scale alignment < 0.05 m).
- [ ] Tests: `explore_euroc_mh01_tracks` (MH_01 cam0: tracking kept ≥ 90 % of frames, ≥ 1 loop
      closure, ATE after Sim(3) alignment reported; record the number, target < 0.10 m),
      `explore_mode_needs_no_field_json`, `viewer_logs_all_entities` (Rerun recording file contains
      every entity above).
- [ ] Screenshot of the viewer on MH_01 saved to `beacon-native/bench/screenshots/` for the
      orchestrator to compare against the reference videos.

---

## Workstream J: Java / Hub — owns `TeamCode/src/main/java/.../beacon/**`, its tests, `TeamCode/build.gradle`, `TeamCode/src/main/cpp/`. Sole Gradle user.

### Task J1: Fix path rename + time-sized history (I4)

**Files:** Rename `TagFix.java`→`FieldFix.java`, `TagFixSource.java`→`FieldFixSource.java`;
modify `BeaconLocalizer.java`, `BeaconConfig.java`, `BeaconRig.java`, all tests. Park
`LimelightTagSource.java`, `LimelightFixFilter.java`, `LimelightTest.java` under
`beacon/parked/` (excluded from the rig).

- [ ] Tests: existing 115 pass after rename; new `historyCoversHalfSecondAtPinpointRate`
      (1.5 kHz samples, fix 0.5 s old is applied, not "older than history"),
      `sigmaWeightsGain` (fix with small sigma moves pose more than one with large sigma),
      `configTagSectionRejectedWithHint` (old `"tag"` key → error naming `"fix"`).
- [ ] `./gradlew :TeamCode:testDebugUnitTest --offline` → BUILD SUCCESSFUL, 0 failures.

### Task J2: Native bridge (JNI) with safe fallback

**Files:** Create `beacon/vision/NativeBeacon.java` (JNI declarations), `beacon/vision/VisionFixSource.java`
(implements `FieldFixSource`; owns frame + odometry feeding thread), `TeamCode/src/main/cpp/beacon_jni.cpp`,
`TeamCode/src/main/cpp/CMakeLists.txt` (adds `../../../../beacon-native`); modify `TeamCode/build.gradle`
(`externalNativeBuild`, `abiFilters 'arm64-v8a'`).

**Interfaces:** Consumes I3. Produces `VisionFixSource(FrameSource frames, PoseSource odom, BeaconConfig cfg)`,
`String debugLine()`, `boolean isNativeLoaded()`.

- [ ] Tests (JVM, native absent): `missingNativeLibFallsBackToPinpoint` (rig runs, status line says
      "vision: native library not loaded"), `odomFedWithoutBlockingUpdate`.
- [ ] `./gradlew :TeamCode:assembleDebug --offline` succeeds and the APK contains
      `lib/arm64-v8a/libbeacon.so`.

### Task J3: Frame source + probe / data-collection OpMode (top priority: ready before hardware)

**Files:** Create `beacon/vision/FrameSource.java` (interface: `start()`, `stop()`, `latest()` →
gray frame + capture `tNs`, `debugLine()`), `beacon/vision/LimelightFrameSource.java` (candidate paths
from design §6.2(a), endpoint configurable in beacon.json `"limelight": {"streamUrl": ...}`),
`beacon/vision/DatasetRecorder.java` (writes I2 to `/sdcard/FIRST/beacon/runs/<date>/`, off the OpMode
thread), `beacon/opmodes/BeaconProbeOpMode.java`.

- [ ] Probe OpMode shows: frames/s, resolution, decode time, capture-to-Hub latency estimate and how it
      was measured, dropped frames, CPU; gamepad A = mark (writes `marks.csv` row with the typed mark
      coordinates), B = start/stop recording; checkerboard capture mode for intrinsics.
- [ ] Tests: `frameSourceUnpluggedReportsAndRecovers`, `recorderWritesDatasetLayout` (I2 files, monotonic
      `t_ns`), `recorderNeverBlocksLoop` (full queue drops frames, counts them).

### Task J4: Desktop fusion replay

**Files:** Create test-source `beacon/ReplayFusion.java` + `ReplayFusionTest.java`: reads `odom.csv`,
`start.csv`, a `fixes.csv` (C1 format), runs the real `BeaconLocalizer` with a fake clock, writes
`fused.csv` (`t_ns,x,y,heading`). Path via system property `beacon.replay.run`.

- [ ] Test: `replayWithNoFixesEqualsOdometry`, `replayWithExactFixesTracksTruth` (synthetic fixes equal
      to truth → fused within 5 mm).

---

## Execution order

1. D1 (orchestrator) → start F, C, J in parallel.
2. J3 first in J (hardware arrives in 3-4 days), then J1, J4, J2.
3. F4 → F1 → F2 → F3 (F4 first: the team can use the Limelight on a Mac now); C1 uses its own stub data until F2 lands; C4/C5 gates need F2, F3, J4.
4. Orchestrator re-runs every claimed test, then a reviewer per workstream (`orchestration/reviews/`).
