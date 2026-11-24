# beacon-native

The Beacon vision core (C++17). One codebase, two modes:

- **field** (robot): Pinpoint odometry is the motion prior; each Limelight frame is aligned to the
  BIOBUZZ field model (perimeter rails, posts, wall-floor line, static hive frame edges) and an
  absolute field-frame fix is published with a 1-sigma, when the gate accepts it.
- **explore** (Mac bench, the team's reference videos): general monocular ORB SLAM, field-free:
  ORB features anywhere, two-view initialization, keyframes, local bundle adjustment, bag-of-words
  place recognition, loop closing with essential-graph optimization, relocalization. With Pinpoint
  odometry (`beacon_push_odom` / `odom.csv`) the map is metric from the first keyframe, odometry
  factors enter local BA and the pose graph, and the odometry carries the pose through frames that
  cannot be tracked.

C API: `include/beacon/beacon.h` (plan interface I3, plus `beacon_reset_odom_world`). MIT, see
`LICENSE`.

## Build

Dependencies are built once into `third_party/` (gitignored; versions pinned in `VERSIONS.md`):

```sh
cd beacon-native
mkdir -p third_party/src   # put the pinned archives here (see VERSIONS.md for URLs and sha256)
./scripts/build_opencv.sh host        # OpenCV 4.12.0, minimal static
./scripts/build_opencv.sh android
./scripts/build_ceres_rerun.sh ceres-host
./scripts/build_ceres_rerun.sh ceres-android
./scripts/build_ceres_rerun.sh rerun-host     # Rerun C++ SDK 0.41.0 (+ Arrow 18.0.0)

cmake -S . -B build && cmake --build build -j10          # macOS arm64: library, tests, bench tools
./build/beacon_tests                                     # GoogleTest
./scripts/build_android.sh                               # build-android/libbeacon.so (arm64-v8a)
```

The Android library is built with the NDK 27 toolchain file directly (no Gradle). Only
`include/` is a public include directory; `src/` is private.

## Bench tools (host only)

| Tool | What it does |
| --- | --- |
| `beacon_replay --run DIR [--out fixes.csv] [--mode field\|explore] [--no-odom]` | replays an I2 data set (odometry and frames merged in time, each frame delivered 40 ms after capture). Writes `t_ns,x,y,heading,sigma_xy,sigma_heading,inliers,kind`. |
| `beacon_replay --euroc DIR [--traj traj.txt]` | EuRoC cam0 in explore mode; prints tracking %, loop closures and ATE after Sim(3) alignment. |
| `beacon_live [--url http://limelight.local:5800] [--camera camera.json] [--mode explore\|field]` | live Limelight MJPEG stream on the Mac, viewer on by default (`--no-view` to turn it off, `--rrd FILE` to record, `--record DIR` to save an I2 run). Exit code 2 when the stream is unreachable. |
| `scripts/fused_gate.sh RUN OUT [stage1\|stage2]` | the C4/C5 gate path: core fixes -> Java ReplayFusion (holds `.gradle.lock`) -> `tools/eval/eval_run.py` |
| `beacon_overlay --run DIR --frame N --out PNG [--pose x,y,h]` | draws the field model projected with the truth (or a given) pose over a frame: checks intrinsics, mount and field-model conventions by eye. |
| `beacon_vocab_train --out vocab.bin --euroc DIR \| --run DIR` | trains Beacon's own ORB vocabulary (no ORB-SLAM vocabulary). |
| `beacon_make_testdata --out DIR` | minimal synthetic perimeter-only runs (used by the unit tests; the Field engineer's `tools/sim/make_dataset.py` is the real simulator). |

All tools: `--view` (live Rerun viewer), `--rrd FILE` (save a recording), `--params FILE_OR_JSON`.

### Viewer (Rerun 0.41.0)

Entities match the reference videos' RViz displays (plan Task C6): `beacon/world/grid`,
`beacon/world/tf`, `beacon/points`, `beacon/backend_pose_imu_viz`, `beacon/image/cam0` (+ `/tracked`
green, `/new` blue, `/field` projected model), `beacon/loop_closures`, `beacon/pose_imu`,
`beacon/kfs`, `beacon/frontend_map`, `beacon/pose_graph`, `beacon/lc_database_map`,
`beacon/backend_points`, plus `beacon/world/camera`, `beacon/world/field`, `beacon/status`. Layout
and default visibility come from `bench/viewer/beacon.rbl` (regenerate with
`third_party/venv/bin/python bench/viewer/make_blueprint.py bench/viewer/beacon.rbl`). To look at a
recording: `third_party/src/rerun-cli-0.41.0 run.rrd bench/viewer/beacon.rbl`.

### Mac + Limelight this week (explore mode, no odometry)

1. Plug the Limelight into the Mac, check `http://limelight.local:5801`, load the passthrough
   SnapScript (`tools/limelight/README.md`).
2. `./build/beacon_live --camera camera.json` (intrinsics from `tools/limelight/ll_calibrate.py`;
   without it the 3A defaults are used: 640x480, HFOV 54.5 deg).
3. Move the camera sideways first (monocular initialization needs parallax). Scale is arbitrary
   without odometry. The fix `kind` carries `BEACON_FIX_NO_ODOM` in this bench-only mode.
4. Explore mode needs a vocabulary for loop closing and relocalization:
   `--params '{"vocabularyPath": "bench/vocab/beacon_orb_v1_01.bin"}'` (train it with
   `beacon_vocab_train`; for the field, train on practice-field recordings).

Frame times in `beacon_live` are Mac arrival time minus `--latency-ms` (30); not capture time.

## params_json (all optional; unknown keys are an error)

| Key | Default | Meaning |
| --- | --- | --- |
| `mode` | `"field"` | `"field"` or `"explore"` (`field_json` may be NULL in explore mode) |
| `synchronous` | false | bench only: process frames inside `beacon_push_frame` (deterministic replays) |
| `lostAfterS` | 1.0 | status LOST (vision degraded) after this long without an accepted fix |
| `odomMaxExtrapolateS`, `odomHistoryS` | 0.10, 4.0 | odometry prediction limits |
| `startSigmaXY`, `startSigmaHeading` | 0.15 m, 0.10 rad | trust in the typed start pose |
| `driftPerMetre`, `driftHeadingPerRad`, `driftHeadingPerS` | 0.01, 0.005, 0.0005 | odometry drift model for the internal alignment |
| `orbFeatures`, `orbScaleFactor`, `orbLevels`, `orbFastThreshold` | 1000, 1.2, 8, 20 | ORB front end |
| `edgeSampleStepPx`, `edgeSearchMaxPx`, `edgeSearchMinPx`, `edgeMinGradient`, `edgeHuberPx`, `edgeSigmaPx`, `edgeCorrelationInflation`, `edgeMaxRangeM`, `edgeMaxSamplesPerEdge` | 8, 60, 6, 25, 2, 1.5, 3, 5, 80 | model-edge tracking |
| `edgeUsePolarity` | true | black rails: horizontal wall edges must have the rail on their known side |
| `edgeWindowFrames`, `edgeWideWindowFrames`, `edgeWindowMinSpacingS` | 2, 6, 0.15 s | field mode solves one odometry->field alignment over the newest 2 frames (tracking) or 6 (wide search, re-verification); frames at least 0.15 s apart |
| `edgeSeamWeight`, `edgeSeamMaxRangeM`, `edgeSeamMinContrast` | 0.5, 2.0 m, 8 | tile seams as thin dark lines on the floor (weight 0 = off) |
| `hiveStartEvidence`, `hiveEvidenceDecay`, `hiveEvidenceGain`, `hiveDecide`, `hiveMinSamples`, `hiveSupportNeutral` | 3, 0.9, 4, 1, 20, 0.2 | two-state tilt estimate of each tipping hive part; its edges are used only in the estimated state |
| `relockCheckS`, `relockGridXY`, `relockGridHeading`, `relockMargin` | 2.0 s, 0.05 m, 0.035 rad, 0.08 | periodic re-verification of the lock against nearby alternatives |
| `fixSigmaPerQXY`, `fixSigmaPerQHeading`, `fixSigmaQuadratic`, `fixSigmaKneeQ`, `fixSigmaFloorXY`, `fixSigmaFloorHeading` | 0.0055 m, 0.004 rad, 0.006 m, 2.4, 0.008 m, 0.005 rad | calibrated sigma from q = residual RMS px / inlier ratio^2 (linear + quadratic tail); used for the gate, the core's own alignment update and the published fix |
| `searchSolvesPerFrame`, `searchBudgetMs`, `searchRescoreTop` | 3, 8 ms, 2 | wide searches and lock re-checks run a few hypotheses per frame; on the robot also stop after `searchBudgetMs` (ignored in synchronous replays for determinism) |
| `partMinSamples`, `partMinInlierFrac` | 30, 0.2 | a model part (a wall, the hive frame) with this many samples but fewer inliers blocks the fix and forces a re-check |
| `stillRelockMinInliers`, `stillRelockMaxSigmaXY` | 150, 0.015 m | lost precision while standing still: local re-lock accepted only above this quality |
| `relockWideSigmaXY`, `relockWideSigmaHeading` | 0.06 m, 0.05 rad | after 3x `relockAfterRejects` consecutive rejections: full widening (wide search) |
| `odomScaleEstimate`, `odomScaleMinPathM`, ... | false | online per-axis odometry scale from fix pairs (measured: no gain; off) |
| `wideSearchSigmaXY`, `wideSearchSigmaHeading` | 0.04 m, 0.03 rad | above this prediction sigma, search 45 alignment hypotheses (start pose, after slips) |
| `wideMinSpreadM`, `wideMinTurnRad` | 0.25 m, 0.35 rad | the wide search waits until the window holds this much motion (a still view can be ambiguous) |
| `wideSearchPriorWeight`, `wideAmbiguityMinInliers`, `wideAmbiguityFraction` | 1, 15, 0.05 | ranking and the ambiguity test (an ambiguous answer is never published) |
| `relockAfterRejects`, `relockSigmaXY`, `relockSigmaHeading` | 8, 0.06 m, 0.05 rad | after this many fixes rejected for disagreeing with the prediction, widen the prediction (odometry slip / heading jump) |
| `fixSigmaFloorXY`, `fixSigmaFloorHeading` | 0.015 m, 0.01 rad | lower bounds on published sigmas (systematic errors are not in the edge covariance) |
| `gateMaxOmegaRadS`, `gateMaxSpeedMps`, `gateMinInliers`, `gateMinInlierRatio`, `gateMaxSigmaXY`, `gateMaxSigmaHeading`, `gateMaxChi2` | 1.5, 2.0, 25, 0.5, 0.03, 0.03, 16 | fix gate |
| `vocabularyPath` | "" | explore: ORB vocabulary file (loop closing, relocalization) |
| `exploreInitMinMatches`, `exploreInitMinParallaxDeg`, `exploreLoopClosing`, `exploreLocalWindow`, `exploreGlobalBA`, `exploreGlobalBAIterations`, `exploreMinFramesBetweenKf`, `exploreMaxFramesBetweenKf`, `exploreReobsMinGapKf` | 100, 1.0, true, 10, true, 30, 3, 20, 100 | explore mode (re-observation loops: a new keyframe sharing points with one 100+ keyframes older; then a global BA) |

## Status codes and threading

`beacon_status` returns the `BeaconStatus` codes (0 INVALID, 1 INITIALIZING, 2 TRACKING, 3 LOST =
vision degraded). Threading rules are in the header: every call except `beacon_destroy` may come
from any thread; `beacon_destroy` exactly once, after all other calls returned.

## Layout

`src/` core (api, field engine: `predict`, `edges`, `gate`, `field_model`; explore engine:
`orb_features`, `vocab`, `map`, `matcher`, `optimizer`, `init_mono`, `loop`), `bench/` desktop tools
and viewer, `tests/` GoogleTest, `scripts/` dependency and Android builds.
