# Beacon: ORB-SLAM-style field localization for FTC (design)

Date: 2026-11-17. Status: draft for approval. Supersedes the localization approach in
`docs/archive/astro-spec/` (phone-camera ARCore/VINS, specs 03/04) and the AprilTag/MegaTag fix path of the
2026-11-16 odometry tier. `docs/archive/astro-spec/00-PROJECT.md` conventions (units, frames, timestamps,
threading, logging, config) still apply unless this document says otherwise.

## 1. Intent

Beacon is ORB-SLAM ported to FTC: monocular visual SLAM that runs inside the FTC Robot
Controller app on a REV Control Hub (or an Android phone acting as the RC). It takes frames from
one dedicated Limelight 3A, and its job is to keep a goBILDA Pinpoint's odometry pose accurate on
the field indefinitely. Team code reads one field-frame pose through `BeaconLocalizer` (Road
Runner / Pedro adapters).

Ambition (team): **the most accurate SLAM-based localizer ever built for FTC.** The name is
Beacon everywhere; "ASTRO" appears only when citing the reference videos.

Why: dead wheels are fast and smooth but drift without bound; the team wants dead-wheel feel
with error that does not grow over a whole practice session, without AprilTags and without any
per-venue setup.

## 2. Goals, non-goals, anti-goals

**Goals** (accuracy holds **while driving**, not only when parked)
- **Stage 1 (ship first, ASAP):** position error **< 5 cm** while driving, after a full
  2:30 match.
- **Stage 2 (full target):** position error **≤ 2-3 cm**, heading about 1°, **bounded for
  15-20 minutes** of continuous driving (practice-session length).
- **Drivetrain (team):** 104 mm mecanum wheels, 6000-6200 rpm motors, 113:9 reduction
  (12.56:1). Free speed ≈ 478-494 wheel rpm ≈ **2.6-2.7 m/s** forward (≈ 2.1 m/s at a typical
  80 % loaded; strafing slower). Turn rate depends on wheelbase/track (unknown); for a
  ~0.35 m half-sum it is ≈ 7 rad/s free.
- **Design speeds (builder's call):** handle up to 2.7 m/s and 7 rad/s. Vision corrections may
  be gated off above lower rates (blur); Pinpoint coasts. At 2.7 m/s every 1 ms of timestamp
  error is 2.7 mm, so corrections taken at speed need timestamp error of a few ms or less, or
  must be gated to slower moments.
- Error is measured **relative to the physical field elements of the field the robot is on**,
  not to the nominal drawing. Real fields are hand-assembled and differ from the drawing by
  about the target itself. **The field frame is defined by the perimeter walls** (builder's
  call); the hive's actual placement on each field is estimated online as a small offset from
  its nominal pose, during INIT and the first seconds of the run.
- Works **cold on any BIOBUZZ field**: no mapping run, no venue-specific setup. Up to ~10 min of
  optional venue checks (camera view looks right, start pose correct); never required.
- Tuned once on the team's practice field; only parameters carry over, never a map.
- Pose to team code at Pinpoint rate (~1 kHz reads, one per `update()`), `update()` < 0.5 ms.

**Non-goals**
- AprilTags. The team cannot or will not use them. No tag detection anywhere.
- Venue features (bleachers, banners, people, lights). Never in the map.
- Multi-robot, dense mapping, iOS, a second camera, stereo.
- A persistent map between runs or venues.

**Anti-goals (failure even if it "works")**
- A correction that teleports the pose or makes velocity spike (bad for path followers).
- A wrong pose reported as TRACKING. When vision is unsure, Beacon must fall back to pure
  Pinpoint and say so, not guess.
- Anything that throws, blocks, or returns NaN on the OpMode thread.
- Anything that needs the robot to move during INIT (G304 H) or a network/laptop in the match.

## 3. Constraints

- **Hardware:** REV Control Hub (Rockchip RK3328, 4× Cortex-A53 @ 1.5 GHz; RAM 1-2 GB,
  unconfirmed) or phone RC. goBILDA Pinpoint on I2C. One Limelight 3A in the Hub's USB 3.0 port,
  dedicated to Beacon (no other pipeline). 3A: global or rolling shutter and exact FOV/resolution
  to be confirmed from its datasheet (spec 00 says rolling shutter; vertical FOV about 42°).
- **Camera mount (team's choice):** not pitched down. Level to at most ~10° down, as high as the
  robot allows, so one frame holds the perimeter, the hive and the floor tiles. Floor becomes
  visible roughly 0.6-1 m ahead (estimate, depends on height).
- **Rules (see `RULES.md`, 2026-11-17 audit):** Limelight 3A is the only permitted vision
  coprocessor and reprogramming it is permitted (R702, Table 12-9); powered from the Hub USB 3.0
  port only; no live video leaving the robot (R704 D); robot motionless after INIT (G304 H).
  Frames moving Limelight → Hub over USB stay on the robot.
- **Compute:** everything vision runs off the OpMode thread. Pinpoint carries the pose between
  vision updates, so vision can run at a low rate (target ≥ 5-10 corrections/s; to be measured).
- **Licensing:** Beacon is **MIT-licensed** (team: "ours needs to be licensed"; MIT as in spec
  00). The reference system (ASTRO) has no license, which means all rights reserved, NOT free to
  copy: Beacon must match and beat its capabilities with its own implementation, never by copying
  its code. Only BSD/MIT/Apache components (OpenCV, g2o or Ceres, DBoW2 /
  FBoW with our own trained vocabulary, Eigen). No ORB-SLAM source or ORB-SLAM vocabulary file
  (GPL-3).
- **Environment:** macOS arm64 dev machine, no device attached; anything on the Hub is run by
  the team.

## 4. Approaches considered

| | Approach | Trade-off |
| --- | --- | --- |
| A | Port ORB-SLAM3 as-is (NDK, monocular), align its map to Pinpoint for scale | Proven, but GPL-3, heavy (~40 MB vocabulary), fragile monocular init, cannot use wheel odometry, and its map would happily include venue features |
| B | Fork ORB-SLAM3, replace IMU path with Pinpoint odometry | Metric from frame 1, but deep surgery in a large GPL codebase, still heavy on the A53s |
| **C** | **Lean ORB-SLAM-style core written for FTC (chosen)** | Same building blocks (ORB features, keyframes, local BA, place recognition, relocalization) with Pinpoint as the motion prior, known mount, flat floor and a known field model. Cheap, no GPL, no fragile init; more of our own code, so it needs a strong desktop test bench |

Dropped: pure Java on the SDK's OpenCV (too slow for bundle adjustment).
The team chose C.

## 5. What the team decided

1. **Pinpoint is the continuous pose; Beacon vision corrects it.** Scale comes from the wheels.
2. **No AprilTags.**
3. **Limelight 3A dedicated to Beacon, level-ish mount** seeing perimeter, hive and tiles.
4. **Landmarks are tied only to the BIOBUZZ field**, in this order of trust:
   1. **Hive and perimeter** as known shapes from a per-season field model.
   2. **ORB features that lie on a known field surface** (floor plane, hive surfaces). Learned
      during the run; a feature whose 3-D position is not on a field surface is rejected. This
      is what makes it ORB-SLAM while keeping venue features out. **Perimeter panels are clear
      polycarbonate**, so texture "on" a wall is usually the venue behind it: walls contribute
      only through their edges (top rail, posts, wall-floor line), never ORB points (builder's
      call).
   3. **Tile seams** (24" grid, Pinpoint resolves which seam), low weight, can be switched off;
      kept only if the desktop bench shows they help.
5. **Start pose** is the team's known starting position (as in auto), set in INIT; Beacon
   refines it from the walls and hive while the robot is still.
6. **Ground truth:** the team parks near known tape marks on the practice field (each mark's
   coordinates measured from two perimeter walls), measures the robot's offset from the mark by
   tape measure, and compares with Beacon's telemetry pose. The robot drives between checks,
   so a parked check after driving measures accumulated error while driving. Pass: Stage 1,
   every check < 5 cm through a 2:30 match-style run; Stage 2, every check within 2-3 cm
   (heading about 1°) through a 15-20 min session; each repeated over several sessions.
7. **Practice field** has the official perimeter and a real hive.
8. **Hardware is not confirmed yet** (robot, Limelight mount, camera height unknown). The team
   cannot run anything or send logs for 3-4 days and wants the build as soon as possible.

### Builder's calls from review
- Control Hub only for now; phone RC later.
- Limelight code stays within stock firmware (pipelines + Python). Option (b) in §6.2 is
  therefore unlikely to carry enough data; (a), decoding frames on the Hub, is the main path.
- Whether full-rate Limelight → Hub frames over USB count as a "continuous video stream" under
  R704 D is unaudited; added to the Q&A list in `RULES.md` for the team to submit.
- Camera height and pitch are parameters; the synthetic bench sweeps them until the robot
  exists.

## 6. Design

### 6.1 Data flow

```
Limelight 3A ──frames (USB net)──▶ FrameGrabber (Hub, own thread) ──▶ beacon-native (C++, own threads)
                                                                       │  tracking + local map
Pinpoint ──I2C, 1 read per update()──▶ PinpointPoseSource ──history──▶│  (Pinpoint pose at frame time
                                                                       │   is the motion prior)
                                                                       ▼
                                             VisionFix (field pose at capture time, covariance)
                                                                       ▼
                       BeaconLocalizer (existing): blends fix at capture time ─▶ team code
```

### 6.2 Frame path (first unknown; milestone 0)

How frames reach the Hub, at what resolution, rate, latency and compression, is **not known**.
Options to evaluate, in order: (a) the Limelight's camera stream over its USB network interface,
decoded on the Hub; (b) custom code on the Limelight (permitted by R702) that runs the ORB
front end on its stronger A72 cores and sends keypoints + descriptors instead of images.
Overlays, JPEG artifacts and exposure control must be checked for (a). The chosen path must give
a **capture timestamp on the Hub's `elapsedRealtimeNanos` clock** with known, bounded error;
without it nothing downstream is accurate. A rough check: at 1 m/s, 10 ms of timestamp error is
1 cm.

### 6.3 Field model (season file)

One file per season (`beacon-field-biobuzz.json` or similar) holding, in the FTC field frame:
floor plane and tile grid (pitch, origin), perimeter wall planes with height and the
floor-wall and top edges, and the hive as a set of planar faces / edges. Built from the official
BIOBUZZ field drawings. Next season = new file, no code change. The field-dimension tolerance
from the Competition Manual must be looked up and recorded with the file.

### 6.4 Tracking and mapping (beacon-native)

- **State:** robot pose is planar (x, y, heading) on the floor; camera pose = robot pose ∘
  calibrated `T_robot_cam`. Intrinsics and distortion from a checkerboard calibration of this
  Limelight.
- **Per frame:** predict the camera pose from Pinpoint at the frame's capture time; extract ORB;
  match against local map points (projected with the prediction); match model edges (wall-floor
  line, wall top, hive edges) and, optionally, seams; solve a planar pose with robust weights.
- **New map points:** because every accepted point lies on a known surface, a point can be
  created from a **single view** by intersecting its ray with the surface (floor, wall, hive
  face), then refined across keyframes. Points that do not stay consistent with a surface are
  culled. This sidesteps monocular initialization.
- **Local optimization:** a sliding window of keyframes with Pinpoint odometry factors between
  them, reprojection factors to map points, surface-membership priors on points, model-edge
  factors, and low-weight seam factors.
- **Relocalization:** place recognition over keyframes (BoW with a vocabulary trained on field
  images) plus model-based pose search around the Pinpoint pose when tracking is lost.
- **Gating:** a correction is published only when it is well constrained (covariance below a
  threshold, enough inliers, consistent with Pinpoint within a jump limit). Otherwise Beacon
  reports vision-degraded and the pose is pure Pinpoint.
- **Motion blur / rolling shutter:** frames taken during fast rotation are down-weighted or
  skipped; whether rolling-shutter modelling is needed is decided on the bench.

### 6.4a Explore mode and the viewer (team requirement, 2026-11-17)

The team's reference videos (`~/Downloads/robotics-slam-demo-1.mp4`, handheld camera in a house;
`~/Downloads/slam-test-3.mp4`, EuRoC Machine Hall) show how Beacon must look and behave when the
Limelight is plugged into the Mac: "should work like that exactly". What they show:
- **Camera panel:** the live grayscale image with tracked features drawn on it (green = tracked
  map points, blue = new/candidate).
- **3-D panel:** ground grid, the camera trajectory as it moves, keyframe markers along it, a
  sparse map point cloud growing as the camera explores, loop-closure links and a pose graph,
  the map snapping into place when a loop closes. About 11-15 fps.
- Topics in the videos: points, backend pose, image, loop closures, keyframes, frontend map,
  pose graph, loop-closure database map, backend points.

So the core has two modes over one codebase:
- **Explore mode (Mac bench; general monocular ORB-SLAM):** features anywhere in the scene,
  monocular initialization, keyframes, local BA, place recognition with loop closing and pose-graph
  optimization. No Pinpoint, so scale is arbitrary (set at initialization); the videos were
  visual-inertial, the Limelight has no IMU, so this is the only visible difference. Validated on
  EuRoC MH_01 (already in `desktop-harness/data/euroc/`, cam0 only) and on live Limelight
  footage.
- **Field mode (robot):** the same machinery plus Pinpoint as the motion prior (metric scale),
  field-model anchoring, and the field-surface rule of §5.4 for anything that produces an absolute
  correction. Venue features may help frame-to-frame tracking but never anchor the pose.
- **Viewer:** Rerun (macOS-native, no ROS), with entities matching the panels above, fed live by
  `beacon_live` and by replays. Never runs on the robot (R704 D).

### 6.5 Hub integration (existing code reused)

- Keep `BeaconLocalizer`, `PinpointPoseSource`, `BeaconConfig`, `PoseLogWriter`, OpModes,
  G304 start rule, world-epoch handling. 115 JVM tests pass (re-run 2026-11-17).
- Generalize `TagFixSource`/`TagFix` into a vision-fix source that carries capture time,
  pose and covariance. `LimelightTagSource`/`LimelightFixFilter`/MegaTag code is parked.
- **The localizer's 128-sample history covers only ~85 ms at Pinpoint's ~1.5 kHz**; vision
  latency is likely longer. History must cover at least the worst vision latency (size by time,
  not count).
- Native code: `beacon-native/` (C++17, CMake, NDK arm64-v8a), JNI shim in TeamCode. Build
  changes only in `TeamCode/build.gradle`.
- Telemetry shows vision status, correction rate, last correction size, latency, CPU time.

### 6.6 Desktop bench (needed because no robot is attached here)

- **Synthetic:** render the BIOBUZZ field model with textures, plus clutter (other robots,
  game pieces, a random venue backdrop that must be rejected), along known trajectories with
  simulated Pinpoint drift and slip. Measures accuracy against exact truth over 20-minute runs.
- **Recorded:** replay logged Limelight frames + Pinpoint logs from the practice field.
- Same native core compiled for macOS arm64. The existing OpenVINS harness under
  `desktop-harness/` is parked.

## 7. Milestones

Ordered for "build ASAP, no hardware for 3-4 days": everything before the hardware arrives runs
on the desktop, and one probe OpMode is ready on day one of hardware access.

0. **Probe + data-collection OpMode (written now, run when hardware exists):** gets Limelight
   frames on the Hub, measures rate, latency, timestamp error and image quality, and records
   frames + Pinpoint logs + a checkerboard sequence for intrinsics. One OpMode, so a single
   field session gathers everything.
1. **Field model** for BIOBUZZ + synthetic renderer (with clutter and a venue backdrop that
   must be rejected) + evaluation scripts.
2. **Core on the desktop bench:** first to Stage 1 (< 5 cm after 2:30 while driving) on
   synthetic runs, then Stage 2 (≤ 2 cm over 20 min); then on recorded logs once they exist.
3. **NDK build, JNI, Hub integration** (can start in parallel with 2); measure CPU and
   correction rate on the Hub.
4. **Practice-field acceptance** with tape-mark checks (§5.6), Stage 1 then Stage 2, then a
   second, different field.

## 8. Left to the builder

Exact optimizer (g2o vs Ceres), feature counts and image resolution, vocabulary training,
keyframe policy, window size, gating thresholds, seam detector, renderer tooling, file formats
(follow existing CSV/JSON conventions), test structure.

## 9. Risks

- Frame path may be too slow, too compressed, or too poorly timestamped (milestone 0 decides).
- Hub CPU may limit correction rate; Pinpoint coasting between corrections must stay within budget.
- Long stretches with hive and walls occluded by robots leave only floor features and seams.
- Lighting differences between venues (exposure control on the 3A).
- Field build tolerance vs the 2-3 cm target (addressed by defining accuracy against the
  physical field, §2).
