# Decisions

Append-only log of spec findings and choices. Newest at the bottom.

## 2026-11-16 orchestrator: Product name is Beacon, not ASTRO
The spec bundle uses the working name ASTRO (which 08-RELEASE notes collides with the Astro web
framework). All code, paths and config use Beacon: package `...teamcode.beacon`, `BeaconLocalizer`,
`beacon.json`, `/sdcard/FIRST/beacon/`, `beacon-native/`, `beacon_*` C API. Mapping table in
`orchestration/CHARTER.md`.

## 2026-11-16 orchestrator: Repo root is the spec's `ftc/` directory
This repo is an FtcRobotController v12.0 fork, so `desktop-harness/`, `calibration/`,
`beacon-native/` and `tools/` sit at the root beside `TeamCode/` instead of beside an `ftc/` folder.

## 2026-11-16 orchestrator: Device-gated acceptance is recorded as pending, not passed
No Android phone or robot is available in the build session. Every acceptance item in 03, 04 and
06 that needs a phone, a robot or a field is listed in `TESTING.md` as NOT RUN. Spec rule "do not
start 04 before 02 and 03 have passing acceptance tests" is narrowed to: 04 code may be written and
cross-compiled once 02's JVM tests pass and 03 compiles, but 04 must not be tested on a robot until
03's on-device acceptance has been run and passed by the team.

## 2026-11-16 Rules Auditor: spec 07 and 02 corrected against the TU09 manual
Read the BIOBUZZ Competition Manual TU09 (HTML), Section 12 TU08 PDF, TU00-TU09, and Inspection Checklist V26-27.3.
Corrections: R603/R608 (not R602/R613) govern hub power, R602 B-ii allows a USB battery pack for the phone; R202 F/J are "additional scrutiny", not bans;
a UVC webcam is legal (R707 A, R708) and a USB hub is explicitly legal (R703), so 02 option (b) "Not legal" is wrong;
Expansion Hub address is changed in the RC app (Advanced Settings > Expansion Hub Address Change), not the REV Hardware Client (02 line 10 and 07 wiring notes);
"legal but unsupported" is not manual wording. New constraints: R704 D bans live streaming (no live Rerun from the phone), G304 H robot must be motionless after INIT (VIO init), R202 C no AprilTag-like imagery on the robot.
Details in RULES.md and WIRING.md.

## 2026-11-16 core-java: Frame contract for pose sources (camera optical frame in a z-up world)
Spec 02 left the camera body frame as "whatever the source reports", so one `T_robot_cam` would mean
different things per tier. Contract (documented on `PoseSource`, `BeaconConfig`, `FrameMath`):
- Every `PoseSource` emits the pose of the camera **optical frame** (x right, y down, z forward out
  of the lens, OpenCV) in a **z-up, gravity-aligned world** with arbitrary origin and yaw. Each source
  converts itself; `FrameMath.fromArCoreCameraPose`, `rotateWorld`, `rotateBody`, `compose` do it.
  (ARCore: world y-up -> z-up is `(x,y,z)->(x,-z,y)`; body OpenGL x-right/y-up/z-back -> optical.)
- `T_robot_cam` in beacon.json is the pose of that optical frame in the robot frame (+x fwd, +y left,
  +z up, origin at the centre of rotation). Localizer: `T_sw_robot = T_sw_cam * inverse(T_robot_cam)`.
- roll/pitch/yaw are defined against a baseline of "camera looking straight forward, upright", so
  all zeros is a sane mount, not a camera looking along robot +z:
  `R_robot_cam = Rz(yaw) * Ry(pitch) * Rx(roll) * R_baseline` (extrinsic about robot axes, roll first;
  same as intrinsic yaw, pitch, roll). R_baseline maps optical x->robot -y, y->-z, z->+x. Right-hand
  rule about robot axes: yaw>0 looks left, pitch>0 looks DOWN, roll>0 raises the image-left edge.
  A portrait-mounted phone is roll = +/- pi/2. Angles are radians; |angle| > 2 pi is rejected as
  probable degrees. Worked examples (straight ahead; pitched down 15 deg) are in the `BeaconConfig`
  Javadoc and asserted in `FrameMathTest`.
- Caveat for source authors: verify on a device which ARCore pose accessor (`getPose` sensor-oriented vs
  `getDisplayOrientedPose`) matches the "OpenGL camera, sensor orientation" assumption in
  `fromArCoreCameraPose`; a portrait/landscape mismatch shows up as a constant roll of +/- pi/2.

## 2026-11-16 core-java: Field alignment is planar and tag fixes edit it
`T_field_sourceworld` is (ax, ay, ayaw). `setPose` sets it from the last good sample (or on the first
good sample if none yet). A tag fix is compared with the localizer's pose at the fix time (history of
128 samples, interpolated; fixes older than 1 s are rejected), then: rotate the alignment about the
robot's position at that time by `gainHeading * wrap(err_heading)` (position unchanged), then translate
by `gainXY * err_xy`. Output is therefore continuous apart from the gained correction itself.
Velocity is computed from source-world samples, so corrections never appear as velocity spikes.
Added config `tag.maxJumpRad` (default 0.5) so the jump test covers heading (spec only had maxJumpM).
The first fix after the source returned from LOST/INITIALIZING (and any fix before alignment exists) skips
the jump test and uses gain 1 ("unless status is initializing or lost" in the spec).

## 2026-11-16 core-java: BeaconConfig validates what the file contains, parses with MiniJson
Spec says to throw on a non-normalized quaternion, but beacon.json holds roll/pitch/yaw, so no
quaternion exists to normalize. Validated instead: finite numbers; gains in [0,1]; lever arm
(|x,y,z|) <= 1 m; angles within +/- 2 pi; maxJumpM in (0,10]; maxJumpRad in (0,pi]; known tier;
`vins.configPath` when tier is vins; `sourceTimeoutS` in [0.01,60]. Unknown keys and a missing
`T_robot_cam` are warnings (`cfg.warnings`), a missing file is defaults plus one warning, a malformed
or invalid file throws `BeaconException`. JSON parsing: `org.json` is a throwing stub in JVM unit tests
and Gson is not on the TeamCode compile classpath (checked with `:TeamCode:dependencies`), so a 250-line
strict parser (`MiniJson`, package-private, pure Java) is used; no new runtime dependency.
Config additions beyond the spec: `tag.maxJumpRad`, `sourceTimeoutS` (default 0.5 s; no new sample for
that long means LOST).

## 2026-11-16 core-java: Localizer failure behaviour and status mapping
Only samples with source status TRACKING move the pose. Non-finite pose, |q| not within 1% of 1, time
going backwards, a vertical robot x axis, or an exception from the source or tag source: sample
rejected, last good pose held, status LOST, counter and `getLastRejectReason()` updated, never thrown.
A repeated timestamp is ignored silently. Source INITIALIZING/LOST/INVALID map to the same localizer
status. INITIALIZING also until the alignment is set. `update()` returns the same `Pose2dStamped` object
when nothing changed. `BeaconLocalizer.start(ctx)` never throws (records `getStartError()`).
`setPose` has `(x, y, heading)` and `Pose2D` overloads: FTC SDK 12.0 ships
`org.firstinspires.ftc.robotcore.external.navigation.Pose2D` and `Pose3D` (checked in
RobotCore-12.0.0-api.jar); there is no `Pose2d` in the SDK (that is Road Runner's).

## 2026-11-16 core-java: CSV logs
Columns `t_ns,x,y,z,qx,qy,qz,qw,status`. A pose source logs its camera pose in its own world. The
localizer logs the robot pose in the FIELD frame (x, y; z = robot height in source world; yaw-only
quaternion; status = localizer status), one row per new source sample, written after any tag fix of that
update. `PoseLogWriter` back-pressure: SPSC ring (4096 rows by default), the loop copies a row and
returns; when the ring is full the NEWEST row is dropped and `getDroppedCount()` increments; after an
I/O failure all later rows are counted as dropped. One producer thread per writer. `FakePoseSource.fromCsv`
replays source-style logs (camera poses), not localizer logs.

## 2026-11-16 orchestrator: Primary tier is Hub-side odometry + Limelight 3A (phone camera is not legal)
Per the team, a phone camera on the robot is not an option. New tier `"odometry"` (BeaconConfig.TIER_ODOMETRY),
runs in the Control Hub RC app: `PinpointPoseSource` (goBILDA Pinpoint, dead wheels + IMU) is the continuous pose,
`LimelightTagSource` (Limelight 3A, MegaTag2) supplies absolute field fixes through the existing `TagFixSource`
path, `BeaconRig` wires both into one unchanged `BeaconLocalizer`. ARCore/VINS tiers (specs 03, 04) stay in the tree but
are not the build target. Pinpoint reports the robot pose, so `PinpointPoseSource` composes it with `T_robot_cam` and
the localizer undoes it exactly (the mount has no effect for this tier). Assumptions NOT verified on hardware:
the Limelight MT2 botpose frame equals the localizer's field frame (FTC field coordinate system, origin at field centre,
heading CCW from +x), `LLResult.getStaleness()` is the result age in ms, and `getControlHubTimeStampNanos()` changes per
result. Pod offsets, encoder directions and the Limelight robot-space camera offset must be measured and set
(`BeaconHardwareConfig`, Limelight web UI).

## 2026-11-16 orchestrator: Phone-tier work stopped mid-flight
After the switch to the Control Hub odometry tier, the desktop harness (spec 01, OpenVINS on EuRoC) and
calibration (spec 05, phone camera/IMU) agents were stopped before finishing. `desktop-harness/` and
`calibration/` are partial and unverified: no results in them have been checked by the orchestrator. The
ARCore tier (spec 03) was never written. Specs 01, 03, 04, 05 are parked.

## 2026-11-17 Rules Auditor: RULES.md and WIRING.md rewritten for Control Hub + Pinpoint + Limelight 3A
Audit pass 2 against the same TU09 manual (HTML last modified 13 Nov 2026; no TU10). Hardware is now REV Control Hub (R701 A),
optional Expansion Hub, goBILDA Pinpoint on I2C, Limelight 3A on the Control Hub USB 3.0 port. Phone rows moved to a short
"not applicable" list in RULES.md. Findings: Limelight 3A is allowed by R702 Table 12-9 (SDK-native, "reprogramming them is permitted";
checklist exempts it from the coprocessor-software check), R707/R708 B; Limelight 3G is named prohibited and the Limelight 4 is not in Table 12-9.
No numeric limit on vision devices. Pinpoint is not named in the manual; allowed by R303 J, R702 examples 2 and 3 and absence of any sensor rule
(keep manufacturer firmware). Limelight must be powered only from the hub (R602, R613 A), never a USB battery pack. R704 C/D: configure the Limelight
with a laptop plugged directly into it, no laptop on RC Wi-Fi in a match, no live streaming. R711: Control Hub needs a non-default Wi-Fi password and
Bluetooth off; Airplane Mode is phone-only. R606 B is about the Control Hub diagnostic lights. G304 H now helps: the Pinpoint IMU zero offset is taken
while stationary (about 0.25 s, user guide), so place, INIT, then leave the robot still until the LED is green.
WIRING.md corrections: Pinpoint offsets are from a free "tracking point", not necessarily the centre of rotation; IMU calibration is ~0.25 s not "about a second";
no Expansion Hub address change is needed with a Control Hub (REV page ties it to an Android-device RC); address change, when needed, is in the RC app not the REV Hardware Client.
Unverified: Control Hub USB 3.0 current vs Limelight 4 W, Limelight radios (none documented), web UI reachability over RC Wi-Fi, I2C port 0 note, Q&A archive.

## 2026-11-17 tier1-engineer: Odometry tier verified against the SDK 12.0 sources
Read in the Gradle cache (Hardware-12.0.0-sources.jar, RobotCore-12.0.0-sources.jar):
- `GoBildaPinpointDriver`: position kept in mm, heading in rad (unnormalized; `getHeading(AngleUnit)` wraps via
  `AngleUnit.fromRadians`). `setOffsets` Javadoc: X pod offset = sideways distance of the forward pod, LEFT positive;
  Y pod offset = forward distance of the strafe pod, FORWARD positive. `setEncoderDirections`: X pod must count up
  moving forward, Y pod up moving LEFT, so the frame is x forward, y left. Heading sign is not written down; CCW
  positive follows from the right-handed x-fwd/y-left frame and is checked on the robot by the calibration OpMode.
  `update()` is one 40-byte bulk read; all getters used afterwards read cached fields (no extra I2C).
  `DeviceStatus`: NOT_READY (powering up), READY, CALIBRATING (gyro zeroing, ~0.25 s, robot must be still),
  FAULT_X/Y/NO_PODS, FAULT_IMU_RUNAWAY, FAULT_BAD_READ ("the result reported is a duplicate of the last good read").
  `resetPosAndIMU()` resets to 0,0,0 and recalibrates.
- `Limelight3A`: the SDK polls `/results` every 10 ms on its own executor; `getLatestResult()` returns that cached
  object and never blocks, and it is NOT cleared on disconnect (the last result stays). `updateRobotOrientation` and
  `pipelineSwitch` are HTTP POSTs with a 100 ms connect and 15 s read timeout: never on the loop.
- `LLResult`: `getControlHubTimeStamp()` is `System.currentTimeMillis()` when the Hub parsed the JSON;
  `getStaleness()` = wall-clock ms since then; `getCaptureLatency()` (`cl`) and `getTargetingLatency()` (`tl`) are ms
  (the SDK sample adds them as "LL Latency"); `getTimestamp()` is "Limelight-local monotonic". `getBotpose()` (MT1) and
  `getBotpose_MT2()` build a Pose3D from metres and degrees (yaw = element 5); missing arrays become all zeros.
  `getBotposeAvgDist()` is metres per its Javadoc. `getPipelineIndex()` is "pID".
Corrections to the 2026-11-16 odometry code: (1) the fix time was receipt time; it is now
`now - (staleness + cl + tl)`; (2) the result id was `getControlHubTimeStampNanos()`, which changes on every 10 ms
re-poll of the SAME camera frame (duplicate fixes); frames are now identified by the Limelight timestamp plus pose;
(3) `pipelineSwitch` ran in INIT on the OpMode thread (up to 15 s) and is now on the Limelight thread with retry;
(4) `hw.get` threw for a missing device; `tryGet` is used and the problem becomes a telemetry line; (5) FAULT_BAD_READ
was mapped to LOST; it now publishes nothing (the values are stale copies) and only a timeout makes it LOST;
(6) `T_robot_cam` was composed in and undone again; the source now declares `reportsRobotPose()` and the mount is
not used at all. Remaining residual on the fix time: HTTP transfer plus up to one 10 ms poll interval (stamp late).

## 2026-11-17 tier1-engineer: MegaTag2 corrects position only; MegaTag1 is the heading reference
MegaTag2 solves position GIVEN the heading we send, so its heading is our own value echoed back. Using it to
correct heading would be a loop with no information; using it for position is right, but its position error is
about range x heading error (2 m, 1 deg -> 3.5 cm). Design (`LimelightFixFilter`, `TagFix.hasPosition/hasHeading`):
- MT2 fix = position only, and only if the localizer is TRACKING (only then is a heading sent) and our heading was
  POSTed successfully within `limelight.headingFreshMs` (50) of the capture time.
- MT1 (full independent solve) supplies heading, through `tag.gainHeading`, only with >= `mt1MinTags` (2) tags within
  `mt1MaxRangeM` (2.0 m); single-tag MT1 heading is too ambiguous. MT1 position is used when MT2 is unusable.
- When the localizer is not TRACKING, only a full MT1 fix is produced (rescue path that can set the alignment).
- Every fix: valid, configured pipeline, staleness <= `maxStalenessMs` (150; catches an unplugged Limelight), pose not
  all zeros, finite, |x|,|y| <= `fieldHalfM` (1.83 m), tag count and average distance limits. In the localizer: robot
  speed <= `tag.maxSpeedMps` (0.75) and turn rate <= `tag.maxOmegaRadPs` (0.35) AT THE CAPTURE TIME (history), since
  turning makes the heading the Limelight used wrong for that image, plus the existing jump test.
Heading drift therefore has one independent corrector (MT1 multi-tag); with Pinpoint IMU drift small, gain 0.2 and
several fixes per second is expected to hold heading, but that is unmeasured.

## 2026-11-17 tier1-engineer: Localizer review findings 1-11 closed; world epoch; G304 start rule
- F1: the jump-test waiver (gain 1) applies only to the first fix captured within `tag.recoveryWindowS` (1 s) after
  tracking resumed or the world reset. F2: `Pose3dStamped.worldEpoch`; on a change the localizer re-aligns so the
  first new sample continues the last output (extrapolated with the last velocity over gaps <= sourceTimeoutS) and
  opens the window. The Pinpoint source bumps it after a reboot (NOT_READY/CALIBRATING seen after READY: brown-out;
  it also re-sends offsets/resolution/directions) and after a read-to-read jump faster than 5 m/s or 25 rad/s.
  F3: fixes > sourceTimeoutS ahead of the newest sample are rejected without advancing the last-fix time; samples
  arriving "backwards" for longer than sourceTimeoutS are taken as a new time base (history cleared, window opened).
  F4: tag-source exceptions only reject the fix. F5: pitch limited to +/- pi/2, whole-number radians >= 2 warn.
  F6: moot for this tier (T_robot_cam ignored, warning if present). F7: velocity is zero unless TRACKING.
  F8: |coordinates| > 1e4 m rejected; final finite guard in publish. F9: held pose stamped with the last good sample.
  F10: FrameMath per-sample transforms are allocation-free. F11: PoseLogWriter refuses to restart while the old
  thread drains and appends instead of truncating; unknown top-level config sections are errors; UTF-8 BOM and `//`
  comments accepted; class Javadoc pivot wording fixed.
- G304 H (motionless after INIT): `setPose` in INIT, `onStart()` at START. Tracking at START: the start pose is
  re-asserted against the newest sample (removes INIT drift). Not tracking at START (Pinpoint still calibrating):
  the start pose EXPIRES, status stays INITIALIZING until a full tag fix; telemetry says so. The OpModes wait in INIT
  showing "keep still" until the Pinpoint reports READY.
- `PoseSource.poll()` (default no-op) is where a source reads its device: exactly one Pinpoint I2C read per update().
- Config: default tier is now "odometry"; "arcore"/"vins" are rejected as parked; new keys `tag.recoveryWindowS`,
  `tag.maxSpeedMps`, `tag.maxOmegaRadPs` and a `limelight` section; `tag.minDecisionMargin` and `vins` removed (the
  Limelight does its own decoding). Hardware names and pod values stay in BeaconHardwareConfig.java.

## 2026-11-17 tier1-engineer: Assumptions only the robot can settle (exposed in the OpModes)
1. `updateRobotOrientation(yaw)` wants DEGREES in the Limelight field-map frame (SDK Javadoc gives no unit).
   Check: "MT2 echo" in Beacon Test (MT2 yaw minus sent heading) must stay near 0.
2. The Limelight field map's frame (origin, axes, CCW yaw) equals the frame teams use for setPose. Check:
   calibration step 4 (frame advice) and step 5 (turn sign).
3. Pinpoint heading is CCW positive with the device right side up. Check: calibration step 1.
4. `getBotposeTagCount/AvgDist` refer to the tags used by both MT1 and MT2 (one set of fields exists).
5. Wall-clock (`currentTimeMillis`) does not jump during a match (staleness uses it). A jump shows as "result is N ms
   old" and fixes stop; it cannot corrupt the pose.
6. The Limelight's own `ts` changes once per camera frame (used to drop re-polled duplicates; the pose is compared too).
7. CPU cost on a Control Hub of the 100 Hz heading POST plus the SDK's 100 Hz GET is unmeasured; lower
   `limelight.headingHz` if the loop suffers.

## 2026-11-17 tier1-engineer: Findings from the parked ARCore investigation (for whoever unparks it)
ARCore 1.48.0 AAR has no aar-metadata minCompileSdk, minSdk 21, needs Play Services for AR >= 250210000; 1.57.0 needs
>= 262070000. `Camera.getPose()` is the readout-oriented OpenGL pose (docs), so `FrameMath.fromArCoreCameraPose`'s
assumption holds; `getImageIntrinsics()` is unrotated for the CPU image. `Frame.getTimestamp()`'s time base is not
documented; `Session.update()` throws MissingGlContextException without a GL context. FTC SDK 12.0's
`AprilTagGameDatabase.getBioBuzzTagLibrary()` places all four BIOBUZZ tag clusters at (0,0,0) with identity
orientation (placeholders), so SDK `robotPose` is meaningless this season, and cluster detections carry no decision
margin. VisionPortal webcam frame times are `System.nanoTime()` (CameraFrame.getCaptureTime), not elapsedRealtimeNanos.
Nothing ARCore-related was added to the build or manifest.

## 2026-11-17 tier1-engineer: Odometry tier bring-up (first session on the robot)
See the numbered procedure in the tier1-engineer report of this date; BeaconTestTeleOp's Javadoc points here.
In short: (1) robot config names; (2) INIT shows READY within ~1 s, still; (3) push 1 m forward / left in Beacon
Test, x/y signs; (4) calibration spin, forward, left; (5) Limelight pipeline, staleness < 30 ms, MT2 echo ~0;
(6) calibration steps 4 and 5 at a measured spot; (7) tag blend on: fixes applied, rejected reasons sane;
(8) unplug the Limelight mid-run: odometry continues, reason says "old"; (9) stop and re-run the OpMode.

## 2026-11-17 orchestrator: Beacon is ORB-SLAM-style field localization (supersedes phone VIO and AprilTag fixes)
Team restated the product: Beacon is ORB-SLAM ported to FTC, on a REV Control Hub (phone RC later),
with one dedicated Limelight 3A (monocular), correcting a goBILDA Pinpoint. Team decisions: no
AprilTags; no venue features and no mapping run (must work cold on any BIOBUZZ field); level camera
mount (<= 10 deg down) seeing hive, perimeter and floor; landmarks are hive + perimeter edges, ORB points
constrained to floor/hive surfaces (clear wall panels give edges only), optional low-weight tile seams;
accuracy while driving, Stage 1 < 5 cm after a 2:30 match, Stage 2 2-3 cm over 15-20 min, measured
against the physical field (perimeter walls define the frame); ground truth by tape-mark checks.
Drivetrain 104 mm mecanum, 6000-6200 rpm motors, 113:9 -> ~2.6-2.7 m/s free; design speeds 2.7 m/s,
7 rad/s. Approach C (lean MIT core, Pinpoint as motion prior) chosen over porting/forking ORB-SLAM3
(GPL-3, heavy, no wheel odometry). Design: docs/superpowers/specs/2026-11-17-beacon-orbslam-design.md;
plan: docs/superpowers/plans/2026-11-17-beacon-orbslam.md. LimelightTagSource / MegaTag code is parked.
Pinpoint tier, localizer hardening, G304 start rule and world epoch carry over (115 JVM tests re-run
by the orchestrator 2026-11-17: 0 failures). Specs 01/03/04/05 superseded; desktop-harness OpenVINS parked.

## 2026-11-17 field-tools-engineer: Mac Limelight tools (F4) conventions
- Stream: plain-Python MJPEG reader (http.client), default `http://limelight.local:5800/`; bare host or host:port
  accepted (port defaults to 5800). Boundary matched without leading dashes (servers differ). Parts with or without
  Content-Length. Every connection/stream failure is one `Error:` line, exit 2 (could not start) or 1 (lost part-way,
  frames kept); no tracebacks.
- `t_ns` in a Mac recording = ARRIVAL time of the frame's first JPEG bytes on `time.monotonic_ns()`, not capture
  time (capture-to-arrival latency is unknown; milestone 0 measures it on the Hub). Reader runs on its own thread so
  disk writes do not delay timestamps; a full queue drops and counts frames.
- I2 addition (no change to existing files): `ll_record.py` also writes `camera.json`. With `--camera` it copies a
  calibrated file; without, it writes UNCALIBRATED Limelight 3A datasheet values (HFOV 54.5 deg, centred principal
  point, zero distortion, zero mount, rollingShutterReadoutS 0) sized to the received frames, so a Mac recording can
  be replayed by `beacon_replay --mode explore` at once. The plan said "frames.csv + frames/ only"; this is the
  smallest change that makes it replayable. `--field` copies a field model in as `field.json`.
- `ll_calibrate.py --board 9x6` counts INNER corners (OpenCV convention, 10x7 squares). k3 fixed at 0 unless
  `--free-k3`. Keeps an existing camera.json's mount and rollingShutterReadoutS; otherwise writes zeros. Extra
  results go to a separate `calibration_report.json` so camera.json keeps exactly the I2 keys.
- Rolling-shutter time convention used by the simulator (I2 clarification): a frame's `t_ns` is the exposure time of
  the image's centre row; row r is exposed at t + (r - (H-1)/2) * rollingShutterReadoutS / H.

## 2026-11-17 orchestrator: Beacon is unlicensed for now; goal is the most accurate SLAM localizer in FTC
Team: Beacon is currently unlicensed. No LICENSE file and no license headers in our code (supersedes
spec 00's "MIT for everything we write" and the MIT lines in the design/plan). Dependencies stay
BSD/MIT/Apache only, with their notices kept, so any later license choice stays open; no GPL.
Name is Beacon everywhere, never ASTRO. Team ambition: the most accurate SLAM-based localizer ever
built for FTC. Mac viewer must be 1:1 with the reference videos' displays (plan Task C6 table).
Team confirmed the Limelight 3A MJPEG stream works on their Mac.

## 2026-11-17 orchestrator: CORRECTION - Beacon is MIT-licensed; ASTRO is the unlicensed one
The previous entry misread the team. Beacon must be licensed: MIT (spec 00's choice). The reference
system ASTRO (the videos) is unlicensed. No license means all rights reserved, not free to copy:
Beacon replicates and improves on ASTRO's behaviour and capabilities as seen in the videos with its
own implementation; no ASTRO source code is copied into this repo. Dependencies stay BSD/MIT/Apache.

## 2026-11-17 java-hub-engineer: Limelight frame path on the Hub (J3): stream URL, capture timestamps, recordings
- Path: the Limelight 3A's built-in MJPEG stream over its USB-Ethernet link, decoded on the Hub with Android
  `BitmapFactory` (reused bitmap + buffers) to BT.601 gray. Reader thread ("BeaconFrameReader") and decode thread
  ("BeaconFrameDecode") are separate; the hand-off is a one-slot mailbox, newest wins, overwritten JPEGs counted
  (`dropped before decode`). Nothing runs on the OpMode thread. Code: `beacon/vision/LimelightFrameSource.java`.
- How the SDK reaches the device (read from Hardware-12.0.0 sources): `Limelight3A` does HTTP to
  `http://<ip>:5807` (REST: /results polled at 100 Hz, /status, ...). `<ip>` is the per-device address in the RC
  configuration; `EthernetOverUsbConfiguration` defaults it to the Hub's own eth address with the last octet set
  to 1 (Hub 172.29.0.x -> Limelight 172.29.0.1). The SDK has no stream access; port 5807 is REST only.
- Stream URL candidates, tried in order, each reported on telemetry ("Tried: ..."): beacon.json
  `limelight.streamUrl` (bare host / host:port completed to `http://host:5800/`); the RC-configured Limelight
  address on port 5800 (read from the SDK's private `inetAddress` field by reflection, fallback: derive from
  `getConnectionInfo()` like the SDK does); `http://172.29.0.1:5800/`; `http://limelight.local:5800/` (last: Android
  rarely resolves .local and the lookup is not bounded by the connect timeout). A root URL that answers with
  non-MJPEG content (e.g. HTML) is retried at `/stream.mjpg` and `/?action=stream`. The last working URL is tried
  first on reconnect. Team confirmed limelight.local streams on a Mac; nothing confirmed on a Hub yet.
- Capture timestamp (GrayFrame.tNs, elapsedRealtimeNanos): `arrival of the part's first byte (measured) - latency`.
  Latency = `limelight.latencyMs` if set (measured with the probe's LED test), else Limelight-reported cl+tl of the
  newest fresh (< 250 ms) result, EMA 0.2 (a LOWER bound: excludes half the exposure, JPEG encode, transfer; and may
  belong to a different frame), else `limelight.defaultLatencyMs` (30 ms, a guess, telemetry says UNMEASURED).
  Expected errors (not measured): arrival stamp 0.1-2 ms (reader wake-up); cl+tl path late by ~5-20 ms; LED-test
  path ~frame period/sqrt(12N) (< 1 ms at 90 fps, N = 30) + Hub LED command time (shown on telemetry) + jitter.
  Stamps are forced strictly increasing (+1 ns, counted). Until the LED test is done, frames are for recording and
  bench work only, not corrections at speed (2.7 mm per ms at 2.7 m/s).
- LED latency test (`vision/LedLatencyMeter`, `opmodes/HubLedBlinker`): Control Hub status LED toggled green/off
  at random 350-650 ms intervals via `LynxModule.setConstant` (pattern saved and restored); the block of a 16x12
  brightness grid with the largest on/off contrast is the LED (min contrast 20 gray levels, else "not visible",
  nothing guessed); latency = mean(first frame showing the change - change time) - frame period/2.
- Recorder (`vision/DatasetRecorder`) writes I2 to `/sdcard/FIRST/beacon/runs/<yyyyMMdd_HHmmss>[_checker]/`.
  Additions, no change to I2 files: `frames_meta.csv` (idx,t_ns,arrival_ns,last_byte_ns,latency_ms,latency_source,
  jpeg_bytes,decode_ms,width,height) so the bench can re-derive capture times later; `camera_notes.txt`;
  `probe_summary.txt`. camera.json keeps exactly the I2 keys with UNCALIBRATED intrinsics using the F4/sim
  convention (fx = fy from HFOV 54.5 deg, principal point ((w-1)/2, (h-1)/2), zero distortion,
  rollingShutterReadoutS 0); mount = beacon.json T_robot_cam. odom.csv = raw Pinpoint world (TRACKING samples only;
  Pinpoint world resets are listed in probe_summary.txt). PNG is written by our own 8-bit gray encoder
  (java.util.zip only, Sub filter, deflate level 1); default `record.maxHz` 30.
- beacon.json additions: `limelight.{streamUrl, connectTimeoutMs, readTimeoutMs, latencyMs, defaultLatencyMs}`,
  `record.{dir, maxHz, frameQueue, fieldFile, checkerboardHz}`, `probe.{start: [x,y,h], marks: [[x,y,h],...]}`.
- Interface deviation (plan J3 said `latest()` -> frame): `FrameSource.latest(GrayFrame out)` copies the newest frame
  into a caller-owned container (returns false if not newer), plus `setListener` for per-frame delivery on the decode
  thread (recorder now, native core in J2). Reason: no per-frame allocation and no shared mutable buffer.
- Test-side deviation: the Android unit-test classpath has no javax.imageio, so JVM tests use a trivial "fake JPEG"
  container (SOI + size + raw gray + EOI) for the stream plumbing and an independent PNG reader (all filters, CRC
  checks) to verify PNG output. BitmapFactory decoding itself is only exercised on the robot. `MiniJson` made public.
- RISK to check on day one (probe details page shows it): the Control Hub's Android ABI list. If the Hub reports
  only armeabi-v7a, an arm64-v8a-only libbeacon.so (plan J2) will not load there.

## 2026-11-17 field-tools-engineer: BIOBUZZ field model from the official CAD (F1); I1 additions
- Source: FIRST "Field CAD (STEP, .ZIP)" v26-27.2 (2026-09-15, Onshape export 2026-09-09), the official field
  representation per Competition Manual 9.1 (general tolerance +/- 1 in -> toleranceM 0.0254). Pivot height 43.95 in and
  30 deg tilt from Manual Section 9 (TU06) Figs 9-8/9-10; the CAD reproduces them (cells symmetric about the pivot to
  0.001 in) and the manual/checklist numbers (hive bottom 30.6 in, opening 53.5/65.6 in, frame 49.46 x 38.95 in,
  25.5 in hive spacing). Derivation script kept at beacon-field/derive/extract_biobuzz.py (offline; cascadio/trimesh
  in a scratch venv, not Beacon dependencies).
- Frame: FTC field coordinates per ftc-docs red-wall rule; BIOBUZZ red alliance is on the audience's left, so +x toward
  the audience, +y toward blue, +z up, origin field centre on the tile top.
- Deviations from the plan's I1 example values (values, not schema): floor.halfSizeM = 1.7929 (CAD inner rail face,
  141.2 in) instead of 1.8288 (manual "approximately 144 in"); tilePitchM = 0.59694 (CAD seam pitch, 23.50 in) instead
  of 0.6096. Plan test "inner half-size 1.8288 +/- 0.03 m" became: equals the CAD value to 2 mm AND within 0.04 m of
  1.8288 (the CAD is 3.6 cm under the nominal).
- The hive TIPS during a match (two stable states, +/-30 deg about a pivot on +y; red and blue independent). I1 had no
  way to say this. Smallest change, all ADDITIVE optional keys (I1 readers keep working): face "material" and "part"
  (frame | hive_red | hive_blue), hive "pivots" (point, axis, startTiltRad, tiltRad) and "edgeLabels"; wall
  "bottomRailTopM", "panelInsetM"; floor "tiles", "tape"; top-level "flowers" (boxes), "toleranceSource",
  "fieldFrame". Faces are given in the match-start state (Event Field Setup Guide 11.1: red audience CELL up, blue
  scoring CELL up). Native core: treat hive_red/hive_blue geometry as two-state, or anchor only to the frame until the
  state is estimated.
- Rib faces (pentagonal rings) are written as single weakly-simple "keyhole" polygons (outer ring, slit, inner ring
  reversed); skip material rib_* if your polygon code cannot handle that.
- Only opaque prints are textured:true (4 AprilTag-cluster stickers used purely as texture, no decoding; 2 BIOBUZZ
  panel stickers). Cell skins are clear plastic (Initial Field Element Assembly Guide 6.1-6.7) -> textured:false.

## 2026-11-17 java-hub-engineer: Fix path rename (I4), time-sized history, sigma gains, AprilTag code parked (J1)
- `TagFix` -> `FieldFix` (+ `sigmaXY` m, `sigmaHeading` rad; NaN = unknown), `TagFixSource` -> `FieldFixSource`,
  config `"tag"` -> `"fix"` (same keys). A beacon.json that still has `"tag"` is REJECTED with "renamed to fix".
  Localizer API renamed to match: `setFixBlendEnabled`, `isFixBlendEnabled`, `getFixesApplied`, `getFixesRejected`,
  `getHeadingIfTrackingOrNaN` (was `getHeadingForTagsOrNaN`).
- Gain from sigma: `gain = p^2 / (p^2 + sigma^2)` with new keys `fix.priorSigmaXYM` (default 0.02 m) and
  `fix.priorSigmaHeadingRad` (0.02 rad) = assumed error of our own pose between fixes; NaN sigma -> `fix.gainXY` /
  `fix.gainHeading` as before; recovery waiver still uses gain 1. Defaults are guesses, to be tuned on the bench (J4/C4).
  Negative or infinite sigma makes the fix invalid (rejected, counted).
- History sized by TIME: `fix.historyS` (default 0.75 s); ring capacity = ceil(historyS * 4000 Hz) + 16 samples, so the
  window holds at any Pinpoint rate up to 4 kHz (~3016 samples, ~96 KB). Fixes older than historyS are rejected
  ("older than history"); the old fixed 1 s / 128-sample limit is gone.
- Deviation (small, within J1's intent): the speed / turn-rate gate at a fix's capture time is now measured over a
  +/- 20 ms time window instead of "two older, one newer" samples, because at 1.5 kHz three samples span ~2 ms and
  the rate is quantization noise. All existing gate tests pass unchanged. NOT changed: `getVelocity()` still
  differences the last three samples (same noise issue at 1.5 kHz); flagged for a later look, not in J1's scope.
- Parked under `beacon/parked/` (package `...beacon.parked`): `LimelightTagSource`, `LimelightFixFilter`,
  new `LimelightTagConfig` (the MegaTag settings, own parser), and `LimelightTest`. They still compile and their tests
  run (so the 115 originals still run), but nothing in BeaconRig or any OpMode uses them. MegaTag keys left in
  beacon.json's `"limelight"` section only produce a warning. BeaconCalibrationOpMode lost its two AprilTag steps
  (LIMELIGHT SPOT, TURN SIGN); `CalibrationMath.frameAdvice/turnSignAdvice` stay (pure math, still tested), marked
  parked-path helpers.
- Behaviour change: `T_robot_cam` is no longer "ignored by the odometry tier" (warning removed): Beacon's vision and
  every camera.json use it. Test `cameraAnglesGivenForTheOdometryTierAreFlaggedAsIgnored` became
  `cameraMountIsUsedByTheOdometryTierNow`.

## 2026-11-17 field-tools-engineer: Synthetic bench (F2) and evaluation (F3) conventions
- Renderer: exact per-pixel ray casting of textured planar polygons (numpy), sharing one camera model with
  projection (OpenCV distortion, BeaconConfig mount convention). Rendered checkerboard corners vs analytic projection:
  no bias > 0.03 px, RMS < 0.1 px (OpenCV's corner finder scatters ~0.06 px), so the plan's "+/-0.1 px" test is split
  into an exact project() test and this statistical render test.
- Odometry model (plan defaults): per-axis scale U(-0.5 %, +0.5 %); heading random walk "0.5 deg/min" implemented as
  sigma = 0.5 deg after 60 s (sqrt-time growth); 1-3 cm slips over 0.1 s at collisions; 200 Hz; Pinpoint world starts
  at (0, 0, 0). Finding: with these defaults pure odometry aligned with a perfect start pose gave max 4.68 cm over the
  150 s synthetic match seed 1 (passes stage1 by itself), so the stage1 bench gate is weak unless the start pose is
  wrong or the odometry model is made harsher; the core team should also run with --start-error-cm.
- Dataset extras beyond I2 (additive): hive.csv (true hive tilts) and meta.json (parameters, slips, collisions, flips).
  Headings wrapped to (-pi, pi]; t_ns starts at 10 s.
- eval_run.py: truth.csv is interpolated at estimate times (headings unwrapped); with marks.csv the estimate is
  interpolated at mark times. stage1 = every truth sample up to t = 150 s (every mark, with marks.csv) < 0.05 m AND the
  estimate reaches t = 150 s; stage2 = <= 0.02 m and <= 1 deg everywhere. t = 0 is start.csv's t_ns.

## 2026-11-17 field-tools-engineer: Hard odometry profile (F2b)
- `make_dataset.py --odom-profile {nominal,hard}` (default nominal; nominal runs are bit-identical to before: the
  heading-jump draw only happens when the profile has jumps). hard = per-axis scale U(+/-1.5 %), heading random walk
  2 deg SD after 60 s, 2-5 cm slip and U(+/-2 deg) heading jump (spread over 0.1 s) at each collision, collisions
  topped up with random times to U{4..8} per 150 s (scaled by duration; clutter contacts count), start error 5 cm /
  3 deg unless --start-error-cm/--start-error-deg are given. meta.json records odomProfile, odomParams, startErrorCmDeg.
- New baseline tool tools/eval/odom_only.py: aligns odom.csv to the field once via start.csv and never corrects (pure
  Pinpoint). Measured stage1 max error, odometry only, match seeds 1-5: nominal 4.68 / 4.63 / 5.01 / 6.97 / 4.43 cm
  (3 of 5 pass); hard with exact start 15.7 / 18.4 / 33.1 / 11.8 / 25.7 cm; hard with default start error
  33.2 / 29.7 / 32.6 / 29.6 / 47.9 cm (all fail).
- Shared runs for the native engineer live in tools/sim/runs/ (gitignored): hard-match-s1..s5 (30 fps) and
  hard-session-s1 (10 fps, to fit the disk: 14 GB free when generated).
