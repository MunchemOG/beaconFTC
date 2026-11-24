## Beacon core-java review (read-only; nothing in the repo was changed, no Gradle run)

**Bottom line:** I found no errors in the frame math, the transform order, the yaw extraction, the tag-blend geometry or the velocity. I checked each one against my own first-principles version and they agree to about 1e-15. The real problems are in failure handling and recovery. Four of them can teleport the pose or freeze the localizer during a match.

### How I checked
- I copied the sources into the scratchpad with stubs for `Context` and `Pose2D`, compiled them with `javac`, and drove them through a small harness (`Driver.java`, `Probe.java`, `CfgProbe.java`).
- The expected values come from Python/numpy scripts built from the documented conventions. Those scripts do not use FrameMath or FakePoseSource.
- Everything is in `/private/tmp/claude-501/-Users-Mithu-StudioProjects-beaconFTC/ede7afc8-8f7b-4629-8dce-da43a6ae7ae9/scratchpad/review/`. Run with `tools/.venv/bin/python check_frames.py`, `check_loc.py`, `check_fail.py`, `check_cfg.py`, `check_more.py`; the Java probe is `java -cp out org.firstinspires.ftc.teamcode.beacon.Probe`.

## Findings, worst first

**1. A one-frame source dropout disables the tag jump test indefinitely, at gain 1 (confirmed).**
- Where: `BeaconLocalizer.java` lines 311-313 set `needRecover`; lines 514-516 skip the jump test and force gain 1.
- `needRecover` is set by any non-TRACKING sample, however short. It is cleared only by the next tag fix or `setPose`, so the waiver lasts as long as no fix arrives.
- Repro:
  - The source is LOST for one sample, then tracks normally for 5 s with the robot parked at (0.2, 0, 0).
  - An outlier fix arrives at (2.2, 1.0, 1.2 rad), with maxJumpM=0.5 and gains 0.3/0.2.
  - Output jumps to (2.200, 1.000, 1.200) with status TRACKING. It should have been rejected.
- The spec only waives the test while "status **is** initializing or lost". ARCore drops to PAUSED for a frame on collisions and fast spins, so this will happen in matches. The existing test `tagFixAfterLossIsAcceptedWholeEvenIfFar` locks this behaviour in.
- Smallest fix: on the first good sample after a loss, store `recoverUntilTNs = s.tNs + 1e9`. Waive the test only if `needRecover && f.tNs <= recoverUntilTNs`, and clear `needRecover` once that window passes.

**2. A source that resets its world frame gives a wrong pose with status TRACKING (confirmed; design gap).**
- Where: `processSample`, lines 303-336. The old alignment is reused after LOST → INITIALIZING → TRACKING.
- Repro:
  - `setPose(1, 1, π/2)`, then drive 0.99 m. The source goes LOST, then re-inits with a new origin at the robot, as VINS-Fusion does.
  - Output is (1.00, 1.04) TRACKING; the truth is (1.00, 2.04).
  - A 5 m world jump while the source stays TRACKING is also accepted silently.
- Without a tag in view, the robot drives off a 1 m error.
- Smallest fix: add a world-epoch counter to the PoseSource contract (field on `Pose3dStamped` or a `worldEpoch()` method). When it changes, call `alignTo(firstNewSample, lastOutput)` so the pose continues from the held one, and set `needRecover`.

**3. One bad timestamp freezes the localizer or silently disables tags (confirmed).**
- Source sample (lines 292-300): one sample stamped 10 s in the future is accepted, and `lastTNs` stays at that value. Every later real sample is rejected as "time went backwards". Repro: LOST for all of the next 190 samples, pose frozen; this lasts until real time passes the bad stamp.
- Tag fix (lines 444-452, 473):
  - A fix stamped in the future is accepted with no limit and compared against the newest sample.
  - It also moves `lastFixTNs` forward, so every later correct fix is dropped. They are counted as neither applied nor rejected (repro: `A=1 R=0` after a valid fix 10 cm off).
- Clock-base mixups are likely: the SDK's `frameAcquisitionNanoTime` is `System.nanoTime`, and ARCore's frame timestamp base is undefined.
- Smallest fix:
  - Reject fixes with `f.tNs > rt[newest] + ~0.2 s`, count them, and do not advance `lastFixTNs` for rejected fixes.
  - For samples, if they keep arriving "backwards" for longer than `sourceTimeoutS`, accept the new time base: reset `lastTNs` and the history ring, and set `needRecover`.

**4. A tag source that throws makes the localizer report LOST permanently (confirmed).**
- Where: lines 270-279. The single try/catch sets `state = ST_BAD` for exceptions from the tag source too.
- Repro: with a throwing `TagFixSource`, the localizer is LOST on 50 of 50 updates and `samplesRejected=50`, while the pose itself is fine.
- The existing test only checks that it doesn't throw.
- Fix: give `processTagFix()` its own try/catch that calls `fixRejected(...)` and leaves `state` alone.

**5. Angles typed in degrees up to 6 pass validation (confirmed).**
- Where: `BeaconConfig.requireAngle`, line 224.
- Repro (true mount: pitch 5°, lens 0.22 m from the centre):
  - `"pitch": 5` is accepted. Spinning in place then gives 0.232 m of position error.
  - `"yaw": 3` gives 0.399 m of error, and `getVelocity()` reports vx = −0.99 instead of +1.0. Reversed velocity is poison for Pedro's controllers.
- Fix: limit pitch to ±π/2 (no loss of generality) and warn on integer-valued angles of magnitude ≥ 2. Alternatively, accept `rollDeg/pitchDeg/yawDeg` keys.

**6. The portrait roll sign is not documented.**
- Where: the docs in `BeaconConfig` (lines 51-52) and DECISIONS.
- Effect of the wrong sign (confirmed): 0.375 m error while spinning in place; heading is unaffected.
- Which sign is right (suspected, from ARCore/Camera2 docs, not tested on a device):
  - `getPose()` uses the sensor readout orientation.
  - A rear camera with `SENSOR_ORIENTATION` = 90, phone upright in portrait, gives roll = **+π/2**.
- Fix: document the sign, and have the calibration OpMode's spin check flag it.

**7. Velocity goes stale while LOST from a timeout or a rejected sample (confirmed).**
- Repro: after 2 s of source silence, status is LOST but `vx` is still 1.0.
- Fix: in `publish`, use `Velocity2d.ZERO` whenever the status is not TRACKING.

**8. Huge finite inputs reach the output as Infinity/NaN (confirmed, unlikely input).**
- Repro: a source position of 1.5e308 publishes y = Infinity with status TRACKING, and velocity becomes NaN. A huge tag fix during the recovery waiver can do the same to the alignment.
- Fix: reject |x|, |y|, |z| > 1e4 m on samples and fixes, and add a final `isFinite` guard in `publish`.

**9. A held pose still gets newer timestamps (confirmed).**
- Line 553 stamps the pose with `lastTNs`, which advances on rejected and non-tracking samples. Repro: the held pose is stamped 1.5e9 while its data is from 1.09e9.
- Fix: stamp with the time of the last good sample.

**10. FrameMath allocates despite its Javadoc (confirmed).**
- The doc (lines 4-6) says the methods don't allocate, but `compose`, `invert`, `transformPoint`, `robotFromCamera` and `fromArCoreCameraPose` build temporary arrays.
- Measured with escape analysis turned off (ART may not remove these allocations): 184 B per `compose` call and 584 B per `fromArCoreCameraPose` call. A pose source calls the latter every frame.
- This is not on the OpMode hot path. `update()` itself measured 0 B per idle call and 96 B per new sample (the two result objects, as documented), with no I/O.

**11. Lower priority:**
- **PoseLogWriter restart (suspected):** if `close()`'s 1.5 s join times out and `start()` is called again, the old drain thread sees `running == true` and keeps going, so two consumers share one ring. `start()` also truncates an existing file on restart.
- **Config usability (confirmed by probe):**
  - A misspelled section such as `"T_robot_camera"` only produces warnings; the robot runs with the camera assumed at the centre.
  - A UTF-8 BOM gives an error about an invisible character.
  - The JSON example in the `BeaconConfig` Javadoc contains `//` comments, so a student who copies it gets a parse error.
- **Doc nit:** the class Javadoc (lines 30-31) says the heading correction pivots on the robot's current position. The code correctly pivots on its position at the fix time.

## Checked and correct
- **Mount transform:** `robotFromCamera` matches `Rz(yaw)·Ry(pitch)·Rx(roll)·R_baseline` to 6.7e-16 over 205 cases. The baseline has determinant +1. The physical meanings hold: pitch>0 looks down, yaw>0 looks left, roll>0 raises the image-left edge, with +x forward, +y left, +z up.
- **ARCore conversion:** `fromArCoreCameraPose` equals `Rx(+90°)·T_ar·diag(1,−1,−1)` (y-up world to z-up; GL x-right/y-up/z-back body to optical) to 8.9e-16 over 201 random poses. A camera yawed 90° about ARCore up looks along −x, as expected.
- **Yaw with tilt:** `yawFromQuat` is correct with roll and pitch present (1.1e-15).
- **Whole pipeline:** `T_sw_cam·inv(T_robot_cam)`, projection and alignment, on Python-generated camera poses: four awkward mounts (including portrait ±π/2 and a 180° roll), a robot on uneven floor, arbitrary alignment. Max error 2.7e-15.
- **Tag blending:**
  - Tested with a stale fix (25 samples old) carrying both heading and position error while the robot drives an arc, at gains 1, 0.5 and 0.3. Java matches my analytic result to 1e-9.
  - Gain 1 recovers the truth exactly, which shows the pivot-at-fix-time choice is right.
  - Partial gain moves the pose toward the fix by exactly that fraction at the fix time (heading exactly g; the current position shift is about g, because the motion since the fix is also rotated, which is correct).
  - The correction persists.
- **Velocity:** on an arc through ±π with a tag correction mid-way, vx, vy and ω match the analytic chord to within 1e-13. There is no spike from the correction.
- **Other failure inputs:** NaN/Inf inputs, non-unit quaternions, repeated and backwards timestamps (single events), a silent source going to LOST, `update()` before `start()`, and `stop()` twice are all handled.
- **Concurrency:** immutable final-field poses behind volatile references are published safely. The PoseLogWriter SPSC ring is ordered correctly for one producer and one consumer: the volatile `head` is written after the row data, and `tail` after the read.
- **Config parsing:** trailing comma, number in quotes, single quotes, duplicate keys, 1e999, angles beyond 2π, lever arm in millimetres or inches, and gains as percentages all give clear errors.

## Test strength
- `BeaconLocalizerTest` builds its camera poses with `FakePoseSource.cameraSample`, which uses the same `toTransform()` the localizer inverts. A convention error in the mount math would cancel out there; only `FrameMathTest`'s physical-direction checks, and my independent check, protect against it.
- The ARCore tests use only an identity rotation.
- Neither tag-latency test separates pivoting at the fix time from pivoting at the current position. One fix is at the newest time; the other has zero error.
- No tests cover future timestamps, world resets, stale velocity after a timeout, a waiver long after recovery, or degree-valued angles below 2π.
- Two tests enshrine weak behaviour: one locks in finding 1, and the tag-exception test only checks "no throw", which hides finding 4.
