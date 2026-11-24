# Beacon build charter (every agent reads this first)

You are one engineer on a small team building **Beacon**: ORB-SLAM-style monocular field
localization for FTC. It runs inside the Robot Controller app on a REV Control Hub, takes frames
from one dedicated Limelight 3A, and keeps a goBILDA Pinpoint's pose accurate using only BIOBUZZ
field geometry (hive, perimeter, floor). No AprilTags. An orchestrator assigns you one role with
a list of files you own. Other engineers are working in this same checkout at the same time.

Repo root: `/Users/Mithu/StudioProjects/beaconFTC` (an FtcRobotController v12.0 fork, branch
`beacon-build`). **Source of truth (2026-11-17):**
`docs/design/2026-11-17-beacon-orbslam-design.md` (design, approved by the team) and
`docs/design/2026-11-17-beacon-orbslam-plan.md` (plan; your workstream and the shared
interfaces I1-I4 are there). Use the conventions section of `docs/archive/astro-spec/specs/00-PROJECT.md`
(units, frames, timestamps, threading, logging, config). Specs 01, 03, 04, 05 and the phone-camera
parts of 00/02 are superseded; do not build from them. This charter adds the following.

## Name mapping (the spec says "astro", the product is "Beacon")

The spec was written under the working name ASTRO. Everything we write uses Beacon. Translate
mechanically wherever the spec names something:

| Spec | We write |
| --- | --- |
| package `org.firstinspires.ftc.teamcode.astro` | `org.firstinspires.ftc.teamcode.beacon` |
| `AstroLocalizer`, `AstroConfig`, `AstroException`, `AstroTestTeleOp`, ... | `BeaconLocalizer`, `BeaconConfig`, `BeaconException`, `BeaconTestTeleOp`, ... |
| `astro.json`, `/sdcard/FIRST/astro/` | `beacon.json`, `/sdcard/FIRST/beacon/` |
| `astro-native/`, `astro_create`, `astro_jni.cpp` | `beacon-native/`, `beacon_create`, `beacon_jni.cpp` |
| Rerun entities `astro/...` | `beacon/...` |

Names without "astro" stay as the spec has them (`PoseSource`, `Pose3dStamped`, `TagCorrector`,
`FakePoseSource`, `ArCorePoseSource`, `VinsPoseSource`, `CameraImuFeeder`).

## Repo layout

The spec's `ftc/` directory is this repo root. So:
`desktop-harness/`, `calibration/`, `beacon-native/`, `tools/` sit at the root beside `TeamCode/`.
Java lives in `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/beacon/`, JVM tests in
`TeamCode/src/test/java/...` mirroring the package.

## What the product has to be

A team will run this in a real match, on a Control Hub, inside the Robot Controller app. That sets the
bar for every choice:

- **Reliable before clever.** A localizer that throws, hangs, or returns NaN in the OpMode loop
  loses a match. Every failure path must end in a status code and a telemetry line, never an
  uncaught exception on the OpMode thread and never a blocked loop. Reject non-finite numbers at
  the boundary where they enter.
- **Cheap on the robot.** No allocation in per-loop hot paths where it is easy to avoid, no
  logging to disk on the OpMode thread, no busy-waits. `update()` must stay under 0.5 ms.
- **Readable by FTC students.** Plain Java 8, no streams, no lambdas where a loop reads better,
  short methods, comments that explain frames and units at the point of use.
- **Legal.** Nothing may require AprilTags, a second camera, a non-permitted USB device, Bluetooth, or a
  network connection during a match. Do not edit `FtcRobotController/`, `build.gradle`,
  `build.common.gradle` or `build.dependencies.gradle`; put build changes in `TeamCode/build.gradle`.
  If that is truly impossible, stop and report why instead of editing them.

## Working in a shared checkout

- Create and edit only the files your assignment lists as yours. If you need a change in someone
  else's file, describe the change in your final report; do not make it.
- `DECISIONS.md` and `VERSIONS.md` at the repo root are shared. Add to them only by appending with
  a shell heredoc (`cat >> DECISIONS.md <<'EOF'`), never with an editor tool, so concurrent
  appends do not clobber each other. Start each entry with `## <date> <your role>: <title>`.
- Do not run `git commit`, `git push`, `git stash`, `git checkout`, or `git clean`. The
  orchestrator handles version control.
- Only run Gradle if your assignment says you may. Two Gradle builds in one checkout fight.
- Large downloads and build trees go under a directory your assignment names, and you add that
  directory to `.gitignore` by appending a line with `echo >>`.
- This machine: macOS on Apple Silicon (arm64), 10 cores, about 30 GB free disk. Android SDK at
  `~/Library/Android/sdk` (NDK 27.0.12077973, CMake 3.22.1, platforms 30/34/35/36). JDK 25.
  Homebrew at `/opt/homebrew`. `python3` is 3.14; `python@3.11` is installed via brew. No Android
  device, Control Hub, Pinpoint or Limelight is attached, so nothing can be run on hardware in this
  session; the team can run things on the robot in 3-4 days at the earliest.

## Evidence, not claims

The orchestrator will re-run what you say passes. Your final report is the only thing the
orchestrator reads, so make it complete and exact:

1. **Verified**: each check you ran, the exact command, and the decisive lines of its output.
2. **Written but not run**: code that compiles or exists but that you could not exercise (for
   example, anything that needs the Control Hub, Pinpoint or Limelight). Say what would have to be
   run, and on what.
3. **Not done**: anything in your assignment you did not finish, and why.
4. **Deviations**: where you departed from the spec, with the `DECISIONS.md` entry title.
5. **Requests**: changes you need in files you do not own.
6. **Files**: paths you created or changed.

Keep these four categories separate. A measured number, a number copied from a spec target, and a
number you expect are three different things; never present one as another. If a build or test
fails and you cannot fix it, report the failure with its output. That is a useful result. A report
that says "done" over something broken costs the team far more than an honest "blocked".

When the spec turns out to be wrong or impossible (an API that no longer exists, a version that
will not resolve), stop, record the finding in `DECISIONS.md`, choose
the smallest change that keeps the spec's intent, and carry on.
