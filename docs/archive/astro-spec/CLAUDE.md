> **SUPERSEDED IN PART (2026-11-17).** Beacon is now ORB-SLAM-style monocular field localization
> on a REV Control Hub with a dedicated Limelight 3A and a goBILDA Pinpoint, no AprilTags. Build
> from `docs/design/2026-11-17-beacon-orbslam-design.md` and
> `docs/design/2026-11-17-beacon-orbslam-plan.md`. Specs 01, 03, 04, 05 and the phone-camera
> parts of 00 and 02 are superseded. The conventions section of `00-PROJECT.md` still applies.
> Name mapping ASTRO -> Beacon: `docs/process/CHARTER.md`.

# Working instructions for Claude Code on this repo

Read `specs/00-PROJECT.md` first, then the spec for the task you were given. Specs are numbered
in build order; do not start `04` before `02` and `03` have passing acceptance tests.

Rules:
- Follow the conventions section of `00-PROJECT.md` exactly (units, frames, timestamps,
  threading, logging, config). If a spec and a convention disagree, the convention wins; note the
  conflict in `DECISIONS.md`.
- Every spec has an Acceptance section. A task is not done until those checks run and pass; put
  the command and its output in the PR description.
- Pin every third-party dependency (git commit or version) in `VERSIONS.md`.
- Prefer boring Java that FTC students can read over clever Kotlin or streams.
- Never block the OpMode loop. Never use wall-clock time for sensor data.
- When something in a spec turns out to be wrong or impossible (an API that no longer exists, an
  ARCore limitation), stop, write the finding in `DECISIONS.md`, and propose the smallest change
  to the spec before implementing a workaround.
- Do not touch rules text in `RULES.md` without a link to the current Competition Manual section.

Suggested task sizes (one Claude Code session each):
0. Environment is **macOS-only**: never add Linux/ROS/Docker requirements to the main workflow
   (Kalibr-in-Docker is an optional cross-check only).
1. `01` harness setup (OpenVINS ROS-free + Rerun) + EuRoC run + `ate.py` + `speed.py`
2. `02` package skeleton + `FakePoseSource` + unit tests
3. `02` `AstroLocalizer` + `TagCorrector` + OpModes
4. `03` `ArCorePoseSource`
5. `05` calibration tooling and docs
6. `04` native build of OpenVINS for Android arm64 (desktop arm64 build from 01 first)
7. `04` `CameraImuFeeder` + JNI + replay test
8. `06` tools (`plot_log.py`, `compare_logs.py`), `07` docs, `08` release
