# Beacon

Vision-based field localization for *FIRST* Tech Challenge robots.

Beacon runs monocular, ORB-SLAM-style visual localization on a REV Control Hub, using frames from
one Limelight 3A. Its job is to keep a goBILDA Pinpoint's odometry pose accurate on the field for
a whole match or practice session, with no AprilTags and no per-venue setup. Team code reads one
field-frame pose through `BeaconLocalizer`.

> **Status:** in development for the 2026-27 BIOBUZZ season. The vision core and the Java
> localizer run on recorded and simulated data. On-robot validation has not been done yet.

## How it works

- **Odometry first.** The Pinpoint gives a fast, smooth pose. Beacon only corrects the drift.
- **Field mode (robot).** Each Limelight frame is aligned to a model of the field: perimeter
  rails, posts, the wall-floor line and the static hive frame. ORB points are added only on known
  surfaces (the floor and the hive faces). A field-frame fix with a calibrated 1-sigma is published
  when it passes the gate.
- **Explore mode (bench).** General monocular ORB SLAM with no field model: two-view
  initialization, keyframes, local bundle adjustment, bag-of-words place recognition, loop
  closing, relocalization and multiple maps. It is used to develop and test the core on a Mac.
- **Latency-aware fusion.** `BeaconLocalizer` compares each fix with the pose the robot had when
  the frame was captured, so camera latency does not turn into a position error.

## Repository layout

| Path | Contents |
| --- | --- |
| `TeamCode/.../teamcode/beacon/` | Java localizer, Pinpoint source, config, and the test, probe and calibration OpModes |
| `beacon-native/` | C++17 vision core (field and explore modes), JNI glue, bench tools and Rerun viewer. See its [README](beacon-native/README.md). |
| `beacon-field/` | Field models, one JSON file per season, derived from FIRST's field CAD |
| `tools/` | Log plotting and comparison, evaluation, the simulator and Limelight scripts |
| `calibration/` | Camera and IMU calibration tooling |
| `desktop-harness/` | Desktop replay harness for public datasets |
| `docs/design/` | Design spec and implementation plan |
| `docs/reviews/`, `docs/process/` | Code reviews and the working charter |
| `docs/archive/` | Earlier phone-camera spec bundle, kept for reference |
| `DECISIONS.md` | Log of design decisions and spec findings |
| `WIRING.md`, `RULES.md` | Wiring and mounting notes, and legality notes for inspection |
| `VERSIONS.md` | Pinned versions of all third-party dependencies |
| `beacon.json` | Robot configuration: the Limelight mount pose relative to the robot |

Everything else (`FtcRobotController/`, the Gradle files, `doc/`) is the stock FTC SDK v12.0.
Its original README is in [doc/FTC_SDK_README.md](doc/FTC_SDK_README.md).

## Building

**Robot Controller app.** Open the project in Android Studio (Narwhal 3 Feature Drop or later)
and deploy as with any FTC SDK project. The JVM tests run on the desktop:

```sh
./gradlew :TeamCode:testDebugUnitTest
```

**Vision core (macOS arm64).** Third-party libraries are built once into
`beacon-native/third_party/`. The steps are in [beacon-native/README.md](beacon-native/README.md).
After that:

```sh
cd beacon-native
cmake -S . -B build && cmake --build build -j10
./build/beacon_tests                 # unit tests
./scripts/build_android.sh           # libbeacon.so for the Control Hub (arm64-v8a)
```

## Hardware

REV Control Hub, Limelight 3A, and a goBILDA Pinpoint Odometry Computer with two odometry pods.
See [WIRING.md](WIRING.md) for wiring, mounting and start placement.

## License

The FTC SDK portions are under FIRST's license ([LICENSE](LICENSE)). `beacon-native/` is MIT
([beacon-native/LICENSE](beacon-native/LICENSE)).
