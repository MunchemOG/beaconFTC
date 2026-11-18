# Pinned versions

Every third-party dependency, with the exact version or git commit. Append-only.

| Component | Version / commit | Where used | Added by |
| --- | --- | --- | --- |
| FTC SDK (FtcRobotController) | 12.0.0 | RC app | orchestrator |
| Android Gradle Plugin | 8.13.2 | build | orchestrator |
| Gradle wrapper | 9.1.0 | build | orchestrator |
| Android NDK (installed) | 27.0.12077973 | Tier 2 native | orchestrator |

## core-java (2026-11-16)
- JUnit 4.13.2 (`testImplementation` in TeamCode/build.gradle; resolved from the local Gradle cache).

## tier1-engineer (2026-11-17)
- No new dependencies. The odometry tier uses `GoBildaPinpointDriver` and `Limelight3A`/`LLResult` from
  `org.firstinspires.ftc:Hardware:12.0.0` (already in build.dependencies.gradle). ARCore was evaluated (1.48.0 /
  1.57.0) but is NOT in the build.

## field-tools-engineer (2026-11-17)
- opencv-python 4.14.0.94 (wheel cp37-abi3-macosx_13_0_arm64), installed into `tools/.venv` (Python 3.11.14) with
  numpy 2.4.6 (already there). Listed in `tools/limelight/requirements.txt` and `tools/sim/requirements.txt`. Used by
  tools/limelight (JPEG decode, PNG write, window, checkerboard calibration) and tools/sim (resize, PNG write).

## 2026-11-17 native-core (Workstream C, beacon-native/)
Sources downloaded into `beacon-native/third_party/src/` (gitignored), built by `beacon-native/scripts/*.sh`.
| Component | Version / commit | License | Where used | sha256 of the downloaded archive |
| --- | --- | --- | --- | --- |
| OpenCV (core, imgproc, features2d, flann, calib3d; host also imgcodecs, highgui) | 4.12.0 (github tag) | Apache-2.0 | core (host + Android, static) | 44c106d5bb47efec04e531fd93008b3fcd1d27138985c5baf4eafac0e1ec9e9d |
| Eigen | 3.4.0 (gitlab tag) | MPL-2.0 (headers; we use no LGPL modules) | core | 8586084f71f9bde545ee7fa6d00288b264a2b7ac3607b974e54d13e7162c1c72 |
| Ceres Solver | 2.2.0 (github tag; miniglog, no SuiteSparse/LAPACK) | BSD-3 | core (local BA, pose graph) | 12efacfadbfdc1bbfa203c236e96f4d3c210bed96994288b3ff0c8e7c6f350d4 |
| nlohmann/json | 3.11.3 (single header) | MIT | core (field model, params) | 9bea4c8066ef4a1c206b2be5a36302f8926f7fdc6087af5d20b417d0cf103ea6 |
| GoogleTest | 1.15.2 | BSD-3 | tests (host) | 7b42b4d6ed48810c5362c265a17faebe90dc2373c885e5216439d37927f02926 |
| Rerun C++ SDK | 0.41.0 (rerun_cpp_sdk-0.41.0-multiplatform.zip; bundles Arrow C++ 18.0.0, Apache-2.0) | MIT/Apache-2.0 | bench viewer only (host) | 9b41c4ec0ca2cc9d4f53e0e4c6d0b4f8ff1090d1dd6419385fc925eb6990e226 |
| Rerun CLI / viewer | 0.41.0 (rerun-cli-0.41.0-aarch64-apple-darwin) | MIT/Apache-2.0 | bench viewer only | db4f6ece9eea4c6b8d997e4c9be6d0f923d49ef06ea196869e94f1b833df8233 |
| rerun-sdk (Python, venv in third_party/venv) | 0.41.0 | MIT/Apache-2.0 | writes bench/viewer/beacon.rbl only | pip |
| Android NDK / CMake | 27.0.12077973 / 3.22.1 (installed) | - | libbeacon.so cross-build | - |
Host-only bench binaries also link OpenCV's bundled libpng/libjpeg-turbo/zlib (permissive, not in libbeacon.so).
- Offline only (field-model derivation, scratch venv, NOT Beacon dependencies): cascadio 0.1.1 (MIT; bundles
  OpenCASCADE, LGPL-2.1), trimesh 5.2.1 (MIT), opencv-python-headless 4.14.0.94, pypdf 5.4.0 (BSD) for reading the
  FIRST PDFs, Python 3.11.14.
