#!/usr/bin/env bash
# Beacon desktop harness setup (macOS, Apple Silicon, Homebrew at /opt/homebrew). Idempotent.
#
#   1. Homebrew build dependencies (versions verified in VERSIONS.md)
#   2. rpng/open_vins at the pinned commit -> third_party/open_vins, plus our patches/
#   3. OpenVINS ROS-free build (-DENABLE_ROS=OFF) -> build/ov_msckf, installed to build/install
#   4. our EuRoC driver -> build/driver/euroc_driver
#   5. Python venv for eval/ and viz/ -> .venv (python3.11; evo and rerun pinned in requirements.txt)
#
# No ROS, no Docker, no Rosetta.
set -euo pipefail

OPENVINS_REPO=https://github.com/rpng/open_vins.git
OPENVINS_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b   # master, 2025-11-30 (after v2.7)
BREW=/opt/homebrew/bin/brew
PY=/opt/homebrew/bin/python3.11
JOBS="$(sysctl -n hw.ncpu 2>/dev/null || echo 8)"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$HERE"
OV="$HERE/third_party/open_vins"
PREFIX="$HERE/build/install"

log() { printf '\n==> %s\n' "$*"; }

[[ "$(uname -s)" == Darwin ]] || { echo "macOS only" >&2; exit 1; }
[[ "$(uname -m)" == arm64 ]] || echo "warning: not arm64 ($(uname -m)); paths below assume /opt/homebrew" >&2
[[ -x "$BREW" ]] || { echo "Homebrew not found at $BREW" >&2; exit 1; }
xcode-select -p >/dev/null 2>&1 || { echo "install Xcode Command Line Tools: xcode-select --install" >&2; exit 1; }

# ---- 1. Homebrew --------------------------------------------------------------------------
# opencv@4, not opencv (5.x): the OpenVINS CMake asks for OpenCV 3 or 4, and the Android build uses
# the OpenCV 4 Android SDK, so desktop and phone share a major version. eigen is Eigen 5.x, which
# the Homebrew ceres-solver is built against; patch 0001 makes OpenVINS compile with it.
log "Homebrew dependencies"
DEPS=(cmake eigen boost opencv@4 ceres-solver pkgconf python@3.11)
MISSING=()
for d in "${DEPS[@]}"; do
  "$BREW" list --versions "$d" >/dev/null 2>&1 || MISSING+=("$d")
done
if ((${#MISSING[@]})); then
  HOMEBREW_NO_AUTO_UPDATE=1 "$BREW" install "${MISSING[@]}"
fi
"$BREW" list --versions "${DEPS[@]}"
export PATH="/opt/homebrew/bin:$PATH"
OPENCV_PREFIX="$("$BREW" --prefix opencv@4)"

# ---- 2. OpenVINS source at the pinned commit + patches ------------------------------------
log "OpenVINS $OPENVINS_COMMIT"
mkdir -p "$HERE/third_party"
if [[ ! -d "$OV/.git" ]]; then
  git clone --quiet "$OPENVINS_REPO" "$OV"
fi
HEAD_NOW="$(git -C "$OV" rev-parse HEAD)"
if [[ "$HEAD_NOW" != "$OPENVINS_COMMIT" ]]; then
  if ! git -C "$OV" diff --quiet; then
    echo "third_party/open_vins has local changes and is not at the pinned commit; remove it and re-run" >&2
    exit 1
  fi
  git -C "$OV" fetch --quiet origin "$OPENVINS_COMMIT" || git -C "$OV" fetch --quiet origin
  git -C "$OV" -c advice.detachedHead=false switch --quiet --detach "$OPENVINS_COMMIT"
fi
for p in "$HERE"/patches/*.patch; do
  [[ -e "$p" ]] || continue
  if git -C "$OV" apply --reverse --check "$p" 2>/dev/null; then
    echo "patch already applied: $(basename "$p")"
  elif git -C "$OV" apply --check "$p"; then
    git -C "$OV" apply "$p"
    echo "applied: $(basename "$p")"
  else
    echo "patch does not apply: $p (is the clone at $OPENVINS_COMMIT with no other edits?)" >&2
    exit 1
  fi
done

# ---- 3. OpenVINS ROS-free build -----------------------------------------------------------
# CMAKE_POLICY_VERSION_MINIMUM: OpenVINS declares cmake_minimum_required(3.3); CMake 4 refuses
#   anything below 3.5 unless told to assume 3.5 policies.
# CMAKE_PREFIX_PATH (not OpenCV_DIR): OpenVINS first probes find_package(OpenCV 3 QUIET), which
#   resets OpenCV_DIR to NOTFOUND when it sees 4.x; the prefix path survives that probe.
# ENABLE_ARUCO_TAGS=OFF: ArUco lives in opencv_contrib, which the OpenCV Android SDK lacks; Beacon
#   does its AprilTag work in the FTC SDK, not in the estimator.
log "OpenVINS ROS-free build"
cmake -S "$OV/ov_msckf" -B "$HERE/build/ov_msckf" -Wno-dev \
  -DCMAKE_BUILD_TYPE=Release \
  -DENABLE_ROS=OFF \
  -DENABLE_ARUCO_TAGS=OFF \
  -DDISABLE_MATPLOTLIB=ON \
  -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
  -DCMAKE_PREFIX_PATH="$OPENCV_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$PREFIX"
cmake --build "$HERE/build/ov_msckf" -j "$JOBS"
cmake --install "$HERE/build/ov_msckf" >/dev/null
ls -la "$PREFIX/lib/libov_msckf_lib.dylib"

# ---- 4. our driver ------------------------------------------------------------------------
log "EuRoC driver"
cmake -S "$HERE/driver" -B "$HERE/build/driver" -Wno-dev \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="$OPENCV_PREFIX" \
  -DOPENVINS_PREFIX="$PREFIX"
cmake --build "$HERE/build/driver" -j "$JOBS"
ls -la "$HERE/build/driver/euroc_driver"

# ---- 5. Python venv -----------------------------------------------------------------------
log "Python venv (.venv)"
if [[ ! -x "$HERE/.venv/bin/python" ]]; then
  "$PY" -m venv "$HERE/.venv"
fi
"$HERE/.venv/bin/pip" install --quiet --upgrade pip
"$HERE/.venv/bin/pip" install --quiet -r "$HERE/requirements.txt"
"$HERE/.venv/bin/python" -c "import evo, rerun; print('evo', evo.__version__, '| rerun', rerun.__version__)"

log "setup complete. Next: data/get_euroc.sh MH_01_easy && ./run.sh MH_01_easy"
