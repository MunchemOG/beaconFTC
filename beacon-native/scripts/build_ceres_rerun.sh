#!/bin/bash
# Builds Ceres 2.2.0 (miniglog, Eigen 3.4.0, no SuiteSparse/LAPACK, static) for host or Android,
# and the Rerun C++ SDK 0.41.0 (with its pinned Arrow 18.0.0) for the host.
# Usage: build_ceres_rerun.sh ceres-host|ceres-android|rerun-host
set -euo pipefail
HERE="$(cd "$(dirname "$0")/../third_party" && pwd)"
CERES_COMMON=(
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
  -DBUILD_BENCHMARKS=OFF -DBUILD_DOCUMENTATION=OFF -DMINIGLOG=ON -DGFLAGS=OFF -DSUITESPARSE=OFF
  -DLAPACK=OFF -DACCELERATESPARSE=OFF -DCUDA=OFF -DUSE_CUDA=OFF -DEIGENSPARSE=ON
  -DSCHUR_SPECIALIZATIONS=OFF -DPROVIDE_UNINSTALL_TARGET=OFF -DMINIGLOG_MAX_LOG_LEVEL=-1
  -DEigen3_DIR="$HERE/eigen-install/share/eigen3/cmake"
)
eigen_install() {
  if [ ! -f "$HERE/eigen-install/share/eigen3/cmake/Eigen3Config.cmake" ]; then
    cmake -S "$HERE/eigen" -B "$HERE/eigen-build" -DCMAKE_INSTALL_PREFIX="$HERE/eigen-install" \
      -DBUILD_TESTING=OFF -DEIGEN_BUILD_DOC=OFF >/dev/null
    cmake --build "$HERE/eigen-build" --target install >/dev/null
  fi
}
case "$1" in
  ceres-host)
    eigen_install
    cmake -S "$HERE/ceres" -B "$HERE/ceres-build-host" "${CERES_COMMON[@]}" \
      -DCMAKE_INSTALL_PREFIX="$HERE/ceres-host"
    cmake --build "$HERE/ceres-build-host" -j10 --target install
    ;;
  ceres-android)
    eigen_install
    NDK="$HOME/Library/Android/sdk/ndk/27.0.12077973"
    SDKCMAKE="$HOME/Library/Android/sdk/cmake/3.22.1/bin"
    "$SDKCMAKE/cmake" -S "$HERE/ceres" -B "$HERE/ceres-build-android" "${CERES_COMMON[@]}" -G Ninja \
      -DCMAKE_MAKE_PROGRAM="$SDKCMAKE/ninja" \
      -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
      -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-24 -DANDROID_STL=c++_static \
      -DCMAKE_FIND_ROOT_PATH_MODE_PACKAGE=BOTH \
      -DCMAKE_CXX_FLAGS="-ffunction-sections -fdata-sections" \
      -DCMAKE_INSTALL_PREFIX="$HERE/ceres-android"
    "$SDKCMAKE/cmake" --build "$HERE/ceres-build-android" -j10 --target install
    ;;
  rerun-host)
    cmake -S "$HERE/rerun_cpp_sdk" -B "$HERE/rerun-build-host" -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$HERE/rerun-host" -DRERUN_DOWNLOAD_AND_BUILD_ARROW=ON
    cmake --build "$HERE/rerun-build-host" -j10 --target install
    ;;
  *) echo "usage: $0 ceres-host|ceres-android|rerun-host"; exit 2;;
esac
