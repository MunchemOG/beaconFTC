#!/bin/bash
# Cross-builds libbeacon.so for Android arm64-v8a with the NDK toolchain file (no Gradle).
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
NDK="$HOME/Library/Android/sdk/ndk/27.0.12077973"
SDKCMAKE="$HOME/Library/Android/sdk/cmake/3.22.1/bin"
"$SDKCMAKE/cmake" -S "$ROOT" -B "$ROOT/build-android" -G Ninja \
  -DCMAKE_MAKE_PROGRAM="$SDKCMAKE/ninja" \
  -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
  -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-24 -DANDROID_STL=c++_static \
  -DCMAKE_BUILD_TYPE=Release
"$SDKCMAKE/cmake" --build "$ROOT/build-android" --target beacon -j10
ls -l "$ROOT/build-android/libbeacon.so"
