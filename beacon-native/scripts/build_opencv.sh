#!/bin/bash
# Builds a minimal static OpenCV 4.12.0 for the host (macOS arm64) and Android arm64-v8a.
# Usage: build_opencv.sh host|android
set -euo pipefail
HERE="$(cd "$(dirname "$0")/../third_party" && pwd)"
SRC="$HERE/opencv"
COMMON=(
  -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=OFF
  -DBUILD_TESTS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_DOCS=OFF
  -DBUILD_opencv_apps=OFF -DBUILD_JAVA=OFF -DBUILD_opencv_python3=OFF -DBUILD_opencv_python2=OFF
  -DBUILD_ANDROID_PROJECTS=OFF -DBUILD_ANDROID_EXAMPLES=OFF -DINSTALL_ANDROID_EXAMPLES=OFF
  -DWITH_IPP=OFF -DWITH_ITT=OFF -DWITH_OPENCL=OFF -DWITH_OPENCLAMDFFT=OFF -DWITH_OPENCLAMDBLAS=OFF
  -DWITH_EIGEN=OFF -DWITH_LAPACK=OFF -DWITH_PROTOBUF=OFF -DWITH_ADE=OFF -DWITH_KLEIDICV=OFF
  -DWITH_FFMPEG=OFF -DWITH_GSTREAMER=OFF -DWITH_AVFOUNDATION=OFF -DWITH_TIFF=OFF -DWITH_WEBP=OFF
  -DWITH_OPENJPEG=OFF -DWITH_JASPER=OFF -DWITH_OPENEXR=OFF -DWITH_IMGCODEC_HDR=OFF
  -DWITH_IMGCODEC_SUNRASTER=OFF -DWITH_IMGCODEC_PXM=OFF -DWITH_IMGCODEC_PFM=OFF -DWITH_AVIF=OFF
  -DWITH_JPEGXL=OFF -DWITH_SPNG=OFF -DWITH_OPENVX=OFF -DWITH_VTK=OFF -DWITH_QT=OFF -DWITH_GTK=OFF
  -DOPENCV_GENERATE_PKGCONFIG=OFF -DCV_TRACE=OFF -DWITH_PTHREADS_PF=ON
)
case "$1" in
  host)
    cmake -S "$SRC" -B "$HERE/opencv-build-host" "${COMMON[@]}" \
      -DCMAKE_INSTALL_PREFIX="$HERE/opencv-host" \
      -DBUILD_LIST=core,imgproc,features2d,flann,calib3d,imgcodecs,highgui \
      -DBUILD_ZLIB=ON -DBUILD_PNG=ON -DBUILD_JPEG=ON -DWITH_PNG=ON -DWITH_JPEG=ON
    cmake --build "$HERE/opencv-build-host" -j10 --target install
    ;;
  android)
    NDK="$HOME/Library/Android/sdk/ndk/27.0.12077973"
    CMAKE="$HOME/Library/Android/sdk/cmake/3.22.1/bin/cmake"
    "$CMAKE" -S "$SRC" -B "$HERE/opencv-build-android" "${COMMON[@]}" -G Ninja \
      -DCMAKE_MAKE_PROGRAM="$HOME/Library/Android/sdk/cmake/3.22.1/bin/ninja" \
      -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
      -DANDROID_ABI=arm64-v8a -DANDROID_PLATFORM=android-24 -DANDROID_STL=c++_static \
      -DCMAKE_INSTALL_PREFIX="$HERE/opencv-android" \
      -DBUILD_LIST=core,imgproc,features2d,flann,calib3d \
      -DWITH_PNG=OFF -DWITH_JPEG=OFF -DBUILD_ZLIB=ON -DWITH_CAROTENE=ON \
      -DCMAKE_C_FLAGS="-ffunction-sections -fdata-sections" \
      -DCMAKE_CXX_FLAGS="-ffunction-sections -fdata-sections"
    "$CMAKE" --build "$HERE/opencv-build-android" -j10 --target install
    ;;
  *) echo "usage: $0 host|android"; exit 2;;
esac
