# Beacon native core. Host (macOS arm64): library + tests + bench tools.
# Android (NDK toolchain, arm64-v8a): libbeacon.so only.
# Dependencies are built once by scripts/build_opencv.sh and scripts/build_ceres_rerun.sh into
# third_party/ (pinned versions in VERSIONS.md).
cmake_minimum_required(VERSION 3.22)
project(beacon_native CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(BEACON_TP ${CMAKE_CURRENT_SOURCE_DIR}/third_party)
option(BEACON_BUILD_BENCH "Build tests and bench tools (host only)" ON)
if(ANDROID)
  set(BEACON_BUILD_BENCH OFF)
endif()

# --- dependencies -----------------------------------------------------------------------------
if(ANDROID)
  set(OpenCV_DIR ${BEACON_TP}/opencv-android/sdk/native/jni CACHE PATH "")
  set(Ceres_DIR ${BEACON_TP}/ceres-android/lib/cmake/Ceres CACHE PATH "")
else()
  set(OpenCV_DIR ${BEACON_TP}/opencv-host/lib/cmake/opencv4 CACHE PATH "")
  set(Ceres_DIR ${BEACON_TP}/ceres-host/lib/cmake/Ceres CACHE PATH "")
endif()
set(Eigen3_DIR ${BEACON_TP}/eigen-install/share/eigen3/cmake CACHE PATH "")
list(APPEND CMAKE_FIND_ROOT_PATH ${BEACON_TP})
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d NO_CMAKE_FIND_ROOT_PATH)
find_package(Eigen3 3.4 REQUIRED NO_MODULE NO_CMAKE_FIND_ROOT_PATH)
find_package(Ceres REQUIRED NO_CMAKE_FIND_ROOT_PATH)

# --- core library ------------------------------------------------------------------------------
add_library(beacon_core STATIC
  src/api.cpp
  src/edges.cpp
  src/explore_engine.cpp
  src/orb_features.cpp
  src/field_engine.cpp
  src/field_model.cpp
  src/gate.cpp
  src/geometry.cpp
  src/init_mono.cpp
  src/loop.cpp
  src/map.cpp
  src/matcher.cpp
  src/optimizer.cpp
  src/params.cpp
  src/predict.cpp
  src/vocab.cpp
)
# Only include/ is public: src/ stays private so its headers never shadow system headers
# for code that links the core (the JNI shim sees just beacon/beacon.h).
target_include_directories(beacon_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src ${BEACON_TP}/nlohmann/include)
target_link_libraries(beacon_core PUBLIC opencv_core opencv_imgproc opencv_features2d opencv_calib3d
  Eigen3::Eigen Ceres::ceres)
if(NOT MSVC)
  target_compile_options(beacon_core PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-missing-field-initializers
    -ffunction-sections -fdata-sections)
endif()
find_package(Threads REQUIRED)
target_link_libraries(beacon_core PUBLIC Threads::Threads)

# --- shared library (the thing the RC app loads) ----------------------------------------------
add_library(beacon SHARED src/api.cpp)  # api.cpp again so the C API is exported
target_link_libraries(beacon PRIVATE beacon_core)
target_include_directories(beacon PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
if(ANDROID)
  target_link_libraries(beacon PRIVATE log)
  target_link_options(beacon PRIVATE -Wl,--gc-sections -Wl,--exclude-libs,ALL -s
    -Wl,--version-script=${CMAKE_CURRENT_SOURCE_DIR}/src/beacon.map)
  # 16 KB page alignment (Android 15+ devices); harmless on the Control Hub.
  target_link_options(beacon PRIVATE -Wl,-z,max-page-size=16384)
elseif(APPLE)
  target_link_options(beacon PRIVATE -Wl,-dead_strip)
endif()

# --- tests and bench (host only) ---------------------------------------------------------------
if(BEACON_BUILD_BENCH)
  enable_testing()
  find_package(OpenCV REQUIRED COMPONENTS imgcodecs highgui NO_CMAKE_FIND_ROOT_PATH)
  set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
  add_subdirectory(${BEACON_TP}/googletest ${CMAKE_BINARY_DIR}/googletest EXCLUDE_FROM_ALL)

  set(rerun_sdk_DIR ${BEACON_TP}/rerun-host/lib/cmake/rerun_sdk CACHE PATH "")
  find_package(rerun_sdk REQUIRED NO_CMAKE_FIND_ROOT_PATH)

  add_library(beacon_bench STATIC
    bench/dataset.cpp
    bench/render.cpp
    bench/eval.cpp
    bench/mjpeg.cpp
    bench/replay.cpp
    bench/viewer_rerun.cpp
  )
  target_compile_definitions(beacon_bench PUBLIC BEACON_SOURCE_DIR_BENCH="${CMAKE_CURRENT_SOURCE_DIR}")
  target_include_directories(beacon_bench PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/bench
    ${CMAKE_CURRENT_SOURCE_DIR}/src PRIVATE ${BEACON_TP}/nlohmann/include)
  target_link_libraries(beacon_bench PUBLIC beacon_core opencv_imgcodecs opencv_highgui rerun_sdk)

  foreach(tool beacon_replay beacon_live beacon_make_testdata beacon_vocab_train beacon_overlay)
    add_executable(${tool} bench/${tool}.cpp)
    target_link_libraries(${tool} PRIVATE beacon_bench)
    target_include_directories(${tool} PRIVATE ${BEACON_TP}/nlohmann/include)
  endforeach()

  file(GLOB BEACON_TESTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_*.cpp)
  add_executable(beacon_tests ${BEACON_TESTS})
  target_link_libraries(beacon_tests PRIVATE beacon_bench GTest::gtest_main)
  target_include_directories(beacon_tests PRIVATE ${BEACON_TP}/nlohmann/include)
  target_compile_definitions(beacon_tests PRIVATE
    BEACON_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
    BEACON_BINARY_DIR="${CMAKE_BINARY_DIR}"
    BEACON_REPO_DIR="${CMAKE_CURRENT_SOURCE_DIR}/..")
  include(GoogleTest)
  gtest_discover_tests(beacon_tests DISCOVERY_TIMEOUT 60 PROPERTIES TIMEOUT 1800)
endif()
