# Built by Gradle (TeamCode/build.gradle, externalNativeBuild) for arm64-v8a only.
# Produces libbeacon.so (the native core, built from beacon-native/ as a subproject) and
# libbeacon_jni.so (this directory: the JNI shim, which links libbeacon.so).
# beacon-native needs its prebuilt Android dependencies in beacon-native/third_party
# (scripts/build_opencv.sh and scripts/build_ceres_rerun.sh; versions in VERSIONS.md).
cmake_minimum_required(VERSION 3.22)
project(beacon_jni CXX C)

set(BEACON_NATIVE ${CMAKE_CURRENT_SOURCE_DIR}/../../../../beacon-native)
get_filename_component(BEACON_NATIVE ${BEACON_NATIVE} ABSOLUTE)
if(NOT EXISTS ${BEACON_NATIVE}/CMakeLists.txt)
  message(FATAL_ERROR "beacon-native not found at ${BEACON_NATIVE}")
endif()
if(NOT ANDROID_ABI STREQUAL "arm64-v8a")
  message(FATAL_ERROR "beacon-native's Android dependencies exist for arm64-v8a only, not ${ANDROID_ABI}")
endif()

add_subdirectory(${BEACON_NATIVE} ${CMAKE_CURRENT_BINARY_DIR}/beacon-native)

# Workaround (requested upstream): beacon-native/src/features.h shadows bionic's system
# <features.h>, which libc++ includes, when beacon-native/src is a -I directory. beacon-native
# includes its own headers with quotes only, so it is put on the quote-only path instead.
foreach(tgt beacon_core beacon)
  foreach(prop INCLUDE_DIRECTORIES INTERFACE_INCLUDE_DIRECTORIES)
    get_target_property(dirs ${tgt} ${prop})
    if(dirs)
      list(FILTER dirs EXCLUDE REGEX "beacon-native/src/?$")
      set_property(TARGET ${tgt} PROPERTY ${prop} ${dirs})
    endif()
  endforeach()
  target_compile_options(${tgt} PRIVATE -iquote ${BEACON_NATIVE}/src)
endforeach()

add_library(beacon_jni SHARED beacon_jni.cpp)
target_include_directories(beacon_jni PRIVATE ${BEACON_NATIVE}/include)
target_compile_options(beacon_jni PRIVATE -Wall -Wextra)
target_link_libraries(beacon_jni PRIVATE beacon)
# 16 KB page alignment (Android 15+ devices); harmless on the Control Hub.
target_link_options(beacon_jni PRIVATE -Wl,-z,max-page-size=16384)
