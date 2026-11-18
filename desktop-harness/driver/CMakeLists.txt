cmake_minimum_required(VERSION 3.16)
project(beacon_euroc_driver CXX)

# Builds the EuRoC replay driver against the OpenVINS ROS-free library that setup.sh installs into
# OPENVINS_PREFIX (headers in include/open_vins, libov_msckf_lib in lib).

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif ()

set(OPENVINS_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/../build/install" CACHE PATH "OpenVINS install prefix")

find_package(Eigen3 REQUIRED)
# calib3d: CamRadtan/CamEqui (header-inline in ov_core) call cv::undistortPoints.
find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs calib3d)
# VioManager.h includes boost/filesystem.hpp and boost/date_time; headers only for us.
find_package(Boost REQUIRED CONFIG COMPONENTS filesystem)

find_library(OV_MSCKF_LIB ov_msckf_lib PATHS "${OPENVINS_PREFIX}/lib" NO_DEFAULT_PATH REQUIRED)

add_executable(euroc_driver euroc_driver.cpp)
target_include_directories(euroc_driver PRIVATE "${OPENVINS_PREFIX}/include/open_vins" ${OpenCV_INCLUDE_DIRS})
# Must match how the library was compiled (ROS off, ArUco off).
target_compile_definitions(euroc_driver PRIVATE ROS_AVAILABLE=0 ENABLE_ARUCO_TAGS=0)
target_link_libraries(euroc_driver PRIVATE "${OV_MSCKF_LIB}" Eigen3::Eigen ${OpenCV_LIBS} Boost::filesystem)
set_target_properties(euroc_driver PROPERTIES BUILD_RPATH "${OPENVINS_PREFIX}/lib")
