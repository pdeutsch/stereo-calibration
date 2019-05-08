cmake_minimum_required(VERSION 2.8.11)
project(CALIB_CAM)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(OpenCV REQUIRED)
include_directories($(OpenCV_INCLUDE_DIRS))

add_executable(calibrate calib_intrinsic.cpp popt_pp.h)
target_link_libraries(calibrate ${OpenCV_LIBS} "-lpopt")

add_executable(read read_images.cpp)
target_link_libraries(read ${OpenCV_LIBS} "-lpopt")

add_executable(calibrate_stereo calib_stereo.cpp)
target_link_libraries(calibrate_stereo ${OpenCV_LIBS} "-lpopt")

add_executable(undistort_rectify undistort_rectify.cpp)
target_link_libraries(undistort_rectify ${OpenCV_LIBS} "-lpopt")

add_executable(show_both show_both.cpp)
target_link_libraries(show_both ${OpenCV_LIBS} "-lpopt")

add_executable(show_quasi show_quasi.cpp)
target_link_libraries(show_quasi ${OpenCV_LIBS} "-lpopt")

add_executable(stereo_live stereo_live.cpp)
target_link_libraries(stereo_live ${OpenCV_LIBS} "-lpopt")
