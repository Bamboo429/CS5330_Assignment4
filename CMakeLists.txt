
cmake_minimum_required(VERSION 3.21)
project(Calibration_and_Augmented_Reality)

find_package(OpenCV)
include_directories(${/usr/local/include/})

set(CMAKE_CXX_STANDARD 14)

add_executable(Calibration_and_Augmented_Reality camera_calibration.cpp aruco.cpp txt_util.cpp txt_util.h harris_corner.cpp object_projection.cpp)
target_link_libraries(Calibration_and_Augmented_Reality ${OpenCV_LIBS})
