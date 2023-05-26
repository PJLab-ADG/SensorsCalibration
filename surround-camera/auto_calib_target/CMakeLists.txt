cmake_minimum_required(VERSION 2.8)
project(AVM_Calibration)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED 
             PATHS "/usr/local/opencv-2.4.13/share/OpenCV/" NO_DEFAULT_PATH)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(run_AVM_Calibration src/calibration.cpp)
target_link_libraries(run_AVM_Calibration ${OpenCV_LIBS})
add_executable(run_stitching src/stitching.cpp)
target_link_libraries(run_stitching ${OpenCV_LIBS})