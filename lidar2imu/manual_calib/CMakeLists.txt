cmake_minimum_required(VERSION 2.8.3)
project(LidarToImu)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} libjsoncpp.a ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})

add_executable(run_lidar2imu src/run_lidar2imu.cpp)
target_link_libraries(run_lidar2imu ${PROJECT_NAME})
target_link_libraries(run_lidar2imu ${PCL_LIBRARIES})
target_link_libraries(run_lidar2imu ${Pangolin_LIBRARIES})

add_executable(pcd_view_stitcher src/pcd_view_stitcher.cpp)
target_link_libraries(pcd_view_stitcher ${PROJECT_NAME})
target_link_libraries(pcd_view_stitcher ${PCL_LIBRARIES})
target_link_libraries(pcd_view_stitcher ${Pangolin_LIBRARIES})
