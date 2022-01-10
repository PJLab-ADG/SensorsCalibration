cmake_minimum_required(VERSION 2.8.3)
project(LidarToImu)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
## Get Pangolin
find_package(Pangolin 0.4 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

find_package(PCL REQUIRED)
find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_subdirectory(ceres)
include_directories(${PROJECT_SOURCE_DIR}/eigen3)
include_directories(${PROJECT_SOURCE_DIR}/ceres/include)

include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PCL_INCLUDE_DIRS})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} libjsoncpp.a ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ceres)

add_executable(run_lidar2imu src/run_lidar2imu.cpp)
target_link_libraries(run_lidar2imu ${PROJECT_NAME})
target_link_libraries(run_lidar2imu ${PCL_LIBRARIES})
