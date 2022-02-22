cmake_minimum_required(VERSION 2.8.3)
project(LiDAR2Camera)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set(3RDPARTY_DIR ${CMAKE_SOURCE_DIR}/3rdparty)
add_subdirectory(3rdparty)

include_directories(${3RDPARTY_DIR}/eigen3)
include_directories(${3RDPARTY_DIR}/ceres/include)

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} ceres libjsoncpp.a ${OpenCV_LIBS})

add_executable(lidar2camera src/lidar2camera.cpp)
target_link_libraries(lidar2camera ${PROJECT_NAME})
target_link_libraries(lidar2camera ceres)