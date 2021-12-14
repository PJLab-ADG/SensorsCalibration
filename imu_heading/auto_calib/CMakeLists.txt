cmake_minimum_required(VERSION 3.5)
project(rec_parser C CXX)
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
find_package(PCL REQUIRED)
find_package(Boost REQUIRED system)

include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} libjsoncpp.a ${Boost_SYSTEM_LIBRARY})

add_executable(run_imu_heading src/run_imu_heading.cpp )
target_link_libraries(run_imu_heading ${PROJECT_NAME})
