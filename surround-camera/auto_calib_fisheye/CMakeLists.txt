cmake_minimum_required(VERSION 2.8)
project(online_Calibration)
set(CMAKE_CXX_FLAGS "-std=c++17 -g -Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")
SET(CMAKE_BUILD_TYPE "Release")

include_directories("/usr/include/eigen3")
find_package(OpenCV REQUIRED )

include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib)
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(run_AVM_Calibration src/calibration.cpp)
target_link_libraries(run_AVM_Calibration ${PROJECT_NAME})

