cmake_minimum_required(VERSION 3.5)
project(radar2carcenter)
message(STATUS "${BoldGreen}[${PROJECT_NAME}]${ColourReset} Building....")

string(REGEX MATCH "/.*senseauto" SENSEAUTO_DIR ${CMAKE_CURRENT_SOURCE_DIR})
list(APPEND CMAKE_MODULE_PATH ${SENSEAUTO_DIR}/system/cmake)

set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")

set(EIGEN_ROOT "${PROJECT_SOURCE_DIR}/eigen3/")
find_package(Boost REQUIRED system)
# find_package(OpenCV REQUIRED)
# link_directories(${OpenCV_LIBRARY_DIRS})
# include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} libjsoncpp.a ${Boost_SYSTEM_LIBRARY})

add_executable(run_radar2car apps/run_radar2car.cpp)
target_link_libraries(run_radar2car ${PROJECT_NAME})