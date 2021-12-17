cmake_minimum_required(VERSION 2.8.3)
project(CornerDetect)

set(CMAKE_BUILD_TYPE "Debug") # Release
set(CMAKE_CXX_FLAGS "-std=gnu++14 -g -Wall")

set(3RDPARTY_DIR ${CMAKE_SOURCE_DIR}/3rdparty)
add_subdirectory(3rdparty)

include_directories(${3RDPARTY_DIR}/eigen3)
include_directories(${3RDPARTY_DIR}/ceres/include)

find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

find_package(Boost REQUIRED system)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE UTILS src/utils/*.cpp)
add_library(utils STATIC ${UTILS})

file(GLOB_RECURSE PARSER_PATH src/*.cpp src/*.cc)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} ceres libjsoncpp.a ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY} ${PCL_LIBRARIES})

add_executable(run_board_detect src/run_board_detect.cpp)
target_link_libraries(run_board_detect ${PROJECT_NAME})
target_link_libraries(run_board_detect utils)
target_link_libraries(run_board_detect ceres)

add_executable(run_lidar_detect src/run_lidar_detect.cpp)
target_link_libraries(run_lidar_detect ${PROJECT_NAME})
target_link_libraries(run_lidar_detect utils)
target_link_libraries(run_lidar_detect ceres)

add_executable(run_homo_vp_calib src/run_homo_vp_calib.cpp) 
target_link_libraries(run_homo_vp_calib ${PROJECT_NAME})
target_link_libraries(run_homo_vp_calib utils)
target_link_libraries(run_homo_vp_calib ceres)

add_executable(run_extrinsic_calib src/run_extrinsic_calib.cpp)
target_link_libraries(run_extrinsic_calib ${PROJECT_NAME})
target_link_libraries(run_extrinsic_calib utils)
target_link_libraries(run_extrinsic_calib ceres)
