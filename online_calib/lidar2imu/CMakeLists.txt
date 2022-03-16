cmake_minimum_required(VERSION 3.5)
project(lidar2carcenter)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


# set(EIGEN_ROOT "${PROJECT_SOURCE_DIR}/3rdparty/eigen3/")
# include_directories(${PROJECT_SOURCE_DIR}/3rdparty/eigen3)
# add_subdirectory(${PROJECT_SOURCE_DIR}/3rdparty/ceres)
# include_directories(${PROJECT_SOURCE_DIR}/3rdparty/ceres/include)

# set(EIGEN_ROOT "${PROJECT_SOURCE_DIR}/eigen3/")
include_directories(${PROJECT_SOURCE_DIR}/eigen3)
add_subdirectory(eigen3)
add_subdirectory(ceres)
include_directories(${PROJECT_SOURCE_DIR}/ceres/include)

## Get OpenCV
# find_package(OpenCV REQUIRED 
#              PATHS "/usr/local/opencv-2.4.13/share/OpenCV/" NO_DEFAULT_PATH)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib)


## Get boost
find_package(Boost REQUIRED filesystem system)
include_directories(${BOOST_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} ceres libjsoncpp.a ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})

add_executable(run_lidar2car apps/run_lidar2car.cpp)
target_link_libraries(run_lidar2car ceres)
target_link_libraries(run_lidar2car ${PROJECT_NAME})
target_link_libraries(run_lidar2car ${PCL_LIBRARIES})
