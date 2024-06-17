cmake_minimum_required(VERSION 2.8.3)
project(lidar2camera)
set(CMAKE_CXX_FLAGS "-g3 -O -Wall")
set(CMAKE_CXX_STANDARD 17)

find_package(Pangolin 0.6 REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})
link_directories(${Pangolin_LIBRARY_DIRS})

find_package(PCL 1.9.1 EXACT REQUIRED)
find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)
find_package(jsoncpp REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)

link_directories(${OpenCV_LIBRARY_DIRS})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(run_lidar2camera src/run_lidar2camera.cpp)
target_link_libraries(run_lidar2camera jsoncpp_lib ${PCL_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})
