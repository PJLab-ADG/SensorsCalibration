cmake_minimum_required(VERSION 2.8)
project(pose_sensor2car)

# compile option
SET(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")  

# compile output path 
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# Get boost
find_package(Boost REQUIRED filesystem)
include_directories(${BOOST_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

# Get Eigen
find_package(Eigen3 REQUIRED)
include_directories("/usr/include/eigen3")

# Get PCL
find_package(PCL 1.9 REQUIRED 
             PATHS "/usr/local/pcl-1.9/share/pcl-1.9/" NO_DEFAULT_PATH)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Get OpenCV
find_package(OpenCV REQUIRED 
             PATHS "/usr/local/opencv-2.4.13/share/OpenCV/" NO_DEFAULT_PATH)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_INSTALL_PATH}/lib) 

# Get splinter
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/splinter-3.0/include)
link_directories(${PROJECT_SOURCE_DIR}/lib)

# Get matplotlib
include_directories("/usr/include/python2.7")
include_directories(${PROJECT_SOURCE_DIR}/3rdparty/matplotlib-cpp)

# other dependence
include_directories(${PROJECT_SOURCE_DIR}/include)

SET(DEP_LIBS
    ${Boost_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OpenCV_LIBS}
    splinter-static-3-0
    python2.7
)

file(GLOB_RECURSE SRC_LIST src/*.cpp)

add_library(${PROJECT_NAME} STATIC ${SRC_LIST})

TARGET_LINK_LIBRARIES(${PROJECT_NAME} ${DEP_LIBS})

ADD_EXECUTABLE(run_imu2car ${PROJECT_SOURCE_DIR}/src/run_imu2car.cpp)
TARGET_LINK_LIBRARIES(run_imu2car ${PROJECT_NAME})
