# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

project(SPLINTER CXX)
cmake_minimum_required(VERSION 2.8)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

# Read the SPLINTER version from version file
file(STRINGS "version" VERSION)

# Default configuration values
set(DEFAULT_BUILD_TYPE "Release")
set(DEFAULT_ARCH "x86-64")
set(DEFAULT_HEADER_INSTALL_DIRECTORY "include")
set(DEFAULT_LIBRARY_INSTALL_DIRECTORY "lib")
set(DEFAULT_EIGEN_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Eigen)

# Detect platform
string(COMPARE EQUAL "Linux"   ${CMAKE_SYSTEM_NAME} LINUX)
string(COMPARE EQUAL "Windows" ${CMAKE_SYSTEM_NAME} WINDOWS)
string(COMPARE EQUAL "Darwin"  ${CMAKE_SYSTEM_NAME} OS_X)
if(LINUX)
    set(OS_STRING "linux")
elseif(WINDOWS)
    set(OS_STRING "windows")
elseif(OS_X)
    set(OS_STRING "osx")
else()
    set(OS_STRING "Unknown")
endif()

# Detect compiler
if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    set(CLANG 1)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "AppleClang")
    set(CLANG 1)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(GCC 1)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
    set(INTEL 1)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
    set(MSVC 1)
endif()

# Set build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE ${DEFAULT_BUILD_TYPE})
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

# Set architecture to default if not set
if(NOT ARCH)
    set(ARCH ${DEFAULT_ARCH})
endif()

# Set bitness
if(${ARCH} STREQUAL "x86-64")
	set(BITNESS "64")
elseif(${ARCH} STREQUAL "x86")
	set(BITNESS "32")
else()
    set(BITNESS "32")
    set(ARCH "x86")
    message("Unknown architecture selected, defaulting to x86")
endif()

if(WINDOWS AND GCC)
	# MinGW (GCC on Windows) version 4.8.1 has a bug where _hypot is not found
	# when using any level of optimization. Defining __NO_INLINE__ fixes the bug.
	# The bug probably exists on other versions of MinGW as well, but that has not been verified by us.
    if(CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL "4.8.1")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -D__NO_INLINE__")
    endif()
    if(CMAKE_CXX_COMPILER_VERSION VERSION_EQUAL "4.9.2")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O2")
		# From the GCC man pages:
		# If you use multiple -O options, with or without level numbers, the last such option is the one that is effective.
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -finline-functions")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -funswitch-loops")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fpredictive-commoning")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fgcse-after-reload")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ftree-loop-vectorize")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ftree-loop-distribute-patterns")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ftree-slp-vectorize")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fvect-cost-model")
		set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -ftree-partial-pre")
		# -fipa-cp-clone causes a SegFault in Eigen when enabled with MinGW i686 4.9.2 with dwarf exception model
		#set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -fipa-cp-clone")
	endif()
	
	# To avoid an external dependency (libgcc_s_seh11.dll or libgcc_s_dw2-1.dll)
	# we statically link the libraries required by MinGW runtimes
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -static")
endif()

if(MSVC)
    # User cannot specify bitness with MSVC, so set it to whatever the generator is.
    string(TOLOWER ${CMAKE_GENERATOR} GENERATOR)
    if(GENERATOR MATCHES ".*win64.*")
        set(BITNESS "64")
    else()
        set(BITNESS "32")
    endif()
	set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Ox")

elseif(GCC OR CLANG)
    # Treat warning return-type as error to avoid undefined behaviour
    # when a non-void function does not return a value.
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m${BITNESS} -std=c++11 -Werror=return-type")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Wno-long-long")

elseif(INTEL)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -m${BITNESS} -std=c++11")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
endif()

if(NOT EIGEN_DIRECTORY)
	set(EIGEN_DIRECTORY ${DEFAULT_EIGEN_DIRECTORY})
endif()
# CACHE STRING = Display the option with help text in CMakeCache.txt
set(EIGEN_DIRECTORY ${EIGEN_DIRECTORY} CACHE STRING "Directory where the Eigen library is located.")

if(NOT HEADER_INSTALL_DIRECTORY)
    set(HEADER_INSTALL_DIRECTORY ${DEFAULT_HEADER_INSTALL_DIRECTORY})
endif()
# CACHE STRING = Display the option with help text in CMakeCache.txt
set(HEADER_INSTALL_DIRECTORY ${HEADER_INSTALL_DIRECTORY} CACHE STRING "Absolute path, or, if relative, relative to CMAKE_INSTALL_PREFIX to install the header files.")

if(NOT LIBRARY_INSTALL_DIRECTORY)
    set(LIBRARY_INSTALL_DIRECTORY ${DEFAULT_LIBRARY_INSTALL_DIRECTORY})
endif()
# CACHE STRING = Display the option with help text in CMakeCache.txt
set(LIBRARY_INSTALL_DIRECTORY ${LIBRARY_INSTALL_DIRECTORY} CACHE STRING "Absolute path, or, if relative, relative to CMAKE_INSTALL_PREFIX to install the library file.")

# Header directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${EIGEN_DIRECTORY})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/test)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/Catch)

set(CINTERFACE_SRC_LIST
    include/cinterface/cinterface.h
    include/cinterface/utilities.h
    src/cinterface/bspline.cpp
    src/cinterface/bsplinebuilder.cpp
    src/cinterface/cinterface.cpp
    src/cinterface/datatable.cpp
    src/cinterface/utilities.cpp
)
# These are the sources we need for compilation of the library
set(SRC_LIST
    ${CINTERFACE_SRC_LIST}
    include/bspline.h
    include/bsplinebasis.h
    include/bsplinebasis1d.h
    include/knots.h
    include/bsplinebuilder.h
    include/datapoint.h
    include/datatable.h
    include/function.h
    include/definitions.h
    include/linearsolvers.h
    include/mykroneckerproduct.h
    include/serializer.h
    include/utilities.h
    include/saveable.h
    src/bspline.cpp
    src/bsplinebasis.cpp
    src/bsplinebasis1d.cpp
    src/knots.cpp
    src/bsplinebuilder.cpp
    src/datapoint.cpp
    src/datatable.cpp
    src/function.cpp
    src/mykroneckerproduct.cpp
    src/serializer.cpp
    src/utilities.cpp
)
set(TEST_SRC_LIST
    ${SRC_LIST}
    test/main.cpp
    test/approximation/bspline.cpp
    test/approximation/pspline.cpp
    test/general/bspline.cpp
    test/general/datatable.cpp
    test/general/utilities.cpp
    test/serialization/datatable.cpp
    test/serialization/bspline.cpp
    test/operatoroverloads.h
    test/operatoroverloads.cpp
    test/testfunction.h
    test/testfunction.cpp
    test/testfunctions.h
    test/testfunctions.cpp
    test/testingutilities.h
    test/testingutilities.cpp
    test/bsplinetestingutilities.h
    test/bsplinetestingutilities.cpp
    test/serialization/eigentypes.cpp
    test/unit/bsplinebasis1d.cpp
    test/unit/knots.cpp)

set(SHARED_LIBRARY ${PROJECT_NAME_LOWER}-${VERSION})
set(STATIC_LIBRARY ${PROJECT_NAME_LOWER}-static-${VERSION})

set(SHARED_LIBRARY_TEST "${PROJECT_NAME_LOWER}-shared-test")
set(STATIC_LIBRARY_TEST "${PROJECT_NAME_LOWER}-static-test")
set(TEST "${PROJECT_NAME_LOWER}-test")

# Add output library: add_library(libname [SHARED | STATIC] sourcelist)
add_library(${SHARED_LIBRARY} SHARED ${SRC_LIST})
add_library(${STATIC_LIBRARY} STATIC ${SRC_LIST})

# Testing executable
add_executable(${TEST} ${TEST_SRC_LIST})
target_link_libraries(${TEST} ${STATIC_LIBRARY})

# Make the directory structure expected by the MatLab interface
# License file
install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-matlab/
)
# Matlab files (.m)
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/matlab
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-matlab
)
# Version file (used for deducing the name of the binary when loading it)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/splinter-matlab/version ${VERSION})
install(
        FILES include/cinterface/cinterface.h
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-matlab/include
)
install(
        TARGETS ${SHARED_LIBRARY}
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-matlab/lib/${OS_STRING}/${ARCH}
)

# Make the directory structure expected by the Python interface
install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/LICENSE
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-python/
)
# Python files (.py)
install(
        DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-python
        PATTERN "*.pyc" EXCLUDE
        PATTERN "__pycache__*" EXCLUDE
)
# Version file (used for deducing the name of the binary when loading it)
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/splinter-python/version ${VERSION})
install(
        FILES include/cinterface/cinterface.h
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-python/include
)
install(
        TARGETS ${SHARED_LIBRARY}
        DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/splinter-python/lib/${OS_STRING}/${ARCH}
)

# Install the header files (including Eigen) to the header directory
install(
    DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    ${EIGEN_DIRECTORY}/Eigen
    ${EIGEN_DIRECTORY}/unsupported
    DESTINATION ${HEADER_INSTALL_DIRECTORY}/SPLINTER
)
# Install the shared library file
install(
    TARGETS ${SHARED_LIBRARY}
    DESTINATION ${LIBRARY_INSTALL_DIRECTORY}
)
# Install the static library file
install(
    TARGETS ${STATIC_LIBRARY}
    DESTINATION ${LIBRARY_INSTALL_DIRECTORY}
)

# Doxygen documentation
find_package(Doxygen)

if(DOXYGEN_FOUND)
  configure_file(
    ${PROJECT_SOURCE_DIR}/docs/Doxyfile.in
    ${PROJECT_BINARY_DIR}/Doxyfile
    @ONLY)

  add_custom_target(
    doc
#      ALL
    ${DOXYGEN_EXECUTABLE}
    ${PROJECT_BINARY_DIR}/Doxyfile
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM)
    
endif()

if(CMAKE_BUILD_TYPE STREQUAL DEBUG)
    message("Detected compiler and platform:")
	message("Clang:   ${CLANG}")
	message("GCC:     ${GCC}")
	message("Intel:   ${INTEL}")
	message("MSVC:    ${MSVC}")
	message("Linux:   ${LINUX}")
	message("Windows: ${WINDOWS}")
	message("OS X:    ${OS_X}")
endif()

message("Configuring ${PROJECT_NAME} version ${VERSION} in ${CMAKE_BUILD_TYPE} mode for ${ARCH} (${BITNESS} bit)")
message("Compiler flags: ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}}")
