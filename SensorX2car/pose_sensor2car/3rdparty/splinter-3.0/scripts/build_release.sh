#!/bin/bash
# This file is part of the SPLINTER library.
# Copyright (C) 2012 Bjarne Grimstad (bjarne.grimstad@gmail.com).
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.

# ====================== Configuration ======================
# Note how Windows paths has to be written: C:/Program Files/ => /C/Program Files/

# Binary directory of CMake (the directory containing cmake.exe)
CMAKE_PATH="/C/Program Files (x86)/CMake/bin"

# MinGW config (comment out if you don't have the respective version installed)
MINGW_32_BIT="/C/mingw-w64/i686-4.9.2-posix-dwarf-rt_v4-rev3/mingw32/bin"
MINGW_64_BIT="/C/mingw-w64/x86_64-4.9.2-posix-seh-rt_v4-rev3/mingw64/bin"

# MSVC config
# Visual Studio 2012:
#MSBUILD_DIR="/C/Program Files (x86)/MSBuild/12.0/Bin"
#VCVARSALL_DIR="/C/Program Files (x86)/Microsoft Visual Studio 12.0/VC"
#MSVC_GENERATOR="Visual Studio 12 2013"

# Visual Studio 2015:
MSBUILD_DIR="/C/Program Files (x86)/MSBuild/14.0/Bin"
VCVARSALL_DIR="/C/Program Files (x86)/Microsoft Visual Studio 14.0/VC"
MSVC_GENERATOR="Visual Studio 14 2015"

# ====================== End configuration ==================





ROOT=$(pwd)
OS="unknown"
COMPILER="unknown"
NPROC="unknown"
CMAKE_CMD="cmake"

# Capture the command argument for use in help messages
COMMAND=$0

# Thanks to http://stackoverflow.com/questions/192249/how-do-i-parse-command-line-arguments-in-bash#14203146
# for this great command line argument parsing algorithm
# Use > 0 to consume one or more arguments per pass in the loop (e.g.
# some arguments don't have a corresponding value to go with it such
# as in the --default example).
while [[ $# > 0 ]]
do
key="$1"

case $key in
	-c|--cmake-binary-dir)
	CMAKE_PATH="$2"
	shift # past argument
	;;
	-vc|--vcvarsall-dir)
	VCVARSALL_DIR="$2"
	shift # past argument
	;;
	-mb|--msbuild-dir)
	MSBUILD_DIR="$2"
	shift # past argument
	;;
	*)
	# No preceding: path to SPLINTER (has no effect as of now)
	SPLINTER_DIR="$1"
	;;
esac
shift # past argument or value
done

PATH="$CMAKE_PATH:$PATH"

# Parent directory of this file (build_release.sh)
SPLINTER_DIR=$(cd `dirname "${BASH_SOURCE[0]}"` && pwd)/..
# Verify that CMakeLists.txt exists in $SPLINTER_DIR
if [ ! -f $SPLINTER_DIR/CMakeLists.txt ]; then
	echo "Error: Unable to locate CMakeLists.txt!"
	exit 1
fi

# Make sure SPLINTER_DIR is an absolute path
if [[ $SPLINTER_DIR != /* ]]; then
	SPLINTER_DIR="$(pwd)/$SPLINTER_DIR"
fi

BUILD_ROOT=$SPLINTER_DIR/build

# Read the SPLINTER version from version file
SPLINTER_VERSION=$(cat $SPLINTER_DIR/version)

# Check that we can find CMake
if [[ $(which cmake) == "" ]]; then
	echo "Error: Can't find CMake, make sure it is in your PATH environment variable"
	echo "and try again!"
	echo "If you don't want to add CMake to your PATH, you can specify the path to it with:"
	echo "$COMMAND -c /path/to/cmake/binary/directory"
	echo "Note that on Windows, the path \"C:/Program Files (x86)/CMake/bin\" has to be written as \"/C/Program Files (x86)/CMake/bin\""
	exit 1
fi

# Gets the id of the last commit to the repository at $SPLINTER_DIR
function get_commit_id {
	COMMIT_ID=$(git -C "$SPLINTER_DIR" log -n 1 --pretty=format:"%H")
}

function update_commit_id {
	get_commit_id
	echo $COMMIT_ID > "$BUILD_ROOT/$OS/$COMPILER/commit_id"
}

function update_compiler_version {
	echo $COMPILER_VERSION > "$BUILD_ROOT/$OS/$COMPILER/compiler_version"
}

function copy_header_files {
	cp -r $SPLINTER_DIR/include $BUILD_ROOT/$OS/$COMPILER/
	cp -r $SPLINTER_DIR/thirdparty/Eigen/Eigen $BUILD_ROOT/$OS/$COMPILER/include
	cp -r $SPLINTER_DIR/thirdparty/Eigen/unsupported $BUILD_ROOT/$OS/$COMPILER/include
}

function build_gcc_clang {
	ARCH=$1
	COMPILER=$2

	mkdir -p $BUILD_ROOT/$OS/$COMPILER/$ARCH
	mkdir -p $BUILD_ROOT/.build/$COMPILER/$ARCH
	cd $BUILD_ROOT/.build/$COMPILER/$ARCH
	
	rm CMakeCache.txt
	echo "Building SPLINTER for $ARCH with $COMPILER"
	"$CMAKE_CMD" "$SPLINTER_DIR" -DCMAKE_BUILD_TYPE=release -DARCH=$ARCH -G "Unix Makefiles" -DCMAKE_MAKE_PROGRAM="$MAKE_CMD"
	"$MAKE_CMD" -j$NPROC
}

function build_linux {
	echo "Building for Linux"
	OS=linux
	
	MAKE_CMD=$(which make)
	NPROC=$(nproc)
	
	GPP=$(which g++)
	if [[ $GPP != "" ]]; then
		export CXX=$GPP
		COMPILER=gcc
		COMPILER_VERSION=$($CXX -dumpversion)
		
		build_gcc_clang x86 $COMPILER
		cp libsplinter-$SPLINTER_VERSION.so libsplinter-static-$SPLINTER_VERSION.a "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
		# MatLab for Linux only exists as 64bit, so we don't need this
#		"$MAKE_CMD" install
#		cp -r splinter-matlab $BUILD_ROOT
#		cp -r splinter-python $BUILD_ROOT
		
		build_gcc_clang x86-64 $COMPILER
		cp libsplinter-$SPLINTER_VERSION.so libsplinter-static-$SPLINTER_VERSION.a "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
		
		# Have CMake create the interface directory structures, then copy them to the build root
		"$MAKE_CMD" install
		cp -r splinter-matlab $BUILD_ROOT
		cp -r splinter-python $BUILD_ROOT
		
		copy_header_files
		
		# Write down the commit id this was compiled from
		update_commit_id
		update_compiler_version
	fi
	
	CLANG=$(which clang++)
	if [[ $CLANG != "" ]]; then
		export CXX=$CLANG
		COMPILER=clang
		COMPILER_VERSION=$($CXX -dumpversion)
		
		build_gcc_clang x86 $COMPILER
		cp libsplinter-$SPLINTER_VERSION.so libsplinter-static-$SPLINTER_VERSION.a "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
		
		build_gcc_clang x86-64 $COMPILER
		cp libsplinter-$SPLINTER_VERSION.so libsplinter-static-$SPLINTER_VERSION.a "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
		
		# Have CMake create the interface directory structures, then copy them to the build root
		"$MAKE_CMD" install
		cp -r splinter-matlab $BUILD_ROOT
		cp -r splinter-python $BUILD_ROOT
		
		copy_header_files
		
		# Write down the commit id this was compiled from
		update_commit_id
		update_compiler_version
	fi
}

function build_msvc {
	ARCH=$1
	COMPILER=$2
	
	mkdir -p $BUILD_ROOT/$OS/$COMPILER/$ARCH
	mkdir -p $BUILD_ROOT/.build/$COMPILER/$ARCH
	cd $BUILD_ROOT/.build/$COMPILER/$ARCH
	
	# Need this so msbuild.exe can find the project file
#	export PATH="$ROOT/build/$COMPILER/$ARCH/:$PATH"
	rm CMakeCache.txt
	
	if [[ $ARCH == "x86" ]]; then
		cmd "/C vcvarsall.bat x86"
		GENERATOR=$MSVC_GENERATOR
	elif [[ $ARCH == "x86-64" ]]; then
		cmd "/C vcvarsall.bat x64"
		GENERATOR="$MSVC_GENERATOR Win64"
	else
		echo "Error: Unknown architecture given to build_msvc: $ARCH"
		exit 1
	fi
	
	"$CMAKE_CMD" "$SPLINTER_DIR" -DCMAKE_BUILD_TYPE=Release -DARCH=$ARCH -G "$GENERATOR"
	
	"$MSBUILD" ALL_BUILD.vcxproj -p:Configuration=Release -maxcpucount:$NPROC
	
	# Install
	mkdir -p "$BUILD_ROOT/splinter-matlab/lib/$OS/$ARCH/"
	mkdir -p "$BUILD_ROOT/splinter-python/lib/$OS/$ARCH/"
	mkdir -p "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
	
	cp "Release/splinter-$SPLINTER_VERSION.dll" "$BUILD_ROOT/splinter-matlab/lib/$OS/$ARCH/"
	cp "Release/splinter-$SPLINTER_VERSION.dll" "$BUILD_ROOT/splinter-python/lib/$OS/$ARCH/"
	cp "Release/splinter-$SPLINTER_VERSION.dll" "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
	cp "Release/splinter-static-$SPLINTER_VERSION.lib" "$BUILD_ROOT/$OS/$COMPILER/$ARCH"

	copy_header_files
}

function build_windows {
	echo "Building for Windows"
	OS=windows
	
	export PATH="$MSBUILD_DIR:$PATH"
	export PATH="$VCVARSALL_DIR:$PATH"
	
	# Get number of processors for use with -maxcpucount
	NPROC_STRING=$(cmd "/C echo %NUMBER_OF_PROCESSORS%")
	NPROC="${NPROC_STRING//[!0-9]/}"
	
	# First build with MinGW if it is installed and in PATH
	GPP="g++"
	MAKE_CMD="mingw32-make"
	if [[ $GPP != "" && $MAKE_CMD != "" && $MINGW_32_BIT != "" && $MINGW_64_BIT != "" ]]; then
		export CXX=$GPP
		COMPILER=gcc

		if [[ $MINGW_32_BIT != "" ]]; then
			export PATH="$MINGW_32_BIT:$PATH"
			build_gcc_clang x86 $COMPILER
			cp libsplinter-$SPLINTER_VERSION.dll libsplinter-static-$SPLINTER_VERSION.a "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
		fi
		

		if [[ $MINGW_64_BIT != "" ]]; then
			export PATH="$MINGW_64_BIT:$PATH"
			build_gcc_clang x86-64 $COMPILER
			cp libsplinter-$SPLINTER_VERSION.dll libsplinter-static-$SPLINTER_VERSION.a "$BUILD_ROOT/$OS/$COMPILER/$ARCH"
		fi
		
		# Have CMake create the interface directory structures, then copy them to the build root
		"$MAKE_CMD" install
		cp -r splinter-matlab $BUILD_ROOT
		cp -r splinter-python $BUILD_ROOT
		
		COMPILER_VERSION=$($CXX -dumpversion)
		
		copy_header_files
		
		# Write down the commit id this was compiled from
		update_commit_id
		update_compiler_version
	fi

	MSBUILD=$(which msbuild.exe)
	if [[ $MSBUILD != "" ]]; then
		COMPILER=msvc
		COMPILER_VERSION=$(msbuild.exe "-version" | grep '^[[:digit:]]\+.[[:digit:]]\+.[[:digit:]]\+.[[:digit:]]\+$')
		
		last_compiled_commit_id=$(cat $BUILD_ROOT/$OS/$COMPILER/commit_id)
		get_commit_id
		if [[ $last_compiled_commit_id == $COMMIT_ID ]]; then
			echo "No new commits since last compile with $COMPILER, skipping."
			
		else
			build_msvc "x86" $COMPILER
			build_msvc "x86-64" $COMPILER
			
			# Write down the commit id this was compiled from
			update_commit_id
			update_compiler_version
		fi
	fi
}


mkdir -p $BUILD_ROOT # -p to avoid error message when it already exists
cd $BUILD_ROOT

PLATFORM=$(uname)
#if [[ $PLATFORM == MINGW* ]]; then
#	rm -r $BUILD_ROOT/windows
#	build_windows
	
#elif [[ $PLATFORM == Linux ]]; then
#	rm -r $BUILD_ROOT/linux
#	build_linux
	
#else
#	echo "Unknown platform: $PLATFORM"
#fi

cd $BUILD_ROOT
# Check that all commit ids are the same
# If they are we can make a release
# TODO: Add osx
# List of required OS's
OSES="windows
linux"
COMMIT_ID=""
for os_dir in $OSES
do
	if [[ ! -d $os_dir ]]; then
		echo "Cannot make release because an OS directory ($os_dir) is missing."
		exit 1
	fi
	
	for compiler in $(ls $BUILD_ROOT/$os_dir)
	do
		if [[ $COMMIT_ID == "" ]]; then
			COMMIT_ID=$(cat $BUILD_ROOT/$os_dir/$compiler/commit_id)
		else
			if [[ $(cat $BUILD_ROOT/$os_dir/$compiler/commit_id) != $COMMIT_ID ]]; then
				echo "Commit id mismatch, $os_dir/$compiler differs."
				echo "Cannot make release."
				exit 1
			fi
		fi
	done
done

echo "All builds were built from the same commit, proceeding to make release."

# If tar is installed, and all commit ids are the same,
# then we make a release
TAR=$(which tar)
ZIP=$(which zip)
if [[ $TAR == ""  || $ZIP == "" ]]; then
	echo "Error: Missing either tar or zip, need both to create release."
	exit 1
fi

RELEASE_DIR=$BUILD_ROOT/releases
mkdir -p $RELEASE_DIR
rm $RELEASE_DIR/* # In case theres an old release there
for os_dir in $OSES
do
	cd $BUILD_ROOT/$os_dir
	for compiler_dir in $(echo */) # echo */ gives us a list of the directories
	do
		compiler_name=${compiler_dir%?} # compiler_dir includes the last /, remove it.
		cd $BUILD_ROOT/$os_dir/$compiler_dir
		files=""
		for arch in $(echo */)
		do
			files="$arch $files"
		done

		filename=$os_dir"_"$compiler_name$(cat compiler_version)
		full_filename=$RELEASE_DIR/$filename

		OLDWD=$(pwd)
		cd $BUILD_ROOT/$os_dir/$compiler_dir

		echo "Creating archive $filename.tar.gz"
		$TAR -czf $full_filename.tar.gz $files > /dev/null

		echo "Creating archive $filename.zip"
		$ZIP -r $full_filename.zip $files > /dev/null
		cd $OLDWD
	done
done

# Make an archive of splinter-matlab and splinter-python
cd $BUILD_ROOT

filename="splinter-matlab"
full_filename="$RELEASE_DIR/$filename"
files="splinter-matlab"
echo "Creating archive $filename.tar.gz"
$TAR -czf $full_filename.tar.gz $files > /dev/null

echo "Creating archive $filename.zip"
$ZIP -r $full_filename $files > /dev/null

filename="splinter-python"
full_filename="$RELEASE_DIR/$filename"
files="splinter-python"
echo "Creating archive $filename.tar.gz"
$TAR -czf $full_filename.tar.gz $files > /dev/null

echo "Creating archive $filename.zip"
$ZIP -r $full_filename $files > /dev/null
