#Compilation

*Please note that we provide precompiled binaries of SPLINTER [here](https://github.com/bgrimstad/splinter/releases)*.

If you want to compile SPLINTER with the least amount of effort, please see [this guide](#easy) for how to compile the library with our release script.

If you want more control and avoid compiling for all architectures and with all available compilers you can use these guides:

##Easy

Note: This script compiles the library for all architectures with all compilers available (that it knows about), and therefore probably takes more time (at least if you plan to compile the library more than once).

####Requirements
* [CMake](http://www.cmake.org/)
* [Git](http://git-scm.com/)
* UNIX: [GCC](https://gcc.gnu.org/) and/or [Clang](http://clang.llvm.org/)
* Windows: [MinGW](https://sourceforge.net/projects/mingw-w64/) and/or [Visual Studio Community](https://www.visualstudio.com/en-us/downloads/download-visual-studio-vs.aspx)
* Ubuntu: build-essentials (`apt-get install build-essentials`)

####MinGW note:
MinGW is not multilib-enabled, meaning it can only compile either x86 or x86-64. If you want to compile against both architectures, you need two MinGW installs, one for x86 (i686) and one for x86-64. These are the versions we have tested:
* x86: 4.9.2 with dwarf exception model, posix threads and build revision 3
* x86-64: 4.9.2 with seh exception model, posix threads and build revision 3

You should avoid installing MinGW to a path containing spaces (that includes Program Files!) as that may lead to problems with CMake not finding it.

####Clang note:
The scripts looks for clang++, but on Ubuntu (tested with 14.04.2 LTS) /usr/bin/clang++ is not symlinked to /usr/bin/clang++-3.5 (assuming 3.5 is the version of Clang you have installed), so the script can't find it. You can fix this by doing

`sudo ln -s /usr/bin/clang++-3.5 /usr/bin/clang++`

###Compilation
All the commands shown here should be entered into a terminal window if on UNIX, or Git Bash if on Windows. Git Bash should've been installed when you installed Git.

1. `git clone https://github.com/bgrimstad/splinter.git`
2. `cd splinter`
3. Windows: open scripts/build_release.sh and make sure the configuration corresponds to your install. If you do not plan to compile with a specific compiler, you can comment out the lines regarding that compiler.
4. UNIX: make sure the compilers you intend to use are in your PATH (export PATH=/path/to/compiler:$PATH).
5. `./scripts/build_release.sh`

A new directory called build will be created, and the binaries can be found in build/OS/COMPILER/ARCHITECTURE, where the capitalized directories are variables depending on your configuration.

##Advanced
Please use the guide corresponding to your platform:
* [UNIX](#compile-on-unix-tested-with-ubuntu)
* [Windows](#compile-on-windows-mingw)

Note that most of the [options](#options-both-platforms) to CMake are the same for both guides.

###Compile on UNIX (tested with Ubuntu)
####Requirements
* make: Ubuntu: `apt-get install build-essentials`, OSX: XCode should provide this (not tested).
* [CMake](http://www.cmake.org/)
* [Git](http://git-scm.com/)
* [GCC](https://gcc.gnu.org/) or [Clang](http://clang.llvm.org/)

1. `git clone https://github.com/bgrimstad/splinter.git`
2. `cd splinter`
3. `mkdir build && cd build`
4. Choose compiler: `export CXX=$(which g++)` or `export CXX=$(which clang++)`
5. `cmake ..` Also, see [options](#options-both-platforms)
6. `make`
7. `make install`


####Troubleshooting
`make: *** [install] Error 1`: You probably need elevated rights to install the library because you are trying to write to a directory you don't have permission to write to. Either change the install paths via the options, or run step #8 again like this: `sudo make install`.

`fatal error: bits/c++config.h: No such file or directory`: If your OS is 64 bits and you try to compile the library as a 32 bits executable with GCC you may get this error. To resolve this you need to install g++-multilib (`sudo apt-get install g++-multilib on Ubuntu`).

---

###Compile on Windows (MinGW)

* [CMake](http://www.cmake.org/)
* [Git](http://git-scm.com/)
* [MinGW](http://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/installer/mingw-w64-install.exe/download) (please see [these](#mingw-note) notes on MinGW)

Before proceeding you need to make sure the binary directory of MinGW (the directory containing mingw32-make.exe) is in your PATH. For that, see [this](http://www.computerhope.com/issues/ch000549.htm) guide.

1. `git clone https://github.com/bgrimstad/splinter.git`
2. `cd splinter`
3. `mkdir build && cd build`
4. `cmake .. -DCMAKE_MAKE_PROGRAM=mingw32-make -G "Unix Makefiles"` Also, see [options](#options-both-platforms)
5. `mingw32-make`

The output files should be in your build folder. If you wish to have them installed you can type `mingw32-make install`.

####Troubleshooting
`cc1plus.exe:-1: error: out of memory allocating (...).`: You need to use GCC version <= 4.7.0 or >= 4.9.1, as other versions has this bug. [This](http://sourceforge.net/projects/mingw-w64/files/Toolchains%20targetting%20Win32/Personal%20Builds/mingw-builds/4.9.2/threads-posix/dwarf/i686-4.9.2-release-posix-dwarf-rt_v4-rev3.7z/download) version of MinGW should be able to compile the library. To install this version of MinGW you can just delete everything in C:\mingw and replace it with what you find in the archive you downloaded.

---

##Options (all platforms)

| Variable name             | Default value     | Description                                                                   |
| ------------------------- | ----------------- | ----------------------------------------------------------------------------- |
| EIGEN_DIRECTORY           | thirdparty/Eigen  | Path to the Eigen lib.                                                        |
| HEADER_INSTALL_DIRECTORY  | include           | Where the headers should be installed.                                        |
| LIBRARY_INSTALL_DIRECTORY | lib               | Where to install the library file.                                            |
| ARCH                      | x86               | Architecture of the generated binary (x86 / x86-64). Only works with GCC/Clang. |
| CMAKE_BUILD_TYPE          | Release           | Build type (Debug / Release)                                                  |

These options go along with the cmake step, and are used like this:

    UNIX:
    cmake .. -DEIGEN_DIRECTORY=/path/to/eigen -DHEADER_INSTALL_DIRECTORY=/home/me/splinter/includes -DARCH=x86-64
    
    
    Windows (in the arguments field):
    -DEIGEN_DIRECTORY=C:/path/to/eigen -DHEADER_INSTALL_DIRECTORY=C:/Users/me/splinter/includes

The syntax is: `-D<VARIABLE_NAME>=<VARIABLE_VALUE>`. If you have any spaces in your value you must surround it with double quotes (").

Note for the header and library paths:
If the path is relative (the first character is not / on UNIX or C:/ (or equivalent) on Windows), then the actual path used will be relative to [CMAKE_INSTALL_PREFIX](http://www.cmake.org/cmake/help/v2.8.12/cmake.html#variable:CMAKE_INSTALL_PREFIX).
