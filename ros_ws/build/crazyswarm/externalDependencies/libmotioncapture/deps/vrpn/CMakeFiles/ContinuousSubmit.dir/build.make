# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/swarmslab/crazyswarm/ros_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/swarmslab/crazyswarm/ros_ws/build

# Utility rule file for ContinuousSubmit.

# Include the progress variables for this target.
include crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/progress.make

crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit:
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyswarm/externalDependencies/libmotioncapture/deps/vrpn && /usr/bin/ctest -D ContinuousSubmit

ContinuousSubmit: crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit
ContinuousSubmit: crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/build.make

.PHONY : ContinuousSubmit

# Rule to build all files generated by this target.
crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/build: ContinuousSubmit

.PHONY : crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/build

crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/clean:
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyswarm/externalDependencies/libmotioncapture/deps/vrpn && $(CMAKE_COMMAND) -P CMakeFiles/ContinuousSubmit.dir/cmake_clean.cmake
.PHONY : crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/clean

crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/depend:
	cd /home/swarmslab/crazyswarm/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/swarmslab/crazyswarm/ros_ws/src /home/swarmslab/crazyswarm/ros_ws/src/crazyswarm/externalDependencies/libmotioncapture/deps/vrpn /home/swarmslab/crazyswarm/ros_ws/build /home/swarmslab/crazyswarm/ros_ws/build/crazyswarm/externalDependencies/libmotioncapture/deps/vrpn /home/swarmslab/crazyswarm/ros_ws/build/crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyswarm/externalDependencies/libmotioncapture/deps/vrpn/CMakeFiles/ContinuousSubmit.dir/depend

