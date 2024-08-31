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

# Include any dependencies generated for this target.
include crazyflie_tools/CMakeFiles/console.dir/depend.make

# Include the progress variables for this target.
include crazyflie_tools/CMakeFiles/console.dir/progress.make

# Include the compile flags for this target's objects.
include crazyflie_tools/CMakeFiles/console.dir/flags.make

crazyflie_tools/CMakeFiles/console.dir/src/console.cpp.o: crazyflie_tools/CMakeFiles/console.dir/flags.make
crazyflie_tools/CMakeFiles/console.dir/src/console.cpp.o: /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/console.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/swarmslab/crazyswarm/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object crazyflie_tools/CMakeFiles/console.dir/src/console.cpp.o"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/console.dir/src/console.cpp.o -c /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/console.cpp

crazyflie_tools/CMakeFiles/console.dir/src/console.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/console.dir/src/console.cpp.i"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/console.cpp > CMakeFiles/console.dir/src/console.cpp.i

crazyflie_tools/CMakeFiles/console.dir/src/console.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/console.dir/src/console.cpp.s"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/console.cpp -o CMakeFiles/console.dir/src/console.cpp.s

# Object files for target console
console_OBJECTS = \
"CMakeFiles/console.dir/src/console.cpp.o"

# External object files for target console
console_EXTERNAL_OBJECTS =

/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console: crazyflie_tools/CMakeFiles/console.dir/src/console.cpp.o
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console: crazyflie_tools/CMakeFiles/console.dir/build.make
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console: /home/swarmslab/crazyswarm/ros_ws/devel/lib/libcrazyflie_cpp.so
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console: crazyflie_tools/CMakeFiles/console.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/swarmslab/crazyswarm/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/console.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crazyflie_tools/CMakeFiles/console.dir/build: /home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/console

.PHONY : crazyflie_tools/CMakeFiles/console.dir/build

crazyflie_tools/CMakeFiles/console.dir/clean:
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && $(CMAKE_COMMAND) -P CMakeFiles/console.dir/cmake_clean.cmake
.PHONY : crazyflie_tools/CMakeFiles/console.dir/clean

crazyflie_tools/CMakeFiles/console.dir/depend:
	cd /home/swarmslab/crazyswarm/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/swarmslab/crazyswarm/ros_ws/src /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools /home/swarmslab/crazyswarm/ros_ws/build /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools/CMakeFiles/console.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyflie_tools/CMakeFiles/console.dir/depend

