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
include crazyflie_tools/CMakeFiles/battery.dir/depend.make

# Include the progress variables for this target.
include crazyflie_tools/CMakeFiles/battery.dir/progress.make

# Include the compile flags for this target's objects.
include crazyflie_tools/CMakeFiles/battery.dir/flags.make

crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o: crazyflie_tools/CMakeFiles/battery.dir/flags.make
crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o: /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/battery.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/swarmslab/crazyswarm/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/battery.dir/src/battery.cpp.o -c /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/battery.cpp

crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/battery.dir/src/battery.cpp.i"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/battery.cpp > CMakeFiles/battery.dir/src/battery.cpp.i

crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/battery.dir/src/battery.cpp.s"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools/src/battery.cpp -o CMakeFiles/battery.dir/src/battery.cpp.s

# Object files for target battery
battery_OBJECTS = \
"CMakeFiles/battery.dir/src/battery.cpp.o"

# External object files for target battery
battery_EXTERNAL_OBJECTS =

/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery: crazyflie_tools/CMakeFiles/battery.dir/src/battery.cpp.o
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery: crazyflie_tools/CMakeFiles/battery.dir/build.make
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery: /home/swarmslab/crazyswarm/ros_ws/devel/lib/libcrazyflie_cpp.so
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery: /usr/lib/x86_64-linux-gnu/libboost_program_options.so.1.71.0
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery: /usr/lib/x86_64-linux-gnu/libusb-1.0.so
/home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery: crazyflie_tools/CMakeFiles/battery.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/swarmslab/crazyswarm/ros_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery"
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/battery.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
crazyflie_tools/CMakeFiles/battery.dir/build: /home/swarmslab/crazyswarm/ros_ws/devel/lib/crazyflie_tools/battery

.PHONY : crazyflie_tools/CMakeFiles/battery.dir/build

crazyflie_tools/CMakeFiles/battery.dir/clean:
	cd /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools && $(CMAKE_COMMAND) -P CMakeFiles/battery.dir/cmake_clean.cmake
.PHONY : crazyflie_tools/CMakeFiles/battery.dir/clean

crazyflie_tools/CMakeFiles/battery.dir/depend:
	cd /home/swarmslab/crazyswarm/ros_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/swarmslab/crazyswarm/ros_ws/src /home/swarmslab/crazyswarm/ros_ws/src/crazyflie_tools /home/swarmslab/crazyswarm/ros_ws/build /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools /home/swarmslab/crazyswarm/ros_ws/build/crazyflie_tools/CMakeFiles/battery.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : crazyflie_tools/CMakeFiles/battery.dir/depend

