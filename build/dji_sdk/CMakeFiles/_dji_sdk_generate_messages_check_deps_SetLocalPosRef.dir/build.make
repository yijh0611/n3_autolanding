# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /home/aims/.local/lib/python2.7/site-packages/cmake/data/bin/cmake

# The command to remove a file.
RM = /home/aims/.local/lib/python2.7/site-packages/cmake/data/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/aims/catkin_ws/build/dji_sdk

# Utility rule file for _dji_sdk_generate_messages_check_deps_SetLocalPosRef.

# Include any custom commands dependencies for this target.
include CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/progress.make

CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef:
	catkin_generated/env_cached.sh /usr/bin/python2 /opt/ros/melodic/share/genmsg/cmake/../../../lib/genmsg/genmsg_check_deps.py dji_sdk /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/srv/SetLocalPosRef.srv 

_dji_sdk_generate_messages_check_deps_SetLocalPosRef: CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef
_dji_sdk_generate_messages_check_deps_SetLocalPosRef: CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/build.make
.PHONY : _dji_sdk_generate_messages_check_deps_SetLocalPosRef

# Rule to build all files generated by this target.
CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/build: _dji_sdk_generate_messages_check_deps_SetLocalPosRef
.PHONY : CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/build

CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/cmake_clean.cmake
.PHONY : CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/clean

CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/depend:
	cd /home/aims/catkin_ws/build/dji_sdk && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk /home/aims/catkin_ws/build/dji_sdk /home/aims/catkin_ws/build/dji_sdk /home/aims/catkin_ws/build/dji_sdk/CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/_dji_sdk_generate_messages_check_deps_SetLocalPosRef.dir/depend

