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

# Include any dependencies generated for this target.
include CMakeFiles/talker_ros_17.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/talker_ros_17.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/talker_ros_17.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/talker_ros_17.dir/flags.make

CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o: CMakeFiles/talker_ros_17.dir/flags.make
CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o: /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/src/talker_ros_17.cpp
CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o: CMakeFiles/talker_ros_17.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/aims/catkin_ws/build/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o -MF CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o.d -o CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o -c /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/src/talker_ros_17.cpp

CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/src/talker_ros_17.cpp > CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.i

CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk/src/talker_ros_17.cpp -o CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.s

# Object files for target talker_ros_17
talker_ros_17_OBJECTS = \
"CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o"

# External object files for target talker_ros_17
talker_ros_17_EXTERNAL_OBJECTS =

/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: CMakeFiles/talker_ros_17.dir/src/talker_ros_17.cpp.o
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: CMakeFiles/talker_ros_17.dir/build.make
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libcv_bridge.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libimage_geometry.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_calib3d.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_core.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_features2d.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_flann.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_highgui.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_imgcodecs.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_imgproc.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_ml.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_objdetect.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_photo.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_shape.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_stitching.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_superres.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_video.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_videoio.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_videostab.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_viz.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_aruco.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_bgsegm.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_bioinspired.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_ccalib.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_datasets.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_dpm.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_face.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_freetype.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_fuzzy.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_hdf.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_line_descriptor.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_optflow.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_phase_unwrapping.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_plot.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_reg.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_rgbd.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_saliency.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_stereo.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_structured_light.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_surface_matching.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_text.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_ximgproc.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_xobjdetect.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libopencv_xphoto.so.3.2.0
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libimage_transport.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libnodeletlib.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libbondcpp.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libuuid.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libclass_loader.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/libPocoFoundation.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libdl.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libroslib.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/librospack.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libtf.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libtf2_ros.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libactionlib.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libmessage_filters.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libroscpp.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libtf2.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/librosconsole.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/librostime.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /opt/ros/melodic/lib/libcpp_common.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: /usr/local/lib/libdjiosdk-core.a
/home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17: CMakeFiles/talker_ros_17.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/aims/catkin_ws/build/dji_sdk/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/talker_ros_17.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/talker_ros_17.dir/build: /home/aims/catkin_ws/devel/.private/dji_sdk/lib/dji_sdk/talker_ros_17
.PHONY : CMakeFiles/talker_ros_17.dir/build

CMakeFiles/talker_ros_17.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/talker_ros_17.dir/cmake_clean.cmake
.PHONY : CMakeFiles/talker_ros_17.dir/clean

CMakeFiles/talker_ros_17.dir/depend:
	cd /home/aims/catkin_ws/build/dji_sdk && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk /home/aims/catkin_ws/src/Onboard-SDK-ROS-3.8.1/dji_sdk /home/aims/catkin_ws/build/dji_sdk /home/aims/catkin_ws/build/dji_sdk /home/aims/catkin_ws/build/dji_sdk/CMakeFiles/talker_ros_17.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/talker_ros_17.dir/depend

