# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_COMMAND = /opt/local/bin/cmake

# The command to remove a file.
RM = /opt/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking

# Include any dependencies generated for this target.
include CMakeFiles/Tracking.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Tracking.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Tracking.dir/flags.make

CMakeFiles/Tracking.dir/main.cpp.o: CMakeFiles/Tracking.dir/flags.make
CMakeFiles/Tracking.dir/main.cpp.o: main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Tracking.dir/main.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Tracking.dir/main.cpp.o -c /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/main.cpp

CMakeFiles/Tracking.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Tracking.dir/main.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/main.cpp > CMakeFiles/Tracking.dir/main.cpp.i

CMakeFiles/Tracking.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Tracking.dir/main.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/main.cpp -o CMakeFiles/Tracking.dir/main.cpp.s

CMakeFiles/Tracking.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/Tracking.dir/main.cpp.o.requires

CMakeFiles/Tracking.dir/main.cpp.o.provides: CMakeFiles/Tracking.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Tracking.dir/build.make CMakeFiles/Tracking.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Tracking.dir/main.cpp.o.provides

CMakeFiles/Tracking.dir/main.cpp.o.provides.build: CMakeFiles/Tracking.dir/main.cpp.o

CMakeFiles/Tracking.dir/UKF.cpp.o: CMakeFiles/Tracking.dir/flags.make
CMakeFiles/Tracking.dir/UKF.cpp.o: UKF.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/Tracking.dir/UKF.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Tracking.dir/UKF.cpp.o -c /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/UKF.cpp

CMakeFiles/Tracking.dir/UKF.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Tracking.dir/UKF.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/UKF.cpp > CMakeFiles/Tracking.dir/UKF.cpp.i

CMakeFiles/Tracking.dir/UKF.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Tracking.dir/UKF.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/UKF.cpp -o CMakeFiles/Tracking.dir/UKF.cpp.s

CMakeFiles/Tracking.dir/UKF.cpp.o.requires:
.PHONY : CMakeFiles/Tracking.dir/UKF.cpp.o.requires

CMakeFiles/Tracking.dir/UKF.cpp.o.provides: CMakeFiles/Tracking.dir/UKF.cpp.o.requires
	$(MAKE) -f CMakeFiles/Tracking.dir/build.make CMakeFiles/Tracking.dir/UKF.cpp.o.provides.build
.PHONY : CMakeFiles/Tracking.dir/UKF.cpp.o.provides

CMakeFiles/Tracking.dir/UKF.cpp.o.provides.build: CMakeFiles/Tracking.dir/UKF.cpp.o

# Object files for target Tracking
Tracking_OBJECTS = \
"CMakeFiles/Tracking.dir/main.cpp.o" \
"CMakeFiles/Tracking.dir/UKF.cpp.o"

# External object files for target Tracking
Tracking_EXTERNAL_OBJECTS =

Tracking: CMakeFiles/Tracking.dir/main.cpp.o
Tracking: CMakeFiles/Tracking.dir/UKF.cpp.o
Tracking: CMakeFiles/Tracking.dir/build.make
Tracking: /usr/local/lib/libopencv_videostab.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_video.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_ts.a
Tracking: /usr/local/lib/libopencv_superres.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_stitching.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_photo.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_ocl.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_objdetect.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_nonfree.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_ml.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_legacy.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_imgproc.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_highgui.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_gpu.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_flann.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_features2d.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_core.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_contrib.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_calib3d.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_nonfree.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_ocl.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_gpu.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_photo.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_objdetect.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_legacy.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_video.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_ml.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_calib3d.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_features2d.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_highgui.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_imgproc.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_flann.2.4.9.dylib
Tracking: /usr/local/lib/libopencv_core.2.4.9.dylib
Tracking: CMakeFiles/Tracking.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Tracking"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Tracking.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Tracking.dir/build: Tracking
.PHONY : CMakeFiles/Tracking.dir/build

CMakeFiles/Tracking.dir/requires: CMakeFiles/Tracking.dir/main.cpp.o.requires
CMakeFiles/Tracking.dir/requires: CMakeFiles/Tracking.dir/UKF.cpp.o.requires
.PHONY : CMakeFiles/Tracking.dir/requires

CMakeFiles/Tracking.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Tracking.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Tracking.dir/clean

CMakeFiles/Tracking.dir/depend:
	cd /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking /Users/georgyguryev/Desktop/Skoltech/Robotics/skoltech-robotics-people-detection/Tracking/CMakeFiles/Tracking.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Tracking.dir/depend

