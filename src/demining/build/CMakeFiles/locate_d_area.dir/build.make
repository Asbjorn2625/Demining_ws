# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/ros/project/src/demining

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ros/project/src/demining/build

# Include any dependencies generated for this target.
include CMakeFiles/locate_d_area.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/locate_d_area.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/locate_d_area.dir/flags.make

CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o: CMakeFiles/locate_d_area.dir/flags.make
CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o: ../src/locate_d_area.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ros/project/src/demining/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o"
	/usr/bin/x86_64-linux-gnu-g++-5   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o -c /home/ros/project/src/demining/src/locate_d_area.cpp

CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.i"
	/usr/bin/x86_64-linux-gnu-g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ros/project/src/demining/src/locate_d_area.cpp > CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.i

CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.s"
	/usr/bin/x86_64-linux-gnu-g++-5  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ros/project/src/demining/src/locate_d_area.cpp -o CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.s

CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.requires:

.PHONY : CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.requires

CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.provides: CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.requires
	$(MAKE) -f CMakeFiles/locate_d_area.dir/build.make CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.provides.build
.PHONY : CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.provides

CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.provides.build: CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o


# Object files for target locate_d_area
locate_d_area_OBJECTS = \
"CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o"

# External object files for target locate_d_area
locate_d_area_EXTERNAL_OBJECTS =

devel/lib/demining/locate_d_area: CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o
devel/lib/demining/locate_d_area: CMakeFiles/locate_d_area.dir/build.make
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libcv_bridge.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libimage_transport.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libmessage_filters.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libclass_loader.so
devel/lib/demining/locate_d_area: /usr/lib/libPocoFoundation.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libdl.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libroslib.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/librospack.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libpython2.7.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_program_options.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libtinyxml.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libactionlib.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libroscpp.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_signals.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/librosconsole.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_regex.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libxmlrpcpp.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libroscpp_serialization.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/librostime.so
devel/lib/demining/locate_d_area: /opt/ros/kinetic/lib/libcpp_common.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_system.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_thread.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libpthread.so
devel/lib/demining/locate_d_area: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
devel/lib/demining/locate_d_area: CMakeFiles/locate_d_area.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ros/project/src/demining/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable devel/lib/demining/locate_d_area"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/locate_d_area.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/locate_d_area.dir/build: devel/lib/demining/locate_d_area

.PHONY : CMakeFiles/locate_d_area.dir/build

CMakeFiles/locate_d_area.dir/requires: CMakeFiles/locate_d_area.dir/src/locate_d_area.cpp.o.requires

.PHONY : CMakeFiles/locate_d_area.dir/requires

CMakeFiles/locate_d_area.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/locate_d_area.dir/cmake_clean.cmake
.PHONY : CMakeFiles/locate_d_area.dir/clean

CMakeFiles/locate_d_area.dir/depend:
	cd /home/ros/project/src/demining/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ros/project/src/demining /home/ros/project/src/demining /home/ros/project/src/demining/build /home/ros/project/src/demining/build /home/ros/project/src/demining/build/CMakeFiles/locate_d_area.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/locate_d_area.dir/depend

