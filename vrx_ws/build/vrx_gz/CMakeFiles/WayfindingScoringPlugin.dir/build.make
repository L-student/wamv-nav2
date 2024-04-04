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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/momesso/vrx_ws/src/vrx/vrx_gz

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/momesso/vrx_ws/build/vrx_gz

# Include any dependencies generated for this target.
include CMakeFiles/WayfindingScoringPlugin.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/WayfindingScoringPlugin.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/WayfindingScoringPlugin.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/WayfindingScoringPlugin.dir/flags.make

CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o: CMakeFiles/WayfindingScoringPlugin.dir/flags.make
CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o: /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WayfindingScoringPlugin.cc
CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o: CMakeFiles/WayfindingScoringPlugin.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/momesso/vrx_ws/build/vrx_gz/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o -MF CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o.d -o CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o -c /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WayfindingScoringPlugin.cc

CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WayfindingScoringPlugin.cc > CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.i

CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WayfindingScoringPlugin.cc -o CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.s

CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o: CMakeFiles/WayfindingScoringPlugin.dir/flags.make
CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o: /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WaypointMarkers.cc
CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o: CMakeFiles/WayfindingScoringPlugin.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/momesso/vrx_ws/build/vrx_gz/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o -MF CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o.d -o CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o -c /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WaypointMarkers.cc

CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WaypointMarkers.cc > CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.i

CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/momesso/vrx_ws/src/vrx/vrx_gz/src/WaypointMarkers.cc -o CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.s

# Object files for target WayfindingScoringPlugin
WayfindingScoringPlugin_OBJECTS = \
"CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o" \
"CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o"

# External object files for target WayfindingScoringPlugin
WayfindingScoringPlugin_EXTERNAL_OBJECTS =

libWayfindingScoringPlugin.so: CMakeFiles/WayfindingScoringPlugin.dir/src/WayfindingScoringPlugin.cc.o
libWayfindingScoringPlugin.so: CMakeFiles/WayfindingScoringPlugin.dir/src/WaypointMarkers.cc.o
libWayfindingScoringPlugin.so: CMakeFiles/WayfindingScoringPlugin.dir/build.make
libWayfindingScoringPlugin.so: libScoringPlugin.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-sim7.so.7.7.0
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-fuel_tools8.so.8.1.0
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-gui7.so.7.2.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2-loader.so.2.0.2
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5QuickControls2.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Quick.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5QmlModels.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Qml.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Network.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Widgets.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Gui.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libQt5Core.so.5.15.3
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libtinyxml2.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-physics6.so.6.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-plugin2.so.2.0.2
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-rendering7.so.7.4.2
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-profiler.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-events.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-av.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libswscale.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavdevice.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavformat.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavcodec.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libavutil.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-io.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-testing.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-geospatial.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5-graphics.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-common5.so.5.5.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-transport12-parameters.so.12.2.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-transport12.so.12.2.1
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libuuid.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-msgs9.so.9.5.0
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libsdformat13.so.13.6.0
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libprotobuf.so
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-math7.so.7.4.0
libWayfindingScoringPlugin.so: /usr/lib/x86_64-linux-gnu/libgz-utils2.so.2.2.0
libWayfindingScoringPlugin.so: CMakeFiles/WayfindingScoringPlugin.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/momesso/vrx_ws/build/vrx_gz/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX shared library libWayfindingScoringPlugin.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/WayfindingScoringPlugin.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/WayfindingScoringPlugin.dir/build: libWayfindingScoringPlugin.so
.PHONY : CMakeFiles/WayfindingScoringPlugin.dir/build

CMakeFiles/WayfindingScoringPlugin.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/WayfindingScoringPlugin.dir/cmake_clean.cmake
.PHONY : CMakeFiles/WayfindingScoringPlugin.dir/clean

CMakeFiles/WayfindingScoringPlugin.dir/depend:
	cd /home/momesso/vrx_ws/build/vrx_gz && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/momesso/vrx_ws/src/vrx/vrx_gz /home/momesso/vrx_ws/src/vrx/vrx_gz /home/momesso/vrx_ws/build/vrx_gz /home/momesso/vrx_ws/build/vrx_gz /home/momesso/vrx_ws/build/vrx_gz/CMakeFiles/WayfindingScoringPlugin.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/WayfindingScoringPlugin.dir/depend

