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
CMAKE_SOURCE_DIR = /home/juanma/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/juanma/catkin_ws/build

# Include any dependencies generated for this target.
include magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/depend.make

# Include the progress variables for this target.
include magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/progress.make

# Include the compile flags for this target's objects.
include magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/flags.make

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o: magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/flags.make
magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o: /home/juanma/catkin_ws/src/magabot-idmind-ros/src/magabot/src/magabot_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/juanma/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o"
	cd /home/juanma/catkin_ws/build/magabot-idmind-ros/src/magabot && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o -c /home/juanma/catkin_ws/src/magabot-idmind-ros/src/magabot/src/magabot_node.cpp

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/magabot_node.dir/src/magabot_node.cpp.i"
	cd /home/juanma/catkin_ws/build/magabot-idmind-ros/src/magabot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/juanma/catkin_ws/src/magabot-idmind-ros/src/magabot/src/magabot_node.cpp > CMakeFiles/magabot_node.dir/src/magabot_node.cpp.i

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/magabot_node.dir/src/magabot_node.cpp.s"
	cd /home/juanma/catkin_ws/build/magabot-idmind-ros/src/magabot && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/juanma/catkin_ws/src/magabot-idmind-ros/src/magabot/src/magabot_node.cpp -o CMakeFiles/magabot_node.dir/src/magabot_node.cpp.s

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.requires:

.PHONY : magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.requires

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.provides: magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.requires
	$(MAKE) -f magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/build.make magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.provides.build
.PHONY : magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.provides

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.provides.build: magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o


# Object files for target magabot_node
magabot_node_OBJECTS = \
"CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o"

# External object files for target magabot_node
magabot_node_EXTERNAL_OBJECTS =

/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/build.make
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /home/juanma/catkin_ws/devel/lib/libcereal_port.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libtf.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libtf2_ros.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libactionlib.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libmessage_filters.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libroscpp.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libtf2.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/librosconsole.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/liblog4cxx.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/librostime.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/juanma/catkin_ws/devel/lib/magabot/magabot_node: magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/juanma/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/juanma/catkin_ws/devel/lib/magabot/magabot_node"
	cd /home/juanma/catkin_ws/build/magabot-idmind-ros/src/magabot && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/magabot_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/build: /home/juanma/catkin_ws/devel/lib/magabot/magabot_node

.PHONY : magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/build

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/requires: magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/src/magabot_node.cpp.o.requires

.PHONY : magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/requires

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/clean:
	cd /home/juanma/catkin_ws/build/magabot-idmind-ros/src/magabot && $(CMAKE_COMMAND) -P CMakeFiles/magabot_node.dir/cmake_clean.cmake
.PHONY : magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/clean

magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/depend:
	cd /home/juanma/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/juanma/catkin_ws/src /home/juanma/catkin_ws/src/magabot-idmind-ros/src/magabot /home/juanma/catkin_ws/build /home/juanma/catkin_ws/build/magabot-idmind-ros/src/magabot /home/juanma/catkin_ws/build/magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : magabot-idmind-ros/src/magabot/CMakeFiles/magabot_node.dir/depend

