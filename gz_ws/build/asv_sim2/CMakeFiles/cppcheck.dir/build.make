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
CMAKE_SOURCE_DIR = /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/blueboat_sitl/gz_ws/build/asv_sim2

# Utility rule file for cppcheck.

# Include any custom commands dependencies for this target.
include CMakeFiles/cppcheck.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/cppcheck.dir/progress.make

CMakeFiles/cppcheck:
	/usr/bin/cppcheck -q --inline-suppr -j 4 --language=c++ --std=c++17 --force --enable=style,performance,portability,information -I/home/blueboat_sitl/gz_ws/build/asv_sim2 -I/home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/test/performance -UM_PI\ --rule-file=/usr/share/gz/gz-cmake3/codecheck/header_guard.rule\ --rule-file=/usr/share/gz/gz-cmake3/codecheck/namespace_AZ.rule `/usr/bin/find /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/src /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/include /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/test/performance -name '*.cc' -o -name '*.hh' -o -name '*.c' -o -name '*.h'`
	/usr/bin/cppcheck -q --inline-suppr -j 4 --language=c++ --std=c++17 --force --enable=missingInclude `/usr/bin/find /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/src /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/include /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins/test/performance -name '*.cc' -o -name '*.hh' -o -name '*.c' -o -name '*.h'`

cppcheck: CMakeFiles/cppcheck
cppcheck: CMakeFiles/cppcheck.dir/build.make
.PHONY : cppcheck

# Rule to build all files generated by this target.
CMakeFiles/cppcheck.dir/build: cppcheck
.PHONY : CMakeFiles/cppcheck.dir/build

CMakeFiles/cppcheck.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cppcheck.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cppcheck.dir/clean

CMakeFiles/cppcheck.dir/depend:
	cd /home/blueboat_sitl/gz_ws/build/asv_sim2 && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins /home/blueboat_sitl/gz_ws/src/asv_sim/asv_sim_gazebo_plugins /home/blueboat_sitl/gz_ws/build/asv_sim2 /home/blueboat_sitl/gz_ws/build/asv_sim2 /home/blueboat_sitl/gz_ws/build/asv_sim2/CMakeFiles/cppcheck.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cppcheck.dir/depend

