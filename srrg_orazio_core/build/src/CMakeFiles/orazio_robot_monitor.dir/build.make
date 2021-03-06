# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/giorgio/source/srrg/srrg_orazio_core

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/giorgio/source/srrg/srrg_orazio_core/build

# Include any dependencies generated for this target.
include src/CMakeFiles/orazio_robot_monitor.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/orazio_robot_monitor.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/orazio_robot_monitor.dir/flags.make

src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o: src/CMakeFiles/orazio_robot_monitor.dir/flags.make
src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o: ../src/orazio_robot_monitor.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/giorgio/source/srrg/srrg_orazio_core/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o"
	cd /home/giorgio/source/srrg/srrg_orazio_core/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o -c /home/giorgio/source/srrg/srrg_orazio_core/src/orazio_robot_monitor.cpp

src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.i"
	cd /home/giorgio/source/srrg/srrg_orazio_core/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/giorgio/source/srrg/srrg_orazio_core/src/orazio_robot_monitor.cpp > CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.i

src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.s"
	cd /home/giorgio/source/srrg/srrg_orazio_core/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/giorgio/source/srrg/srrg_orazio_core/src/orazio_robot_monitor.cpp -o CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.s

src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.requires:
.PHONY : src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.requires

src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.provides: src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/orazio_robot_monitor.dir/build.make src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.provides.build
.PHONY : src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.provides

src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.provides.build: src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o

# Object files for target orazio_robot_monitor
orazio_robot_monitor_OBJECTS = \
"CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o"

# External object files for target orazio_robot_monitor
orazio_robot_monitor_EXTERNAL_OBJECTS =

src/orazio_robot_monitor: src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o
src/orazio_robot_monitor: src/CMakeFiles/orazio_robot_monitor.dir/build.make
src/orazio_robot_monitor: src/liborazio_robot_library.a
src/orazio_robot_monitor: /usr/lib/x86_64-linux-gnu/libcurses.so
src/orazio_robot_monitor: src/CMakeFiles/orazio_robot_monitor.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable orazio_robot_monitor"
	cd /home/giorgio/source/srrg/srrg_orazio_core/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/orazio_robot_monitor.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/orazio_robot_monitor.dir/build: src/orazio_robot_monitor
.PHONY : src/CMakeFiles/orazio_robot_monitor.dir/build

src/CMakeFiles/orazio_robot_monitor.dir/requires: src/CMakeFiles/orazio_robot_monitor.dir/orazio_robot_monitor.cpp.o.requires
.PHONY : src/CMakeFiles/orazio_robot_monitor.dir/requires

src/CMakeFiles/orazio_robot_monitor.dir/clean:
	cd /home/giorgio/source/srrg/srrg_orazio_core/build/src && $(CMAKE_COMMAND) -P CMakeFiles/orazio_robot_monitor.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/orazio_robot_monitor.dir/clean

src/CMakeFiles/orazio_robot_monitor.dir/depend:
	cd /home/giorgio/source/srrg/srrg_orazio_core/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/giorgio/source/srrg/srrg_orazio_core /home/giorgio/source/srrg/srrg_orazio_core/src /home/giorgio/source/srrg/srrg_orazio_core/build /home/giorgio/source/srrg/srrg_orazio_core/build/src /home/giorgio/source/srrg/srrg_orazio_core/build/src/CMakeFiles/orazio_robot_monitor.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/orazio_robot_monitor.dir/depend

