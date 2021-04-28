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
CMAKE_SOURCE_DIR = /home/robproj/panda/libfranka

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robproj/panda/libfranka

# Include any dependencies generated for this target.
include examples/CMakeFiles/mpc_target.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/mpc_target.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/mpc_target.dir/flags.make

examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o: examples/CMakeFiles/mpc_target.dir/flags.make
examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o: examples/mpc_target.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robproj/panda/libfranka/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o"
	cd /home/robproj/panda/libfranka/examples && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/mpc_target.dir/mpc_target.cpp.o -c /home/robproj/panda/libfranka/examples/mpc_target.cpp

examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/mpc_target.dir/mpc_target.cpp.i"
	cd /home/robproj/panda/libfranka/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robproj/panda/libfranka/examples/mpc_target.cpp > CMakeFiles/mpc_target.dir/mpc_target.cpp.i

examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/mpc_target.dir/mpc_target.cpp.s"
	cd /home/robproj/panda/libfranka/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robproj/panda/libfranka/examples/mpc_target.cpp -o CMakeFiles/mpc_target.dir/mpc_target.cpp.s

examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.requires:

.PHONY : examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.requires

examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.provides: examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/mpc_target.dir/build.make examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.provides.build
.PHONY : examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.provides

examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.provides.build: examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o


# Object files for target mpc_target
mpc_target_OBJECTS = \
"CMakeFiles/mpc_target.dir/mpc_target.cpp.o"

# External object files for target mpc_target
mpc_target_EXTERNAL_OBJECTS = \
"/home/robproj/panda/libfranka/examples/solver.o" \
"/home/robproj/panda/libfranka/examples/ldl.o" \
"/home/robproj/panda/libfranka/examples/util.o" \
"/home/robproj/panda/libfranka/examples/matrix_support.o"

examples/mpc_target: examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o
examples/mpc_target: examples/solver.o
examples/mpc_target: examples/ldl.o
examples/mpc_target: examples/util.o
examples/mpc_target: examples/matrix_support.o
examples/mpc_target: examples/CMakeFiles/mpc_target.dir/build.make
examples/mpc_target: examples/libexamples_common.a
examples/mpc_target: libfranka.so.0.7.1
examples/mpc_target: examples/CMakeFiles/mpc_target.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robproj/panda/libfranka/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable mpc_target"
	cd /home/robproj/panda/libfranka/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpc_target.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/mpc_target.dir/build: examples/mpc_target

.PHONY : examples/CMakeFiles/mpc_target.dir/build

examples/CMakeFiles/mpc_target.dir/requires: examples/CMakeFiles/mpc_target.dir/mpc_target.cpp.o.requires

.PHONY : examples/CMakeFiles/mpc_target.dir/requires

examples/CMakeFiles/mpc_target.dir/clean:
	cd /home/robproj/panda/libfranka/examples && $(CMAKE_COMMAND) -P CMakeFiles/mpc_target.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/mpc_target.dir/clean

examples/CMakeFiles/mpc_target.dir/depend:
	cd /home/robproj/panda/libfranka && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robproj/panda/libfranka /home/robproj/panda/libfranka/examples /home/robproj/panda/libfranka /home/robproj/panda/libfranka/examples /home/robproj/panda/libfranka/examples/CMakeFiles/mpc_target.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/mpc_target.dir/depend
