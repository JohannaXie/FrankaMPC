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
include examples/CMakeFiles/MPC_openloop.dir/depend.make

# Include the progress variables for this target.
include examples/CMakeFiles/MPC_openloop.dir/progress.make

# Include the compile flags for this target's objects.
include examples/CMakeFiles/MPC_openloop.dir/flags.make

examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o: examples/CMakeFiles/MPC_openloop.dir/flags.make
examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o: examples/MPC_openloop.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robproj/panda/libfranka/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o"
	cd /home/robproj/panda/libfranka/examples && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o -c /home/robproj/panda/libfranka/examples/MPC_openloop.cpp

examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.i"
	cd /home/robproj/panda/libfranka/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robproj/panda/libfranka/examples/MPC_openloop.cpp > CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.i

examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.s"
	cd /home/robproj/panda/libfranka/examples && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robproj/panda/libfranka/examples/MPC_openloop.cpp -o CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.s

examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.requires:

.PHONY : examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.requires

examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.provides: examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.requires
	$(MAKE) -f examples/CMakeFiles/MPC_openloop.dir/build.make examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.provides.build
.PHONY : examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.provides

examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.provides.build: examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o


# Object files for target MPC_openloop
MPC_openloop_OBJECTS = \
"CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o"

# External object files for target MPC_openloop
MPC_openloop_EXTERNAL_OBJECTS = \
"/home/robproj/panda/libfranka/examples/solver.o" \
"/home/robproj/panda/libfranka/examples/ldl.o" \
"/home/robproj/panda/libfranka/examples/util.o" \
"/home/robproj/panda/libfranka/examples/matrix_support.o"

examples/MPC_openloop: examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o
examples/MPC_openloop: examples/solver.o
examples/MPC_openloop: examples/ldl.o
examples/MPC_openloop: examples/util.o
examples/MPC_openloop: examples/matrix_support.o
examples/MPC_openloop: examples/CMakeFiles/MPC_openloop.dir/build.make
examples/MPC_openloop: examples/libexamples_common.a
examples/MPC_openloop: libfranka.so.0.7.1
examples/MPC_openloop: examples/CMakeFiles/MPC_openloop.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robproj/panda/libfranka/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable MPC_openloop"
	cd /home/robproj/panda/libfranka/examples && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/MPC_openloop.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
examples/CMakeFiles/MPC_openloop.dir/build: examples/MPC_openloop

.PHONY : examples/CMakeFiles/MPC_openloop.dir/build

examples/CMakeFiles/MPC_openloop.dir/requires: examples/CMakeFiles/MPC_openloop.dir/MPC_openloop.cpp.o.requires

.PHONY : examples/CMakeFiles/MPC_openloop.dir/requires

examples/CMakeFiles/MPC_openloop.dir/clean:
	cd /home/robproj/panda/libfranka/examples && $(CMAKE_COMMAND) -P CMakeFiles/MPC_openloop.dir/cmake_clean.cmake
.PHONY : examples/CMakeFiles/MPC_openloop.dir/clean

examples/CMakeFiles/MPC_openloop.dir/depend:
	cd /home/robproj/panda/libfranka && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robproj/panda/libfranka /home/robproj/panda/libfranka/examples /home/robproj/panda/libfranka /home/robproj/panda/libfranka/examples /home/robproj/panda/libfranka/examples/CMakeFiles/MPC_openloop.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : examples/CMakeFiles/MPC_openloop.dir/depend
