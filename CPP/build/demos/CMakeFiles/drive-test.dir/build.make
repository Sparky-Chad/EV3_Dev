# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_SOURCE_DIR = /home/lapech/Programming/Source_EV3/ev3dev-lang-cpp

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lapech/Programming/EV3_Dev/CPP/build

# Include any dependencies generated for this target.
include demos/CMakeFiles/drive-test.dir/depend.make

# Include the progress variables for this target.
include demos/CMakeFiles/drive-test.dir/progress.make

# Include the compile flags for this target's objects.
include demos/CMakeFiles/drive-test.dir/flags.make

demos/CMakeFiles/drive-test.dir/drive-test.cpp.o: demos/CMakeFiles/drive-test.dir/flags.make
demos/CMakeFiles/drive-test.dir/drive-test.cpp.o: /home/lapech/Programming/Source_EV3/ev3dev-lang-cpp/demos/drive-test.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lapech/Programming/EV3_Dev/CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object demos/CMakeFiles/drive-test.dir/drive-test.cpp.o"
	cd /home/lapech/Programming/EV3_Dev/CPP/build/demos && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/drive-test.dir/drive-test.cpp.o -c /home/lapech/Programming/Source_EV3/ev3dev-lang-cpp/demos/drive-test.cpp

demos/CMakeFiles/drive-test.dir/drive-test.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/drive-test.dir/drive-test.cpp.i"
	cd /home/lapech/Programming/EV3_Dev/CPP/build/demos && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lapech/Programming/Source_EV3/ev3dev-lang-cpp/demos/drive-test.cpp > CMakeFiles/drive-test.dir/drive-test.cpp.i

demos/CMakeFiles/drive-test.dir/drive-test.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/drive-test.dir/drive-test.cpp.s"
	cd /home/lapech/Programming/EV3_Dev/CPP/build/demos && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lapech/Programming/Source_EV3/ev3dev-lang-cpp/demos/drive-test.cpp -o CMakeFiles/drive-test.dir/drive-test.cpp.s

# Object files for target drive-test
drive__test_OBJECTS = \
"CMakeFiles/drive-test.dir/drive-test.cpp.o"

# External object files for target drive-test
drive__test_EXTERNAL_OBJECTS =

demos/drive-test: demos/CMakeFiles/drive-test.dir/drive-test.cpp.o
demos/drive-test: demos/CMakeFiles/drive-test.dir/build.make
demos/drive-test: libev3dev.a
demos/drive-test: demos/CMakeFiles/drive-test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lapech/Programming/EV3_Dev/CPP/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable drive-test"
	cd /home/lapech/Programming/EV3_Dev/CPP/build/demos && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/drive-test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
demos/CMakeFiles/drive-test.dir/build: demos/drive-test

.PHONY : demos/CMakeFiles/drive-test.dir/build

demos/CMakeFiles/drive-test.dir/clean:
	cd /home/lapech/Programming/EV3_Dev/CPP/build/demos && $(CMAKE_COMMAND) -P CMakeFiles/drive-test.dir/cmake_clean.cmake
.PHONY : demos/CMakeFiles/drive-test.dir/clean

demos/CMakeFiles/drive-test.dir/depend:
	cd /home/lapech/Programming/EV3_Dev/CPP/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lapech/Programming/Source_EV3/ev3dev-lang-cpp /home/lapech/Programming/Source_EV3/ev3dev-lang-cpp/demos /home/lapech/Programming/EV3_Dev/CPP/build /home/lapech/Programming/EV3_Dev/CPP/build/demos /home/lapech/Programming/EV3_Dev/CPP/build/demos/CMakeFiles/drive-test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : demos/CMakeFiles/drive-test.dir/depend
