# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.17

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
CMAKE_COMMAND = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake

# The command to remove a file.
RM = /Applications/CLion.app/Contents/bin/cmake/mac/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug

# Include any dependencies generated for this target.
include src/CMakeFiles/sampa_raw_data_decoder.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/sampa_raw_data_decoder.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/sampa_raw_data_decoder.dir/flags.make

src/CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.o: src/CMakeFiles/sampa_raw_data_decoder.dir/flags.make
src/CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.o: ../src/sampa_raw_data_decoder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.o"
	cd /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/src && /usr/local/Cellar/gcc/10.2.0/bin/gcc-10  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.o -c /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/sampa_raw_data_decoder.cpp

src/CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.i"
	cd /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/src && /usr/local/Cellar/gcc/10.2.0/bin/gcc-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/sampa_raw_data_decoder.cpp > CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.i

src/CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.s"
	cd /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/src && /usr/local/Cellar/gcc/10.2.0/bin/gcc-10 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/sampa_raw_data_decoder.cpp -o CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.s

# Object files for target sampa_raw_data_decoder
sampa_raw_data_decoder_OBJECTS = \
"CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.o"

# External object files for target sampa_raw_data_decoder
sampa_raw_data_decoder_EXTERNAL_OBJECTS =

src/sampa_raw_data_decoder: src/CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.cpp.o
src/sampa_raw_data_decoder: src/CMakeFiles/sampa_raw_data_decoder.dir/build.make
src/sampa_raw_data_decoder: src/CMakeFiles/sampa_raw_data_decoder.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sampa_raw_data_decoder"
	cd /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sampa_raw_data_decoder.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/sampa_raw_data_decoder.dir/build: src/sampa_raw_data_decoder

.PHONY : src/CMakeFiles/sampa_raw_data_decoder.dir/build

src/CMakeFiles/sampa_raw_data_decoder.dir/clean:
	cd /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/src && $(CMAKE_COMMAND) -P CMakeFiles/sampa_raw_data_decoder.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/sampa_raw_data_decoder.dir/clean

src/CMakeFiles/sampa_raw_data_decoder.dir/depend:
	cd /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/src /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/cmake-build-debug/src/CMakeFiles/sampa_raw_data_decoder.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/sampa_raw_data_decoder.dir/depend
