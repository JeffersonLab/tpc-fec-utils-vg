# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

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
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.15.1/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.15.1/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src

# Include any dependencies generated for this target.
include CMakeFiles/sampa_raw_data_decoder.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sampa_raw_data_decoder.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sampa_raw_data_decoder.dir/flags.make

CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.o: CMakeFiles/sampa_raw_data_decoder.dir/flags.make
CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.o: sampa_raw_data_decoder.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.o -c /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/sampa_raw_data_decoder.cpp

CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/sampa_raw_data_decoder.cpp > CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.i

CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/sampa_raw_data_decoder.cpp -o CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.s

# Object files for target sampa_raw_data_decoder
sampa_raw_data_decoder_OBJECTS = \
"CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.o"

# External object files for target sampa_raw_data_decoder
sampa_raw_data_decoder_EXTERNAL_OBJECTS =

sampa_raw_data_decoder: CMakeFiles/sampa_raw_data_decoder.dir/sampa_raw_data_decoder.o
sampa_raw_data_decoder: CMakeFiles/sampa_raw_data_decoder.dir/build.make
sampa_raw_data_decoder: CMakeFiles/sampa_raw_data_decoder.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sampa_raw_data_decoder"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sampa_raw_data_decoder.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sampa_raw_data_decoder.dir/build: sampa_raw_data_decoder

.PHONY : CMakeFiles/sampa_raw_data_decoder.dir/build

CMakeFiles/sampa_raw_data_decoder.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sampa_raw_data_decoder.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sampa_raw_data_decoder.dir/clean

CMakeFiles/sampa_raw_data_decoder.dir/depend:
	cd /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src /Users/gurjyan/Devel/Ersap/sampa/tpc-fec-utils/src/CMakeFiles/sampa_raw_data_decoder.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sampa_raw_data_decoder.dir/depend

