# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /opt/homebrew/Cellar/cmake/3.30.3/bin/cmake

# The command to remove a file.
RM = /opt/homebrew/Cellar/cmake/3.30.3/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build

# Include any dependencies generated for this target.
include CMakeFiles/SatelliteSim.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/SatelliteSim.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/SatelliteSim.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/SatelliteSim.dir/flags.make

CMakeFiles/SatelliteSim.dir/src/main.cpp.o: CMakeFiles/SatelliteSim.dir/flags.make
CMakeFiles/SatelliteSim.dir/src/main.cpp.o: /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/main.cpp
CMakeFiles/SatelliteSim.dir/src/main.cpp.o: CMakeFiles/SatelliteSim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/SatelliteSim.dir/src/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SatelliteSim.dir/src/main.cpp.o -MF CMakeFiles/SatelliteSim.dir/src/main.cpp.o.d -o CMakeFiles/SatelliteSim.dir/src/main.cpp.o -c /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/main.cpp

CMakeFiles/SatelliteSim.dir/src/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/SatelliteSim.dir/src/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/main.cpp > CMakeFiles/SatelliteSim.dir/src/main.cpp.i

CMakeFiles/SatelliteSim.dir/src/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/SatelliteSim.dir/src/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/main.cpp -o CMakeFiles/SatelliteSim.dir/src/main.cpp.s

CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o: CMakeFiles/SatelliteSim.dir/flags.make
CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o: /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Satellite.cpp
CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o: CMakeFiles/SatelliteSim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o -MF CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o.d -o CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o -c /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Satellite.cpp

CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Satellite.cpp > CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.i

CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Satellite.cpp -o CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.s

CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o: CMakeFiles/SatelliteSim.dir/flags.make
CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o: /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/LQRController.cpp
CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o: CMakeFiles/SatelliteSim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o -MF CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o.d -o CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o -c /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/LQRController.cpp

CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/LQRController.cpp > CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.i

CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/LQRController.cpp -o CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.s

CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o: CMakeFiles/SatelliteSim.dir/flags.make
CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o: /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Controller.cpp
CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o: CMakeFiles/SatelliteSim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o -MF CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o.d -o CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o -c /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Controller.cpp

CMakeFiles/SatelliteSim.dir/src/Controller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/SatelliteSim.dir/src/Controller.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Controller.cpp > CMakeFiles/SatelliteSim.dir/src/Controller.cpp.i

CMakeFiles/SatelliteSim.dir/src/Controller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/SatelliteSim.dir/src/Controller.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Controller.cpp -o CMakeFiles/SatelliteSim.dir/src/Controller.cpp.s

CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o: CMakeFiles/SatelliteSim.dir/flags.make
CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o: /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Quaternion.cpp
CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o: CMakeFiles/SatelliteSim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o -MF CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o.d -o CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o -c /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Quaternion.cpp

CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Quaternion.cpp > CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.i

CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/Quaternion.cpp -o CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.s

CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o: CMakeFiles/SatelliteSim.dir/flags.make
CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o: /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/ReactionWheel.cpp
CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o: CMakeFiles/SatelliteSim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o -MF CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o.d -o CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o -c /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/ReactionWheel.cpp

CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/ReactionWheel.cpp > CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.i

CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/ReactionWheel.cpp -o CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.s

CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o: CMakeFiles/SatelliteSim.dir/flags.make
CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o: /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/WheelController.cpp
CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o: CMakeFiles/SatelliteSim.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o -MF CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o.d -o CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o -c /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/WheelController.cpp

CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/WheelController.cpp > CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.i

CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/src/WheelController.cpp -o CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.s

# Object files for target SatelliteSim
SatelliteSim_OBJECTS = \
"CMakeFiles/SatelliteSim.dir/src/main.cpp.o" \
"CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o" \
"CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o" \
"CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o" \
"CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o" \
"CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o" \
"CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o"

# External object files for target SatelliteSim
SatelliteSim_EXTERNAL_OBJECTS =

SatelliteSim: CMakeFiles/SatelliteSim.dir/src/main.cpp.o
SatelliteSim: CMakeFiles/SatelliteSim.dir/src/Satellite.cpp.o
SatelliteSim: CMakeFiles/SatelliteSim.dir/src/LQRController.cpp.o
SatelliteSim: CMakeFiles/SatelliteSim.dir/src/Controller.cpp.o
SatelliteSim: CMakeFiles/SatelliteSim.dir/src/Quaternion.cpp.o
SatelliteSim: CMakeFiles/SatelliteSim.dir/src/ReactionWheel.cpp.o
SatelliteSim: CMakeFiles/SatelliteSim.dir/src/WheelController.cpp.o
SatelliteSim: CMakeFiles/SatelliteSim.dir/build.make
SatelliteSim: CMakeFiles/SatelliteSim.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX executable SatelliteSim"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SatelliteSim.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/SatelliteSim.dir/build: SatelliteSim
.PHONY : CMakeFiles/SatelliteSim.dir/build

CMakeFiles/SatelliteSim.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/SatelliteSim.dir/cmake_clean.cmake
.PHONY : CMakeFiles/SatelliteSim.dir/clean

CMakeFiles/SatelliteSim.dir/depend:
	cd /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build /Users/agastyakrothapalli/Documents/Coding/Satellite-Attitude-Controller/build/CMakeFiles/SatelliteSim.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/SatelliteSim.dir/depend

