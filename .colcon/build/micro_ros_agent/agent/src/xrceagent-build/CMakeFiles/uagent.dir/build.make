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
CMAKE_SOURCE_DIR = /overlay_ws/build/micro_ros_agent/agent/src/xrceagent

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build

# Utility rule file for uagent.

# Include any custom commands dependencies for this target.
include CMakeFiles/uagent.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/uagent.dir/progress.make

CMakeFiles/uagent: CMakeFiles/uagent-complete

CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-install
CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-mkdir
CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-download
CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-update
CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-patch
CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-configure
CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-build
CMakeFiles/uagent-complete: uagent-prefix/src/uagent-stamp/uagent-install
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Completed 'uagent'"
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles/uagent-complete
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-done

uagent-prefix/src/uagent-stamp/uagent-build: uagent-prefix/src/uagent-stamp/uagent-configure
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Performing build step for 'uagent'"
	$(MAKE)
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-build

uagent-prefix/src/uagent-stamp/uagent-configure: uagent-prefix/tmp/uagent-cfgcmd.txt
uagent-prefix/src/uagent-stamp/uagent-configure: uagent-prefix/tmp/uagent-cache-.cmake
uagent-prefix/src/uagent-stamp/uagent-configure: uagent-prefix/src/uagent-stamp/uagent-patch
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Performing configure step for 'uagent'"
	/usr/bin/cmake "-GUnix Makefiles" -C/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/tmp/uagent-cache-.cmake /overlay_ws/build/micro_ros_agent/agent/src/xrceagent
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-configure

uagent-prefix/src/uagent-stamp/uagent-download: uagent-prefix/src/uagent-stamp/uagent-mkdir
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "No download step for 'uagent'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-download

uagent-prefix/src/uagent-stamp/uagent-install: uagent-prefix/src/uagent-stamp/uagent-build
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "No install step for 'uagent'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-install

uagent-prefix/src/uagent-stamp/uagent-mkdir:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Creating directories for 'uagent'"
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/tmp
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src
	/usr/bin/cmake -E make_directory /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-mkdir

uagent-prefix/src/uagent-stamp/uagent-patch: uagent-prefix/src/uagent-stamp/uagent-update
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "No patch step for 'uagent'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-patch

uagent-prefix/src/uagent-stamp/uagent-update: uagent-prefix/src/uagent-stamp/uagent-download
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "No update step for 'uagent'"
	/usr/bin/cmake -E echo_append
	/usr/bin/cmake -E touch /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/uagent-prefix/src/uagent-stamp/uagent-update

uagent: CMakeFiles/uagent
uagent: CMakeFiles/uagent-complete
uagent: uagent-prefix/src/uagent-stamp/uagent-build
uagent: uagent-prefix/src/uagent-stamp/uagent-configure
uagent: uagent-prefix/src/uagent-stamp/uagent-download
uagent: uagent-prefix/src/uagent-stamp/uagent-install
uagent: uagent-prefix/src/uagent-stamp/uagent-mkdir
uagent: uagent-prefix/src/uagent-stamp/uagent-patch
uagent: uagent-prefix/src/uagent-stamp/uagent-update
uagent: CMakeFiles/uagent.dir/build.make
.PHONY : uagent

# Rule to build all files generated by this target.
CMakeFiles/uagent.dir/build: uagent
.PHONY : CMakeFiles/uagent.dir/build

CMakeFiles/uagent.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/uagent.dir/cmake_clean.cmake
.PHONY : CMakeFiles/uagent.dir/clean

CMakeFiles/uagent.dir/depend:
	cd /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /overlay_ws/build/micro_ros_agent/agent/src/xrceagent /overlay_ws/build/micro_ros_agent/agent/src/xrceagent /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build /overlay_ws/build/micro_ros_agent/agent/src/xrceagent-build/CMakeFiles/uagent.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/uagent.dir/depend

