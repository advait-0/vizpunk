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
CMAKE_SOURCE_DIR = /home/lakshaya/esp/hello_world

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lakshaya/esp/hello_world/build

# Include any dependencies generated for this target.
include esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/flags.make

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/flags.make
esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj: /home/lakshaya/esp/esp-idf/components/esp_driver_cam/esp_cam_ctlr.c
esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj -MF CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj.d -o CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_driver_cam/esp_cam_ctlr.c

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_driver_cam/esp_cam_ctlr.c > CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.i

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_driver_cam/esp_cam_ctlr.c -o CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.s

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/flags.make
esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj: /home/lakshaya/esp/esp-idf/components/esp_driver_cam/dvp_share_ctrl.c
esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj -MF CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj.d -o CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_driver_cam/dvp_share_ctrl.c

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_driver_cam/dvp_share_ctrl.c > CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.i

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_driver_cam/dvp_share_ctrl.c -o CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.s

# Object files for target __idf_esp_driver_cam
__idf_esp_driver_cam_OBJECTS = \
"CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj" \
"CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj"

# External object files for target __idf_esp_driver_cam
__idf_esp_driver_cam_EXTERNAL_OBJECTS =

esp-idf/esp_driver_cam/libesp_driver_cam.a: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/esp_cam_ctlr.c.obj
esp-idf/esp_driver_cam/libesp_driver_cam.a: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/dvp_share_ctrl.c.obj
esp-idf/esp_driver_cam/libesp_driver_cam.a: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/build.make
esp-idf/esp_driver_cam/libesp_driver_cam.a: esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking C static library libesp_driver_cam.a"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && $(CMAKE_COMMAND) -P CMakeFiles/__idf_esp_driver_cam.dir/cmake_clean_target.cmake
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_esp_driver_cam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/build: esp-idf/esp_driver_cam/libesp_driver_cam.a
.PHONY : esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/build

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/clean:
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam && $(CMAKE_COMMAND) -P CMakeFiles/__idf_esp_driver_cam.dir/cmake_clean.cmake
.PHONY : esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/clean

esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/depend:
	cd /home/lakshaya/esp/hello_world/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lakshaya/esp/hello_world /home/lakshaya/esp/esp-idf/components/esp_driver_cam /home/lakshaya/esp/hello_world/build /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/esp_driver_cam/CMakeFiles/__idf_esp_driver_cam.dir/depend

