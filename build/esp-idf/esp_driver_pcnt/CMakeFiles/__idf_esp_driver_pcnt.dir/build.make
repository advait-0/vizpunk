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
include esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/flags.make

esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj: esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/flags.make
esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj: /home/lakshaya/esp/esp-idf/components/esp_driver_pcnt/src/pulse_cnt.c
esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj: esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj -MF CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj.d -o CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_driver_pcnt/src/pulse_cnt.c

esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_driver_pcnt/src/pulse_cnt.c > CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.i

esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_driver_pcnt/src/pulse_cnt.c -o CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.s

# Object files for target __idf_esp_driver_pcnt
__idf_esp_driver_pcnt_OBJECTS = \
"CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj"

# External object files for target __idf_esp_driver_pcnt
__idf_esp_driver_pcnt_EXTERNAL_OBJECTS =

esp-idf/esp_driver_pcnt/libesp_driver_pcnt.a: esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/src/pulse_cnt.c.obj
esp-idf/esp_driver_pcnt/libesp_driver_pcnt.a: esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/build.make
esp-idf/esp_driver_pcnt/libesp_driver_pcnt.a: esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking C static library libesp_driver_pcnt.a"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt && $(CMAKE_COMMAND) -P CMakeFiles/__idf_esp_driver_pcnt.dir/cmake_clean_target.cmake
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_esp_driver_pcnt.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/build: esp-idf/esp_driver_pcnt/libesp_driver_pcnt.a
.PHONY : esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/build

esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/clean:
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt && $(CMAKE_COMMAND) -P CMakeFiles/__idf_esp_driver_pcnt.dir/cmake_clean.cmake
.PHONY : esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/clean

esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/depend:
	cd /home/lakshaya/esp/hello_world/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lakshaya/esp/hello_world /home/lakshaya/esp/esp-idf/components/esp_driver_pcnt /home/lakshaya/esp/hello_world/build /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt /home/lakshaya/esp/hello_world/build/esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/esp_driver_pcnt/CMakeFiles/__idf_esp_driver_pcnt.dir/depend

