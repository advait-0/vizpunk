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
include esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/adc_oneshot.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj -MF CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/adc_oneshot.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/adc_oneshot.c > CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/adc_oneshot.c -o CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/adc_common.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj -MF CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/adc_common.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/adc_common.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/adc_common.c > CMakeFiles/__idf_esp_adc.dir/adc_common.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/adc_common.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/adc_common.c -o CMakeFiles/__idf_esp_adc.dir/adc_common.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj -MF CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/adc_cali.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali.c > CMakeFiles/__idf_esp_adc.dir/adc_cali.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/adc_cali.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali.c -o CMakeFiles/__idf_esp_adc.dir/adc_cali.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali_curve_fitting.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj -MF CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali_curve_fitting.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali_curve_fitting.c > CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/adc_cali_curve_fitting.c -o CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp_adc_cal_common_legacy.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj -MF CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp_adc_cal_common_legacy.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp_adc_cal_common_legacy.c > CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp_adc_cal_common_legacy.c -o CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/adc_continuous.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj -MF CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/adc_continuous.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/adc_continuous.c > CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/adc_continuous.c -o CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_dma.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj -MF CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_dma.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_dma.c > CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_dma.c -o CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_cali_line_fitting.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj -MF CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_cali_line_fitting.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_cali_line_fitting.c > CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/esp32/adc_cali_line_fitting.c -o CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.s

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/flags.make
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj: /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp32/esp_adc_cal_legacy.c
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building C object esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj -MF CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj.d -o CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj -c /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp32/esp_adc_cal_legacy.c

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp32/esp_adc_cal_legacy.c > CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.i

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/esp_adc/deprecated/esp32/esp_adc_cal_legacy.c -o CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.s

# Object files for target __idf_esp_adc
__idf_esp_adc_OBJECTS = \
"CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj" \
"CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj"

# External object files for target __idf_esp_adc
__idf_esp_adc_EXTERNAL_OBJECTS =

esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_oneshot.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_common.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_cali_curve_fitting.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp_adc_cal_common_legacy.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/adc_continuous.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_dma.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/esp32/adc_cali_line_fitting.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/deprecated/esp32/esp_adc_cal_legacy.c.obj
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/build.make
esp-idf/esp_adc/libesp_adc.a: esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking C static library libesp_adc.a"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && $(CMAKE_COMMAND) -P CMakeFiles/__idf_esp_adc.dir/cmake_clean_target.cmake
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_esp_adc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/build: esp-idf/esp_adc/libesp_adc.a
.PHONY : esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/build

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/clean:
	cd /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc && $(CMAKE_COMMAND) -P CMakeFiles/__idf_esp_adc.dir/cmake_clean.cmake
.PHONY : esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/clean

esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/depend:
	cd /home/lakshaya/esp/hello_world/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lakshaya/esp/hello_world /home/lakshaya/esp/esp-idf/components/esp_adc /home/lakshaya/esp/hello_world/build /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc /home/lakshaya/esp/hello_world/build/esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/esp_adc/CMakeFiles/__idf_esp_adc.dir/depend

