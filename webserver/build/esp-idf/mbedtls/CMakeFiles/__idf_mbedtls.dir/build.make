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
CMAKE_SOURCE_DIR = /home/lakshaya/vizpunk/webserver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lakshaya/vizpunk/webserver/build

# Include any dependencies generated for this target.
include esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/flags.make

x509_crt_bundle.S: /home/lakshaya/esp/esp-idf/tools/cmake/scripts/data_file_embed_asm.cmake
x509_crt_bundle.S: esp-idf/mbedtls/x509_crt_bundle
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/lakshaya/vizpunk/webserver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating ../../x509_crt_bundle.S"
	/usr/bin/cmake -D DATA_FILE=/home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls/x509_crt_bundle -D SOURCE_FILE=/home/lakshaya/vizpunk/webserver/build/x509_crt_bundle.S -D FILE_TYPE=BINARY -P /home/lakshaya/esp/esp-idf/tools/cmake/scripts/data_file_embed_asm.cmake

esp-idf/mbedtls/x509_crt_bundle:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/lakshaya/vizpunk/webserver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating x509_crt_bundle"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && /home/lakshaya/.espressif/python_env/idf5.4_py3.10_env/bin/python /home/lakshaya/esp/esp-idf/components/mbedtls/esp_crt_bundle/gen_crt_bundle.py --input /home/lakshaya/esp/esp-idf/components/mbedtls/esp_crt_bundle/cacrt_all.pem /home/lakshaya/esp/esp-idf/components/mbedtls/esp_crt_bundle/cacrt_local.pem -q --max-certs 200

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj: esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/flags.make
esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj: /home/lakshaya/esp/esp-idf/components/mbedtls/esp_crt_bundle/esp_crt_bundle.c
esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj: esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/vizpunk/webserver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj -MF CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj.d -o CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj -c /home/lakshaya/esp/esp-idf/components/mbedtls/esp_crt_bundle/esp_crt_bundle.c

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.i"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/mbedtls/esp_crt_bundle/esp_crt_bundle.c > CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.i

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.s"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/mbedtls/esp_crt_bundle/esp_crt_bundle.c -o CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.s

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.obj: esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/flags.make
esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.obj: x509_crt_bundle.S
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/vizpunk/webserver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building ASM object esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.obj"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -o CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.obj -c /home/lakshaya/vizpunk/webserver/build/x509_crt_bundle.S

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing ASM source to CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.i"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -E /home/lakshaya/vizpunk/webserver/build/x509_crt_bundle.S > CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.i

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling ASM source to assembly CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.s"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(ASM_DEFINES) $(ASM_INCLUDES) $(ASM_FLAGS) -S /home/lakshaya/vizpunk/webserver/build/x509_crt_bundle.S -o CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.s

# Object files for target __idf_mbedtls
__idf_mbedtls_OBJECTS = \
"CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj" \
"CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.obj"

# External object files for target __idf_mbedtls
__idf_mbedtls_EXTERNAL_OBJECTS =

esp-idf/mbedtls/libmbedtls.a: esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/esp_crt_bundle/esp_crt_bundle.c.obj
esp-idf/mbedtls/libmbedtls.a: esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/__/__/x509_crt_bundle.S.obj
esp-idf/mbedtls/libmbedtls.a: esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/build.make
esp-idf/mbedtls/libmbedtls.a: esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lakshaya/vizpunk/webserver/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Linking C static library libmbedtls.a"
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && $(CMAKE_COMMAND) -P CMakeFiles/__idf_mbedtls.dir/cmake_clean_target.cmake
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_mbedtls.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/build: esp-idf/mbedtls/libmbedtls.a
.PHONY : esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/build

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/clean:
	cd /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls && $(CMAKE_COMMAND) -P CMakeFiles/__idf_mbedtls.dir/cmake_clean.cmake
.PHONY : esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/clean

esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/depend: esp-idf/mbedtls/x509_crt_bundle
esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/depend: x509_crt_bundle.S
	cd /home/lakshaya/vizpunk/webserver/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lakshaya/vizpunk/webserver /home/lakshaya/esp/esp-idf/components/mbedtls /home/lakshaya/vizpunk/webserver/build /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls /home/lakshaya/vizpunk/webserver/build/esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/mbedtls/CMakeFiles/__idf_mbedtls.dir/depend

