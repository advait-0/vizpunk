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
include esp-idf/pthread/CMakeFiles/__idf_pthread.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include esp-idf/pthread/CMakeFiles/__idf_pthread.dir/compiler_depend.make

# Include the progress variables for this target.
include esp-idf/pthread/CMakeFiles/__idf_pthread.dir/progress.make

# Include the compile flags for this target's objects.
include esp-idf/pthread/CMakeFiles/__idf_pthread.dir/flags.make

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/flags.make
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.obj: /home/lakshaya/esp/esp-idf/components/pthread/pthread.c
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building C object esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.obj -MF CMakeFiles/__idf_pthread.dir/pthread.c.obj.d -o CMakeFiles/__idf_pthread.dir/pthread.c.obj -c /home/lakshaya/esp/esp-idf/components/pthread/pthread.c

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_pthread.dir/pthread.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/pthread/pthread.c > CMakeFiles/__idf_pthread.dir/pthread.c.i

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_pthread.dir/pthread.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/pthread/pthread.c -o CMakeFiles/__idf_pthread.dir/pthread.c.s

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/flags.make
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj: /home/lakshaya/esp/esp-idf/components/pthread/pthread_cond_var.c
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building C object esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj -MF CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj.d -o CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj -c /home/lakshaya/esp/esp-idf/components/pthread/pthread_cond_var.c

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/pthread/pthread_cond_var.c > CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.i

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/pthread/pthread_cond_var.c -o CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.s

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/flags.make
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj: /home/lakshaya/esp/esp-idf/components/pthread/pthread_local_storage.c
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building C object esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj -MF CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj.d -o CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj -c /home/lakshaya/esp/esp-idf/components/pthread/pthread_local_storage.c

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/pthread/pthread_local_storage.c > CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.i

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/pthread/pthread_local_storage.c -o CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.s

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/flags.make
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj: /home/lakshaya/esp/esp-idf/components/pthread/pthread_rwlock.c
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building C object esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj -MF CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj.d -o CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj -c /home/lakshaya/esp/esp-idf/components/pthread/pthread_rwlock.c

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/pthread/pthread_rwlock.c > CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.i

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/pthread/pthread_rwlock.c -o CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.s

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/flags.make
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj: /home/lakshaya/esp/esp-idf/components/pthread/pthread_semaphore.c
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building C object esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -MD -MT esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj -MF CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj.d -o CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj -c /home/lakshaya/esp/esp-idf/components/pthread/pthread_semaphore.c

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.i"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -E /home/lakshaya/esp/esp-idf/components/pthread/pthread_semaphore.c > CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.i

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.s"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && /home/lakshaya/.espressif/tools/xtensa-esp-elf/esp-14.2.0_20241119/xtensa-esp-elf/bin/xtensa-esp32-elf-gcc $(C_DEFINES) $(C_INCLUDES) $(C_FLAGS) -S /home/lakshaya/esp/esp-idf/components/pthread/pthread_semaphore.c -o CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.s

# Object files for target __idf_pthread
__idf_pthread_OBJECTS = \
"CMakeFiles/__idf_pthread.dir/pthread.c.obj" \
"CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj" \
"CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj" \
"CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj" \
"CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj"

# External object files for target __idf_pthread
__idf_pthread_EXTERNAL_OBJECTS =

esp-idf/pthread/libpthread.a: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread.c.obj
esp-idf/pthread/libpthread.a: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_cond_var.c.obj
esp-idf/pthread/libpthread.a: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_local_storage.c.obj
esp-idf/pthread/libpthread.a: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_rwlock.c.obj
esp-idf/pthread/libpthread.a: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/pthread_semaphore.c.obj
esp-idf/pthread/libpthread.a: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/build.make
esp-idf/pthread/libpthread.a: esp-idf/pthread/CMakeFiles/__idf_pthread.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lakshaya/esp/hello_world/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking C static library libpthread.a"
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && $(CMAKE_COMMAND) -P CMakeFiles/__idf_pthread.dir/cmake_clean_target.cmake
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/__idf_pthread.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
esp-idf/pthread/CMakeFiles/__idf_pthread.dir/build: esp-idf/pthread/libpthread.a
.PHONY : esp-idf/pthread/CMakeFiles/__idf_pthread.dir/build

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/clean:
	cd /home/lakshaya/esp/hello_world/build/esp-idf/pthread && $(CMAKE_COMMAND) -P CMakeFiles/__idf_pthread.dir/cmake_clean.cmake
.PHONY : esp-idf/pthread/CMakeFiles/__idf_pthread.dir/clean

esp-idf/pthread/CMakeFiles/__idf_pthread.dir/depend:
	cd /home/lakshaya/esp/hello_world/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lakshaya/esp/hello_world /home/lakshaya/esp/esp-idf/components/pthread /home/lakshaya/esp/hello_world/build /home/lakshaya/esp/hello_world/build/esp-idf/pthread /home/lakshaya/esp/hello_world/build/esp-idf/pthread/CMakeFiles/__idf_pthread.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : esp-idf/pthread/CMakeFiles/__idf_pthread.dir/depend

