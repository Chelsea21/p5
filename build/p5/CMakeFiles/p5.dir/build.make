# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/chelsea/Documents/GraphicsProjects/p5/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/chelsea/Documents/GraphicsProjects/p5/build

# Include any dependencies generated for this target.
include p5/CMakeFiles/p5.dir/depend.make

# Include the progress variables for this target.
include p5/CMakeFiles/p5.dir/progress.make

# Include the compile flags for this target's objects.
include p5/CMakeFiles/p5.dir/flags.make

p5/CMakeFiles/p5.dir/collisions.cpp.o: p5/CMakeFiles/p5.dir/flags.make
p5/CMakeFiles/p5.dir/collisions.cpp.o: /home/chelsea/Documents/GraphicsProjects/p5/src/p5/collisions.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p5/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p5/CMakeFiles/p5.dir/collisions.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p5.dir/collisions.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p5/src/p5/collisions.cpp

p5/CMakeFiles/p5.dir/collisions.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p5.dir/collisions.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p5/src/p5/collisions.cpp > CMakeFiles/p5.dir/collisions.cpp.i

p5/CMakeFiles/p5.dir/collisions.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p5.dir/collisions.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p5/src/p5/collisions.cpp -o CMakeFiles/p5.dir/collisions.cpp.s

p5/CMakeFiles/p5.dir/collisions.cpp.o.requires:
.PHONY : p5/CMakeFiles/p5.dir/collisions.cpp.o.requires

p5/CMakeFiles/p5.dir/collisions.cpp.o.provides: p5/CMakeFiles/p5.dir/collisions.cpp.o.requires
	$(MAKE) -f p5/CMakeFiles/p5.dir/build.make p5/CMakeFiles/p5.dir/collisions.cpp.o.provides.build
.PHONY : p5/CMakeFiles/p5.dir/collisions.cpp.o.provides

p5/CMakeFiles/p5.dir/collisions.cpp.o.provides.build: p5/CMakeFiles/p5.dir/collisions.cpp.o

p5/CMakeFiles/p5.dir/main.cpp.o: p5/CMakeFiles/p5.dir/flags.make
p5/CMakeFiles/p5.dir/main.cpp.o: /home/chelsea/Documents/GraphicsProjects/p5/src/p5/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p5/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p5/CMakeFiles/p5.dir/main.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p5.dir/main.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p5/src/p5/main.cpp

p5/CMakeFiles/p5.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p5.dir/main.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p5/src/p5/main.cpp > CMakeFiles/p5.dir/main.cpp.i

p5/CMakeFiles/p5.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p5.dir/main.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p5/src/p5/main.cpp -o CMakeFiles/p5.dir/main.cpp.s

p5/CMakeFiles/p5.dir/main.cpp.o.requires:
.PHONY : p5/CMakeFiles/p5.dir/main.cpp.o.requires

p5/CMakeFiles/p5.dir/main.cpp.o.provides: p5/CMakeFiles/p5.dir/main.cpp.o.requires
	$(MAKE) -f p5/CMakeFiles/p5.dir/build.make p5/CMakeFiles/p5.dir/main.cpp.o.provides.build
.PHONY : p5/CMakeFiles/p5.dir/main.cpp.o.provides

p5/CMakeFiles/p5.dir/main.cpp.o.provides.build: p5/CMakeFiles/p5.dir/main.cpp.o

p5/CMakeFiles/p5.dir/physics.cpp.o: p5/CMakeFiles/p5.dir/flags.make
p5/CMakeFiles/p5.dir/physics.cpp.o: /home/chelsea/Documents/GraphicsProjects/p5/src/p5/physics.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p5/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p5/CMakeFiles/p5.dir/physics.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p5.dir/physics.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p5/src/p5/physics.cpp

p5/CMakeFiles/p5.dir/physics.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p5.dir/physics.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p5/src/p5/physics.cpp > CMakeFiles/p5.dir/physics.cpp.i

p5/CMakeFiles/p5.dir/physics.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p5.dir/physics.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p5/src/p5/physics.cpp -o CMakeFiles/p5.dir/physics.cpp.s

p5/CMakeFiles/p5.dir/physics.cpp.o.requires:
.PHONY : p5/CMakeFiles/p5.dir/physics.cpp.o.requires

p5/CMakeFiles/p5.dir/physics.cpp.o.provides: p5/CMakeFiles/p5.dir/physics.cpp.o.requires
	$(MAKE) -f p5/CMakeFiles/p5.dir/build.make p5/CMakeFiles/p5.dir/physics.cpp.o.provides.build
.PHONY : p5/CMakeFiles/p5.dir/physics.cpp.o.provides

p5/CMakeFiles/p5.dir/physics.cpp.o.provides.build: p5/CMakeFiles/p5.dir/physics.cpp.o

p5/CMakeFiles/p5.dir/planebody.cpp.o: p5/CMakeFiles/p5.dir/flags.make
p5/CMakeFiles/p5.dir/planebody.cpp.o: /home/chelsea/Documents/GraphicsProjects/p5/src/p5/planebody.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p5/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p5/CMakeFiles/p5.dir/planebody.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p5.dir/planebody.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p5/src/p5/planebody.cpp

p5/CMakeFiles/p5.dir/planebody.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p5.dir/planebody.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p5/src/p5/planebody.cpp > CMakeFiles/p5.dir/planebody.cpp.i

p5/CMakeFiles/p5.dir/planebody.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p5.dir/planebody.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p5/src/p5/planebody.cpp -o CMakeFiles/p5.dir/planebody.cpp.s

p5/CMakeFiles/p5.dir/planebody.cpp.o.requires:
.PHONY : p5/CMakeFiles/p5.dir/planebody.cpp.o.requires

p5/CMakeFiles/p5.dir/planebody.cpp.o.provides: p5/CMakeFiles/p5.dir/planebody.cpp.o.requires
	$(MAKE) -f p5/CMakeFiles/p5.dir/build.make p5/CMakeFiles/p5.dir/planebody.cpp.o.provides.build
.PHONY : p5/CMakeFiles/p5.dir/planebody.cpp.o.provides

p5/CMakeFiles/p5.dir/planebody.cpp.o.provides.build: p5/CMakeFiles/p5.dir/planebody.cpp.o

p5/CMakeFiles/p5.dir/spherebody.cpp.o: p5/CMakeFiles/p5.dir/flags.make
p5/CMakeFiles/p5.dir/spherebody.cpp.o: /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spherebody.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p5/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p5/CMakeFiles/p5.dir/spherebody.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p5.dir/spherebody.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spherebody.cpp

p5/CMakeFiles/p5.dir/spherebody.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p5.dir/spherebody.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spherebody.cpp > CMakeFiles/p5.dir/spherebody.cpp.i

p5/CMakeFiles/p5.dir/spherebody.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p5.dir/spherebody.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spherebody.cpp -o CMakeFiles/p5.dir/spherebody.cpp.s

p5/CMakeFiles/p5.dir/spherebody.cpp.o.requires:
.PHONY : p5/CMakeFiles/p5.dir/spherebody.cpp.o.requires

p5/CMakeFiles/p5.dir/spherebody.cpp.o.provides: p5/CMakeFiles/p5.dir/spherebody.cpp.o.requires
	$(MAKE) -f p5/CMakeFiles/p5.dir/build.make p5/CMakeFiles/p5.dir/spherebody.cpp.o.provides.build
.PHONY : p5/CMakeFiles/p5.dir/spherebody.cpp.o.provides

p5/CMakeFiles/p5.dir/spherebody.cpp.o.provides.build: p5/CMakeFiles/p5.dir/spherebody.cpp.o

p5/CMakeFiles/p5.dir/spring.cpp.o: p5/CMakeFiles/p5.dir/flags.make
p5/CMakeFiles/p5.dir/spring.cpp.o: /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spring.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p5/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p5/CMakeFiles/p5.dir/spring.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p5.dir/spring.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spring.cpp

p5/CMakeFiles/p5.dir/spring.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p5.dir/spring.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spring.cpp > CMakeFiles/p5.dir/spring.cpp.i

p5/CMakeFiles/p5.dir/spring.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p5.dir/spring.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p5/src/p5/spring.cpp -o CMakeFiles/p5.dir/spring.cpp.s

p5/CMakeFiles/p5.dir/spring.cpp.o.requires:
.PHONY : p5/CMakeFiles/p5.dir/spring.cpp.o.requires

p5/CMakeFiles/p5.dir/spring.cpp.o.provides: p5/CMakeFiles/p5.dir/spring.cpp.o.requires
	$(MAKE) -f p5/CMakeFiles/p5.dir/build.make p5/CMakeFiles/p5.dir/spring.cpp.o.provides.build
.PHONY : p5/CMakeFiles/p5.dir/spring.cpp.o.provides

p5/CMakeFiles/p5.dir/spring.cpp.o.provides.build: p5/CMakeFiles/p5.dir/spring.cpp.o

p5/CMakeFiles/p5.dir/trianglebody.cpp.o: p5/CMakeFiles/p5.dir/flags.make
p5/CMakeFiles/p5.dir/trianglebody.cpp.o: /home/chelsea/Documents/GraphicsProjects/p5/src/p5/trianglebody.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/chelsea/Documents/GraphicsProjects/p5/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object p5/CMakeFiles/p5.dir/trianglebody.cpp.o"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/p5.dir/trianglebody.cpp.o -c /home/chelsea/Documents/GraphicsProjects/p5/src/p5/trianglebody.cpp

p5/CMakeFiles/p5.dir/trianglebody.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/p5.dir/trianglebody.cpp.i"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/chelsea/Documents/GraphicsProjects/p5/src/p5/trianglebody.cpp > CMakeFiles/p5.dir/trianglebody.cpp.i

p5/CMakeFiles/p5.dir/trianglebody.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/p5.dir/trianglebody.cpp.s"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/chelsea/Documents/GraphicsProjects/p5/src/p5/trianglebody.cpp -o CMakeFiles/p5.dir/trianglebody.cpp.s

p5/CMakeFiles/p5.dir/trianglebody.cpp.o.requires:
.PHONY : p5/CMakeFiles/p5.dir/trianglebody.cpp.o.requires

p5/CMakeFiles/p5.dir/trianglebody.cpp.o.provides: p5/CMakeFiles/p5.dir/trianglebody.cpp.o.requires
	$(MAKE) -f p5/CMakeFiles/p5.dir/build.make p5/CMakeFiles/p5.dir/trianglebody.cpp.o.provides.build
.PHONY : p5/CMakeFiles/p5.dir/trianglebody.cpp.o.provides

p5/CMakeFiles/p5.dir/trianglebody.cpp.o.provides.build: p5/CMakeFiles/p5.dir/trianglebody.cpp.o

# Object files for target p5
p5_OBJECTS = \
"CMakeFiles/p5.dir/collisions.cpp.o" \
"CMakeFiles/p5.dir/main.cpp.o" \
"CMakeFiles/p5.dir/physics.cpp.o" \
"CMakeFiles/p5.dir/planebody.cpp.o" \
"CMakeFiles/p5.dir/spherebody.cpp.o" \
"CMakeFiles/p5.dir/spring.cpp.o" \
"CMakeFiles/p5.dir/trianglebody.cpp.o"

# External object files for target p5
p5_EXTERNAL_OBJECTS =

p5/p5: p5/CMakeFiles/p5.dir/collisions.cpp.o
p5/p5: p5/CMakeFiles/p5.dir/main.cpp.o
p5/p5: p5/CMakeFiles/p5.dir/physics.cpp.o
p5/p5: p5/CMakeFiles/p5.dir/planebody.cpp.o
p5/p5: p5/CMakeFiles/p5.dir/spherebody.cpp.o
p5/p5: p5/CMakeFiles/p5.dir/spring.cpp.o
p5/p5: p5/CMakeFiles/p5.dir/trianglebody.cpp.o
p5/p5: application/libapplication.a
p5/p5: math/libmath.a
p5/p5: scene/libscene.a
p5/p5: tinyxml/libtinyxml.a
p5/p5: /usr/lib/i386-linux-gnu/libSDLmain.a
p5/p5: /usr/lib/i386-linux-gnu/libSDL.so
p5/p5: /usr/lib/i386-linux-gnu/libpng.so
p5/p5: /usr/lib/i386-linux-gnu/libz.so
p5/p5: /usr/lib/i386-linux-gnu/libGLU.so
p5/p5: /usr/lib/i386-linux-gnu/libGL.so
p5/p5: /usr/lib/i386-linux-gnu/libSM.so
p5/p5: /usr/lib/i386-linux-gnu/libICE.so
p5/p5: /usr/lib/i386-linux-gnu/libX11.so
p5/p5: /usr/lib/i386-linux-gnu/libXext.so
p5/p5: /usr/lib/i386-linux-gnu/libglut.so
p5/p5: /usr/lib/i386-linux-gnu/libXmu.so
p5/p5: /usr/lib/i386-linux-gnu/libXi.so
p5/p5: /usr/lib/i386-linux-gnu/libGLEW.so
p5/p5: p5/CMakeFiles/p5.dir/build.make
p5/p5: p5/CMakeFiles/p5.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable p5"
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/p5.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
p5/CMakeFiles/p5.dir/build: p5/p5
.PHONY : p5/CMakeFiles/p5.dir/build

p5/CMakeFiles/p5.dir/requires: p5/CMakeFiles/p5.dir/collisions.cpp.o.requires
p5/CMakeFiles/p5.dir/requires: p5/CMakeFiles/p5.dir/main.cpp.o.requires
p5/CMakeFiles/p5.dir/requires: p5/CMakeFiles/p5.dir/physics.cpp.o.requires
p5/CMakeFiles/p5.dir/requires: p5/CMakeFiles/p5.dir/planebody.cpp.o.requires
p5/CMakeFiles/p5.dir/requires: p5/CMakeFiles/p5.dir/spherebody.cpp.o.requires
p5/CMakeFiles/p5.dir/requires: p5/CMakeFiles/p5.dir/spring.cpp.o.requires
p5/CMakeFiles/p5.dir/requires: p5/CMakeFiles/p5.dir/trianglebody.cpp.o.requires
.PHONY : p5/CMakeFiles/p5.dir/requires

p5/CMakeFiles/p5.dir/clean:
	cd /home/chelsea/Documents/GraphicsProjects/p5/build/p5 && $(CMAKE_COMMAND) -P CMakeFiles/p5.dir/cmake_clean.cmake
.PHONY : p5/CMakeFiles/p5.dir/clean

p5/CMakeFiles/p5.dir/depend:
	cd /home/chelsea/Documents/GraphicsProjects/p5/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/chelsea/Documents/GraphicsProjects/p5/src /home/chelsea/Documents/GraphicsProjects/p5/src/p5 /home/chelsea/Documents/GraphicsProjects/p5/build /home/chelsea/Documents/GraphicsProjects/p5/build/p5 /home/chelsea/Documents/GraphicsProjects/p5/build/p5/CMakeFiles/p5.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : p5/CMakeFiles/p5.dir/depend

