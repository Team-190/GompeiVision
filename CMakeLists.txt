cmake_minimum_required(VERSION 3.18)
project(GompeiVision VERSION 1.0 LANGUAGES CXX)

# Explicitly generate compile_commands.json for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# -----------------------------------------------------------------------------
# AUTOMATICALLY GENERATE versions.cmake
# This custom command runs the Python script to update versions.json and
# generate versions.cmake during the CMake configure phase.
# -----------------------------------------------------------------------------
set(VERSIONS_JSON_PATH "${CMAKE_SOURCE_DIR}/versions.json")
set(UPDATE_DEPS_SCRIPT "${CMAKE_SOURCE_DIR}/scripts/update_deps.py")
set(GENERATED_VERSIONS_CMAKE_PATH "${CMAKE_SOURCE_DIR}/third_party/versions.cmake")

# -----------------------------------------------------------------------------
# AUTOMATICALLY GENERATE versions.cmake (during CMake configure phase)
# This uses execute_process to run a CMake wrapper script, which in turn runs
# the Python script to update versions.json and generate versions.cmake.
# The wrapper script includes logic to only run the Python part when needed.
# -----------------------------------------------------------------------------
execute_process(
        COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_SOURCE_DIR}/scripts/run_update_deps_wrapper.cmake"
        # Ensure the working directory for this execute_process call is the project source directory.
        # This is important for the relative paths within run_update_deps_wrapper.cmake to work.
        WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
)

# Include the dependency fetching script. This will trigger the downloads
# and set up the library targets (e.g., wpiutil, ntcore).
add_subdirectory(third_party)
add_subdirectory(src)