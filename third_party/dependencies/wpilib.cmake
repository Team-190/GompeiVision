# File: third_party/wpilib.cmake
# This file handles the logic for fetching and configuring all third-party dependencies.

cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0135 NEW)

# The root CMakeLists.txt already ran the script to generate this file.
# Now, we just need to include it to get the version variables.
include(${CMAKE_SOURCE_DIR}/third_party/versions.cmake)

# Ensure the FetchContent module is available
include(FetchContent)

# --- Helper function to fetch a WPILib dependency from separate header/binary zips ---
function(fetch_wpilib_dependency lib_name)
    # --- Define separate variables for path vs. variable lookup ---
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(ARCH_FOR_VAR "X86_64")   # Uppercase for variable names
        set(ARCH_FOR_PATH "x86-64")  # Lowercase with hyphen for directory paths
    elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(ARCH_FOR_VAR "ARM64")
        set(ARCH_FOR_PATH "arm64")
    else ()
        message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif ()

    # Determine build type (Debug vs. Release)
    string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_UPPER)
    if (BUILD_TYPE_UPPER STREQUAL "DEBUG")
        set(BUILD "DEBUG")
    else ()
        set(BUILD "RELEASE") # Default to Release
    endif ()

    # Construct variable names using the UPPERCASE architecture string
    string(TOUPPER ${lib_name} LIB_NAME_UPPER)
    set(HEADERS_URL_VAR "${LIB_NAME_UPPER}_HEADERS_URL")
    set(HEADERS_HASH_VAR "${LIB_NAME_UPPER}_HEADERS_HASH")
    set(BIN_URL_VAR "${LIB_NAME_UPPER}_${ARCH_FOR_VAR}_BIN_${BUILD}_URL")
    set(BIN_HASH_VAR "${LIB_NAME_UPPER}_${ARCH_FOR_VAR}_BIN_${BUILD}_HASH")

    # --- Add a clear diagnostic message to see what CMake is doing ---
    message(STATUS "--> For [${lib_name}] in [${BUILD}] mode:")
    message(STATUS "    Searching for binary URL variable: [${BIN_URL_VAR}]")
    message(STATUS "    Value found: [${${BIN_URL_VAR}}]")
    # ---

    if (NOT DEFINED ${BIN_URL_VAR} OR "${${BIN_URL_VAR}}" STREQUAL "")
        message(FATAL_ERROR "The variable [${BIN_URL_VAR}] is empty! Please check your 'versions.cmake' file to ensure it is defined correctly.")
    endif ()

    # Fetch the header content
    FetchContent_Declare(
            ${lib_name}_headers URL ${${HEADERS_URL_VAR}} URL_HASH ${${HEADERS_HASH_VAR}})
    FetchContent_MakeAvailable(${lib_name}_headers)
    FetchContent_GetProperties(${lib_name}_headers SOURCE_DIR headers_source_dir)

    # Fetch the binary content
    FetchContent_Declare(
            ${lib_name}_bin URL ${${BIN_URL_VAR}} URL_HASH ${${BIN_HASH_VAR}})
    FetchContent_MakeAvailable(${lib_name}_bin)
    FetchContent_GetProperties(${lib_name}_bin SOURCE_DIR bin_source_dir)

    # Create a proper INTERFACE target
    if (NOT TARGET ${lib_name})
        add_library(${lib_name} INTERFACE)
        add_library(wpi::${lib_name} ALIAS ${lib_name})
    endif ()

    # Tell this target where to find the headers
    target_include_directories(${lib_name} INTERFACE "${headers_source_dir}")

    # Construct the FULL, correct path to the directory containing the .so files
    set(SHARED_LIB_DIR "${bin_source_dir}/linux/${ARCH_FOR_PATH}/shared")
    target_link_directories(${lib_name} INTERFACE "${SHARED_LIB_DIR}")

    # Tell this target which libraries to link against
    if (BUILD STREQUAL "DEBUG")
        target_link_libraries(${lib_name} INTERFACE ${lib_name}d) # Links lib<name>d.so
    else ()
        target_link_libraries(${lib_name} INTERFACE ${lib_name})  # Links lib<name>.so
    endif ()
endfunction()

# --- Fetch all WPILib dependencies by calling the helper function ---
# Order can be important for dependencies. Start with the base utilities.
fetch_wpilib_dependency(wpiutil)
fetch_wpilib_dependency(cscore)
fetch_wpilib_dependency(wpinet)
fetch_wpilib_dependency(ntcore)
fetch_wpilib_dependency(wpimath)