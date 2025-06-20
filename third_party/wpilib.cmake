# File: cmake/dependencies.cmake
# This file handles the logic for fetching third-party dependencies.

cmake_minimum_required(VERSION 3.15)

cmake_policy(SET CMP0135 NEW)

# Load the dependency version definitions from the separate file.
include(${CMAKE_SOURCE_DIR}/third_party/versions.cmake)

# Ensure the FetchContent module is available.
include(FetchContent)

# --- Helper function to fetch a WPILib dependency ---
function(fetch_wpilib_dependency lib_name)
    # Determine architecture
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
        set(ARCH "X86_64")
    elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
        set(ARCH "ARM64")
    else ()
        message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
    endif ()

    # Determine build type to select the correct zip file (debug vs release)
    string(TOUPPER "${CMAKE_BUILD_TYPE}" BUILD_TYPE_UPPER)
    if (BUILD_TYPE_UPPER STREQUAL "DEBUG")
        set(BUILD "DEBUG")
    else ()
        set(BUILD "RELEASE")
    endif ()

    # Dynamically construct the variable names to look up from versions.cmake
    string(TOUPPER ${lib_name} LIB_NAME_UPPER)
    set(HEADERS_URL_VAR "${LIB_NAME_UPPER}_HEADERS_URL")
    set(HEADERS_HASH_VAR "${LIB_NAME_UPPER}_HEADERS_HASH")
    set(BIN_URL_VAR "${LIB_NAME_UPPER}_${ARCH}_BIN_${BUILD}_URL")
    set(BIN_HASH_VAR "${LIB_NAME_UPPER}_${ARCH}_BIN_${BUILD}_HASH")

    # Fetch the header and binary content
    FetchContent_Declare(
            ${lib_name}_headers URL ${${HEADERS_URL_VAR}} URL_HASH ${${HEADERS_HASH_VAR}})
    FetchContent_MakeAvailable(${lib_name}_headers)

    FetchContent_Declare(
            ${lib_name}_bin URL ${${BIN_URL_VAR}} URL_HASH ${${BIN_HASH_VAR}})
    FetchContent_MakeAvailable(${lib_name}_bin)

    # --- START OF CORRECTION BASED ON SCREENSHOT ---

    # Set the arch directory name, accounting for 'x86-64' vs 'x86_64'
    set(ARCH_DIR ${CMAKE_SYSTEM_PROCESSOR})
    if (ARCH_DIR STREQUAL "x86_64")
        set(ARCH_DIR "x86-64") # Match the directory name from the screenshot
    endif ()

    # Determine the library filename based on the build type.
    # The screenshot confirms the debug library is named 'lib<name>d.so'.
    if (BUILD STREQUAL "DEBUG")
        set(LIB_FILE_NAME "lib${lib_name}d.so")
    else ()
        # Assume the release library does not have the 'd' suffix.
        set(LIB_FILE_NAME "lib${lib_name}.so")
    endif ()

    # Construct the full, correct path to the library file.
    set(LIB_FULL_PATH "${${lib_name}_bin_SOURCE_DIR}/linux/${ARCH_DIR}/shared/${LIB_FILE_NAME}")

    # Create an imported library pointing to the correct file location.
    add_library(${lib_name}_imported SHARED IMPORTED)
    set_target_properties(${lib_name}_imported PROPERTIES
            IMPORTED_LOCATION "${LIB_FULL_PATH}"
    )

    # --- END OF CORRECTION ---

    # Create a top-level interface library that other targets can link against.
    add_library(${lib_name} INTERFACE)
    target_include_directories(${lib_name} INTERFACE "${${lib_name}_headers_SOURCE_DIR}")
    target_link_libraries(${lib_name} INTERFACE ${lib_name}_imported)

    # Create a modern CMake alias for easier linking
    add_library(wpi::${lib_name} ALIAS ${lib_name})

endfunction()

# --- Fetch all WPILib dependencies by calling the helper function ---
fetch_wpilib_dependency(wpiutil)
fetch_wpilib_dependency(wpinet)
fetch_wpilib_dependency(ntcore)
fetch_wpilib_dependency(cscore)
fetch_wpilib_dependency(wpimath)
