# Detect architecture and triplet
if (NOT DEFINED VCPKG_TARGET_TRIPLET)
    execute_process(
            COMMAND uname -m
            OUTPUT_VARIABLE ARCH_RAW
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
    if (ARCH_RAW STREQUAL "x86_64")
        set(VCPKG_TARGET_TRIPLET "x64-linux" CACHE STRING "")
    elseif (ARCH_RAW STREQUAL "aarch64")
        set(VCPKG_TARGET_TRIPLET "arm64-linux" CACHE STRING "")
    else ()
        message(FATAL_ERROR "Unsupported architecture: ${ARCH_RAW}")
    endif ()
endif ()

# Bootstrap vcpkg if needed
set(VCPKG_DIR "${CMAKE_SOURCE_DIR}/vcpkg")
if (NOT EXISTS "${VCPKG_DIR}/vcpkg")
    message(STATUS "Bootstrapping vcpkg...")
    execute_process(
            COMMAND ${CMAKE_COMMAND} -E chdir ${VCPKG_DIR} ./bootstrap-vcpkg.sh
            RESULT_VARIABLE BOOTSTRAP_RESULT
    )
    if (NOT BOOTSTRAP_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to bootstrap vcpkg.")
    endif ()
endif ()

# Install dependencies if needed
message(STATUS "Installing vcpkg dependencies...")
execute_process(
        COMMAND ${VCPKG_DIR}/vcpkg install
        --triplet ${VCPKG_TARGET_TRIPLET}
        --x-manifest-root=${CMAKE_SOURCE_DIR}
        RESULT_VARIABLE INSTALL_RESULT
)
if (NOT INSTALL_RESULT EQUAL 0)
    message(FATAL_ERROR "vcpkg install failed.")
endif ()

# Now include WPILib dependencies
include(${CMAKE_SOURCE_DIR}/third_party/dependencies/wpilib.cmake)

# Find OpenCV, AprilTag, etc
find_package(OpenCV REQUIRED)
find_package(apriltag CONFIG) # optional

# Define the "super target" interface library
add_library(third_party_libs INTERFACE)

target_link_libraries(third_party_libs INTERFACE
        wpi::wpiutil
        wpi::ntcore
        wpi::cscore
        wpi::wpimath
        wpi::wpinet
        ${OpenCV_LIBS}
        apriltag::apriltag
)
