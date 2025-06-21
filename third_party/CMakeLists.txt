# File: third_party/CMakeLists.txt
# This file is the single entry point for all third-party dependencies.

# 1. Include the scripts that define how to get our dependencies.
#    This makes the wpi::* targets and the target_link_opencv() function available.
include(${CMAKE_SOURCE_DIR}/third_party/dependencies/wpilib.cmake)
include(${CMAKE_SOURCE_DIR}/third_party/dependencies/opencv.cmake) # This is the file from the immersive artifact

# 2. Create a single INTERFACE target to represent ALL third-party libraries.
#    This target doesn't compile anything; it just bundles other targets.
add_library(third_party_libs INTERFACE)

# 3. Link all the individual dependency targets to our "super-target".
#    This transfers all their properties (include paths, library locations, etc.)
#    to our single 'third_party_libs' target.
target_link_libraries(third_party_libs INTERFACE
        # WPILib Targets
        wpi::wpiutil
        wpi::ntcore
        wpi::cscore
        wpi::wpimath
        wpi::wpinet
)

# 4. Use our custom function to configure OpenCV and link it to our super-target.
#    This will download/build OpenCV (if needed) and add it to the bundle.
target_link_opencv(third_party_libs INTERFACE)

message(STATUS "All third-party dependencies have been bundled into the 'third_party_libs' target.")
