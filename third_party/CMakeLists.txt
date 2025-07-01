# Find all third-party packages
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(wpiutil REQUIRED)
find_package(wpinet REQUIRED)
find_package(cscore REQUIRED)
find_package(ntcore REQUIRED)
find_package(wpimath REQUIRED)

# Define the "super target" interface library
add_library(third_party_libs INTERFACE)

# --- Add Include Directories to the Interface ---
# This is the crucial step. It tells any target that links against
# 'third_party_libs' where to find the header files for all dependencies.
# The find_package commands populate the <PackageName>_INCLUDE_DIRS variables.
target_include_directories(third_party_libs INTERFACE
        ${OpenCV_INCLUDE_DIRS}
        ${apriltag_INCLUDE_DIRS}
        ${wpiutil_INCLUDE_DIRS}
        ${wpinet_INCLUDE_DIRS}
        ${cscore_INCLUDE_DIRS}
        ${ntcore_INCLUDE_DIRS}
        ${wpimath_INCLUDE_DIRS}
)

# --- Link Libraries to the Interface ---
# This tells any target that links against 'third_party_libs' which
# libraries to link against.
target_link_libraries(third_party_libs INTERFACE
        wpiutil
        wpinet
        cscore
        ntcore
        wpimath
        ${OpenCV_LIBS}
        apriltag::apriltag
)
