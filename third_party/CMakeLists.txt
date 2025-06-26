# Now include WPILib dependencies
include(${CMAKE_SOURCE_DIR}/third_party/dependencies/wpilib.cmake)

# Find OpenCV, AprilTag, etc
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)

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
