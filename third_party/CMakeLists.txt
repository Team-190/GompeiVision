cmake_minimum_required(VERSION 3.15)
project(ThirdPartyIntegration)

# --- Find all third-party packages ---
find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(wpiutil REQUIRED)
find_package(wpinet REQUIRED)
find_package(cscore REQUIRED)
find_package(ntcore REQUIRED)
find_package(wpimath REQUIRED)

# --- Manually define OpenPnP-Capture as an imported target ---
# NOTE: Adjust paths if installed elsewhere
add_library(OpenPnP::Capture SHARED IMPORTED GLOBAL)

set_target_properties(OpenPnP::Capture PROPERTIES
        IMPORTED_LOCATION /usr/local/lib/libopenpnp-capture.so
        INTERFACE_INCLUDE_DIRECTORIES /usr/local/include
)

# --- Define the "super target" interface library ---
add_library(third_party_libs INTERFACE)

# --- Add Include Directories to the Interface ---
# This makes all includes available to anything that links third_party_libs
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
target_link_libraries(third_party_libs INTERFACE
        wpiutil
        wpinet
        cscore
        ntcore
        wpimath
        ${OpenCV_LIBS}
        apriltag::apriltag
        OpenPnP::Capture
)
