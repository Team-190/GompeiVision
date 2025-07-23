cmake_minimum_required(VERSION 3.15)
project(ThirdPartyIntegration)

# --- Find all third-party packages ---
list(APPEND CMAKE_PREFIX_PATH "/usr/local" "/usr")

find_package(OpenCV REQUIRED)
find_package(apriltag REQUIRED)
find_package(wpiutil REQUIRED)
find_package(wpinet REQUIRED)
find_package(cscore REQUIRED)
find_package(cameraserver REQUIRED)
find_package(ntcore REQUIRED)
find_package(wpimath REQUIRED)
find_package(fieldimages REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(udev REQUIRED libudev)
pkg_check_modules(zstd REQUIRED libzstd)
find_package(httplib REQUIRED)

# Manually create an imported target for udev so we can use it like the others.
add_library(udev::udev INTERFACE IMPORTED)
target_include_directories(udev::udev INTERFACE ${udev_INCLUDE_DIRS})
target_link_libraries(udev::udev INTERFACE ${udev_LIBRARIES})

add_library(zstd::libzstd INTERFACE IMPORTED)
target_include_directories(zstd::libzstd INTERFACE ${ZSTD_INCLUDE_DIRS})
target_link_libraries(zstd::libzstd INTERFACE ${ZSTD_LIBRARIES})

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
        ${cameraserver_INCLUDE_DIRS}
        ${ntcore_INCLUDE_DIRS}
        ${wpimath_INCLUDE_DIRS}
        ${httplib_INCLUDE_DIRS}
        ${udev_INCLUDE_DIRS}
        /usr/local/include
        /usr/local/include/fieldImages
        ${openpnpcapture_INCLUDE_DIRS}
)

# --- Link Libraries to the Interface ---
target_link_libraries(third_party_libs INTERFACE
        ${OpenCV_LIBS}
        apriltag::apriltag
        wpiutil
        wpinet
        cscore
        cameraserver
        ntcore
        wpimath
        fieldImages
        httplib::httplib
        udev::udev
        OpenPnP::Capture
)
