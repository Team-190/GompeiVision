# Set package metadata
set(CPACK_PACKAGE_NAME "${PROJECT_NAME}")
set(CPACK_PACKAGE_VERSION "${PROJECT_VERSION}")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "GompeiVision - An FRC vision solution")
set(CPACK_PACKAGE_VENDOR "FRC 190")
set(CPACK_PACKAGE_CONTACT "ecscher84@gmail.com")

# Set the generator for Debian packages
set(CPACK_GENERATOR "DEB")

# Set Debian-specific package information
set(CPACK_DEBIAN_PACKAGE_MAINTAINER "${CPACK_PACKAGE_CONTACT}")
set(CPACK_DEBIAN_PACKAGE_SECTION "devel")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")

# --- Corrected Dependency Line ---
# This is the crucial part. It tells the final .deb package that it requires
# these other system packages to be installed in order to run.
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)
set(CPACK_DEBIAN_PACKAGE_DEPENDS "")  # Or omit entirely

# --- Architecture Detection ---
# Intelligently determine the Debian architecture based on the build platform
if (CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
elseif (CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
else ()
    # Fallback to the generic processor name if no match
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE ${CMAKE_SYSTEM_PROCESSOR})
endif ()

message(STATUS "CPack: Detected architecture ${CMAKE_SYSTEM_PROCESSOR}, setting package architecture to ${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")

# Include the main CPack module to enable packaging
include(CPack)
