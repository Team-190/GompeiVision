cmake_minimum_required(VERSION 3.21)
project(GompeiVision VERSION 1.0 LANGUAGES CXX)

# --- Set Architecture-Specific Compiler Optimizations ---
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    # Apply optimizations for Intel x86_64 CPUs (like Alder Lake)
    message(STATUS "x86_64 architecture detected. Applying Raptor Lake optimizations (for intel 13th gen).")
    add_compile_options(-O3 -march=core-avx2)

elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    # Apply optimizations for 64-bit ARM CPUs (like Orange Pi)
    message(STATUS "aarch64 architecture detected. Applying ARMv8-a optimizations.")
    add_compile_options(-O3 -march=armv8-a)
endif()

# Explicitly generate compile_commands.json for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Check if the target architecture is x86_64
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    # Only on x86, we find OpenVINO and add a definition
    message(STATUS "x86_64 architecture detected, enabling OpenVINO.")
    find_package(OpenVINO REQUIRED)
    set(USE_OPENVINO ON)
    add_compile_definitions(USE_OPENVINO=1)
else()
    # On other architectures (like ARM), we do nothing
    message(STATUS "Non-x86 architecture detected, OpenVINO will be disabled.")
    set(USE_OPENVINO OFF)
    add_compile_definitions(USE_OPENVINO=0)
endif()

# Include the dependency fetching script. This will trigger the downloads
# and set up the library targets (e.g., wpiutil, ntcore).
add_subdirectory(third_party)
add_subdirectory(src)

install(DIRECTORY /usr/local/lib/
        DESTINATION lib
        FILES_MATCHING
        PATTERN "lib*.so*"
        PATTERN "cmake" EXCLUDE
        PATTERN "pkgconfig" EXCLUDE
        PATTERN "python*" EXCLUDE
)

# Find and install system libraries like libprotobuf that are not in /usr/local/lib
file(GLOB SYSTEM_LIBS
        LIST_DIRECTORIES false
        "/usr/lib/x86_64-linux-gnu/libprotobuf.so*"
        "/usr/lib/x86_64-linux-gnu/libavcodec.so.58"
        "/usr/lib/x86_64-linux-gnu/libavformat.so.58"
        "/usr/lib/x86_64-linux-gnu/libswscale.so.5"
)

install(FILES ${SYSTEM_LIBS}
        DESTINATION lib
)

# Install the models directory to the standard share location.
install(
    DIRECTORY models/
    DESTINATION /usr/local/share/GompeiVision
    FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
    DIRECTORY_PERMISSIONS OWNER_READ OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE
)

# Set up packaging with CPack
include(packaging/CPackConfig.cmake)