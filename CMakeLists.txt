cmake_minimum_required(VERSION 3.21)
project(GompeiVision VERSION 1.0 LANGUAGES CXX)

# Explicitly generate compile_commands.json for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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
        "/usr/lib/x86_64-linux-gnu/libswscale.so*"

)

install(FILES ${SYSTEM_LIBS}
        DESTINATION lib
)

# Set up packaging with CPack
include(packaging/CPackConfig.cmake)