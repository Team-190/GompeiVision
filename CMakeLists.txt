cmake_minimum_required(VERSION 3.30)
project(GompeiVision VERSION 1.0.3 LANGUAGES CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 26)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

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

# Set up packaging with CPack
include(packaging/CPackConfig.cmake)