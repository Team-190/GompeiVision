cmake_minimum_required(VERSION 3.18)
project(GompeiVision VERSION 1.0 LANGUAGES CXX)

# Explicitly generate compile_commands.json for IDEs
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


# Include the dependency fetching script. This will trigger the downloads
# and set up the library targets (e.g., wpiutil, ntcore).
add_subdirectory(third_party)
add_subdirectory(src)