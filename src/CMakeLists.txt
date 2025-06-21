# File: src/CMakeLists.txt
# This file compiles the main application and links against the unified
# 'third_party_libs' target.

add_executable(gompeivision cpp/main.cpp)

# This adds your project's own /include directory.
target_include_directories(gompeivision PUBLIC
        "${CMAKE_SOURCE_DIR}/include"
)

# This links your executable against the single "super-target".
# All include paths and libraries from WPILib and OpenCV will be
# automatically propagated from this one target.
target_link_libraries(gompeivision PRIVATE
        third_party_libs
)