# File: src/CMakeLists.txt
# This file compiles the main application and links against the unified
# 'third_party_libs' target.

file(GLOB_RECURSE VISION_SOURCES "cpp/*.cpp")

# Add all the found source files to your executable.
add_executable(GompeiVision ${VISION_SOURCES})

set_target_properties(GompeiVision PROPERTIES
        INSTALL_RPATH "$ORIGIN/../lib"
)

# This adds your project's own /include directory.
target_include_directories(GompeiVision PUBLIC
        "${CMAKE_SOURCE_DIR}/src/include"
)

# This links your executable against the single "super-target".
# All include paths and libraries from WPILib and OpenCV will be
# automatically propagated from this one target.
target_link_libraries(GompeiVision PRIVATE
        third_party_libs
)

install(TARGETS GompeiVision DESTINATION bin)

add_custom_command(
        TARGET GompeiVision
        POST_BUILD
        COMMAND ${CMAKE_CPACK_COMMAND}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Packaging project into a .deb file..."
        VERBATIM
)