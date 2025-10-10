# File: src/CMakeLists.txt
# This file defines and builds the two main executables for the project:
# 1. GompeiVision: The main manager application.
# 2. GompeiVisionProcess: The worker process that runs a single pipeline.

# --- Source File Definitions ---
# It's best practice to explicitly list sources instead of using GLOB.
# This ensures CMake correctly detects changes when files are added or removed.

# Sources required for the main manager application.
set(MANAGER_SOURCES
        cpp/main.cpp
        cpp/pipeline/PipelineManager.cpp
)

# Sources required for the pipeline worker executable.
set(WORKER_SOURCES
        cpp/capture/Camera.cpp
        cpp/detector/FiducialDetector.cpp
        cpp/detector/ObjectDetector.cpp
        cpp/estimator/CameraPoseEstimator.cpp
        cpp/estimator/SingleTagPoseEstimator.cpp
        cpp/estimator/TagAngleCalculator.cpp
        cpp/estimator/ObjectEstimator.cpp
        cpp/io/ConfigInterface.cpp
        cpp/io/FieldInterface.cpp
        cpp/io/OutputPublisher.cpp
        cpp/pipeline/Pipeline.cpp
        cpp/pipeline/PipelineHelper.cpp
        cpp/pipeline/PipelineProcess.cpp
)


# --- Executable Definitions ---
# Define the two separate executables for our multi-process architecture.
add_executable(GompeiVision ${MANAGER_SOURCES})
add_executable(GompeiVisionProcess ${WORKER_SOURCES})


# --- Target Properties and Linking ---
# Apply properties and link libraries to BOTH executables.

# Set C++ standard for both targets.
set_property(TARGET GompeiVision GompeiVisionProcess PROPERTY CXX_STANDARD 17)
set_property(TARGET GompeiVision GompeiVisionProcess PROPERTY CXX_STANDARD_REQUIRED ON)

# Set the RPATH so the executables can find shared libraries in ../lib
set_target_properties(GompeiVision GompeiVisionProcess PROPERTIES
        INSTALL_RPATH "$ORIGIN/../lib"
)

# --- FIX: Apply include directories to each target explicitly ---
# This ensures both executables know where to find the project headers.
target_include_directories(GompeiVision PUBLIC
        "${CMAKE_SOURCE_DIR}/src/include"
)
target_include_directories(GompeiVisionProcess PUBLIC
        "${CMAKE_SOURCE_DIR}/src/include"
)

# --- FIX: Link libraries to each target explicitly ---
# This is the correct and unambiguous way to link libraries to multiple targets.
target_link_libraries(GompeiVision PRIVATE
        third_party_libs
)
target_link_libraries(GompeiVisionProcess PRIVATE
        third_party_libs
)

if(USE_OPENVINO)
    find_package(OpenVINO REQUIRED)
    target_link_libraries(GompeiVisionProcess PRIVATE openvino::runtime)
endif()

# --- Installation ---
# CRITICAL: Install BOTH executables into the same 'bin' directory.
# This ensures the manager can find the worker using a relative path.
install(TARGETS GompeiVision GompeiVisionProcess DESTINATION bin)


# --- Packaging ---
# This command runs CPack after the build is complete.
# It's attached to the manager, but packages the whole installation.
add_custom_command(
        TARGET GompeiVision
        POST_BUILD
        COMMAND ${CMAKE_CPACK_COMMAND}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Packaging project into a .deb file..."
        VERBATIM
)