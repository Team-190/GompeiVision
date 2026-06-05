cmake_minimum_required(VERSION 3.15)
project(GompeiVisionTests)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

set(TEST_SOURCES
    capture/TestCamera.cpp
    detector/TestFiducialDetector.cpp
    estimator/TestTagAngleCalculator.cpp
    estimator/TestSingleTagPoseEstimator.cpp
    estimator/TestCameraPoseEstimator.cpp
    io/TestConfigInterface.cpp
    io/TestFieldInterface.cpp
    io/TestOutputPublisher.cpp
    pipeline/TestPipeline.cpp
    pipeline/TestPipelineHelper.cpp
    pipeline/TestPipelineManager.cpp
    util/TestPoseUtils.cpp
    util/TestThreadSafeQueue.cpp
    
    # Sources under test
    ../src/cpp/estimator/TagAngleCalculator.cpp
    ../src/cpp/estimator/CameraPoseEstimator.cpp
    ../src/cpp/estimator/SingleTagPoseEstimator.cpp
    ../src/cpp/pipeline/PipelineHelper.cpp
    ../src/cpp/io/ConfigInterface.cpp
    ../src/cpp/io/FieldInterface.cpp
    ../src/cpp/io/OutputPublisher.cpp
    ../src/cpp/capture/Camera.cpp
    ../src/cpp/pipeline/Pipeline.cpp
    ../src/cpp/detector/FiducialDetector.cpp
    ../src/cpp/pipeline/PipelineManager.cpp
)

add_executable(GompeiVisionTests ${TEST_SOURCES})

target_include_directories(GompeiVisionTests PRIVATE
    "${CMAKE_SOURCE_DIR}/src/include"
)

target_link_libraries(GompeiVisionTests PRIVATE
    gtest_main
    third_party_libs
)

# Enable coverage flags
target_compile_options(GompeiVisionTests PRIVATE --coverage)
target_link_options(GompeiVisionTests PRIVATE --coverage)

add_test(NAME GompeiVisionTests COMMAND GompeiVisionTests)

# Automatically run tests and print coverage report after compiling
find_program(GCOVR_PATH gcovr)
if(GCOVR_PATH)
    add_custom_command(
        TARGET GompeiVisionTests
        POST_BUILD
        COMMAND $<TARGET_FILE:GompeiVisionTests>
        COMMAND ${GCOVR_PATH} --root ${CMAKE_SOURCE_DIR} --filter ${CMAKE_SOURCE_DIR}/src/
        COMMENT "Running tests and generating coverage report..."
        VERBATIM
    )
else()
    add_custom_command(
        TARGET GompeiVisionTests
        POST_BUILD
        COMMAND $<TARGET_FILE:GompeiVisionTests>
        COMMAND ${CMAKE_COMMAND} -E echo "Warning: gcovr not found. Skipping coverage report. Rebuild your Docker image to enable it."
        COMMENT "Running tests..."
        VERBATIM
    )
endif()
