set(MANAGER_SOURCES
        cpp/main.cpp
        cpp/pipeline/PipelineManager.cpp
)

set(WORKER_SOURCES
        cpp/capture/Camera.cpp
        cpp/detector/FiducialDetector.cpp
        cpp/estimator/CameraPoseEstimator.cpp
        cpp/estimator/SingleTagPoseEstimator.cpp
        cpp/estimator/TagAngleCalculator.cpp
        cpp/io/ConfigInterface.cpp
        cpp/io/FieldInterface.cpp
        cpp/io/OutputPublisher.cpp
        cpp/pipeline/Pipeline.cpp
        cpp/pipeline/PipelineHelper.cpp
        cpp/pipeline/PipelineProcess.cpp
)

add_executable(GompeiVision ${MANAGER_SOURCES})
add_executable(GompeiVisionProcess ${WORKER_SOURCES})

set_property(TARGET GompeiVision GompeiVisionProcess PROPERTY CXX_STANDARD 17)
set_property(TARGET GompeiVision GompeiVisionProcess PROPERTY CXX_STANDARD_REQUIRED ON)

set_target_properties(GompeiVision GompeiVisionProcess PROPERTIES
        INSTALL_RPATH "$ORIGIN/../lib"
)

target_include_directories(GompeiVision PUBLIC
        "${CMAKE_SOURCE_DIR}/src/include"
)
target_include_directories(GompeiVisionProcess PUBLIC
        "${CMAKE_SOURCE_DIR}/src/include"
)

target_link_libraries(GompeiVision PRIVATE
        third_party_libs
)
target_link_libraries(GompeiVisionProcess PRIVATE
        third_party_libs
)

install(TARGETS GompeiVision GompeiVisionProcess DESTINATION bin)

add_custom_command(
        TARGET GompeiVision
        POST_BUILD
        COMMAND ${CMAKE_CPACK_COMMAND}
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Packaging project into a .deb file..."
        VERBATIM
)