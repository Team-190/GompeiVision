#! target_link_opencv : Adds opencv as dependency for the target. If the library is not found locally, it will be downloaded and (cross-) compiled.
#
# \arg:TARGET_NAME Name of the target
# \arg:LINK_TYPE Optional, specify how to link the module (either INTERFACE, PRIVATE or PUBLIC; defaults to PUBLIC).
function(target_link_opencv TARGET_NAME)
    if (${ARGC} EQUAL 2)
        set(LINK_TYPE ${ARGV1})
    else ()
        set(LINK_TYPE "PUBLIC")
    endif ()

    set(BUILD_LIST "core,imgproc,imgcodecs,calib3d,features2d,flann,highgui,videoio,objdetect,dnn")
    string(REPLACE "," ";" BUILD_MODULES_LIST "${BUILD_LIST}")
    if (NOT TARGET OpenCV)
        set(OPENCV_LIBRARY_VERSION "4.11.0" CACHE STRING "Version of the opencv library to use.")
        message("opencv library was not found! Trying to download library version ${OPENCV_LIBRARY_VERSION} instead!")

        FetchContent_Declare(OpenCV URL https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_LIBRARY_VERSION}.tar.gz)
        FetchContent_GetProperties(OpenCV)
        if (NOT OpenCV)
            FetchContent_Populate(OpenCV)

            set(BUILD_SHARED_LIBS OFF)
            set(WITH_IPP OFF)
            set(BUILD_TBB OFF)
            set(BUILD_EXAMPLES OFF)
            set(BUILD_TESTS OFF)
            set(BUILD_PERF_TESTS OFF)
            set(BUILD_opencv_apps OFF)

            target_include_directories(${TARGET_NAME} ${LINK_TYPE} "${CMAKE_BINARY_DIR}/_deps/opencv-src/include")
            target_include_directories(${TARGET_NAME} ${LINK_TYPE} "${CMAKE_BINARY_DIR}")
            foreach (m ${BUILD_MODULES_LIST})
                target_include_directories(${TARGET_NAME} ${LINK_TYPE} "${CMAKE_BINARY_DIR}/_deps/opencv-src/modules/${m}/include")
            endforeach ()

            add_subdirectory(${opencv_SOURCE_DIR} ${opencv_BINARY_DIR} EXCLUDE_FROM_ALL SYSTEM)
        endif ()
    endif ()

    foreach (m ${BUILD_MODULES_LIST})
        target_link_libraries(${TARGET_NAME} ${LINK_TYPE} opencv_${m})
    endforeach ()
endfunction()