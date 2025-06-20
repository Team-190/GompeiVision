cmake_minimum_required(VERSION 3.18)

add_executable(gompeivision cpp/main.cpp)

target_include_directories(gompeivision PUBLIC
        ${CMAKE_SOURCE_DIR}/include
)

target_link_libraries(gompeivision PRIVATE
        wpi::wpiutil
        wpi::ntcore
        wpi::cscore
        wpi::wpimath
        wpi::wpinet
)