cmake_minimum_required(VERSION 3.16)

add_library(tools STATIC
    crc.cpp
    foxglove_comm.cpp
    logger.cpp
    math_tools.cpp
    extended_kalman_filter.cpp
    trajectory.cpp
    img_tools.cpp
)

target_include_directories(tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/foxglove/include
    ${CMAKE_CURRENT_SOURCE_DIR}/foxglove/include/foxglove-c
    ${PROJECT_SOURCE_DIR}
)

target_link_libraries(tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/foxglove/lib/libfoxglove.a
    ${OpenCV_LIBS}
    Boost::boost
    Threads::Threads
)
