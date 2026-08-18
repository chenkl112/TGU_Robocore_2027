cmake_minimum_required(VERSION 3.16)

# 创建目标 io
add_library(io STATIC
        serial/serial.cpp
        hikrobot/hikrobot.cpp
        gimbal/gimbal.cpp
        usbcamera/usbcamera.cpp
)

target_include_directories(io PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${PROJECT_SOURCE_DIR}
)

target_link_libraries(io
        PUBLIC
        tools
        Boost::system
        Threads::Threads
        Eigen3::Eigen
        ${OpenCV_LIBS}
)
