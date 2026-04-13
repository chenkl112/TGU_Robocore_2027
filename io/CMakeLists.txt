cmake_minimum_required(VERSION 3.16)

# 创建目标 io
add_library(io STATIC
        serial/serial.cpp
#        hikrobot/hikrobot.cpp
#        mindvision/mindvision.cpp
#        usbcamera/usbcamera.cpp
#        camera.cpp
#        cboard.cpp
#        dm_imu/dm_imu.cpp
#        gimbal/gimbal.cpp
#        gimbal/gimbal_nav.cpp
)

target_link_libraries(io
        PUBLIC
#        tools
        boost_system
        pthread
)

