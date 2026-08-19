# test
add_executable(read_toml test/read_toml.cpp)
add_executable(test_serial test/test_serial.cpp)
add_executable(test_logger test/test_logger.cpp)
add_executable(test_foxglove test/test_foxglove.cpp)
add_executable(test_camera test/test_camera.cpp)
add_executable(auto_aim_test test/auto_aim_test.cpp)
add_executable(planner_test test/planner_test.cpp)
add_executable(auto_aim_sentry auto_aim_sentry.cpp)
add_executable(gazebo_auto_aim gazebo_auto_aim.cpp)

target_link_libraries(test_serial PRIVATE io tools)
target_link_libraries(test_logger PRIVATE tools)
target_link_libraries(test_foxglove PRIVATE tools)
target_link_libraries(test_camera PRIVATE ${ARAVIS_LIBRARIES} ${OpenCV_LIBS})
target_link_libraries(
        auto_aim_test
        PRIVATE auto_aim io tools ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads
)
target_link_libraries(
        planner_test
        PRIVATE auto_aim tools ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads
)
target_link_libraries(
        auto_aim_sentry
        PRIVATE auto_aim io tools ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads
)
target_link_libraries(
        gazebo_auto_aim
        PRIVATE auto_aim tools ${OpenCV_LIBS} Eigen3::Eigen Threads::Threads
)
