# test
add_executable(read_toml test/read_toml.cpp)
add_executable(serial_test test/test_serial.cpp)
add_executable(test_logger test/test_logger.cpp)

target_link_libraries(serial_test PRIVATE io tools)
target_link_libraries(test_logger PRIVATE tools)