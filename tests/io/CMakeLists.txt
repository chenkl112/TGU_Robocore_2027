cmake_minimum_required(VERSION 3.16)
project(robocore_io_tests LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
    message(FATAL_ERROR "The serial lifecycle tests require Linux poll and pseudo-terminals.")
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()
include(CheckCXXSourceCompiles)
check_cxx_source_compiles("#include <format>
int main() { return std::format(\"{}\", 42).empty(); }" IO_HAS_STD_FORMAT)
if(NOT IO_HAS_STD_FORMAT)
    message(FATAL_ERROR "I/O tests require C++20 std::format, for example GCC 13+ with libstdc++ 13+.")
endif()
find_package(Threads REQUIRED)
find_package(Boost 1.69 REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
enable_testing()

add_library(io_test_support STATIC
    ../../io/serial/serial.cpp
    ../../tools/logger.cpp
    ../../tools/crc.cpp
)
target_include_directories(io_test_support PUBLIC ../.. ../../io)
target_link_libraries(io_test_support PUBLIC Threads::Threads Boost::boost)

add_executable(serial_tests serial_tests.cpp)
target_link_libraries(serial_tests PRIVATE io_test_support util)
add_executable(gimbal_lifecycle_tests gimbal_lifecycle_tests.cpp ../../io/gimbal/gimbal.cpp)
target_link_libraries(gimbal_lifecycle_tests PRIVATE io_test_support Eigen3::Eigen util)

set(IO_SANITIZER "none" CACHE STRING "none, address (with UBSan), or thread")
set_property(CACHE IO_SANITIZER PROPERTY STRINGS none address thread)
if(IO_SANITIZER STREQUAL "address")
    set(IO_SANITIZER_FLAGS -fsanitize=address,undefined -fno-sanitize-recover=all -fno-omit-frame-pointer -g)
elseif(IO_SANITIZER STREQUAL "thread")
    set(IO_SANITIZER_FLAGS -fsanitize=thread -fno-omit-frame-pointer -g)
elseif(NOT IO_SANITIZER STREQUAL "none")
    message(FATAL_ERROR "IO_SANITIZER must be none, address, or thread.")
endif()
foreach(target io_test_support serial_tests gimbal_lifecycle_tests)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic ${IO_SANITIZER_FLAGS})
    target_link_options(${target} PRIVATE ${IO_SANITIZER_FLAGS})
endforeach()

foreach(scenario IN ITEMS read_timeout close_read fragmented subscriptions isolated_ports
        reopen failed_open disconnect write_timeout close_write queued_write callback_close callback_throw concurrent)
    add_test(NAME serial.${scenario} COMMAND serial_tests ${scenario})
    set_tests_properties(serial.${scenario} PROPERTIES TIMEOUT 15)
endforeach()
foreach(scenario IN ITEMS silent_start idle_shutdown feedback_timeout pose_history mode_session)
    add_test(NAME gimbal.${scenario} COMMAND gimbal_lifecycle_tests ${scenario})
    set_tests_properties(gimbal.${scenario} PROPERTIES TIMEOUT 10)
endforeach()
