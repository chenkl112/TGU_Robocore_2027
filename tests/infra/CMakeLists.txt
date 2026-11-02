cmake_minimum_required(VERSION 3.16)
project(robocore_infra_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

include(CheckCXXSourceCompiles)
check_cxx_source_compiles("
    #include <format>
    #include <string>
    int main() {
        return std::format(\"{}\", 42) == std::string(\"42\") ? 0 : 1;
    }
" INFRA_HAS_STD_FORMAT)
if(NOT INFRA_HAS_STD_FORMAT)
    message(FATAL_ERROR
        "Infrastructure tests require C++20 std::format. Select a compiler and "
        "standard library that support <format> (for example GCC 13+ with "
        "libstdc++ 13+) using -DCMAKE_CXX_COMPILER in a fresh build directory.")
endif()

find_package(Threads REQUIRED)
enable_testing()

add_executable(logger_tests
    logger_tests.cpp
    ../../tools/logger.cpp
)
target_include_directories(logger_tests PRIVATE ../..)
target_link_libraries(logger_tests PRIVATE Threads::Threads)

option(INFRA_ENABLE_TSAN "Run infrastructure tests with ThreadSanitizer" OFF)
if(INFRA_ENABLE_TSAN)
    if(NOT CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang" OR WIN32)
        message(FATAL_ERROR "INFRA_ENABLE_TSAN requires GCC or Clang on a supported Unix platform.")
    endif()
    target_compile_options(logger_tests PRIVATE -fsanitize=thread -fno-omit-frame-pointer -g)
    target_link_options(logger_tests PRIVATE -fsanitize=thread)
endif()

foreach(scenario IN ITEMS reinitialize switch_file toggle_file invalid_path levels concurrent debug_macro)
    add_test(NAME logger.${scenario} COMMAND logger_tests ${scenario})
    set_tests_properties(logger.${scenario} PROPERTIES TIMEOUT 120)
endforeach()
