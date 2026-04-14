cmake_minimum_required(VERSION 3.16)
project(TGU_Robocore_2027)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Boost CONFIG REQUIRED)
find_package(OpenCV REQUIRED)
#find_package(Eigen3 REQUIRED)

include_directories(${PROJECT_SOURCE_DIR})

# ===== 子模块 =====
add_subdirectory(io)
#add_subdirectory(app)
add_subdirectory(task)
add_subdirectory(tools)