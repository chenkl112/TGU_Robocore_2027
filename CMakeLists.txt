cmake_minimum_required(VERSION 3.16)
project(TGU_Robocore_2027 VERSION 0.1.0)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

find_package(Boost CONFIG REQUIRED COMPONENTS system)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Prefer the project-local Python OpenVINO installation created by the README
# setup commands. A system installation or an explicit -DOpenVINO_DIR still
# takes precedence.
if(NOT OpenVINO_DIR)
    file(GLOB OPENVINO_VENV_CMAKE_DIRS LIST_DIRECTORIES true
        "${PROJECT_SOURCE_DIR}/.venv-openvino/lib/python*/site-packages/openvino/cmake"
    )
    list(LENGTH OPENVINO_VENV_CMAKE_DIRS OPENVINO_VENV_CMAKE_DIR_COUNT)
    if(OPENVINO_VENV_CMAKE_DIR_COUNT EQUAL 1)
        list(GET OPENVINO_VENV_CMAKE_DIRS 0 OPENVINO_VENV_CMAKE_DIR)
        set(OpenVINO_DIR "${OPENVINO_VENV_CMAKE_DIR}" CACHE PATH
            "Path to the project-local OpenVINO CMake package"
        )
    endif()
endif()
find_package(OpenVINO REQUIRED COMPONENTS Runtime)
find_package(Threads REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ARAVIS REQUIRED aravis-0.8)

include_directories(${PROJECT_SOURCE_DIR})
include_directories(${ARAVIS_INCLUDE_DIRS})

# ===== 子模块 =====
add_subdirectory(tools)
add_subdirectory(io)
add_subdirectory(app)
add_subdirectory(task)
