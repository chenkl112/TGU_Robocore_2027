cmake_minimum_required(VERSION 3.16)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    yolo.cpp
    yolos/yolov8.cpp
    planner/planner.cpp
    multithread/mt_yolo.cpp
)

target_include_directories(auto_aim PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(auto_aim PUBLIC
    tools
    tinympcstatic
    ${OpenCV_LIBS}
    Eigen3::Eigen
    openvino::runtime
    Threads::Threads
)
