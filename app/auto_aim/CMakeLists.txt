cmake_minimum_required(VERSION 3.16)

add_subdirectory(planner/tinympc)

add_library(auto_aim OBJECT
    aimer.cpp
    armor.cpp
    classifier.cpp
    detector.cpp
    solver.cpp
    shooter.cpp
    target.cpp
    tracker.cpp
    voter.cpp
    yolo.cpp
    yolos/yolov8.cpp
    multithread/mt_yolo.cpp
    planner/planner.cpp
)

target_include_directories(auto_aim PUBLIC
    ${PROJECT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(auto_aim PUBLIC
    tools
    ${OpenCV_LIBS}
    Eigen3::Eigen
    openvino::runtime
    tinympcstatic
    Threads::Threads
)
