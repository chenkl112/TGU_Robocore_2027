cmake_minimum_required(VERSION 3.16)
project(robocore_auto_aim_tests LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core calib3d imgproc videoio)
find_package(Threads REQUIRED)
enable_testing()
set(REPO ${CMAKE_CURRENT_SOURCE_DIR}/../..)
if(NOT TARGET tinympcstatic)
    add_subdirectory(${REPO}/app/auto_aim/planner/tinympc tinympc)
endif()
add_library(auto_aim_regression_support OBJECT
    ${REPO}/app/auto_aim/armor.cpp
    ${REPO}/app/auto_aim/target.cpp
    ${REPO}/app/auto_aim/aimer.cpp
    ${REPO}/app/auto_aim/shooter.cpp
    ${REPO}/app/auto_aim/solver.cpp
    ${REPO}/app/auto_aim/tracker.cpp
    ${REPO}/app/auto_aim/planner/planner.cpp
    ${REPO}/io/usbcamera/usbcamera.cpp
    ${REPO}/tools/logger.cpp
    ${REPO}/tools/math_tools.cpp
    ${REPO}/tools/extended_kalman_filter.cpp
    ${REPO}/tools/trajectory.cpp
)
target_include_directories(auto_aim_regression_support PUBLIC ${REPO} ${OpenCV_INCLUDE_DIRS})
target_link_libraries(auto_aim_regression_support PUBLIC Eigen3::Eigen tinympcstatic ${OpenCV_LIBS} Threads::Threads)
add_executable(auto_aim_regression_tests auto_aim_regression_tests.cpp)
target_link_libraries(auto_aim_regression_tests PRIVATE auto_aim_regression_support)
foreach(scenario IN ITEMS tracking_confirmation priority_switch association_gate planner_expiry
        feedback_alignment perspective_keypoints tilted_keypoints fire_without_feedback shooter_alignment observation_expiry moving_intercept planner_acceleration_bounds
        planner_failed_solve planner_rotating_sequence planner_solve_recovery
        invalid_numeric lost_target_fire pose_validation calibration_validation camera_timeout trajectory)
    add_test(NAME auto_aim.${scenario} COMMAND auto_aim_regression_tests ${scenario} ${REPO}/config/auto_aim.toml)
    set_tests_properties(auto_aim.${scenario} PROPERTIES TIMEOUT 15 WORKING_DIRECTORY ${REPO})
endforeach()

foreach(suite IN ITEMS armor_type yaw_optimization shooter_current_command)
    add_executable(${suite}_tests ${suite}_tests.cpp)
    target_link_libraries(${suite}_tests PRIVATE auto_aim_regression_support)
    add_test(NAME auto_aim.${suite} COMMAND ${suite}_tests ${REPO}/config/auto_aim.toml)
    set_tests_properties(auto_aim.${suite} PROPERTIES TIMEOUT 30 WORKING_DIRECTORY ${REPO})
endforeach()

# These checks exercise the actual OpenVINO worker and production executables.
if(TARGET auto_aim)
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    add_executable(yolo_failure_tests yolo_failure_tests.cpp)
    target_link_libraries(yolo_failure_tests PRIVATE auto_aim tools)
    add_test(NAME auto_aim.worker_failure COMMAND yolo_failure_tests ${REPO}/config/auto_aim.toml)
    set_tests_properties(auto_aim.worker_failure PROPERTIES WORKING_DIRECTORY ${REPO} TIMEOUT 30)
    add_executable(yolo_keypoint_contract_tests yolo_keypoint_contract_tests.cpp)
    target_link_libraries(yolo_keypoint_contract_tests PRIVATE auto_aim tools)
    add_test(NAME auto_aim.model_keypoint_contract COMMAND yolo_keypoint_contract_tests ${REPO}/assets/yolov8_640x480.xml)
    set_tests_properties(auto_aim.model_keypoint_contract PROPERTIES TIMEOUT 30)
    add_executable(fixture_generator fixture_generator.cpp)
    target_include_directories(fixture_generator PRIVATE ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(fixture_generator PRIVATE ${OpenCV_LIBS})
    foreach(scenario IN ITEMS signal_shutdown missing_camera network_camera_silence replay_seek)
        add_test(NAME runtime.${scenario} COMMAND ${Python3_EXECUTABLE}
            ${CMAKE_CURRENT_SOURCE_DIR}/runtime_tests.py ${scenario} ${REPO} ${CMAKE_BINARY_DIR})
        set_tests_properties(runtime.${scenario} PROPERTIES TIMEOUT 40 RUN_SERIAL TRUE)
    endforeach()
endif()
