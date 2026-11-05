cmake_minimum_required(VERSION 3.16)
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    project(robocore_auto_aim_tests LANGUAGES CXX)
    enable_testing()
    option(ROBOCORE_WITH_VISION "Build OpenVINO model regressions" OFF)
    option(ROBOCORE_WITH_IO "Include camera device regression" OFF)
endif()
include("${CMAKE_CURRENT_LIST_DIR}/../../cmake/RobocoreTargets.cmake")
robocore_add_targets(COMPONENTS core legacy)
if(ROBOCORE_WITH_VISION)
    robocore_add_targets(COMPONENTS vision)
endif()
if(ROBOCORE_WITH_IO)
    robocore_add_targets(COMPONENTS camera)
endif()
set(REPO "${ROBOCORE_SOURCE_DIR}")

add_executable(auto_aim_regression_tests auto_aim_regression_tests.cpp
    planner_regression_tests.cpp tracker_regression_tests.cpp
    geometry_runtime_regression_tests.cpp time_contract_regression_tests.cpp)
target_link_libraries(auto_aim_regression_tests PRIVATE auto_aim_core auto_aim_legacy)
set(regression_scenarios tracking_confirmation priority_switch association_gate planner_expiry
    feedback_alignment perspective_keypoints tilted_keypoints fire_without_feedback shooter_alignment
    observation_expiry moving_intercept planner_acceleration_bounds planner_failed_solve
    planner_rotating_sequence planner_rotating_geometry planner_solve_recovery planner_visible_intercept
    mpc_objective invalid_numeric lost_target_fire pose_validation calibration_validation trajectory
    prediction_time_contract planner_time_contract planner_stop_reasons tracker_snapshot)
if(TARGET robocore_camera)
    target_compile_definitions(auto_aim_regression_tests PRIVATE ROBOCORE_TEST_CAMERA=1)
    target_link_libraries(auto_aim_regression_tests PRIVATE robocore_camera)
    list(APPEND regression_scenarios camera_timeout)
endif()
foreach(scenario IN LISTS regression_scenarios)
    add_test(NAME auto_aim.${scenario}
        COMMAND auto_aim_regression_tests ${scenario} "${REPO}/config/auto_aim.toml")
    set_tests_properties(auto_aim.${scenario} PROPERTIES TIMEOUT 15 WORKING_DIRECTORY "${REPO}")
endforeach()
set_tests_properties(auto_aim.planner_rotating_sequence auto_aim.planner_rotating_geometry
    PROPERTIES TIMEOUT 30)

foreach(suite IN ITEMS armor_type yaw_optimization shooter_current_command)
    add_executable(${suite}_tests ${suite}_tests.cpp)
    target_link_libraries(${suite}_tests PRIVATE auto_aim_core auto_aim_legacy)
    add_test(NAME auto_aim.${suite} COMMAND ${suite}_tests "${REPO}/config/auto_aim.toml")
    set_tests_properties(auto_aim.${suite} PROPERTIES TIMEOUT 30 WORKING_DIRECTORY "${REPO}")
endforeach()
set_tests_properties(auto_aim.armor_type PROPERTIES TIMEOUT 60)

# Debug 保留 Eigen 检查且关闭优化，穷举数值回归需要更长的进程时间预算。
# 这里只调整防挂起超时；物理误差、收敛次数和控制边界断言保持相同。
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set_tests_properties(auto_aim.planner_rotating_sequence auto_aim.planner_rotating_geometry
        PROPERTIES TIMEOUT 300)
    set_tests_properties(auto_aim.armor_type PROPERTIES TIMEOUT 600)
endif()

add_executable(config_tests config_tests.cpp)
target_link_libraries(config_tests PRIVATE auto_aim_core auto_aim_legacy)
add_test(NAME auto_aim.config COMMAND config_tests "${REPO}/config/auto_aim.toml")
set_tests_properties(auto_aim.config PROPERTIES TIMEOUT 60 WORKING_DIRECTORY "${REPO}")

if(TARGET auto_aim_vision)
    target_link_libraries(config_tests PRIVATE auto_aim_vision)
    target_compile_definitions(config_tests PRIVATE TGU_TEST_WITH_VISION=1)
    add_executable(yolo_failure_tests yolo_failure_tests.cpp)
    target_link_libraries(yolo_failure_tests PRIVATE auto_aim_vision)
    add_test(NAME auto_aim.worker_failure COMMAND yolo_failure_tests "${REPO}/config/auto_aim.toml")
    set_tests_properties(auto_aim.worker_failure PROPERTIES WORKING_DIRECTORY "${REPO}" TIMEOUT 30)

    add_executable(worker_lifecycle_tests worker_lifecycle_tests.cpp)
    target_link_libraries(worker_lifecycle_tests PRIVATE auto_aim_vision)
    add_test(NAME auto_aim.worker_lifecycle COMMAND worker_lifecycle_tests "${REPO}/config/auto_aim.toml")
    set_tests_properties(auto_aim.worker_lifecycle PROPERTIES WORKING_DIRECTORY "${REPO}" TIMEOUT 40)

    add_executable(yolo_keypoint_contract_tests yolo_keypoint_contract_tests.cpp)
    target_link_libraries(yolo_keypoint_contract_tests PRIVATE auto_aim_vision)
    add_test(NAME auto_aim.model_keypoint_contract
        COMMAND yolo_keypoint_contract_tests "${REPO}/assets/yolov8_640x480.xml")
    set_tests_properties(auto_aim.model_keypoint_contract PROPERTIES TIMEOUT 30)

    add_executable(vision_pipeline_tests vision_pipeline_tests.cpp)
    target_link_libraries(vision_pipeline_tests PRIVATE auto_aim_vision)
    add_test(NAME auto_aim.vision_pipeline COMMAND vision_pipeline_tests "${REPO}/config/auto_aim.toml")
    set_tests_properties(auto_aim.vision_pipeline PROPERTIES TIMEOUT 120 WORKING_DIRECTORY "${REPO}")

    # 进程测试仅在正式入口确实构建时注册，独立算法测试不隐式创建设备或推理程序。
    if(TARGET auto_aim_test)
        find_package(Python3 REQUIRED COMPONENTS Interpreter)
        add_executable(fixture_generator fixture_generator.cpp)
        target_link_libraries(fixture_generator PRIVATE auto_aim_vision)
        set(runtime_scenarios replay_seek)
        if(TARGET auto_aim_sentry)
            list(APPEND runtime_scenarios signal_shutdown missing_camera network_camera_silence
                positive_target_recovery pose_wait_shutdown)
        endif()
        foreach(scenario IN LISTS runtime_scenarios)
            add_test(NAME runtime.${scenario} COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_CURRENT_SOURCE_DIR}/runtime_tests.py" ${scenario} "${REPO}" "${CMAKE_BINARY_DIR}")
            set_tests_properties(runtime.${scenario} PROPERTIES TIMEOUT 40 RUN_SERIAL TRUE)
        endforeach()
        foreach(scenario IN ITEMS replay_deterministic replay_positive_seek replay_invalid_records)
            add_test(NAME runtime.${scenario} COMMAND ${Python3_EXECUTABLE}
                "${CMAKE_CURRENT_SOURCE_DIR}/pipeline_runtime_tests.py" ${scenario}
                "${REPO}" "${CMAKE_BINARY_DIR}")
            set_tests_properties(runtime.${scenario} PROPERTIES TIMEOUT 120 RUN_SERIAL TRUE)
        endforeach()
    endif()
endif()
