#pragma once

#include <Eigen/Core>
#include <array>
#include <chrono>
#include <string>

#include "io/device_config.hpp"

namespace app::auto_aim {

// 配置进入算法前完成单位转换：角度为弧度，距离为米，浮点时间为秒。
// 路径构造函数仅用于兼容独立调用；正式入口只读取一次 AutoAimConfig。
struct CalibrationConfig {
    Eigen::Matrix3d R_gimbal2imubody = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R_camera2gimbal = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_camera2gimbal = Eigen::Vector3d::Zero();
    Eigen::Matrix3d camera_matrix = Eigen::Matrix3d::Identity();
    Eigen::Matrix<double, 1, 5> distort_coeffs = Eigen::Matrix<double, 1, 5>::Zero();
};

struct SolverConfig {
    double max_reprojection_error = 3.0;
    double min_type_error_gap = 0.25;
    double size_noise_floor = 1.5;
    double max_balance_pitch = 1.0471975511965976;
};

struct SolverSettings {
    CalibrationConfig calibration;
    SolverConfig solver;
};

struct ClassifierConfig {
    std::string model_path = "config/armor_classifier.onnx";
};

struct DetectorConfig {
    double threshold = 40.0;
    double max_angle_error = 0.2617993877991494;
    double min_lightbar_ratio = 1.5;
    double max_lightbar_ratio = 8.0;
    double min_lightbar_length = 5.0;
    double min_armor_ratio = 0.8;
    double max_armor_ratio = 4.0;
    double max_side_ratio = 2.5;
    double min_confidence = 0.6;
    double max_rectangular_error = 0.17453292519943295;
};

struct YoloConfig {
    std::string name = "yolov8";
    std::string model_path = "config/yolov8_armor.onnx";
    std::string device = "AUTO";
    bool use_roi = false;
    int roi_x = 0;
    int roi_y = 0;
    int roi_width = -1;
    int roi_height = -1;
    double min_confidence = 0.6;
};

struct VisionConfig {
    YoloConfig yolo;
    ClassifierConfig classifier;
    DetectorConfig detector;
};

struct TrackerConfig {
    std::string enemy_color = "blue";
    int priority_mode = 1;
    int min_detect_count = 3;
    int max_temp_lost_count = 10;
    double max_match_distance = 0.5;
};

struct MpcAxisConfig {
    double max_acc = 50.0;
    std::array<double, 2> Q{9.0e6, 0.0};
    double R = 30.0;
};

struct PlannerConfig {
    double yaw_offset = 0.0;
    double pitch_offset = 0.0;
    double fire_thresh = 0.05;
    double decision_speed = 7.0;
    double high_speed_delay_time = 0.12;
    double low_speed_delay_time = 0.12;
    double max_target_age = 0.1;
    std::chrono::milliseconds feedback_timeout{100};
    MpcAxisConfig yaw;
    MpcAxisConfig pitch;
    bool auto_fire = false;
};

struct AimerConfig {
    double yaw_offset = 0.0;
    double pitch_offset = 0.0;
    double coming_angle = 1.0471975511965976;
    double leaving_angle = 0.3490658503988659;
    double decision_speed = 10.0;
    double high_speed_delay_time = 0.026;
    double low_speed_delay_time = 0.010;
};

struct ShooterConfig {
    double first_tolerance = 0.08726646259971647;
    double second_tolerance = 0.03490658503988659;
    double judge_distance = 3.0;
    bool auto_fire = false;
};

struct AutoAimConfig {
    io::CameraConfig camera;
    io::GimbalConfig gimbal;
    CalibrationConfig calibration;
    SolverConfig solver;
    ClassifierConfig classifier;
    DetectorConfig detector;
    YoloConfig yolo;
    TrackerConfig tracker;
    PlannerConfig planner;
    AimerConfig aimer;
    ShooterConfig shooter;
};

// 相对资源路径以 paths.base_dir 为基准；未设置时保持以启动目录为基准。
// paths.base_dir 自身相对配置文件目录解析，加载后交给模块的模型路径均为绝对路径。
AutoAimConfig load_auto_aim_config(const std::string& path);
SolverSettings load_solver_settings(const std::string& path);
VisionConfig load_vision_config(const std::string& path);
ClassifierConfig load_classifier_config(const std::string& path);
TrackerConfig load_tracker_config(const std::string& path);
PlannerConfig load_planner_config(const std::string& path);
AimerConfig load_aimer_config(const std::string& path);
ShooterConfig load_shooter_config(const std::string& path);

void validate(const CalibrationConfig& config);
void validate(const SolverConfig& config);
void validate(const ClassifierConfig& config);
void validate(const DetectorConfig& config);
void validate(const YoloConfig& config);
void validate(const TrackerConfig& config);
void validate(const PlannerConfig& config);
void validate(const AimerConfig& config);
void validate(const ShooterConfig& config);

} // namespace app::auto_aim
