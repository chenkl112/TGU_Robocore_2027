#include "config.hpp"

#include <Eigen/LU>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string_view>
#include <type_traits>

#include "tools/tomlpp.hpp"

namespace app::auto_aim {
namespace {

using Table = toml::table;
constexpr double radians(double degrees) {
    return degrees * std::numbers::pi / 180.0;
}

[[noreturn]] void invalid(std::string_view field, std::string_view reason) {
    throw std::invalid_argument(std::string(field) + ": " + std::string(reason));
}

template <typename T>
T read(const Table& table, std::string_view section, std::string_view key, T fallback) {
    const auto value = table[section][key];
    if (!value)
        return fallback;
    const auto field = std::string(section) + "." + std::string(key);
    // TOML 的宽松转换会把整数当布尔值，或把整数值浮点数当整数；配置边界禁止隐式转换。
    if constexpr (std::is_same_v<T, bool>) {
        if (!value.is_boolean())
            invalid(field, "expected a boolean");
    } else if constexpr (std::is_same_v<T, std::int64_t>) {
        if (!value.is_integer())
            invalid(field, "expected an integer");
    } else if constexpr (std::is_same_v<T, double>) {
        if (!value.is_integer() && !value.is_floating_point())
            invalid(field, "expected a number");
    } else if constexpr (std::is_same_v<T, std::string>) {
        if (!value.is_string())
            invalid(field, "expected a string");
    }
    const auto parsed = value.value<T>();
    if (!parsed)
        invalid(field, "invalid value type");
    return *parsed;
}

int integer(const Table& table, std::string_view section, std::string_view key, int fallback) {
    const auto value = read<std::int64_t>(table, section, key, fallback);
    if (value < std::numeric_limits<int>::min() || value > std::numeric_limits<int>::max()) {
        invalid(std::string(section) + "." + std::string(key), "integer out of range");
    }
    return static_cast<int>(value);
}

double angle(const Table& table, std::string_view section, std::string_view key, double fallback) {
    return radians(read<double>(table, section, key, fallback));
}

void finite(double value, std::string_view field) {
    if (!std::isfinite(value))
        invalid(field, "must be finite");
}

void positive(double value, std::string_view field) {
    finite(value, field);
    if (value <= 0)
        invalid(field, "must be positive");
}

void nonnegative(double value, std::string_view field) {
    finite(value, field);
    if (value < 0)
        invalid(field, "must be nonnegative");
}

void probability(double value, std::string_view field) {
    nonnegative(value, field);
    if (value > 1)
        invalid(field, "must not exceed 1");
}

double number(const toml::node& node, std::string_view field) {
    if (!node.is_integer() && !node.is_floating_point())
        invalid(field, "expected a number");
    const auto result = node.value<double>();
    if (!result)
        invalid(field, "invalid number");
    finite(*result, field);
    return *result;
}

struct Document {
    Table table;
    std::filesystem::path resource_base;

    explicit Document(const std::string& path) : table(toml::parse_file(path)) {
        for (const auto section : {"camera", "gimbal", "solver", "classifier", "detector", "yolo",
                                   "tracker", "planner", "aimer", "shooter", "paths", "scope"}) {
            if (const auto node = table[section]; node && !node.is_table()) {
                invalid(section, "expected a table");
            }
        }
        resource_base = std::filesystem::current_path();
        if (table["paths"]["base_dir"]) {
            const auto base = read<std::string>(table, "paths", "base_dir", "");
            if (base.empty())
                invalid("paths.base_dir", "must not be empty");
            resource_base = std::filesystem::path(base);
            if (resource_base.is_relative()) {
                resource_base = std::filesystem::absolute(path).parent_path() / resource_base;
            }
        }
        resource_base = resource_base.lexically_normal();
    }

    std::string resource(const std::string& value) const {
        if (value.empty())
            return value;
        const auto path = std::filesystem::path(value);
        return (path.is_absolute() ? path : resource_base / path).lexically_normal().string();
    }
};

template <int Rows, int Columns>
Eigen::Matrix<double, Rows, Columns> matrix(const Table& table, std::string_view key) {
    const auto values = table["camera"][key].as_array();
    const auto field = std::string("camera.") + std::string(key);
    if (!values || values->size() != Rows * Columns) {
        invalid(field, "expected " + std::to_string(Rows * Columns) + " numeric values");
    }
    Eigen::Matrix<double, Rows, Columns> result;
    for (int index = 0; index < Rows * Columns; ++index) {
        result(index / Columns, index % Columns) = number((*values)[index], field);
    }
    return result;
}

CalibrationConfig calibration(const Table& table) {
    CalibrationConfig result;
    result.R_gimbal2imubody = matrix<3, 3>(table, "R_gimbal2imubody");
    result.R_camera2gimbal = matrix<3, 3>(table, "R_camera2gimbal");
    result.t_camera2gimbal = matrix<3, 1>(table, "t_camera2gimbal");
    result.camera_matrix = matrix<3, 3>(table, "camera_matrix");
    result.distort_coeffs = matrix<1, 5>(table, "distort_coeffs");
    validate(result);
    return result;
}

SolverConfig solver(const Table& table) {
    SolverConfig result;
    result.max_reprojection_error = read(table, "solver", "max_reprojection_error", 3.0);
    result.min_type_error_gap = read(table, "solver", "min_type_error_gap", 0.25);
    result.size_noise_floor =
        read(table, "solver", "size_noise_floor", std::min(1.5, result.max_reprojection_error));
    result.max_balance_pitch = angle(table, "solver", "max_balance_pitch", 60.0);
    validate(result);
    return result;
}

ClassifierConfig classifier(const Document& document) {
    ClassifierConfig result;
    result.model_path = document.resource(
        read<std::string>(document.table, "classifier", "classify_model", result.model_path));
    validate(result);
    return result;
}

DetectorConfig detector(const Table& table) {
    DetectorConfig result;
    result.threshold = read(table, "detector", "threshold", result.threshold);
    result.max_angle_error = angle(table, "detector", "max_angle_error", 15.0);
    result.min_lightbar_ratio =
        read(table, "detector", "min_lightbar_ratio", result.min_lightbar_ratio);
    result.max_lightbar_ratio =
        read(table, "detector", "max_lightbar_ratio", result.max_lightbar_ratio);
    result.min_lightbar_length =
        read(table, "detector", "min_lightbar_length", result.min_lightbar_length);
    result.min_armor_ratio = read(table, "detector", "min_armor_ratio", result.min_armor_ratio);
    result.max_armor_ratio = read(table, "detector", "max_armor_ratio", result.max_armor_ratio);
    result.max_side_ratio = read(table, "detector", "max_side_ratio", result.max_side_ratio);
    result.min_confidence = read(table, "detector", "min_confidence", result.min_confidence);
    result.max_rectangular_error = angle(table, "detector", "max_rectangular_error", 10.0);
    validate(result);
    return result;
}

YoloConfig yolo(const Document& document) {
    const auto& table = document.table;
    YoloConfig result;
    result.name = read(table, "yolo", "yolo_name", result.name);
    result.model_path = document.resource(read(table, "yolo", "yolo_model", result.model_path));
    result.device = read(table, "yolo", "device", result.device);
    result.use_roi = read(table, "yolo", "use_roi", false);
    result.roi_x = integer(table, "yolo", "roi_x", 0);
    result.roi_y = integer(table, "yolo", "roi_y", 0);
    result.roi_width = integer(table, "yolo", "roi_width", -1);
    result.roi_height = integer(table, "yolo", "roi_height", -1);
    result.min_confidence = read(table, "detector", "min_confidence", 0.6);
    validate(result);
    return result;
}

TrackerConfig tracker(const Table& table) {
    TrackerConfig result;
    result.enemy_color = read(table, "tracker", "enemy_color", result.enemy_color);
    result.priority_mode = integer(table, "tracker", "priority_mode", result.priority_mode);
    result.min_detect_count =
        integer(table, "tracker", "min_detect_count", result.min_detect_count);
    result.max_temp_lost_count =
        integer(table, "tracker", "max_temp_lost_count", result.max_temp_lost_count);
    result.max_match_distance =
        read(table, "tracker", "max_match_distance", result.max_match_distance);
    validate(result);
    return result;
}

MpcAxisConfig axis(const Table& table, const std::string& name) {
    MpcAxisConfig result;
    result.max_acc = read(table, "planner", "max_" + name + "_acc", 50.0);
    const auto q = table["planner"]["Q_" + name].as_array();
    const auto r = table["planner"]["R_" + name].as_array();
    if (!q || q->size() != 2)
        invalid("planner.Q_" + name, "expected 2 values");
    if (!r || r->size() != 1)
        invalid("planner.R_" + name, "expected 1 value");
    for (int i = 0; i < 2; ++i) {
        result.Q[i] = number((*q)[i], "planner.Q_" + name);
    }
    result.R = number((*r)[0], "planner.R_" + name);
    return result;
}

PlannerConfig planner(const Table& table) {
    PlannerConfig result;
    result.yaw_offset = angle(table, "planner", "yaw_offset", 0.0);
    result.pitch_offset = angle(table, "planner", "pitch_offset", 0.0);
    result.fire_thresh = read(table, "planner", "fire_thresh", result.fire_thresh);
    result.decision_speed = read(table, "planner", "decision_speed", result.decision_speed);
    result.high_speed_delay_time =
        read(table, "planner", "high_speed_delay_time", result.high_speed_delay_time);
    result.low_speed_delay_time =
        read(table, "planner", "low_speed_delay_time", result.low_speed_delay_time);
    result.max_target_age = read(table, "planner", "max_target_age", result.max_target_age);
    result.feedback_timeout =
        std::chrono::milliseconds(integer(table, "gimbal", "feedback_timeout_ms", 100));
    result.auto_fire = read(table, "planner", "auto_fire", false);
    result.yaw = axis(table, "yaw");
    result.pitch = axis(table, "pitch");
    validate(result);
    return result;
}

AimerConfig aimer(const Table& table) {
    AimerConfig result;
    result.yaw_offset = angle(table, "aimer", "yaw_offset", 0.0);
    result.pitch_offset = angle(table, "aimer", "pitch_offset", 0.0);
    result.coming_angle = angle(table, "aimer", "coming_angle", 60.0);
    result.leaving_angle = angle(table, "aimer", "leaving_angle", 20.0);
    result.decision_speed = read(table, "aimer", "decision_speed", result.decision_speed);
    result.high_speed_delay_time =
        read(table, "aimer", "high_speed_delay_time", result.high_speed_delay_time);
    result.low_speed_delay_time =
        read(table, "aimer", "low_speed_delay_time", result.low_speed_delay_time);
    validate(result);
    return result;
}

ShooterConfig shooter(const Table& table) {
    ShooterConfig result;
    result.first_tolerance = angle(table, "shooter", "first_tolerance", 5.0);
    result.second_tolerance = angle(table, "shooter", "second_tolerance", 2.0);
    result.judge_distance = read(table, "shooter", "judge_distance", result.judge_distance);
    result.auto_fire = read(table, "shooter", "auto_fire", false);
    validate(result);
    return result;
}

} // namespace

void validate(const CalibrationConfig& config) {
    for (const auto& [name, rotation] :
         {std::pair{"camera.R_gimbal2imubody", config.R_gimbal2imubody},
          std::pair{"camera.R_camera2gimbal", config.R_camera2gimbal}}) {
        if (!rotation.allFinite() ||
            (rotation.transpose() * rotation - Eigen::Matrix3d::Identity()).norm() > 1e-3 ||
            std::abs(rotation.determinant() - 1.0) > 1e-3) {
            invalid(name, "must be a finite rotation matrix with determinant +1");
        }
    }
    if (!config.t_camera2gimbal.allFinite())
        invalid("camera.t_camera2gimbal", "must be finite");
    const auto& matrix = config.camera_matrix;
    if (!matrix.allFinite() || matrix(0, 0) <= 0 || matrix(1, 1) <= 0 ||
        (matrix.row(2) - Eigen::RowVector3d(0, 0, 1)).norm() > 1e-9) {
        invalid("camera.camera_matrix", "invalid camera intrinsics");
    }
    if (!config.distort_coeffs.allFinite())
        invalid("camera.distort_coeffs", "must be finite");
}

void validate(const SolverConfig& config) {
    positive(config.max_reprojection_error, "solver.max_reprojection_error");
    positive(config.min_type_error_gap, "solver.min_type_error_gap");
    positive(config.size_noise_floor, "solver.size_noise_floor");
    if (config.size_noise_floor > config.max_reprojection_error) {
        invalid("solver.size_noise_floor", "must not exceed max_reprojection_error");
    }
    positive(config.max_balance_pitch, "solver.max_balance_pitch");
    if (config.max_balance_pitch >= std::numbers::pi / 2) {
        invalid("solver.max_balance_pitch", "must be below 90 degrees");
    }
}

void validate(const ClassifierConfig& config) {
    if (config.model_path.empty())
        invalid("classifier.classify_model", "must not be empty");
}

void validate(const DetectorConfig& config) {
    nonnegative(config.threshold, "detector.threshold");
    if (config.threshold > 255)
        invalid("detector.threshold", "must not exceed 255");
    positive(config.max_angle_error, "detector.max_angle_error");
    positive(config.min_lightbar_ratio, "detector.min_lightbar_ratio");
    positive(config.max_lightbar_ratio, "detector.max_lightbar_ratio");
    positive(config.min_lightbar_length, "detector.min_lightbar_length");
    positive(config.min_armor_ratio, "detector.min_armor_ratio");
    positive(config.max_armor_ratio, "detector.max_armor_ratio");
    positive(config.max_side_ratio, "detector.max_side_ratio");
    positive(config.max_rectangular_error, "detector.max_rectangular_error");
    probability(config.min_confidence, "detector.min_confidence");
    if (config.max_lightbar_ratio <= config.min_lightbar_ratio) {
        invalid("detector.max_lightbar_ratio", "must exceed min_lightbar_ratio");
    }
    if (config.max_armor_ratio <= config.min_armor_ratio) {
        invalid("detector.max_armor_ratio", "must exceed min_armor_ratio");
    }
}

void validate(const YoloConfig& config) {
    if (config.name != "yolov8")
        invalid("yolo.yolo_name", "only yolov8 is supported");
    if (config.model_path.empty())
        invalid("yolo.yolo_model", "must not be empty");
    if (config.device.empty())
        invalid("yolo.device", "must not be empty");
    probability(config.min_confidence, "detector.min_confidence");
    if (config.use_roi && (config.roi_width == 0 || config.roi_height == 0 ||
                           config.roi_width < -1 || config.roi_height < -1)) {
        invalid("yolo.roi", "width and height must be positive or -1");
    }
}

void validate(const TrackerConfig& config) {
    if (config.enemy_color != "red" && config.enemy_color != "blue") {
        invalid("tracker.enemy_color", "expected red or blue");
    }
    if (config.priority_mode != 1 && config.priority_mode != 2) {
        invalid("tracker.priority_mode", "expected 1 or 2");
    }
    if (config.min_detect_count < 1)
        invalid("tracker.min_detect_count", "must be positive");
    if (config.max_temp_lost_count < 0)
        invalid("tracker.max_temp_lost_count", "must be nonnegative");
    positive(config.max_match_distance, "tracker.max_match_distance");
}

void validate(const PlannerConfig& config) {
    finite(config.yaw_offset, "planner.yaw_offset");
    finite(config.pitch_offset, "planner.pitch_offset");
    nonnegative(config.fire_thresh, "planner.fire_thresh");
    nonnegative(config.decision_speed, "planner.decision_speed");
    nonnegative(config.high_speed_delay_time, "planner.high_speed_delay_time");
    nonnegative(config.low_speed_delay_time, "planner.low_speed_delay_time");
    positive(config.max_target_age, "planner.max_target_age");
    if (config.feedback_timeout.count() <= 0)
        invalid("gimbal.feedback_timeout_ms", "must be positive");
    if (config.feedback_timeout.count() > std::numeric_limits<int>::max()) {
        invalid("gimbal.feedback_timeout_ms", "milliseconds exceed the supported integer range");
    }
    for (const auto& [name, axis] :
         {std::pair{"yaw", config.yaw}, std::pair{"pitch", config.pitch}}) {
        positive(axis.max_acc, std::string("planner.max_") + name + "_acc");
        positive(axis.R, std::string("planner.R_") + name);
        for (const double weight : axis.Q)
            nonnegative(weight, std::string("planner.Q_") + name);
    }
}

void validate(const AimerConfig& config) {
    finite(config.yaw_offset, "aimer.yaw_offset");
    finite(config.pitch_offset, "aimer.pitch_offset");
    positive(config.coming_angle, "aimer.coming_angle");
    nonnegative(config.leaving_angle, "aimer.leaving_angle");
    nonnegative(config.decision_speed, "aimer.decision_speed");
    nonnegative(config.high_speed_delay_time, "aimer.high_speed_delay_time");
    nonnegative(config.low_speed_delay_time, "aimer.low_speed_delay_time");
}

void validate(const ShooterConfig& config) {
    positive(config.first_tolerance, "shooter.first_tolerance");
    positive(config.second_tolerance, "shooter.second_tolerance");
    positive(config.judge_distance, "shooter.judge_distance");
}

AutoAimConfig load_auto_aim_config(const std::string& path) {
    const Document document(path);
    const auto& table = document.table;
    AutoAimConfig result;
    result.calibration = calibration(table);
    result.solver = solver(table);
    result.classifier = classifier(document);
    result.detector = detector(table);
    result.yolo = yolo(document);
    result.tracker = tracker(table);
    result.planner = planner(table);
    result.aimer = aimer(table);
    result.shooter = shooter(table);

    if (const auto source = table["camera"]["source"]; source) {
        if (source.is_integer()) {
            const int index = integer(table, "camera", "source", 0);
            if (index < 0)
                invalid("camera.source", "camera index must be nonnegative");
            result.camera.source = index;
        } else {
            auto value = read<std::string>(table, "camera", "source", "");
            if (value.empty())
                invalid("camera.source", "must not be empty");
            if (value.find("://") == std::string::npos)
                value = document.resource(value);
            result.camera.source = value;
        }
    }
    result.camera.width = integer(table, "camera", "width", 0);
    result.camera.height = integer(table, "camera", "height", 0);
    result.camera.fps = read(table, "camera", "fps", 200.0);
    io::validate(result.camera);

    result.gimbal.port = read(table, "gimbal", "com_port", result.gimbal.port);
    result.gimbal.baudrate = integer(table, "gimbal", "baudrate", result.gimbal.baudrate);
    result.gimbal.feedback_timeout = result.planner.feedback_timeout;
    result.gimbal.reconnect_delay =
        std::chrono::milliseconds(integer(table, "gimbal", "reconnect_delay_ms", 250));
    result.gimbal.fire_threshold = result.planner.fire_thresh;
    io::validate(result.gimbal);
    return result;
}

SolverSettings load_solver_settings(const std::string& path) {
    const Document document(path);
    return {calibration(document.table), solver(document.table)};
}

VisionConfig load_vision_config(const std::string& path) {
    const Document document(path);
    return {yolo(document), classifier(document), detector(document.table)};
}

ClassifierConfig load_classifier_config(const std::string& path) {
    return classifier(Document(path));
}
TrackerConfig load_tracker_config(const std::string& path) {
    return tracker(Document(path).table);
}
PlannerConfig load_planner_config(const std::string& path) {
    return planner(Document(path).table);
}
AimerConfig load_aimer_config(const std::string& path) {
    return aimer(Document(path).table);
}
ShooterConfig load_shooter_config(const std::string& path) {
    return shooter(Document(path).table);
}

} // namespace app::auto_aim
