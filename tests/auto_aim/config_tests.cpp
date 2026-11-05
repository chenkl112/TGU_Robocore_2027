#include "app/auto_aim/aimer.hpp"
#include "app/auto_aim/config.hpp"
#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/shooter.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/tracker.hpp"
#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"
#ifdef TGU_TEST_WITH_VISION
#include "app/auto_aim/classifier.hpp"
#include "app/auto_aim/detector.hpp"
#include "app/auto_aim/yolo.hpp"
#endif

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>

namespace {
using namespace app::auto_aim;
namespace fs = std::filesystem;

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

void close_to(double actual, double expected, const char* field) {
    require(std::abs(actual - expected) < 1e-14, std::string("Incorrect units for ") + field);
}

struct TemporaryDirectory {
    fs::path path = fs::temp_directory_path() /
                    ("robocore-config-" +
                     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    TemporaryDirectory() {
        require(fs::create_directory(path), "Could not create temporary test directory");
    }
    ~TemporaryDirectory() {
        std::error_code ignored;
        fs::remove_all(path, ignored);
    }
};

struct WorkingDirectory {
    fs::path original = fs::current_path();
    explicit WorkingDirectory(const fs::path& directory) {
        fs::current_path(directory);
    }
    ~WorkingDirectory() {
        std::error_code ignored;
        fs::current_path(original, ignored);
    }
};

void write(const fs::path& path, const toml::table& table) {
    std::ofstream output(path);
    output << table;
    require(output.good(), "Could not write temporary configuration");
}

void rejects(const std::function<void()>& operation, const std::string& field) {
    try {
        operation();
    } catch (const std::exception& error) {
        require(std::string(error.what()).find(field) != std::string::npos,
                "Validation error did not identify " + field + ": " + error.what());
        return;
    }
    throw std::runtime_error("Invalid configuration was accepted for " + field);
}

void unit_conversions(toml::table config, const fs::path& path) {
    config["solver"].as_table()->insert_or_assign("max_balance_pitch", 45.0);
    config["detector"].as_table()->insert_or_assign("max_angle_error", 90.0);
    config["detector"].as_table()->insert_or_assign("max_rectangular_error", 45.0);
    config["planner"].as_table()->insert_or_assign("yaw_offset", 90.0);
    config["planner"].as_table()->insert_or_assign("pitch_offset", -30.0);
    config["planner"].as_table()->insert_or_assign("fire_thresh", 0.123);
    config["aimer"].as_table()->insert_or_assign("yaw_offset", 90.0);
    config["aimer"].as_table()->insert_or_assign("pitch_offset", -30.0);
    config["aimer"].as_table()->insert_or_assign("coming_angle", 60.0);
    config["aimer"].as_table()->insert_or_assign("leaving_angle", 20.0);
    config["shooter"].as_table()->insert_or_assign("first_tolerance", 90.0);
    config["shooter"].as_table()->insert_or_assign("second_tolerance", 30.0);
    config["gimbal"].as_table()->insert_or_assign("feedback_timeout_ms", 123);
    write(path, config);
    const auto loaded = load_auto_aim_config(path.string());
    constexpr double pi = std::numbers::pi;
    close_to(loaded.solver.max_balance_pitch, pi / 4, "solver.max_balance_pitch");
    close_to(loaded.detector.max_angle_error, pi / 2, "detector.max_angle_error");
    close_to(loaded.detector.max_rectangular_error, pi / 4, "detector.max_rectangular_error");
    close_to(loaded.planner.yaw_offset, pi / 2, "planner.yaw_offset");
    close_to(loaded.planner.pitch_offset, -pi / 6, "planner.pitch_offset");
    close_to(loaded.planner.fire_thresh, 0.123, "planner.fire_thresh");
    close_to(loaded.aimer.yaw_offset, pi / 2, "aimer.yaw_offset");
    close_to(loaded.aimer.pitch_offset, -pi / 6, "aimer.pitch_offset");
    close_to(loaded.aimer.coming_angle, pi / 3, "aimer.coming_angle");
    close_to(loaded.aimer.leaving_angle, pi / 9, "aimer.leaving_angle");
    close_to(loaded.shooter.first_tolerance, pi / 2, "shooter.first_tolerance");
    close_to(loaded.shooter.second_tolerance, pi / 6, "shooter.second_tolerance");
    require(loaded.gimbal.feedback_timeout == std::chrono::milliseconds(123) &&
                loaded.planner.feedback_timeout == loaded.gimbal.feedback_timeout,
            "Gimbal and planner did not share the converted feedback timeout");
}

void invalid_documents(const toml::table& baseline, const fs::path& path) {
    using Mutation = std::function<void(toml::table&)>;
    const std::pair<const char*, Mutation> cases[] = {
        {"camera.camera_matrix", [](auto& c) { c["camera"].as_table()->erase("camera_matrix"); }},
        {"camera.R_camera2gimbal",
         [](auto& c) {
             c["camera"].as_table()->insert_or_assign("R_camera2gimbal", toml::array{1, 2});
         }},
        {"camera.distort_coeffs",
         [](auto& c) {
             c["camera"].as_table()->insert_or_assign("distort_coeffs",
                                                      toml::array{0, 0, "bad", 0, 0});
         }},
        {"camera.source",
         [](auto& c) { c["camera"].as_table()->insert_or_assign("source", true); }},
        {"camera.source", [](auto& c) { c["camera"].as_table()->insert_or_assign("source", -1); }},
        {"camera.width", [](auto& c) { c["camera"].as_table()->insert_or_assign("width", -1); }},
        {"camera.fps",
         [](auto& c) {
             c["camera"].as_table()->insert_or_assign("fps",
                                                      std::numeric_limits<double>::quiet_NaN());
         }},
        {"solver.max_reprojection_error",
         [](auto& c) { c["solver"].as_table()->insert_or_assign("max_reprojection_error", 0.0); }},
        {"solver.size_noise_floor",
         [](auto& c) { c["solver"].as_table()->insert_or_assign("size_noise_floor", 100.0); }},
        {"solver.max_balance_pitch",
         [](auto& c) { c["solver"].as_table()->insert_or_assign("max_balance_pitch", 90.0); }},
        {"detector.min_confidence",
         [](auto& c) { c["detector"].as_table()->insert_or_assign("min_confidence", 1.1); }},
        {"tracker.min_detect_count",
         [](auto& c) { c["tracker"].as_table()->insert_or_assign("min_detect_count", "five"); }},
        {"tracker.min_detect_count",
         [](auto& c) {
             c["tracker"].as_table()->insert_or_assign("min_detect_count",
                                                       std::numeric_limits<std::int64_t>::max());
         }},
        {"tracker.priority_mode",
         [](auto& c) { c["tracker"].as_table()->insert_or_assign("priority_mode", 3); }},
        {"tracker.enemy_color",
         [](auto& c) { c["tracker"].as_table()->insert_or_assign("enemy_color", "green"); }},
        {"yolo.use_roi", [](auto& c) { c["yolo"].as_table()->insert_or_assign("use_roi", "yes"); }},
        {"yolo.roi",
         [](auto& c) {
             c["yolo"].as_table()->insert_or_assign("use_roi", true);
             c["yolo"].as_table()->insert_or_assign("roi_width", 0);
         }},
        {"yolo.yolo_name",
         [](auto& c) { c["yolo"].as_table()->insert_or_assign("yolo_name", "unsupported"); }},
        {"planner.Q_yaw", [](auto& c) { c["planner"].as_table()->erase("Q_yaw"); }},
        {"planner.Q_pitch",
         [](auto& c) {
             c["planner"].as_table()->insert_or_assign("Q_pitch", toml::array{1, "bad"});
         }},
        {"planner.R_yaw",
         [](auto& c) { c["planner"].as_table()->insert_or_assign("R_yaw", toml::array{0.0}); }},
        {"gimbal.feedback_timeout_ms",
         [](auto& c) { c["gimbal"].as_table()->insert_or_assign("feedback_timeout_ms", 0); }},
        {"gimbal.reconnect_delay_ms",
         [](auto& c) { c["gimbal"].as_table()->insert_or_assign("reconnect_delay_ms", -1); }},
        {"paths.base_dir",
         [](auto& c) { c.insert_or_assign("paths", toml::table{{"base_dir", 7}}); }},
        {"tracker", [](auto& c) { c.insert_or_assign("tracker", "invalid section type"); }},
        {"yolo", [](auto& c) { c.insert_or_assign("yolo", 5); }},
    };
    // TOML 的宽松 value<T>() 会把整数当布尔、布尔当整数；配置契约要求保留类型错误。
    const std::pair<const char*, Mutation> strict_types[] = {
        {"yolo.use_roi", [](auto& c) { c["yolo"].as_table()->insert_or_assign("use_roi", 2); }},
        {"planner.auto_fire",
         [](auto& c) { c["planner"].as_table()->insert_or_assign("auto_fire", 1); }},
        {"tracker.min_detect_count",
         [](auto& c) { c["tracker"].as_table()->insert_or_assign("min_detect_count", true); }},
        {"tracker.min_detect_count",
         [](auto& c) { c["tracker"].as_table()->insert_or_assign("min_detect_count", 5.0); }},
        {"camera.width", [](auto& c) { c["camera"].as_table()->insert_or_assign("width", 1e100); }},
    };
    for (const auto& [field, mutate] : strict_types) {
        auto changed = baseline;
        mutate(changed);
        write(path, changed);
        rejects([&] { load_auto_aim_config(path.string()); }, field);
    }
    for (const auto& [field, mutate] : cases) {
        auto changed = baseline;
        mutate(changed);
        write(path, changed);
        rejects([&] { load_auto_aim_config(path.string()); }, field);
    }
}

void path_resolution(toml::table config, const fs::path& directory) {
    const auto config_directory = directory / "settings";
    fs::create_directory(config_directory);
    const auto path = config_directory / "aim.toml";
    config.insert_or_assign("paths", toml::table{{"base_dir", "../resources"}});
    config["yolo"].as_table()->insert_or_assign("yolo_model", "models/yolo.xml");
    config["classifier"].as_table()->insert_or_assign("classify_model", "models/classifier.onnx");
    config["camera"].as_table()->insert_or_assign("source", "clips/frame.avi");
    write(path, config);
    WorkingDirectory different_cwd(directory);
    const auto loaded = load_auto_aim_config(path.string());
    const auto base = directory / "resources";
    require(loaded.yolo.model_path == (base / "models/yolo.xml").string() &&
                loaded.classifier.model_path == (base / "models/classifier.onnx").string() &&
                std::get<std::string>(loaded.camera.source) == (base / "clips/frame.avi").string(),
            "Relative paths did not resolve against explicit config-relative base_dir");
    config["yolo"].as_table()->insert_or_assign("yolo_model", "/tmp/absolute-model.xml");
    config["camera"].as_table()->insert_or_assign("source", "rtsp://127.0.0.1/camera");
    write(path, config);
    const auto absolute = load_auto_aim_config(path.string());
    require(absolute.yolo.model_path == "/tmp/absolute-model.xml" &&
                std::get<std::string>(absolute.camera.source) == "rtsp://127.0.0.1/camera",
            "Resource base changed an absolute path or stream URL");
    config.erase("paths");
    config["yolo"].as_table()->insert_or_assign("yolo_model", "legacy/model.xml");
    write(path, config);
    require(load_auto_aim_config(path.string()).yolo.model_path ==
                (directory / "legacy/model.xml").string(),
            "Configurations without base_dir lost the documented current-directory behavior");
}

void typed_validation(const AutoAimConfig& valid) {
    auto calibration = valid.calibration;
    calibration.camera_matrix(0, 0) = 0;
    rejects([&] { validate(calibration); }, "camera.camera_matrix");
    auto solver = valid.solver;
    solver.min_type_error_gap = -1;
    rejects([&] { Solver instance(valid.calibration, solver); }, "solver.min_type_error_gap");
    auto classifier = valid.classifier;
    classifier.model_path.clear();
    rejects([&] { validate(classifier); }, "classifier.classify_model");
    auto detector = valid.detector;
    detector.max_lightbar_ratio = detector.min_lightbar_ratio;
    rejects([&] { validate(detector); }, "detector.max_lightbar_ratio");
    auto yolo = valid.yolo;
    yolo.device.clear();
    rejects([&] { validate(yolo); }, "yolo.device");
    auto tracker = valid.tracker;
    tracker.min_detect_count = 0;
    Solver pose(valid.calibration, valid.solver);
    rejects([&] { Tracker instance(tracker, pose); }, "tracker.min_detect_count");
    auto planner = valid.planner;
    planner.yaw.Q[0] = -1;
    rejects([&] { Planner instance(planner); }, "planner.Q_yaw");
    planner = valid.planner;
    planner.feedback_timeout = std::chrono::milliseconds::max();
    rejects([&] { Planner instance(planner); }, "gimbal.feedback_timeout_ms");
    auto aimer = valid.aimer;
    aimer.decision_speed = std::numeric_limits<double>::quiet_NaN();
    rejects([&] { Aimer instance(aimer); }, "aimer.decision_speed");
    auto shooter = valid.shooter;
    shooter.first_tolerance = 0;
    rejects([&] { Shooter instance(shooter); }, "shooter.first_tolerance");
}

void loaded_values_are_independent(toml::table config, const fs::path& path) {
    // 加载后删除配置并切换工作目录，typed 构造只能使用已解析参数及绝对资源路径。
    write(path, config);
    auto loaded = load_auto_aim_config(path.string());
    fs::remove(path);
    WorkingDirectory different_cwd(path.parent_path());
    Solver solver(loaded.calibration, loaded.solver);
    Tracker tracker(loaded.tracker, solver);
    Planner planner(loaded.planner);
    Aimer aimer(loaded.aimer);
    Shooter shooter(loaded.shooter);
    require(tracker.state() == TrackerState::lost &&
                !planner.plan(std::nullopt, 22, TargetClock::now()).control,
            "Typed algorithm objects did not initialize from the loaded values");
#ifdef TGU_TEST_WITH_VISION
    loaded.yolo.device = "CPU";
    Classifier classifier(loaded.classifier);
    Detector detector(loaded.detector, loaded.classifier, false);
    YOLO yolo(loaded.yolo, loaded.classifier, false);
    require(detector.detect(cv::Mat()).empty() &&
                yolo.detect(cv::Mat::zeros(480, 640, CV_8UC3)).empty(),
            "Typed vision objects failed after the original configuration was removed");
#endif
}
} // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected default configuration path");
        const auto original = load_auto_aim_config(argv[1]);
        auto baseline = toml::parse_file(argv[1]);
        // 临时文件与原配置不在同一目录，显式固定资源根目录以隔离被测路径行为。
        baseline["yolo"].as_table()->insert_or_assign("yolo_model", original.yolo.model_path);
        baseline["classifier"].as_table()->insert_or_assign("classify_model",
                                                            original.classifier.model_path);
        TemporaryDirectory temporary;
        const auto path = temporary.path / "aim.toml";
        unit_conversions(baseline, path);
        invalid_documents(baseline, path);
        path_resolution(baseline, temporary.path);
        typed_validation(original);
        loaded_values_are_independent(baseline, path);
        std::cout << "PASS config units, validation, resource paths and typed construction without "
                     "rereading\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
