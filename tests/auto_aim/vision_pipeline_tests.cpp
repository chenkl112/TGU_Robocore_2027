#include "app/auto_aim/config.hpp"
#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/tracker.hpp"
#include "app/auto_aim/yolo.hpp"
#include "app/auto_aim/yolos/roi.hpp"
#include "tools/logger.hpp"
#include "vision_fixture.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace app::auto_aim;
using namespace std::chrono_literals;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void require_hero(const std::list<Armor>& detections) {
    require(detections.size() == 1, "Expected exactly one real-model armor detection");
    const auto& armor = detections.front();
    require(armor.name == ArmorName::one && armor.type == ArmorType::big &&
                armor.color == Color::red && armor.confidence > 0.8,
            "Real classifier did not identify the red hero with normal confidence");
    require(!armor.pose_valid, "Detector presented an unsolved pose as valid");
}

void same_detection(const Armor& a, const Armor& b) {
    require(a.name == b.name && a.type == b.type && a.box == b.box,
            "Resolution round trip changed the detection metadata");
    for (std::size_t i = 0; i < a.points.size(); ++i) {
        require(cv::norm(a.points[i] - b.points[i]) < 1e-4,
                "Resolution round trip changed physical corner coordinates");
    }
}

void roi_resolution_roundtrip(const VisionConfig& settings, const cv::Mat& image) {
    auto config = settings.yolo;
    config.device = "CPU";
    config.use_roi = true;
    config.roi_x = config.roi_y = 0;
    config.roi_width = config.roi_height = -1;
    YOLO yolo(config, settings.classifier, false);
    const auto initial = yolo.detect(image);
    std::cout << "ROI initial detections=" << initial.size() << std::endl;
    require_hero(initial);
    for (const auto size : {cv::Size(160, 120), cv::Size(960, 540), cv::Size(320, 80)}) {
        require(yolo.detect(cv::Mat::zeros(size, CV_8UC3)).empty(),
                "Reused inference buffer retained a previous detection");
        const auto restored = yolo.detect(image);
        std::cout << "ROI restored after " << size << " detections=" << restored.size()
                  << std::endl;
        require_hero(restored);
        same_detection(initial.front(), restored.front());
    }

    // 固定 ROI 被小图裁剪甚至完全落在图外之后，也必须恢复到配置的范围。
    config.roi_x = 100;
    config.roi_y = 50;
    config.roi_width = 640;
    config.roi_height = 480;
    YOLO offset_yolo(config, settings.classifier, false);
    cv::Mat canvas = cv::Mat::zeros(580, 840, CV_8UC3);
    image.copyTo(canvas(cv::Rect(100, 50, 640, 480)));
    const auto offset_initial = offset_yolo.detect(canvas);
    std::cout << "ROI offset detections=" << offset_initial.size() << std::endl;
    require_hero(offset_initial);
    for (const auto size : {cv::Size(160, 120), cv::Size(80, 40)}) {
        require(offset_yolo.detect(cv::Mat::zeros(size, CV_8UC3)).empty(),
                "Blank clipped ROI produced a detection");
        const auto restored = offset_yolo.detect(canvas);
        require_hero(restored);
        same_detection(offset_initial.front(), restored.front());
    }
    const auto& shifted = offset_initial.front();
    const auto& original = initial.front();
    require(shifted.box == original.box + cv::Point(100, 50),
            "ROI box was not restored to source-image coordinates");
    for (std::size_t i = 0; i < original.points.size(); ++i) {
        require(cv::norm(shifted.points[i] - original.points[i] - cv::Point2f(100, 50)) < 1e-4,
                "ROI keypoint was not restored to source-image coordinates");
    }

    cv::Mat invalid_shape = cv::Mat::zeros(13, 6300, CV_32F);
    bool rejected = false;
    try {
        yolo.postprocess(1, invalid_shape, image, 0);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }
    require(rejected, "Malformed model output was accepted");
    cv::Mat output = cv::Mat::zeros(14, 6300, CV_32F);
    output.at<float>(0, 0) = std::numeric_limits<float>::infinity();
    output.at<float>(4, 0) = 1;
    require(yolo.postprocess(1, output, image, 0).empty(), "Nonfinite model output was accepted");
    require(output.rows == 14 && output.cols == 6300,
            "Postprocessing mutated the caller's tensor shape");
}

void detection_to_planning(const std::string& path, const VisionConfig& settings,
                           const cv::Mat& image) {
    auto yolo_config = settings.yolo;
    yolo_config.device = "CPU";
    yolo_config.use_roi = false;
    YOLO yolo(yolo_config, settings.classifier, false);
    auto calibration = load_solver_settings(path).calibration;
    // 本夹具的相机中心与图像中心一致；相机右/下/前映射到云台前/左/上。
    calibration.camera_matrix << 800, 0, 320, 0, 800, 240, 0, 0, 1;
    calibration.R_camera2gimbal << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    calibration.R_gimbal2imubody.setIdentity();
    calibration.t_camera2gimbal.setZero();
    calibration.distort_coeffs.setZero();
    Solver solver(calibration, load_solver_settings(path).solver);
    auto tracker_config = load_tracker_config(path);
    Tracker tracker(tracker_config, solver);
    Planner planner(load_planner_config(path));
    const auto start = TargetClock::now();
    std::optional<TargetSnapshot> snapshot;
    for (int frame = 0; frame < tracker_config.min_detect_count + 3; ++frame) {
        auto detections = yolo.detect(image, frame);
        require_hero(detections);
        const auto now = start + frame * 10ms;
        snapshot = tracker.track_snapshot(detections, now);
        if (frame + 1 < tracker_config.min_detect_count) {
            require(!snapshot, "Tracker confirmed a target before the configured frame count");
        } else {
            require(snapshot.has_value() && detections.front().pose_valid,
                    "Real model detections failed to produce a solved tracking snapshot");
            const auto plan = planner.plan(snapshot, 22, now);
            require(plan.control && std::isfinite(plan.yaw) && std::isfinite(plan.pitch),
                    "Confirmed real-model target did not produce a finite control plan");
            require(!plan.fire, "Pipeline fired without gimbal feedback");
        }
    }
    std::cout << "PASS real YOLO -> classifier -> pose -> tracker -> planner\n";
}
} // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected configuration path");
        const auto settings = load_vision_config(argv[1]);
        const auto image = vision_fixture::hero_image();
        roi_resolution_roundtrip(settings, image);
        detection_to_planning(argv[1], settings, image);
        std::cout << "PASS ROI resolution round trips, inference reuse and output contracts\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
