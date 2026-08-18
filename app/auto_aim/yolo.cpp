#include "yolo.hpp"

#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

#include "yolos/yolov8.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

YOLO::YOLO(const std::string& config_path, bool debug) {
    auto config = toml::parse_file(config_path);
    auto yolo_name = config["yolo"]["yolo_name"].value_or("yolov8");

    if (yolo_name == "yolov8") {
        yolo_ = std::make_unique<YOLOV8>(config_path, debug);
    } else {
        LOG_ERROR(MODULE, "v0.1 only supports yolov8, got: {}", yolo_name);
        throw std::runtime_error("v0.1 only supports yolov8: " + yolo_name);
    }
}

std::list<Armor> YOLO::detect(const cv::Mat& img, int frame_count) {
    return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(
    double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count) {
    return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

}  // namespace app::auto_aim
