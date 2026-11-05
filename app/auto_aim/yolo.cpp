#include "yolo.hpp"

#include <stdexcept>

#include "tools/logger.hpp"

#include "yolos/yolov8.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

YOLO::YOLO(const std::string& config_path, bool debug)
    : YOLO(load_vision_config(config_path), debug) {}

YOLO::YOLO(const VisionConfig& config, bool debug) : YOLO(config.yolo, config.classifier, debug) {}

YOLO::YOLO(const YoloConfig& config, const ClassifierConfig& classifier, bool debug) {
    validate(config);
    yolo_ = std::make_unique<YOLOV8>(config, classifier, debug);
}

std::list<Armor> YOLO::detect(const cv::Mat& img, int frame_count) {
    return yolo_->detect(img, frame_count);
}

std::list<Armor> YOLO::postprocess(double scale, const cv::Mat& output, const cv::Mat& bgr_img,
                                   int frame_count) {
    return yolo_->postprocess(scale, output, bgr_img, frame_count);
}

} // namespace app::auto_aim
