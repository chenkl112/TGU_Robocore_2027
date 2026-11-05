#include "yolov8.hpp"
#include "keypoints.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <format>
#include <limits>
#include <stdexcept>

#include "armor_image.hpp"
#include "roi.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

YOLOV8::YOLOV8(const std::string& config_path, bool debug)
    : YOLOV8(load_vision_config(config_path), debug) {}

YOLOV8::YOLOV8(const VisionConfig& config, bool debug)
    : YOLOV8(config.yolo, config.classifier, debug) {}

YOLOV8::YOLOV8(const YoloConfig& config, const ClassifierConfig& classifier, bool debug)
    : classifier_(classifier), device_(config.device), model_path_(config.model_path),
      debug_(debug), use_roi_(config.use_roi), min_confidence_(config.min_confidence),
      configured_roi_(config.roi_x, config.roi_y, config.roi_width, config.roi_height) {
    validate(config);
    auto model = core_.read_model(model_path_);
    const auto model_shape = model->input().get_shape();
    const ov::Shape expected_shape = {1, 3, network_height_, network_width_};
    if (model_shape != expected_shape) {
        throw std::runtime_error(std::format("YOLO model input must be NCHW 1x3x{}x{}, got {}",
                                             network_height_, network_width_,
                                             model->input().get_partial_shape().to_string()));
    }
    const ov::Shape expected_output_shape = {1, 14, 6300};
    if (model->output().get_shape() != expected_output_shape) {
        throw std::runtime_error(std::format("YOLO model output must be 1x14x6300, got {}",
                                             model->output().get_partial_shape().to_string()));
    }

    ov::preprocess::PrePostProcessor ppp(model);
    auto& input = ppp.input();

    input.tensor()
        .set_element_type(ov::element::u8)
        .set_shape({1, network_height_, network_width_, 3})
        .set_layout("NHWC")
        .set_color_format(ov::preprocess::ColorFormat::BGR);

    input.model().set_layout("NCHW");

    input.preprocess()
        .convert_element_type(ov::element::f32)
        .convert_color(ov::preprocess::ColorFormat::RGB)
        .scale(255.0);

    model = ppp.build();
    compiled_model_ = core_.compile_model(
        model, device_, ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
    input_image_.create(static_cast<int>(network_height_), static_cast<int>(network_width_),
                        CV_8UC3);
    input_tensor_ =
        ov::Tensor(ov::element::u8, {1, network_height_, network_width_, 3}, input_image_.data);
    infer_request_ = compiled_model_.create_infer_request();
    infer_request_.set_input_tensor(input_tensor_);
    LOG_INFO(MODULE, "Loaded YOLO model {} with 640x480 network input", model_path_);
}

std::list<Armor> YOLOV8::detect(const cv::Mat& raw_img, int frame_count) {
    if (raw_img.empty()) {
        LOG_WARN(MODULE, "Empty img, camera drop!");
        return std::list<Armor>();
    }

    if (raw_img.type() != CV_8UC3) {
        throw std::invalid_argument("YOLO expects an 8-bit BGR image");
    }
    const auto roi = effective_yolo_roi(configured_roi_, use_roi_, raw_img.size());
    if (roi.empty()) {
        LOG_ERROR(MODULE, "Configured YOLO ROI is outside the image");
        return {};
    }
    const cv::Mat bgr_img = raw_img(roi);
    const cv::Point2f offset(static_cast<float>(roi.x), static_cast<float>(roi.y));

    const auto height_scale = static_cast<double>(network_height_) / bgr_img.rows;
    const auto width_scale = static_cast<double>(network_width_) / bgr_img.cols;
    const auto scale = std::min(height_scale, width_scale);
    auto h = static_cast<int>(bgr_img.rows * scale);
    auto w = static_cast<int>(bgr_img.cols * scale);

    if (w <= 0 || h <= 0)
        return {};
    // 清空上一帧的补边，复用固定内存；宽高比变化时也不会遗留旧图像。
    input_image_.setTo(cv::Scalar::all(0));
    cv::resize(bgr_img, input_image_(cv::Rect(0, 0, w, h)), {w, h});
    infer_request_.infer();

    auto output_tensor = infer_request_.get_output_tensor();
    const auto output_shape = output_tensor.get_shape();
    if (output_shape != ov::Shape({1, 14, 6300}) ||
        output_tensor.get_element_type() != ov::element::f32) {
        throw std::runtime_error("YOLO output must be float32 1x14x6300");
    }
    cv::Mat output(static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
                   output_tensor.data<float>());

    return parse(scale, output, raw_img, frame_count, offset);
}

std::list<Armor> YOLOV8::parse(double scale, const cv::Mat& output, const cv::Mat& bgr_img,
                               int frame_count, cv::Point2f offset) {
    if (!std::isfinite(scale) || scale <= 0 || output.dims != 2 || output.type() != CV_32F ||
        output.rows != 14 || output.cols != 6300 || bgr_img.empty() || bgr_img.type() != CV_8UC3) {
        throw std::invalid_argument(
            "YOLO postprocess expects positive scale, float32 14x6300 output and BGR image");
    }
    cv::Mat rows;
    cv::transpose(output, rows);

    std::vector<int> ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armors_key_points;

    for (int r = 0; r < rows.rows; r++) {
        const auto row = rows.row(r);
        if (!cv::checkRange(row))
            continue;
        auto xywh = row.colRange(0, 4);
        auto scores = row.colRange(4, 4 + class_num_);
        auto one_key_points = row.colRange(4 + class_num_, 14);

        std::vector<cv::Point2f> armor_key_points;

        double score;
        cv::Point max_point;
        cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

        if (score < score_threshold_)
            continue;

        auto x = xywh.at<float>(0);
        auto y = xywh.at<float>(1);
        auto w = xywh.at<float>(2);
        auto h = xywh.at<float>(3);
        // 网络异常输出不能进入浮点转整数和 NMS 的边界运算。
        constexpr double coordinate_limit = std::numeric_limits<int>::max() / 8.0;
        if (w <= 0 || h <= 0 || std::abs(x / scale) > coordinate_limit ||
            std::abs(y / scale) > coordinate_limit || w / scale > coordinate_limit ||
            h / scale > coordinate_limit)
            continue;
        auto left = static_cast<int>((x - 0.5 * w) / scale);
        auto top = static_cast<int>((y - 0.5 * h) / scale);
        auto width = static_cast<int>(w / scale);
        auto height = static_cast<int>(h / scale);
        if (width <= 0 || height <= 0)
            continue;

        for (int i = 0; i < 4; i++) {
            float x = one_key_points.at<float>(0, i * 2 + 0) / scale;
            float y = one_key_points.at<float>(0, i * 2 + 1) / scale;
            cv::Point2f kp = {x, y};
            armor_key_points.push_back(kp);
        }
        ids.emplace_back(max_point.x);
        confidences.emplace_back(score);
        boxes.emplace_back(left, top, width, height);
        armors_key_points.emplace_back(armor_key_points);
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, score_threshold_, nms_threshold_, indices);

    std::list<Armor> armors;
    for (const auto& i : indices) {
        if (!decode_yolov8_keypoints(armors_key_points[i]))
            continue;
        armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i], offset);
    }

    for (auto it = armors.begin(); it != armors.end();) {
        it->pattern = get_pattern(bgr_img, *it);
        classifier_.classify(*it);

        if (!check_name(*it)) {
            it = armors.erase(it);
            continue;
        }

        it->type = armor_type_from_name(it->name);

        it->center_norm = get_center_norm(bgr_img, it->center);
        ++it;
    }

    if (debug_)
        draw_detections(bgr_img, armors, frame_count);

    return armors;
}

bool YOLOV8::check_name(const Armor& armor) const {
    const bool name_ok = is_robot_target(armor.name);
    auto confidence_ok = armor.confidence > min_confidence_;
    return name_ok && confidence_ok;
}

cv::Point2f YOLOV8::get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
}

cv::Mat YOLOV8::get_pattern(const cv::Mat& bgr_img, const Armor& armor) const {
    return extract_armor_pattern(bgr_img, armor.points);
}

void YOLOV8::draw_detections(const cv::Mat& img, const std::list<Armor>& armors,
                             int frame_count) const {
    auto detection = img.clone();
    tools::draw_text(detection, std::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
    for (const auto& armor : armors) {
        auto info = std::format("{:.2f} {} {}", armor.confidence, ARMOR_NAMES[armor.name],
                                ARMOR_TYPES[armor.type]);
        tools::draw_points(detection,
                           std::vector<cv::Point2f>(armor.points.begin(), armor.points.end()),
                           {0, 255, 0});
        tools::draw_text(detection, info, armor.center, {0, 255, 0});
    }
    if (use_roi_) {
        cv::rectangle(detection, effective_yolo_roi(configured_roi_, use_roi_, img.size()),
                      cv::Scalar(0, 255, 0), 2);
    }
    cv::resize(detection, detection, {}, 0.5, 0.5);
    cv::imshow("detection", detection);
}

std::list<Armor> YOLOV8::postprocess(double scale, const cv::Mat& output, const cv::Mat& bgr_img,
                                     int frame_count) {
    const auto roi = effective_yolo_roi(configured_roi_, use_roi_, bgr_img.size());
    if (roi.empty())
        return {};
    return parse(scale, output, bgr_img, frame_count,
                 {static_cast<float>(roi.x), static_cast<float>(roi.y)});
}

} // namespace app::auto_aim
