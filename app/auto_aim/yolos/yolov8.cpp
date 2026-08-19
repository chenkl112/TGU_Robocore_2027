#include "yolov8.hpp"

#include <algorithm>
#include <cstddef>
#include <filesystem>
#include <format>
#include <random>
#include <stdexcept>

#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

YOLOV8::YOLOV8(const std::string& config_path, bool debug)
    : classifier_(config_path), debug_(debug) {
    auto config = toml::parse_file(config_path);

    model_path_ = config["yolo"]["yolo_model"].value_or("config/yolov8_armor.onnx");
    device_ = config["yolo"]["device"].value_or("AUTO");
    min_confidence_ = config["detector"]["min_confidence"].value_or(0.6);

    use_roi_ = config["yolo"]["use_roi"].value_or(false);
    int x = config["yolo"]["roi_x"].value_or(0);
    int y = config["yolo"]["roi_y"].value_or(0);
    int width = config["yolo"]["roi_width"].value_or(-1);
    int height = config["yolo"]["roi_height"].value_or(-1);
    roi_ = cv::Rect(x, y, width, height);
    offset_ = cv::Point2f(x, y);

    auto model = core_.read_model(model_path_);
    const auto model_shape = model->input().get_shape();
    const ov::Shape expected_shape = {1, 3, network_height_, network_width_};
    if (model_shape != expected_shape) {
        throw std::runtime_error(std::format(
            "YOLO model input must be NCHW 1x3x{}x{}, got {}", network_height_,
            network_width_, model->input().get_partial_shape().to_string()));
    }
    const ov::Shape expected_output_shape = {1, 14, 6300};
    if (model->output().get_shape() != expected_output_shape) {
        throw std::runtime_error(std::format(
            "YOLO model output must be 1x14x6300, got {}",
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
    LOG_INFO(
        MODULE, "Loaded YOLO model {} with 640x480 network input", model_path_);
}

std::list<Armor> YOLOV8::detect(const cv::Mat& raw_img, int frame_count) {
    if (raw_img.empty()) {
        LOG_WARN(MODULE, "Empty img, camera drop!");
        return std::list<Armor>();
    }

    cv::Mat bgr_img;
    if (use_roi_) {
        if (roi_.width < 0) roi_.width = raw_img.cols - roi_.x;
        if (roi_.height < 0) roi_.height = raw_img.rows - roi_.y;
        roi_ &= cv::Rect(0, 0, raw_img.cols, raw_img.rows);
        if (roi_.empty()) {
            LOG_ERROR(MODULE, "Configured YOLO ROI is outside the image");
            return {};
        }
        offset_ = cv::Point2f(static_cast<float>(roi_.x), static_cast<float>(roi_.y));
        bgr_img = raw_img(roi_);
    } else {
        bgr_img = raw_img;
    }

    const auto height_scale = static_cast<double>(network_height_) / bgr_img.rows;
    const auto width_scale = static_cast<double>(network_width_) / bgr_img.cols;
    const auto scale = std::min(height_scale, width_scale);
    auto h = static_cast<int>(bgr_img.rows * scale);
    auto w = static_cast<int>(bgr_img.cols * scale);

    auto input = cv::Mat(
        static_cast<int>(network_height_), static_cast<int>(network_width_), CV_8UC3,
        cv::Scalar(0, 0, 0));
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(bgr_img, input(roi), {w, h});
    ov::Tensor input_tensor(
        ov::element::u8, {1, network_height_, network_width_, 3}, input.data);

    auto infer_request = compiled_model_.create_infer_request();
    infer_request.set_input_tensor(input_tensor);
    infer_request.infer();

    auto output_tensor = infer_request.get_output_tensor();
    auto output_shape = output_tensor.get_shape();
    constexpr std::size_t expected_output_channels = 14;
    if (output_shape.size() != 3 || output_shape[1] != expected_output_channels) {
        LOG_ERROR(MODULE, "Unexpected YOLO output shape");
        return {};
    }
    cv::Mat output(
        static_cast<int>(output_shape[1]), static_cast<int>(output_shape[2]), CV_32F,
        output_tensor.data<float>());

    return parse(scale, output, raw_img, frame_count);
}

std::list<Armor> YOLOV8::parse(
    double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count) {
    cv::transpose(output, output);

    std::vector<int> ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    std::vector<std::vector<cv::Point2f>> armors_key_points;

    for (int r = 0; r < output.rows; r++) {
        auto xywh = output.row(r).colRange(0, 4);
        auto scores = output.row(r).colRange(4, 4 + class_num_);
        auto one_key_points = output.row(r).colRange(4 + class_num_, 14);

        std::vector<cv::Point2f> armor_key_points;

        double score;
        cv::Point max_point;
        cv::minMaxLoc(scores, nullptr, &score, nullptr, &max_point);

        if (score < score_threshold_) continue;

        auto x = xywh.at<float>(0);
        auto y = xywh.at<float>(1);
        auto w = xywh.at<float>(2);
        auto h = xywh.at<float>(3);
        auto left = static_cast<int>((x - 0.5 * w) / scale);
        auto top = static_cast<int>((y - 0.5 * h) / scale);
        auto width = static_cast<int>(w / scale);
        auto height = static_cast<int>(h / scale);

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
        sort_keypoints(armors_key_points[i]);
        if (use_roi_) {
            armors.emplace_back(
                ids[i], confidences[i], boxes[i], armors_key_points[i], offset_);
        } else {
            armors.emplace_back(ids[i], confidences[i], boxes[i], armors_key_points[i]);
        }
    }

    for (auto it = armors.begin(); it != armors.end();) {
        it->pattern = get_pattern(bgr_img, *it);
        classifier_.classify(*it);

        if (!check_name(*it)) {
            it = armors.erase(it);
            continue;
        }

        it->type = get_type(*it);
        if (!check_type(*it)) {
            it = armors.erase(it);
            continue;
        }

        it->center_norm = get_center_norm(bgr_img, it->center);
        ++it;
    }

    if (debug_) draw_detections(bgr_img, armors, frame_count);

    return armors;
}

bool YOLOV8::check_name(const Armor& armor) const {
    const bool name_ok = is_robot_target(armor.name);
    auto confidence_ok = armor.confidence > min_confidence_;
    return name_ok && confidence_ok;
}

bool YOLOV8::check_type(const Armor& armor) const {
    const bool name_ok = (armor.type == ArmorType::small)
                             ? armor.name != ArmorName::one
                             : armor.name != ArmorName::two && armor.name != ArmorName::sentry;
    return name_ok;
}

ArmorType YOLOV8::get_type(const Armor& armor) {
    if (armor.name == ArmorName::one) {
        return ArmorType::big;
    }
    if (armor.name == ArmorName::two || armor.name == ArmorName::sentry) {
        return ArmorType::small;
    }
    return armor.ratio > 3.0 ? ArmorType::big : ArmorType::small;
}

cv::Point2f YOLOV8::get_center_norm(
    const cv::Mat& bgr_img, const cv::Point2f& center) const {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
}

cv::Mat YOLOV8::get_pattern(const cv::Mat& bgr_img, const Armor& armor) const {
    auto tl =
        (armor.points[0] + armor.points[3]) / 2 -
        (armor.points[3] - armor.points[0]) * 1.125;
    auto bl =
        (armor.points[0] + armor.points[3]) / 2 +
        (armor.points[3] - armor.points[0]) * 1.125;
    auto tr =
        (armor.points[2] + armor.points[1]) / 2 -
        (armor.points[2] - armor.points[1]) * 1.125;
    auto br =
        (armor.points[2] + armor.points[1]) / 2 +
        (armor.points[2] - armor.points[1]) * 1.125;

    auto roi_left = std::max<int>(std::min(tl.x, bl.x), 0);
    auto roi_top = std::max<int>(std::min(tl.y, tr.y), 0);
    auto roi_right = std::min<int>(std::max(tr.x, br.x), bgr_img.cols);
    auto roi_bottom = std::min<int>(std::max(bl.y, br.y), bgr_img.rows);

    if (roi_left < 0 || roi_top < 0 || roi_right <= roi_left || roi_bottom <= roi_top) {
        return cv::Mat();
    }
    if (roi_right > bgr_img.cols || roi_bottom > bgr_img.rows) {
        return cv::Mat();
    }

    return bgr_img(cv::Rect(cv::Point(roi_left, roi_top), cv::Point(roi_right, roi_bottom)));
}

void YOLOV8::draw_detections(
    const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const {
    auto detection = img.clone();
    tools::draw_text(
        detection, std::format("[{}]", frame_count), {10, 30}, {255, 255, 255});
    for (const auto& armor : armors) {
        auto info = std::format(
            "{:.2f} {} {}", armor.confidence, ARMOR_NAMES[armor.name],
            ARMOR_TYPES[armor.type]);
        tools::draw_points(detection, armor.points, {0, 255, 0});
        tools::draw_text(detection, info, armor.center, {0, 255, 0});
    }
    if (use_roi_) {
        cv::rectangle(detection, roi_, cv::Scalar(0, 255, 0), 2);
    }
    cv::resize(detection, detection, {}, 0.5, 0.5);
    cv::imshow("detection", detection);
}

void YOLOV8::sort_keypoints(std::vector<cv::Point2f>& keypoints) {
    if (keypoints.size() != 4) return;

    std::sort(keypoints.begin(), keypoints.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.y < b.y;
    });

    std::vector<cv::Point2f> top_points = {keypoints[0], keypoints[1]};
    std::vector<cv::Point2f> bottom_points = {keypoints[2], keypoints[3]};

    std::sort(top_points.begin(), top_points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
    std::sort(
        bottom_points.begin(), bottom_points.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) { return a.x < b.x; });

    keypoints[0] = top_points[0];
    keypoints[1] = top_points[1];
    keypoints[2] = bottom_points[1];
    keypoints[3] = bottom_points[0];
}

std::list<Armor> YOLOV8::postprocess(
    double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count) {
    return parse(scale, output, bgr_img, frame_count);
}

}  // namespace app::auto_aim
