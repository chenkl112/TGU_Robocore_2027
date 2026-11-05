#include "detector.hpp"

#include <algorithm>
#include <chrono>
#include <filesystem>
#include <format>
#include <iterator>
#include <limits>

#include "armor_image.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include <stdexcept>

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

Detector::Detector(const std::string& config_path, bool debug)
    : Detector(load_vision_config(config_path), debug) {}

Detector::Detector(const VisionConfig& config, bool debug)
    : Detector(config.detector, config.classifier, debug) {}

Detector::Detector(const DetectorConfig& config, const ClassifierConfig& classifier, bool debug)
    : classifier_(classifier), debug_(debug) {
    validate(config);
    threshold_ = config.threshold;
    max_angle_error_ = config.max_angle_error;
    min_lightbar_ratio_ = config.min_lightbar_ratio;
    max_lightbar_ratio_ = config.max_lightbar_ratio;
    min_lightbar_length_ = config.min_lightbar_length;
    min_armor_ratio_ = config.min_armor_ratio;
    max_armor_ratio_ = config.max_armor_ratio;
    max_side_ratio_ = config.max_side_ratio;
    min_confidence_ = config.min_confidence;
    max_rectangular_error_ = config.max_rectangular_error;

    save_path_ = "patterns";
    std::filesystem::create_directory(save_path_);
}

std::list<Armor> Detector::detect(const cv::Mat& bgr_img, int frame_count) {
    if (bgr_img.empty())
        return {};
    if (bgr_img.type() != CV_8UC3) {
        throw std::invalid_argument("Detector expects an 8-bit BGR image");
    }
    cv::Mat gray_img;
    cv::cvtColor(bgr_img, gray_img, cv::COLOR_BGR2GRAY);

    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::size_t lightbar_id = 0;
    std::list<Lightbar> lightbars;
    for (const auto& contour : contours) {
        auto rotated_rect = cv::minAreaRect(contour);
        auto lightbar = Lightbar(rotated_rect, lightbar_id);

        if (!check_geometry(lightbar))
            continue;

        lightbar.color = get_color(bgr_img, contour);
        lightbars.emplace_back(lightbar);
        lightbar_id += 1;
    }

    lightbars.sort([](const Lightbar& a, const Lightbar& b) { return a.center.x < b.center.x; });

    std::list<Armor> armors;
    for (auto left = lightbars.begin(); left != lightbars.end(); left++) {
        for (auto right = std::next(left); right != lightbars.end(); right++) {
            if (left->color != right->color)
                continue;

            auto armor = Armor(*left, *right);
            if (!check_geometry(armor))
                continue;

            armor.pattern = get_pattern(bgr_img, armor);
            classifier_.classify(armor);
            if (!check_name(armor))
                continue;

            armor.type = armor_type_from_name(armor.name);

            armor.center_norm = get_center_norm(bgr_img, armor.center);
            armors.emplace_back(armor);
        }
    }

    for (auto armor1 = armors.begin(); armor1 != armors.end(); armor1++) {
        for (auto armor2 = std::next(armor1); armor2 != armors.end(); armor2++) {
            if (armor1->left.id != armor2->left.id && armor1->left.id != armor2->right.id &&
                armor1->right.id != armor2->left.id && armor1->right.id != armor2->right.id) {
                continue;
            }

            if (armor1->left.id == armor2->left.id || armor1->right.id == armor2->right.id) {
                auto area1 = armor1->pattern.cols * armor1->pattern.rows;
                auto area2 = armor2->pattern.cols * armor2->pattern.rows;
                if (area1 < area2)
                    armor2->duplicated = true;
                else
                    armor1->duplicated = true;
            }

            if (armor1->left.id == armor2->right.id || armor1->right.id == armor2->left.id) {
                if (armor1->confidence < armor2->confidence)
                    armor1->duplicated = true;
                else
                    armor2->duplicated = true;
            }
        }
    }

    armors.remove_if([&](const Armor& a) { return a.duplicated; });

    if (debug_)
        show_result(binary_img, bgr_img, lightbars, armors, frame_count);

    return armors;
}

bool Detector::detect(Armor& armor, const cv::Mat& bgr_img) {
    if (bgr_img.empty() || bgr_img.type() != CV_8UC3)
        return false;
    auto tl = armor.points[0];
    auto tr = armor.points[1];
    auto br = armor.points[2];
    auto bl = armor.points[3];

    auto lt2b = bl - tl;
    auto rt2b = br - tr;
    auto tl1 = (tl + bl) / 2 - lt2b;
    auto bl1 = (tl + bl) / 2 + lt2b;
    auto br1 = (tr + br) / 2 + rt2b;
    auto tr1 = (tr + br) / 2 - rt2b;
    auto tl2tr = tr1 - tl1;
    auto bl2br = br1 - bl1;
    auto tl2 = (tl1 + tr) / 2 - 0.75 * tl2tr;
    auto tr2 = (tl1 + tr) / 2 + 0.75 * tl2tr;
    auto bl2 = (bl1 + br) / 2 - 0.75 * bl2br;
    auto br2 = (bl1 + br) / 2 + 0.75 * bl2br;

    std::vector<cv::Point> points = {tl2, tr2, br2, bl2};
    auto armor_rotaterect = cv::minAreaRect(points);
    cv::Rect boundingBox = armor_rotaterect.boundingRect();

    if (boundingBox.x < 0 || boundingBox.y < 0 ||
        boundingBox.x + boundingBox.width > bgr_img.cols ||
        boundingBox.y + boundingBox.height > bgr_img.rows) {
        return false;
    }

    cv::Mat armor_roi = bgr_img(boundingBox);
    if (armor_roi.empty()) {
        return false;
    }

    cv::Mat gray_img;
    cv::cvtColor(armor_roi, gray_img, cv::COLOR_BGR2GRAY);
    cv::Mat binary_img;
    cv::threshold(gray_img, binary_img, threshold_, 255, cv::THRESH_BINARY);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    std::size_t lightbar_id = 0;
    std::list<Lightbar> lightbars;
    for (const auto& contour : contours) {
        auto rotated_rect = cv::minAreaRect(contour);
        auto lightbar = Lightbar(rotated_rect, lightbar_id);

        if (!check_geometry(lightbar))
            continue;

        lightbar.color = get_color(armor_roi, contour);
        lightbars.emplace_back(lightbar);
        lightbar_id += 1;
    }

    if (lightbars.size() < 2)
        return false;

    lightbars.sort([](const Lightbar& a, const Lightbar& b) { return a.center.x < b.center.x; });

    Lightbar* closest_left_lightbar = nullptr;
    Lightbar* closest_right_lightbar = nullptr;
    float min_distance_tl_bl = std::numeric_limits<float>::max();
    float min_distance_br_tr = std::numeric_limits<float>::max();
    for (auto& lightbar : lightbars) {
        float distance_tl_bl =
            cv::norm(tl - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y))) +
            cv::norm(bl - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y)));
        if (distance_tl_bl < min_distance_tl_bl) {
            min_distance_tl_bl = distance_tl_bl;
            closest_left_lightbar = &lightbar;
        }
        float distance_br_tr =
            cv::norm(br - (lightbar.bottom + cv::Point2f(boundingBox.x, boundingBox.y))) +
            cv::norm(tr - (lightbar.top + cv::Point2f(boundingBox.x, boundingBox.y)));
        if (distance_br_tr < min_distance_br_tr) {
            min_distance_br_tr = distance_br_tr;
            closest_right_lightbar = &lightbar;
        }
    }

    if (closest_left_lightbar && closest_right_lightbar &&
        closest_left_lightbar != closest_right_lightbar &&
        min_distance_br_tr + min_distance_tl_bl < 15) {
        armor.points[0] = closest_left_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
        armor.points[1] = closest_right_lightbar->top + cv::Point2f(boundingBox.x, boundingBox.y);
        armor.points[2] =
            closest_right_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
        armor.points[3] = closest_left_lightbar->bottom + cv::Point2f(boundingBox.x, boundingBox.y);
        armor.pose_valid = false;
        return true;
    }

    return false;
}

bool Detector::check_geometry(const Lightbar& lightbar) const {
    auto angle_ok = lightbar.angle_error < max_angle_error_;
    auto ratio_ok = lightbar.ratio > min_lightbar_ratio_ && lightbar.ratio < max_lightbar_ratio_;
    auto length_ok = lightbar.length > min_lightbar_length_;
    return angle_ok && ratio_ok && length_ok;
}

bool Detector::check_geometry(const Armor& armor) const {
    auto ratio_ok = armor.ratio > min_armor_ratio_ && armor.ratio < max_armor_ratio_;
    auto side_ratio_ok = armor.side_ratio < max_side_ratio_;
    auto rectangular_error_ok = armor.rectangular_error < max_rectangular_error_;
    return ratio_ok && side_ratio_ok && rectangular_error_ok;
}

bool Detector::check_name(const Armor& armor) const {
    const bool name_ok = is_robot_target(armor.name);
    auto confidence_ok = armor.confidence > min_confidence_;

    if (name_ok && !confidence_ok)
        save(armor);

    if (armor.name == ArmorName::five)
        LOG_DEBUG(MODULE, "See pattern 5");

    return name_ok && confidence_ok;
}

Color Detector::get_color(const cv::Mat& bgr_img, const std::vector<cv::Point>& contour) const {
    int red_sum = 0, blue_sum = 0;

    for (const auto& point : contour) {
        red_sum += bgr_img.at<cv::Vec3b>(point)[2];
        blue_sum += bgr_img.at<cv::Vec3b>(point)[0];
    }

    return blue_sum > red_sum ? Color::blue : Color::red;
}

cv::Mat Detector::get_pattern(const cv::Mat& bgr_img, const Armor& armor) const {
    return extract_armor_pattern(bgr_img, armor.points);
}

cv::Point2f Detector::get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const {
    auto h = bgr_img.rows;
    auto w = bgr_img.cols;
    return {center.x / w, center.y / h};
}

void Detector::save(const Armor& armor) const {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto img_path = std::format("{}/{}_{}.jpg", save_path_, static_cast<int>(armor.name), time_t);
    cv::imwrite(img_path, armor.pattern);
}

void Detector::show_result(const cv::Mat& binary_img, const cv::Mat& bgr_img,
                           const std::list<Lightbar>& lightbars, const std::list<Armor>& armors,
                           int frame_count) const {
    auto detection = bgr_img.clone();
    tools::draw_text(detection, std::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

    for (const auto& lightbar : lightbars) {
        auto info = std::format("{:.1f} {:.1f} {:.1f} {}", lightbar.angle_error * 57.3,
                                lightbar.ratio, lightbar.length, COLORS[lightbar.color]);
        tools::draw_text(detection, info, lightbar.top, {0, 255, 255});
        tools::draw_points(detection, lightbar.points, {0, 255, 255}, 3);
    }

    for (const auto& armor : armors) {
        auto info = std::format("{:.2f} {:.2f} {:.1f} {:.2f} {} {}", armor.ratio, armor.side_ratio,
                                armor.rectangular_error * 57.3, armor.confidence,
                                ARMOR_NAMES[armor.name], ARMOR_TYPES[armor.type]);
        tools::draw_points(detection,
                           std::vector<cv::Point2f>(armor.points.begin(), armor.points.end()),
                           {0, 255, 0});
        tools::draw_text(detection, info, armor.left.bottom, {0, 255, 0});
    }

    cv::Mat binary_img2;
    cv::resize(binary_img, binary_img2, {}, 0.5, 0.5);
    cv::resize(detection, detection, {}, 0.5, 0.5);

    cv::imshow("detection", detection);
}

} // namespace app::auto_aim
