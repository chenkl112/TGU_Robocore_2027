#include "armor.hpp"

#include <algorithm>
#include <array>
#include <cmath>

#include <opencv2/opencv.hpp>

namespace app::auto_aim {

bool sort_armor_keypoints(std::vector<cv::Point2f>& keypoints) {
    if (keypoints.size() != 4) return false;
    for (const auto& p : keypoints) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y)) return false;
    }
    std::vector<cv::Point2f> hull;
    cv::convexHull(keypoints, hull);
    if (hull.size() != 4 || std::abs(cv::contourArea(hull)) < 1.0) return false;

    // The two short, opposite edges are the lightbars. Sorting the four points
    // by image y instead pairs the two endpoints of one bar when armor rolls.
    const auto length = [&](int i) { return cv::norm(hull[i] - hull[(i + 1) % 4]); };
    const int first = length(0) + length(2) < length(1) + length(3) ? 0 : 1;
    std::array<cv::Point2f, 2> left{hull[first], hull[(first + 1) % 4]};
    std::array<cv::Point2f, 2> right{hull[(first + 2) % 4], hull[(first + 3) % 4]};
    if (left[0].x + left[1].x > right[0].x + right[1].x) std::swap(left, right);
    if (left[0].y > left[1].y) std::swap(left[0], left[1]);
    if (right[0].y > right[1].y) std::swap(right[0], right[1]);
    keypoints = {left[0], right[0], right[1], left[1]};
    return true;
}

Lightbar::Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id)
    : id(id), rotated_rect(rotated_rect) {
    std::vector<cv::Point2f> corners(4);
    rotated_rect.points(&corners[0]);
    std::sort(
        corners.begin(), corners.end(),
        [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });

    center = rotated_rect.center;
    top = (corners[0] + corners[1]) / 2;
    bottom = (corners[2] + corners[3]) / 2;
    top2bottom = bottom - top;

    points.emplace_back(top);
    points.emplace_back(bottom);

    width = cv::norm(corners[0] - corners[1]);
    angle = std::atan2(top2bottom.y, top2bottom.x);
    angle_error = std::abs(angle - CV_PI / 2);
    length = cv::norm(top2bottom);
    ratio = length / width;
}

Armor::Armor(const Lightbar& left, const Lightbar& right)
    : left(left), right(right), duplicated(false) {
    color = left.color;
    center = (left.center + right.center) / 2;

    points.emplace_back(left.top);
    points.emplace_back(right.top);
    points.emplace_back(right.bottom);
    points.emplace_back(left.bottom);

    auto left2right = right.center - left.center;
    auto width = cv::norm(left2right);
    auto max_lightbar_length = std::max(left.length, right.length);
    auto min_lightbar_length = std::min(left.length, right.length);
    ratio = width / max_lightbar_length;
    side_ratio = max_lightbar_length / min_lightbar_length;

    auto roll = std::atan2(left2right.y, left2right.x);
    auto left_rectangular_error = std::abs(left.angle - roll - CV_PI / 2);
    auto right_rectangular_error = std::abs(right.angle - roll - CV_PI / 2);
    rectangular_error = std::max(left_rectangular_error, right_rectangular_error);
}

Armor::Armor(
    int class_id, float confidence, const cv::Rect& box,
    std::vector<cv::Point2f> armor_keypoints)
    : class_id(class_id), confidence(confidence), box(box), points(armor_keypoints) {
    center =
        (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
    auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
    auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
    auto max_width = std::max(left_width, right_width);
    auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
    auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
    auto max_length = std::max(top_length, bottom_length);
    auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
    auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
    auto left2right = right_center - left_center;
    auto roll = std::atan2(left2right.y, left2right.x);
    auto left_rectangular_error =
        std::abs(std::atan2(
                     (armor_keypoints[3] - armor_keypoints[0]).y,
                     (armor_keypoints[3] - armor_keypoints[0]).x) -
                 roll - CV_PI / 2);
    auto right_rectangular_error =
        std::abs(std::atan2(
                     (armor_keypoints[2] - armor_keypoints[1]).y,
                     (armor_keypoints[2] - armor_keypoints[1]).x) -
                 roll - CV_PI / 2);
    rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

    ratio = max_length / max_width;

    color = class_id == 0 ? Color::blue
                          : class_id == 1 ? Color::red : Color::extinguish;
}

Armor::Armor(
    int class_id, float confidence, const cv::Rect& box,
    std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset)
    : class_id(class_id), confidence(confidence), box(box), points(armor_keypoints) {
    std::transform(
        armor_keypoints.begin(), armor_keypoints.end(), armor_keypoints.begin(),
        [&offset](const cv::Point2f& point) { return point + offset; });
    std::transform(
        points.begin(), points.end(), points.begin(),
        [&offset](const cv::Point2f& point) { return point + offset; });
    center =
        (armor_keypoints[0] + armor_keypoints[1] + armor_keypoints[2] + armor_keypoints[3]) / 4;
    auto left_width = cv::norm(armor_keypoints[0] - armor_keypoints[3]);
    auto right_width = cv::norm(armor_keypoints[1] - armor_keypoints[2]);
    auto max_width = std::max(left_width, right_width);
    auto top_length = cv::norm(armor_keypoints[0] - armor_keypoints[1]);
    auto bottom_length = cv::norm(armor_keypoints[3] - armor_keypoints[2]);
    auto max_length = std::max(top_length, bottom_length);
    auto left_center = (armor_keypoints[0] + armor_keypoints[3]) / 2;
    auto right_center = (armor_keypoints[1] + armor_keypoints[2]) / 2;
    auto left2right = right_center - left_center;
    auto roll = std::atan2(left2right.y, left2right.x);
    auto left_rectangular_error =
        std::abs(std::atan2(
                     (armor_keypoints[3] - armor_keypoints[0]).y,
                     (armor_keypoints[3] - armor_keypoints[0]).x) -
                 roll - CV_PI / 2);
    auto right_rectangular_error =
        std::abs(std::atan2(
                     (armor_keypoints[2] - armor_keypoints[1]).y,
                     (armor_keypoints[2] - armor_keypoints[1]).x) -
                 roll - CV_PI / 2);
    rectangular_error = std::max(left_rectangular_error, right_rectangular_error);

    ratio = max_length / max_width;

    color = class_id == 0 ? Color::blue
                          : class_id == 1 ? Color::red : Color::extinguish;
}

}  // namespace app::auto_aim
