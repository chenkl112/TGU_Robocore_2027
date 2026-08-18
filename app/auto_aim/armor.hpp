#ifndef TGU_ROBOCORE_2027_ARMOR_HPP
#define TGU_ROBOCORE_2027_ARMOR_HPP
#pragma once

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

namespace app::auto_aim {

enum Color { red, blue, extinguish, purple };
inline const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType { big, small };
inline const std::vector<std::string> ARMOR_TYPES = {"big", "small"};

enum ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
inline const std::vector<std::string> ARMOR_NAMES = {
    "one", "two", "three", "four", "five", "sentry", "outpost", "base", "not_armor"};

constexpr bool is_robot_target(ArmorName name) {
    return name == ArmorName::one || name == ArmorName::two ||
           name == ArmorName::three || name == ArmorName::four ||
           name == ArmorName::five || name == ArmorName::sentry;
}

static_assert(is_robot_target(ArmorName::one));
static_assert(is_robot_target(ArmorName::sentry));
static_assert(!is_robot_target(ArmorName::outpost));
static_assert(!is_robot_target(ArmorName::base));
static_assert(!is_robot_target(ArmorName::not_armor));

enum ArmorPriority { first = 1, second, third, forth, fifth };

enum class PriorityMode { mode_one = 1, mode_two };

constexpr ArmorPriority armor_priority(ArmorName name, PriorityMode mode) {
    if (mode == PriorityMode::mode_two) {
        switch (name) {
            case ArmorName::two:
                return ArmorPriority::first;
            case ArmorName::one:
            case ArmorName::three:
            case ArmorName::four:
            case ArmorName::five:
                return ArmorPriority::second;
            case ArmorName::sentry:
            case ArmorName::outpost:
            case ArmorName::base:
            case ArmorName::not_armor:
                return ArmorPriority::third;
        }
    }

    switch (name) {
        case ArmorName::three:
        case ArmorName::four:
            return ArmorPriority::first;
        case ArmorName::one:
            return ArmorPriority::second;
        case ArmorName::five:
        case ArmorName::sentry:
            return ArmorPriority::third;
        case ArmorName::two:
            return ArmorPriority::forth;
        case ArmorName::outpost:
        case ArmorName::base:
        case ArmorName::not_armor:
            return ArmorPriority::fifth;
    }

    return ArmorPriority::fifth;
}

static_assert(
    armor_priority(ArmorName::three, PriorityMode::mode_one) == ArmorPriority::first);
static_assert(
    armor_priority(ArmorName::two, PriorityMode::mode_one) == ArmorPriority::forth);
static_assert(
    armor_priority(ArmorName::two, PriorityMode::mode_two) == ArmorPriority::first);

struct Lightbar {
    std::size_t id;
    Color color = Color::blue;
    cv::Point2f center, top, bottom, top2bottom;
    std::vector<cv::Point2f> points;
    double angle, angle_error, length, width, ratio;
    cv::RotatedRect rotated_rect;

    Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id);
    Lightbar() {};
};

struct Armor {
    Color color = Color::extinguish;
    Lightbar left, right;
    cv::Point2f center;
    cv::Point2f center_norm;
    std::vector<cv::Point2f> points;

    double ratio;
    double side_ratio;
    double rectangular_error;

    ArmorType type = ArmorType::small;
    ArmorName name = ArmorName::not_armor;
    ArmorPriority priority = ArmorPriority::fifth;
    int class_id = -1;
    cv::Rect box;
    cv::Mat pattern;
    double confidence = 0.0;
    bool duplicated = false;

    Eigen::Vector3d xyz_in_gimbal;
    Eigen::Vector3d xyz_in_world;
    Eigen::Vector3d ypr_in_gimbal;
    Eigen::Vector3d ypr_in_world;
    Eigen::Vector3d ypd_in_world;

    double yaw_raw;

    Armor(const Lightbar& left, const Lightbar& right);
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints);
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_ARMOR_HPP
