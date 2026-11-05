#ifndef TGU_ROBOCORE_2027_ARMOR_HPP
#define TGU_ROBOCORE_2027_ARMOR_HPP
#pragma once

#include <Eigen/Dense>
#include <array>
#include <opencv2/opencv.hpp>
#include <string>
#include <utility>
#include <vector>

namespace app::auto_aim {

enum Color { red, blue, extinguish, purple };
inline const std::vector<std::string> COLORS = {"red", "blue", "extinguish", "purple"};

enum ArmorType { big, small, unknown };
inline const std::vector<std::string> ARMOR_TYPES = {"big", "small", "unknown"};

enum ArmorName { one, two, three, four, five, sentry, outpost, base, not_armor };
inline const std::vector<std::string> ARMOR_NAMES = {
    "one", "two", "three", "four", "five", "sentry", "outpost", "base", "not_armor"};

// 步兵可能使用两种物理尺寸，图像中的长宽比不能作为尺寸标签。
constexpr ArmorType armor_type_from_name(ArmorName name) {
    if (name == ArmorName::one)
        return ArmorType::big;
    if (name == ArmorName::two || name == ArmorName::sentry)
        return ArmorType::small;
    return ArmorType::unknown;
}

constexpr bool is_robot_target(ArmorName name) {
    return name == ArmorName::one || name == ArmorName::two || name == ArmorName::three ||
           name == ArmorName::four || name == ArmorName::five || name == ArmorName::sentry;
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

static_assert(armor_priority(ArmorName::three, PriorityMode::mode_one) == ArmorPriority::first);
static_assert(armor_priority(ArmorName::two, PriorityMode::mode_one) == ArmorPriority::forth);
static_assert(armor_priority(ArmorName::two, PriorityMode::mode_two) == ArmorPriority::first);

struct Lightbar {
    std::size_t id = 0;
    Color color = Color::blue;
    cv::Point2f center, top, bottom, top2bottom;
    std::vector<cv::Point2f> points;
    double angle = 0, angle_error = 0, length = 0, width = 0, ratio = 0;
    cv::RotatedRect rotated_rect;

    Lightbar(const cv::RotatedRect& rotated_rect, std::size_t id);
    Lightbar() = default;
};

using ArmorCorners = std::array<cv::Point2f, 4>;

// 检测阶段只填写图像信息；pose_valid 为真时才可读取下面的空间位姿。
struct Armor {
    Color color = Color::extinguish;
    Lightbar left, right;
    cv::Point2f center;
    cv::Point2f center_norm;
    // 固定顺序：左上、右上、右下、左下，和 PnP 的物理角点一一对应。
    ArmorCorners points{};

    double ratio = 0;
    double side_ratio = 0;
    double rectangular_error = 0;

    ArmorType type = ArmorType::unknown;
    ArmorName name = ArmorName::not_armor;
    ArmorPriority priority = ArmorPriority::fifth;
    int class_id = -1;
    cv::Rect box;
    cv::Mat pattern;
    double confidence = 0.0;
    bool duplicated = false;

    bool pose_valid = false;
    // 世界系为权威位姿，云台系和球坐标由几何求解器统一更新；长度为米，角度为弧度。
    Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();
    Eigen::Vector3d xyz_in_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypr_in_gimbal = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypr_in_world = Eigen::Vector3d::Zero();
    Eigen::Vector3d ypd_in_world = Eigen::Vector3d::Zero();

    // yaw 搜索之前的 PnP/尺寸模型拟合结果，保留用于诊断。
    double yaw_raw = 0;

    Armor(const Lightbar& left, const Lightbar& right);
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints);
    Armor(int class_id, float confidence, const cv::Rect& box,
          std::vector<cv::Point2f> armor_keypoints, cv::Point2f offset);
};

// 只读解算结果只能由 Solver 创建，跟踪入口通过此类型区分图像检测和空间观测。
class SolvedObservation {
  public:
    const Armor& armor() const {
        return armor_;
    }

  private:
    friend class Solver;
    explicit SolvedObservation(Armor armor) : armor_(std::move(armor)) {}
    Armor armor_;
};

} // namespace app::auto_aim

#endif // TGU_ROBOCORE_2027_ARMOR_HPP
