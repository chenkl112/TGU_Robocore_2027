#include "solver.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <utility>

namespace app::auto_aim {
namespace {
// 自由 PnP 的横边侧倾超过该范围时，不再将平地安装模型作为新的尺寸证据。
constexpr double MAX_LEVEL_ROLL = 3 * CV_PI / 180;
} // namespace

Solver::Solver(const std::string& config_path) : Solver(load_solver_settings(config_path)) {}

Solver::Solver(const SolverSettings& bundle) : Solver(bundle.calibration, bundle.solver) {}

Solver::Solver(const CalibrationConfig& calibration, const SolverConfig& config)
    : config_(config), geometry_(calibration, config.max_balance_pitch) {
    validate(config);
}

Eigen::Matrix3d Solver::R_gimbal2world() const {
    return geometry_.R_gimbal2world();
}
void Solver::set_R_gimbal2world(const Eigen::Quaterniond& q) {
    geometry_.set_R_gimbal2world(q);
}

std::optional<SolvedObservation>
Solver::resolve_observation(const Armor& detection, std::optional<ArmorType> tracked_type) const {
    auto candidate = detection;
    if (!solve_observation(candidate, tracked_type))
        return std::nullopt;
    return SolvedObservation(std::move(candidate));
}

bool Solver::solve(Armor& armor) const {
    armor.pose_valid = false;
    double error;
    return geometry_.solve_pose(armor, error) && geometry_.finish_pose(armor);
}

bool Solver::solve_observation(Armor& armor, std::optional<ArmorType> tracked_type) const {
    armor.pose_valid = false;
    if (!is_robot_target(armor.name))
        return false;
    const auto known_type = armor_type_from_name(armor.name);
    if (known_type != ArmorType::unknown) {
        auto candidate = armor;
        candidate.type = known_type;
        double error;
        if (!geometry_.solve_pose(candidate, error) || error > config_.max_reprojection_error ||
            !geometry_.finish_pose(candidate))
            return false;
        armor = std::move(candidate);
        return true;
    }

    std::array<Armor, 2> candidates{armor, armor};
    std::array<Armor, 2> unconstrained{armor, armor};
    std::array<double, 2> errors;
    std::array<double, 2> unconstrained_errors;
    bool upright_candidate = false;
    bool tilted_small_candidate = false;
    errors.fill(std::numeric_limits<double>::infinity());
    unconstrained_errors.fill(std::numeric_limits<double>::infinity());
    for (int i = 0; i < 2; ++i) {
        candidates[i].type = i == 0 ? ArmorType::big : ArmorType::small;
        double error;
        if (!geometry_.solve_pose(candidates[i], error,
                                  ArmorGeometry::PoseRefinement::LeastSquares))
            continue;
        unconstrained[i] = candidates[i];
        unconstrained_errors[i] = error;
        // 两种尺寸都无法解释近水平横边时，平地安装假设不成立；不能把小步兵的车身
        // 侧倾误认为平衡步兵允许的俯仰，从而错误切换物理尺寸。
        const double roll = std::abs(candidates[i].ypr_in_world[2]);
        upright_candidate = upright_candidate ||
                            (error <= config_.max_reprojection_error && roll <= MAX_LEVEL_ROLL);
        const bool fitted = geometry_.solve_size_pose(candidates[i], error);
        if (i == 1 && (!fitted || error > config_.size_noise_floor)) {
            // 仅用于否决尺寸结论：若小装甲在安装俯仰 ±15° 和车身侧倾下也能解释
            // 角点，则证据不足；比较拟合像素残差，不直接信任远距离噪声较大的 PnP 俯仰。
            auto tilted = unconstrained[i];
            double tilted_error;
            tilted_small_candidate =
                geometry_.solve_size_pose(tilted, tilted_error,
                                          ArmorGeometry::MountingModel::TiltedSmall) &&
                tilted_error <= config_.size_noise_floor;
        }
        if (fitted && error <= config_.max_reprojection_error)
            errors[i] = error;
    }
    if (!upright_candidate || tilted_small_candidate) {
        // 重力约束失效时不产生新的尺寸结论。已关联轨迹的尺寸可作为先验，
        // 使用自由 PnP 位姿，但调用方仍必须进行三维位置关联。
        if (!tracked_type || *tracked_type == ArmorType::unknown)
            return false;
        const int selected = *tracked_type == ArmorType::big ? 0 : 1;
        if (unconstrained_errors[selected] > config_.max_reprojection_error ||
            !geometry_.finish_pose(unconstrained[selected]))
            return false;
        armor = std::move(unconstrained[selected]);
        return true;
    }
    const int best = errors[0] < errors[1] ? 0 : 1;
    if (!std::isfinite(errors[best]))
        return false;
    int selected = best;
    if (std::max(errors[0], errors[1]) <= config_.size_noise_floor ||
        std::abs(errors[0] - errors[1]) < config_.min_type_error_gap) {
        // 两种尺寸都在角点噪声范围内时，残差略小不代表尺寸更可信。
        // 仅允许已有轨迹先验打破平局，随后仍需通过空间关联。
        if (!tracked_type || *tracked_type == ArmorType::unknown)
            return false;
        selected = *tracked_type == ArmorType::big ? 0 : 1;
    }
    if (!geometry_.finish_pose(candidates[selected]))
        return false;
    armor = std::move(candidates[selected]);
    return true;
}

std::vector<cv::Point2f> Solver::reproject_armor(const Eigen::Vector3d& position, double yaw,
                                                 ArmorType type) const {
    return geometry_.reproject_armor(position, yaw, type);
}

std::vector<cv::Point2f> Solver::world2pixel(const std::vector<cv::Point3f>& points) const {
    return geometry_.world2pixel(points);
}

} // namespace app::auto_aim
