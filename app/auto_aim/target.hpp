#ifndef TGU_ROBOCORE_2027_TARGET_HPP
#define TGU_ROBOCORE_2027_TARGET_HPP
#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace app::auto_aim {

class Target {
public:
    bool observed = false;
    ArmorName name = ArmorName::not_armor;
    ArmorType armor_type = ArmorType::small;
    ArmorPriority priority = ArmorPriority::fifth;
    bool jumped = false;
    int last_id = 0;

    Target() = default;
    Target(
        const Armor& armor, std::chrono::steady_clock::time_point t, double radius,
        int armor_num, Eigen::VectorXd P0_dig);
    Target(double x, double vyaw, double radius, double h);

    void predict(std::chrono::steady_clock::time_point t);
    void predict(double dt);
    auto timestamp() const { return t_; }
    // Time of the last accepted measurement, unaffected by prediction.
    auto observation_timestamp() const { return last_observation_; }
    bool update(const Armor& armor, double max_match_distance = 0.5);

    Eigen::VectorXd ekf_x() const;
    const tools::ExtendedKalmanFilter& ekf() const;
    std::vector<Eigen::Vector4d> armor_xyza_list(double dt = 0.0) const;

    bool diverged() const;
    bool convergened();
    bool isinit = false;
    bool checkinit();

private:
    int armor_num_ = 0;
    int switch_count_;
    int update_count_;

    bool is_switch_, is_converged_;

    tools::ExtendedKalmanFilter ekf_;
    std::chrono::steady_clock::time_point t_;
    std::chrono::steady_clock::time_point last_observation_{};

    void update_ypda(const Armor& armor, int id);
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd& x, int id) const;
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd& x, int id) const;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_TARGET_HPP
