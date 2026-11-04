#ifndef TGU_ROBOCORE_2027_SOLVER_HPP
#define TGU_ROBOCORE_2027_SOLVER_HPP
#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <optional>
#include <string>
#include <vector>

#include "armor.hpp"

namespace app::auto_aim {

class Solver {
public:
    explicit Solver(const std::string& config_path);

    Eigen::Matrix3d R_gimbal2world() const;

    void set_R_gimbal2world(const Eigen::Quaterniond& q);

    bool solve(Armor& armor) const;
    // Resolve a detector observation using calibrated size hypotheses. A prior
    // only breaks an ambiguous size tie; the caller must still associate in 3D.
    bool solve_observation(
        Armor& armor, std::optional<ArmorType> tracked_type = std::nullopt) const;

    std::vector<cv::Point2f> reproject_armor(
        const Eigen::Vector3d& xyz_in_world, double yaw, ArmorType type) const;

    std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f>& worldPoints);

private:
    cv::Mat camera_matrix_;
    cv::Mat distort_coeffs_;
    Eigen::Matrix3d R_gimbal2imubody_;
    Eigen::Matrix3d R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_;
    double max_reprojection_error_ = 3.0;
    double min_type_error_gap_ = 0.25;

    bool solve_pose(Armor& armor, double& reprojection_error) const;
    bool finish_pose(Armor& armor) const;

    void optimize_yaw(Armor& armor) const;

    double armor_reprojection_error(const Armor& armor, double yaw) const;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_SOLVER_HPP
