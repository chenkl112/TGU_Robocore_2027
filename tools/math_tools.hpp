#ifndef TGU_ROBOCORE_2027_MATH_TOOLS_HPP
#define TGU_ROBOCORE_2027_MATH_TOOLS_HPP
#pragma once

#include <Eigen/Geometry>
#include <chrono>

namespace tools {

double limit_rad(double angle);

Eigen::Vector3d eulers(
    Eigen::Quaterniond q, int axis0, int axis1, int axis2, bool extrinsic = false);

Eigen::Vector3d eulers(
    Eigen::Matrix3d R, int axis0, int axis1, int axis2, bool extrinsic = false);

Eigen::Matrix3d rotation_matrix(const Eigen::Vector3d& ypr);

Eigen::Vector3d xyz2ypd(const Eigen::Vector3d& xyz);

Eigen::MatrixXd xyz2ypd_jacobian(const Eigen::Vector3d& xyz);

Eigen::Vector3d ypd2xyz(const Eigen::Vector3d& ypd);

Eigen::MatrixXd ypd2xyz_jacobian(const Eigen::Vector3d& ypd);

double delta_time(
    const std::chrono::steady_clock::time_point& a,
    const std::chrono::steady_clock::time_point& b);

double get_abs_angle(const Eigen::Vector2d& vec1, const Eigen::Vector2d& vec2);

template <typename T>
T square(T const& a) {
    return a * a;
}

double limit_min_max(double input, double min, double max);

}  // namespace tools

#endif  // TGU_ROBOCORE_2027_MATH_TOOLS_HPP
