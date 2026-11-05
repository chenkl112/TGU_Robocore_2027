#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <vector>

#include "armor.hpp"
#include "config.hpp"

namespace app::auto_aim {

// 旋转均以 R_来源2目标 命名；相机系右/下/前，云台和世界系前/左/上。
// 世界系以当前云台旋转中心为原点，通过 IMU 对齐重力方向；长度单位为米。
class ArmorGeometry {
  public:
    enum class PoseRefinement { PlanarSeed, LeastSquares };
    enum class MountingModel { LevelGround, TiltedSmall };

    ArmorGeometry(const CalibrationConfig& calibration, double max_balance_pitch);
    Eigen::Matrix3d R_gimbal2world() const;
    void set_R_gimbal2world(const Eigen::Quaterniond& q);

    bool solve_pose(Armor& armor, double& error,
                    PoseRefinement refinement = PoseRefinement::PlanarSeed) const;
    bool solve_size_pose(Armor& armor, double& error,
                         MountingModel model = MountingModel::LevelGround) const;
    bool finish_pose(Armor& armor) const;
    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d& position, double yaw,
                                             ArmorType type) const;
    std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f>& points) const;

  private:
    cv::Mat camera_matrix_, distort_coeffs_;
    Eigen::Matrix3d R_gimbal2imubody_, R_camera2gimbal_;
    Eigen::Vector3d t_camera2gimbal_;
    Eigen::Matrix3d R_gimbal2world_ = Eigen::Matrix3d::Identity();
    double max_balance_pitch_;
    bool update_derived_pose(Armor& armor) const;
    void optimize_yaw(Armor& armor) const;
    double armor_reprojection_error(const Armor& armor, double yaw) const;
};

} // namespace app::auto_aim
