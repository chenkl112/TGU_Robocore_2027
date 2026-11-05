#pragma once

#include "armor_geometry.hpp"
#include "config.hpp"
#include <optional>
#include <string>

namespace app::auto_aim {

// Solver 负责尺寸假设和歧义策略；坐标变换、PnP 与物理模型拟合由 ArmorGeometry 完成。
class Solver {
  public:
    explicit Solver(const std::string& config_path);
    Solver(const CalibrationConfig& calibration, const SolverConfig& config);

    Eigen::Matrix3d R_gimbal2world() const;
    void set_R_gimbal2world(const Eigen::Quaterniond& q);
    // 已知物理尺寸的几何接口，不执行尺寸歧义或最大残差筛选；生产跟踪使用下面的只读接口。
    bool solve(Armor& armor) const;
    std::optional<SolvedObservation>
    resolve_observation(const Armor& detection,
                        std::optional<ArmorType> tracked_type = std::nullopt) const;
    // 跟踪尺寸只能打破几何歧义，调用方仍须检查与原轨迹的三维关联。
    bool solve_observation(Armor& armor,
                           std::optional<ArmorType> tracked_type = std::nullopt) const;
    std::vector<cv::Point2f> reproject_armor(const Eigen::Vector3d& position, double yaw,
                                             ArmorType type) const;
    std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f>& points) const;

  private:
    explicit Solver(const SolverSettings& bundle);
    SolverConfig config_;
    ArmorGeometry geometry_;
};

} // namespace app::auto_aim
