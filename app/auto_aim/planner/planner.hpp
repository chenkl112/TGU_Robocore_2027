#ifndef TGU_ROBOCORE_2027_PLANNER_HPP
#define TGU_ROBOCORE_2027_PLANNER_HPP
#pragma once

#include <Eigen/Dense>
#include <memory>
#include <optional>
#include <string>

#include "../target.hpp"
#include "io/command.hpp"

namespace app::auto_aim {

struct PlannerConfig;

constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>; // yaw、yaw 角速度、pitch、pitch 角速度

enum class StopReason {
    none,
    no_target,
    invalid_target,
    future_state,
    expired_state,
    future_observation,
    expired_observation,
    invalid_bullet_speed,
    no_visible_armor,
    ballistic_unreachable,
    solver_not_converged,
    invalid_solution,
};
const char* stop_reason_name(StopReason reason);

// 停控时全部控制量保持零；reason 描述停控原因，发弹仍需反馈对齐和可见性校验。
struct Plan {
    bool control = false;
    bool fire = false;
    float target_yaw = 0;
    float target_pitch = 0;
    float yaw = 0;
    float yaw_vel = 0;
    float yaw_acc = 0;
    float pitch = 0;
    float pitch_vel = 0;
    float pitch_acc = 0;
    StopReason reason = StopReason::no_target;
};

class Planner {
  public:
    Eigen::Vector4d debug_xyza = Eigen::Vector4d::Zero();
    explicit Planner(const std::string& config_path);
    explicit Planner(const PlannerConfig& config);
    ~Planner();
    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;

    // 唯一公开入口：状态、最后观测和反馈统一以 now 判龄，随后补偿处理延迟。
    // 回放传入录包时钟；实时入口传入当前 steady_clock 时间。
    Plan plan(const std::optional<TargetSnapshot>& target, double bullet_speed,
              TargetClock::time_point now = TargetClock::now(),
              std::optional<io::GimbalState> feedback = std::nullopt);

  private:
    class AxisSolver;
    std::unique_ptr<AxisSolver> yaw_solver_;
    std::unique_ptr<AxisSolver> pitch_solver_;
    double max_target_age_;
    std::chrono::milliseconds feedback_timeout_{100};
    double yaw_offset_;
    double pitch_offset_;
    double fire_thresh_;
    double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

    struct Interception {
        Eigen::Vector2d angles;
        Eigen::Vector4d armor;
        std::size_t armor_id;
    };
    Plan solve(const TargetSnapshot& target, double bullet_speed, TargetClock::time_point now,
               const std::optional<io::GimbalState>& feedback);
    Interception aim(const TargetSnapshot& target, double bullet_speed,
                     std::optional<std::size_t> armor_id = std::nullopt,
                     bool require_visible = true);
    Trajectory get_trajectory(const TargetSnapshot& target, double yaw0, double bullet_speed,
                              bool& fire_visible);
};

} // namespace app::auto_aim

#endif // TGU_ROBOCORE_2027_PLANNER_HPP
