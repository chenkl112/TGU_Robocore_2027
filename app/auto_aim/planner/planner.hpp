#ifndef TGU_ROBOCORE_2027_PLANNER_HPP
#define TGU_ROBOCORE_2027_PLANNER_HPP
#pragma once

#include <Eigen/Dense>
#include <list>
#include <optional>
#include <string>

#include "../target.hpp"
#include "io/command.hpp"
#include "tinympc/tiny_api.hpp"

namespace app::auto_aim {

constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan {
    bool control;
    bool fire;
    float target_yaw;
    float target_pitch;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
};

class Planner {
public:
    Eigen::Vector4d debug_xyza;
    explicit Planner(const std::string& config_path);
    ~Planner();
    Planner(const Planner&) = delete;
    Planner& operator=(const Planner&) = delete;

    Plan plan(Target target, double bullet_speed,
              std::optional<io::GimbalState> feedback = std::nullopt);
    Plan plan(std::optional<Target> target, double bullet_speed,
              std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now(),
              std::optional<io::GimbalState> feedback = std::nullopt);

private:
    double max_target_age_;
    std::chrono::milliseconds feedback_timeout_{100};
    double yaw_offset_;
    double pitch_offset_;
    double fire_thresh_;
    double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

    TinySolver* yaw_solver_ = nullptr;
    TinySolver* pitch_solver_ = nullptr;

    void cleanup();
    void setup_yaw_solver(const std::string& config_path);
    void setup_pitch_solver(const std::string& config_path);

    struct Interception {
        Eigen::Vector2d angles;
        Eigen::Vector4d armor;
        std::size_t armor_id;
    };
    Interception aim(const Target& target, double bullet_speed,
                     std::optional<std::size_t> armor_id = std::nullopt, bool require_visible = true);
    Trajectory get_trajectory(const Target& target, double yaw0, double bullet_speed,
                              bool& fire_visible);
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_PLANNER_HPP
