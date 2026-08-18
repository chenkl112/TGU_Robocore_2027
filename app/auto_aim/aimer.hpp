#ifndef TGU_ROBOCORE_2027_AIMER_HPP
#define TGU_ROBOCORE_2027_AIMER_HPP
#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "io/command.hpp"
#include "target.hpp"

namespace app::auto_aim {

struct AimPoint {
    bool valid = false;
    Eigen::Vector4d xyza = Eigen::Vector4d::Zero();
};

class Aimer {
public:
    AimPoint debug_aim_point;

    explicit Aimer(const std::string& config_path);

    io::Command aim(
        std::list<Target> targets, std::chrono::steady_clock::time_point timestamp,
        double bullet_speed, bool to_now = true);

private:
    double yaw_offset_;
    double pitch_offset_;
    double coming_angle_;
    double leaving_angle_;
    int lock_id_ = -1;
    double high_speed_delay_time_;
    double low_speed_delay_time_;
    double decision_speed_;

    AimPoint choose_aim_point(const Target& target);
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_AIMER_HPP
