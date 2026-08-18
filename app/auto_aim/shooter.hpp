#ifndef TGU_ROBOCORE_2027_SHOOTER_HPP
#define TGU_ROBOCORE_2027_SHOOTER_HPP
#pragma once

#include <Eigen/Dense>
#include <list>
#include <string>

#include "aimer.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace app::auto_aim {

class Shooter {
public:
    explicit Shooter(const std::string& config_path);

    bool shoot(
        const io::Command& command, const Aimer& aimer,
        const std::list<Target>& targets, const Eigen::Vector3d& gimbal_position);

private:
    io::Command last_command_;
    double judge_distance_;
    double first_tolerance_;
    double second_tolerance_;
    bool auto_fire_;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_SHOOTER_HPP
