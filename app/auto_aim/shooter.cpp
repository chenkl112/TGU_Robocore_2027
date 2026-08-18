#include "shooter.hpp"

#include <cmath>

#include "tools/tomlpp.hpp"

namespace app::auto_aim {

Shooter::Shooter(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    first_tolerance_ = config["shooter"]["first_tolerance"].value_or(5.0) / 57.3;
    second_tolerance_ = config["shooter"]["second_tolerance"].value_or(2.0) / 57.3;
    judge_distance_ = config["shooter"]["judge_distance"].value_or(3.0);
    auto_fire_ = config["shooter"]["auto_fire"].value_or(false);
}

bool Shooter::shoot(
    const io::Command& command, const Aimer& aimer, const std::list<Target>& targets,
    const Eigen::Vector3d& gimbal_position) {
    if (!command.control || targets.empty() || !auto_fire_) return false;

    const double target_x = targets.front().ekf_x()[0];
    const double target_y = targets.front().ekf_x()[2];
    const double target_distance = std::hypot(target_x, target_y);
    const double tolerance =
        target_distance > judge_distance_ ? second_tolerance_ : first_tolerance_;

    const bool command_is_stable =
        std::abs(last_command_.yaw - command.yaw) < tolerance * 2.0;
    const bool gimbal_is_aligned =
        std::abs(gimbal_position[0] - last_command_.yaw) < tolerance;

    last_command_ = command;
    return command_is_stable && gimbal_is_aligned && aimer.debug_aim_point.valid;
}

}  // namespace app::auto_aim
