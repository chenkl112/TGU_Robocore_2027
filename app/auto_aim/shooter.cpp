#include "shooter.hpp"

#include <cmath>

#include "config.hpp"
#include "tools/math_tools.hpp"

namespace app::auto_aim {

Shooter::Shooter(const std::string& config_path) : Shooter(load_shooter_config(config_path)) {}

Shooter::Shooter(const ShooterConfig& config)
    : judge_distance_(config.judge_distance), first_tolerance_(config.first_tolerance),
      second_tolerance_(config.second_tolerance), auto_fire_(config.auto_fire) {
    validate(config);
}

bool Shooter::shoot(const io::Command& command, const Aimer& aimer,
                    const std::list<Target>& targets, const Eigen::Vector3d& gimbal_position) {
    if (!command.control || targets.empty() || !auto_fire_)
        return false;

    const auto state = targets.front().ekf_x();
    const double target_x = state[target_state::X];
    const double target_y = state[target_state::Y];
    const double target_distance = std::hypot(target_x, target_y);
    const double tolerance =
        target_distance > judge_distance_ ? second_tolerance_ : first_tolerance_;

    const bool command_is_stable =
        std::abs(tools::limit_rad(last_command_.yaw - command.yaw)) < tolerance * 2.0 &&
        std::abs(last_command_.pitch - command.pitch) < tolerance * 2.0;
    // 稳定性比较前后两次指令，对齐条件则必须比较当前瞄准指令。
    const bool gimbal_is_aligned =
        std::abs(tools::limit_rad(gimbal_position[0] - command.yaw)) < tolerance &&
        std::abs(gimbal_position[1] - command.pitch) < tolerance;

    last_command_ = command;
    return command_is_stable && gimbal_is_aligned && aimer.debug_aim_point.valid;
}

} // namespace app::auto_aim
