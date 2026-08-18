#include "aimer.hpp"

#include <cmath>
#include <cstdint>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"
#include "tools/trajectory.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

Aimer::Aimer(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    yaw_offset_ = config["aimer"]["yaw_offset"].value_or(0.0) / 57.3;
    pitch_offset_ = config["aimer"]["pitch_offset"].value_or(0.0) / 57.3;
    coming_angle_ = config["aimer"]["coming_angle"].value_or(60.0) / 57.3;
    leaving_angle_ = config["aimer"]["leaving_angle"].value_or(20.0) / 57.3;
    decision_speed_ = config["aimer"]["decision_speed"].value_or(10.0);
    high_speed_delay_time_ =
        config["aimer"]["high_speed_delay_time"].value_or(0.026);
    low_speed_delay_time_ =
        config["aimer"]["low_speed_delay_time"].value_or(0.010);
}

io::Command Aimer::aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp,
    double bullet_speed, bool to_now) {
    debug_aim_point = {};
    if (targets.empty()) return {};

    auto target = targets.front();
    const double delay_time = std::abs(target.ekf_x()[7]) > decision_speed_
                                  ? high_speed_delay_time_
                                  : low_speed_delay_time_;

    if (bullet_speed < 14.0) bullet_speed = 23.0;

    auto future = timestamp;
    if (to_now) {
        const double dt =
            tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
        future += std::chrono::microseconds(static_cast<int64_t>(dt * 1e6));
        target.predict(future);
    } else {
        constexpr double pipeline_delay = 0.005;
        const double dt = pipeline_delay + delay_time;
        future += std::chrono::microseconds(static_cast<int64_t>(dt * 1e6));
        target.predict(future);
    }

    auto aim_point = choose_aim_point(target);
    debug_aim_point = aim_point;
    if (!aim_point.valid) return {};

    const Eigen::Vector3d xyz0 = aim_point.xyza.head<3>();
    const double d0 = std::hypot(xyz0.x(), xyz0.y());
    tools::Trajectory trajectory0(bullet_speed, d0, xyz0.z());
    if (trajectory0.unsolvable) {
        LOG_DEBUG(
            MODULE, "[Aimer] Unsolvable trajectory: speed={:.2f}, d={:.2f}, z={:.2f}",
            bullet_speed, d0, xyz0.z());
        debug_aim_point.valid = false;
        return {};
    }

    double previous_fly_time = trajectory0.fly_time;
    tools::Trajectory current_trajectory = trajectory0;

    for (int iteration = 0; iteration < 10; ++iteration) {
        auto predicted_target = target;
        const auto predict_time =
            future + std::chrono::microseconds(
                         static_cast<int64_t>(previous_fly_time * 1e6));
        predicted_target.predict(predict_time);

        aim_point = choose_aim_point(predicted_target);
        debug_aim_point = aim_point;
        if (!aim_point.valid) return {};

        const Eigen::Vector3d xyz = aim_point.xyza.head<3>();
        const double distance = std::hypot(xyz.x(), xyz.y());
        current_trajectory = tools::Trajectory(bullet_speed, distance, xyz.z());
        if (current_trajectory.unsolvable) {
            LOG_DEBUG(
                MODULE,
                "[Aimer] Unsolvable trajectory in iteration {}: speed={:.2f}, "
                "d={:.2f}, z={:.2f}",
                iteration + 1, bullet_speed, distance, xyz.z());
            debug_aim_point.valid = false;
            return {};
        }

        if (std::abs(current_trajectory.fly_time - previous_fly_time) < 0.001) break;
        previous_fly_time = current_trajectory.fly_time;
    }

    const Eigen::Vector3d final_xyz = debug_aim_point.xyza.head<3>();
    io::Command command;
    command.control = true;
    command.yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
    command.pitch = -(current_trajectory.pitch + pitch_offset_);
    return command;
}

AimPoint Aimer::choose_aim_point(const Target& target) {
    const Eigen::VectorXd ekf_x = target.ekf_x();
    const std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
    const int armor_count = static_cast<int>(armor_xyza_list.size());
    if (armor_count == 0) return {};

    if (!target.jumped) return {true, armor_xyza_list.front()};

    const double center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
    std::vector<double> delta_angles;
    delta_angles.reserve(armor_xyza_list.size());
    for (const auto& armor_xyza : armor_xyza_list) {
        delta_angles.emplace_back(tools::limit_rad(armor_xyza[3] - center_yaw));
    }

    const double angular_speed = ekf_x[7];
    if (std::abs(angular_speed) <= 2.0 && target.name != ArmorName::outpost) {
        std::vector<int> candidates;
        for (int i = 0; i < armor_count; ++i) {
            if (std::abs(delta_angles[i]) <= 60.0 / 57.3) candidates.emplace_back(i);
        }
        if (candidates.empty()) {
            LOG_WARN(MODULE, "[Aimer] No armor is inside the shootable angle");
            return {};
        }

        if (candidates.size() > 1) {
            const int first_id = candidates[0];
            const int second_id = candidates[1];
            if (lock_id_ != first_id && lock_id_ != second_id) {
                lock_id_ = std::abs(delta_angles[first_id]) < std::abs(delta_angles[second_id])
                               ? first_id
                               : second_id;
            }
            return {true, armor_xyza_list[lock_id_]};
        }

        lock_id_ = -1;
        return {true, armor_xyza_list[candidates.front()]};
    }

    double coming_angle = coming_angle_;
    double leaving_angle = leaving_angle_;
    if (target.name == ArmorName::outpost) {
        coming_angle = 70.0 / 57.3;
        leaving_angle = 30.0 / 57.3;
    }

    for (int i = 0; i < armor_count; ++i) {
        if (std::abs(delta_angles[i]) > coming_angle) continue;
        if (angular_speed > 0.0 && delta_angles[i] < leaving_angle) {
            return {true, armor_xyza_list[i]};
        }
        if (angular_speed < 0.0 && delta_angles[i] > -leaving_angle) {
            return {true, armor_xyza_list[i]};
        }
    }

    return {};
}

}  // namespace app::auto_aim
