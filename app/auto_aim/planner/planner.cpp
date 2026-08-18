#include "planner.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"
#include "tools/trajectory.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

using namespace std::chrono_literals;

namespace app::auto_aim {

Planner::Planner(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    yaw_offset_ = config["planner"]["yaw_offset"].value_or(0.0) / 57.3;
    pitch_offset_ = config["planner"]["pitch_offset"].value_or(0.0) / 57.3;
    fire_thresh_ = config["planner"]["fire_thresh"].value_or(0.05);
    decision_speed_ = config["planner"]["decision_speed"].value_or(7.0);
    high_speed_delay_time_ = config["planner"]["high_speed_delay_time"].value_or(0.12);
    low_speed_delay_time_ = config["planner"]["low_speed_delay_time"].value_or(0.12);

    setup_yaw_solver(config_path);
    setup_pitch_solver(config_path);
}

Plan Planner::plan(Target target, double bullet_speed) {
    if (bullet_speed < 10 || bullet_speed > 25) {
        bullet_speed = 22;
    }

    const auto armor_xyza = target.armor_xyza_list();
    if (armor_xyza.empty()) return {false};

    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    auto min_dist = 1e10;
    for (const auto& xyza : armor_xyza) {
        auto dist = xyza.head<2>().norm();
        if (dist < min_dist) {
            min_dist = dist;
            xyz = xyza.head<3>();
        }
    }
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj.unsolvable) {
        LOG_WARN(MODULE, "Unsolvable initial bullet trajectory");
        return {false};
    }
    target.predict(bullet_traj.fly_time);

    double yaw0;
    Trajectory traj;
    try {
        yaw0 = aim(target, bullet_speed)(0);
        traj = get_trajectory(target, yaw0, bullet_speed);
    } catch (const std::exception& e) {
        LOG_WARN(MODULE, "Unsolvable target {:.2f}", bullet_speed);
        return {false};
    }

    Eigen::VectorXd x0(2);
    x0 << traj(0, 0), traj(1, 0);
    tiny_set_x0(yaw_solver_, x0);

    yaw_solver_->work->Xref = traj.block(0, 0, 2, HORIZON);
    tiny_solve(yaw_solver_);

    x0 << traj(2, 0), traj(3, 0);
    tiny_set_x0(pitch_solver_, x0);

    pitch_solver_->work->Xref = traj.block(2, 0, 2, HORIZON);
    tiny_solve(pitch_solver_);

    Plan plan;
    plan.control = true;

    plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
    plan.target_pitch = traj(2, HALF_HORIZON);

    plan.yaw = tools::limit_rad(yaw_solver_->work->x(0, HALF_HORIZON) + yaw0);
    plan.yaw_vel = yaw_solver_->work->x(1, HALF_HORIZON);
    plan.yaw_acc = yaw_solver_->work->u(0, HALF_HORIZON);

    plan.pitch = pitch_solver_->work->x(0, HALF_HORIZON);
    plan.pitch_vel = pitch_solver_->work->x(1, HALF_HORIZON);
    plan.pitch_acc = pitch_solver_->work->u(0, HALF_HORIZON);

    auto shoot_offset_ = 2;
    plan.fire = std::hypot(
                    traj(0, HALF_HORIZON + shoot_offset_)
                        - yaw_solver_->work->x(0, HALF_HORIZON + shoot_offset_),
                    traj(2, HALF_HORIZON + shoot_offset_)
                        - pitch_solver_->work->x(0, HALF_HORIZON + shoot_offset_))
                < fire_thresh_;
    return plan;
}

Plan Planner::plan(std::optional<Target> target, double bullet_speed) {
    if (!target.has_value()) return {false};

    double delay_time = std::abs(target->ekf_x()[7]) > decision_speed_ ? high_speed_delay_time_
                                                                       : low_speed_delay_time_;

    auto future =
        std::chrono::steady_clock::now() + std::chrono::microseconds(int(delay_time * 1e6));

    target->predict(future);

    return plan(*target, bullet_speed);
}

void Planner::setup_yaw_solver(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    auto max_yaw_acc = config["planner"]["max_yaw_acc"].value_or(50.0);

    auto Q_yaw_arr = config["planner"]["Q_yaw"].as_array();
    auto R_yaw_arr = config["planner"]["R_yaw"].as_array();
    std::vector<double> Q_yaw, R_yaw;
    if (Q_yaw_arr) {
        for (auto& v : *Q_yaw_arr) Q_yaw.push_back(v.value_or(0.0));
    }
    if (R_yaw_arr) {
        for (auto& v : *R_yaw_arr) R_yaw.push_back(v.value_or(0.0));
    }
    if (Q_yaw.size() != 2 || R_yaw.size() != 1) {
        throw std::runtime_error("planner.Q_yaw must contain 2 values and R_yaw 1 value");
    }

    Eigen::MatrixXd A{{1, DT}, {0, 1}};
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(Q_yaw.data());
    Eigen::Matrix<double, 1, 1> R(R_yaw.data());
    if (tiny_setup(
            &yaw_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1,
            HORIZON, 0) != 0) {
        throw std::runtime_error("failed to initialize yaw MPC solver");
    }

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_yaw_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_yaw_acc);
    tiny_set_bound_constraints(yaw_solver_, x_min, x_max, u_min, u_max);

    yaw_solver_->settings->max_iter = 10;
}

void Planner::setup_pitch_solver(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    auto max_pitch_acc = config["planner"]["max_pitch_acc"].value_or(50.0);

    auto Q_pitch_arr = config["planner"]["Q_pitch"].as_array();
    auto R_pitch_arr = config["planner"]["R_pitch"].as_array();
    std::vector<double> Q_pitch, R_pitch;
    if (Q_pitch_arr) {
        for (auto& v : *Q_pitch_arr) Q_pitch.push_back(v.value_or(0.0));
    }
    if (R_pitch_arr) {
        for (auto& v : *R_pitch_arr) R_pitch.push_back(v.value_or(0.0));
    }
    if (Q_pitch.size() != 2 || R_pitch.size() != 1) {
        throw std::runtime_error("planner.Q_pitch must contain 2 values and R_pitch 1 value");
    }

    Eigen::MatrixXd A{{1, DT}, {0, 1}};
    Eigen::MatrixXd B{{0}, {DT}};
    Eigen::VectorXd f{{0, 0}};
    Eigen::Matrix<double, 2, 1> Q(Q_pitch.data());
    Eigen::Matrix<double, 1, 1> R(R_pitch.data());
    if (tiny_setup(
            &pitch_solver_, A, B, f, Q.asDiagonal(), R.asDiagonal(), 1.0, 2, 1,
            HORIZON, 0) != 0) {
        throw std::runtime_error("failed to initialize pitch MPC solver");
    }

    Eigen::MatrixXd x_min = Eigen::MatrixXd::Constant(2, HORIZON, -1e17);
    Eigen::MatrixXd x_max = Eigen::MatrixXd::Constant(2, HORIZON, 1e17);
    Eigen::MatrixXd u_min = Eigen::MatrixXd::Constant(1, HORIZON - 1, -max_pitch_acc);
    Eigen::MatrixXd u_max = Eigen::MatrixXd::Constant(1, HORIZON - 1, max_pitch_acc);
    tiny_set_bound_constraints(pitch_solver_, x_min, x_max, u_min, u_max);

    pitch_solver_->settings->max_iter = 10;
}

Eigen::Matrix<double, 2, 1> Planner::aim(const Target& target, double bullet_speed) {
    const auto armor_xyza = target.armor_xyza_list();
    if (armor_xyza.empty()) throw std::runtime_error("target has no armor states");

    Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
    double yaw = 0.0;
    auto min_dist = 1e10;

    for (const auto& xyza : armor_xyza) {
        auto dist = xyza.head<2>().norm();
        if (dist < min_dist) {
            min_dist = dist;
            xyz = xyza.head<3>();
            yaw = xyza[3];
        }
    }
    debug_xyza = Eigen::Vector4d(xyz.x(), xyz.y(), xyz.z(), yaw);

    auto azim = std::atan2(xyz.y(), xyz.x());
    auto bullet_traj = tools::Trajectory(bullet_speed, min_dist, xyz.z());
    if (bullet_traj.unsolvable) throw std::runtime_error("Unsolvable bullet trajectory!");

    return {tools::limit_rad(azim + yaw_offset_), -bullet_traj.pitch - pitch_offset_};
}

Trajectory Planner::get_trajectory(Target& target, double yaw0, double bullet_speed) {
    Trajectory traj;

    target.predict(-DT * (HALF_HORIZON + 1));
    auto yaw_pitch_last = aim(target, bullet_speed);

    target.predict(DT);
    auto yaw_pitch = aim(target, bullet_speed);

    for (int i = 0; i < HORIZON; i++) {
        target.predict(DT);
        auto yaw_pitch_next = aim(target, bullet_speed);

        auto yaw_vel = tools::limit_rad(yaw_pitch_next(0) - yaw_pitch_last(0)) / (2 * DT);
        auto pitch_vel = (yaw_pitch_next(1) - yaw_pitch_last(1)) / (2 * DT);

        traj.col(i) << tools::limit_rad(yaw_pitch(0) - yaw0), yaw_vel, yaw_pitch(1), pitch_vel;

        yaw_pitch_last = yaw_pitch;
        yaw_pitch = yaw_pitch_next;
    }

    return traj;
}

}  // namespace app::auto_aim
