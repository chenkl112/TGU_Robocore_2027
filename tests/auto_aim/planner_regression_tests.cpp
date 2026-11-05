#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/planner/tinympc/admm.hpp"
#include "app/auto_aim/planner/tinympc/tiny_api.hpp"
#include "regression_fixtures.hpp"
#include "tools/tomlpp.hpp"
#include "tools/trajectory.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>

using namespace auto_aim_test;

namespace {
TargetSnapshot rotating_target(Clock::time_point t, double phase, double speed = 2.0) {
    auto target = world_snapshot(t);
    auto& state = target.state;
    state[target_state::X] = 3.0;
    state[target_state::Yaw] = phase;
    state[target_state::YawRate] = speed;
    return target;
}

void require_stopped(const Plan& plan) {
    require(!plan.control && !plan.fire, "Failed MPC solve retained control or fire");
    const std::array values{plan.target_yaw, plan.target_pitch, plan.yaw,       plan.yaw_vel,
                            plan.yaw_acc,    plan.pitch,        plan.pitch_vel, plan.pitch_acc};
    require(std::all_of(values.begin(), values.end(), [](float value) { return value == 0.0f; }),
            "Stopped MPC plan retained a nonzero or nonfinite command");
}

// 独立求每块物理装甲的飞行时间根，使用覆盖本夹具的固定 [0, 2] 秒区间。
// 参考值直接对应真实装甲，避免平滑参考掩盖弹道或换板误差。
Eigen::Vector2d intercept_reference(const TargetSnapshot& target, const toml::table& config,
                                    double* facing_angle = nullptr) {
    const double delay = config["planner"]["low_speed_delay_time"].value_or(0.01);
    double closest = std::numeric_limits<double>::infinity();
    Eigen::Vector2d reference;
    for (std::size_t id = 0; id < target.armor_xyza_list().size(); ++id) {
        double lo = 0, hi = 2;
        for (int i = 0; i < 60; ++i) {
            const double time = (lo + hi) * 0.5;
            const auto armor = target.armor_xyza_list(delay + time)[id];
            const tools::Trajectory flight(22, armor.head<2>().norm(), armor.z());
            require(!flight.unsolvable, "Interception oracle fixture is unreachable");
            if (flight.fly_time > time)
                lo = time;
            else
                hi = time;
        }
        const auto armor = target.armor_xyza_list(delay + (lo + hi) * 0.5)[id];
        if (armor.x() * std::cos(armor.w()) + armor.y() * std::sin(armor.w()) <= 0 ||
            armor.head<2>().norm() >= closest)
            continue;
        closest = armor.head<2>().norm();
        if (facing_angle)
            *facing_angle = tools::limit_rad(armor.w() - std::atan2(armor.y(), armor.x()));
        const tools::Trajectory flight(22, closest, armor.z());
        reference << tools::limit_rad(std::atan2(armor.y(), armor.x()) +
                                      config["planner"]["yaw_offset"].value_or(0.0) * CV_PI /
                                          180.0),
            -flight.pitch - config["planner"]["pitch_offset"].value_or(0.0) * CV_PI / 180.0;
    }
    require(std::isfinite(closest), "Interception oracle found no facing armor");
    return reference;
}

void rotating_sequence(const std::string& config_path, Clock::time_point start, double distance,
                       double speed, double bearing = 0, double radius_difference = 0,
                       double height_difference = 0) {
    const auto config = toml::parse_file(config_path);
    const double yaw_acc = config["planner"]["max_yaw_acc"].value_or(50.0);
    const double pitch_acc = config["planner"]["max_pitch_acc"].value_or(100.0);
    const double fire_thresh = config["planner"]["fire_thresh"].value_or(0.0035);
    Planner planner(config_path);
    int controlled = 0, longest_stop = 0, stop_run = 0, switches = 0, centered_frames = 0;
    double largest_error = 0, largest_jump = 0, centered_error = 0, total_ms = 0, largest_ms = 0;
    Plan previous{};
    for (int frame = 0; frame < 300; ++frame) {
        const auto stamp = start + frame * 10ms;
        auto target = rotating_target(stamp, bearing + speed * frame * 0.01, speed);
        auto& state = target.state;
        state[target_state::X] = distance * std::cos(bearing);
        state[target_state::Y] = distance * std::sin(bearing);
        state[target_state::RadiusDifference] = radius_difference;
        state[target_state::HeightDifference] = height_difference;
        // 上一块装甲的对齐反馈不能授权向新装甲发弹。
        io::GimbalState feedback;
        feedback.received_at = stamp;
        feedback.yaw = previous.target_yaw;
        feedback.pitch = previous.target_pitch;
        const auto began = Clock::now();
        const auto plan = planner.plan(std::optional{target}, 22, stamp, feedback);
        const double elapsed =
            std::chrono::duration<double, std::milli>(Clock::now() - began).count();
        total_ms += elapsed;
        largest_ms = std::max(largest_ms, elapsed);
        if (!plan.control) {
            require_stopped(plan);
            longest_stop = std::max(longest_stop, ++stop_run);
            previous = plan;
            continue;
        }
        ++controlled;
        stop_run = 0;
        double facing_angle = 0;
        const auto reference = intercept_reference(target, config, &facing_angle);
        require(std::hypot(tools::limit_rad(plan.target_yaw - reference.x()),
                           plan.target_pitch - reference.y()) < 1e-5,
                "Rotating target no longer identifies the physical interception plate");
        const double error = std::hypot(tools::limit_rad(plan.yaw - plan.target_yaw),
                                        plan.pitch - plan.target_pitch);
        largest_error = std::max(largest_error, error);
        if (std::abs(facing_angle) < 0.15) {
            ++centered_frames;
            centered_error = std::max(centered_error, error);
            require(error < fire_thresh,
                    "Normal plate tracking remains inaccurate outside a handoff");
        }
        require(std::isfinite(error) && error < 0.25 / distance,
                "Control angle escaped the adjacent-plate handoff region");
        require(std::abs(plan.yaw_acc) <= yaw_acc + 1e-4 &&
                    std::abs(plan.pitch_acc) <= pitch_acc + 1e-4,
                "Rotating sequence exceeded its acceleration bounds");
        if (previous.control) {
            const double yaw_step = tools::limit_rad(plan.yaw - previous.yaw);
            const double pitch_step = plan.pitch - previous.pitch;
            largest_jump = std::max(largest_jump, std::hypot(yaw_step, pitch_step));
            // 同时检查逐帧发送的角度与速度连续性。
            require(std::abs(yaw_step - previous.yaw_vel * DT) < yaw_acc * DT * DT + 1e-4 &&
                        std::abs(pitch_step - previous.pitch_vel * DT) < pitch_acc * DT * DT + 1e-4,
                    "Armor handoff introduced a discontinuous angle command");
            require(std::abs(plan.yaw_vel - previous.yaw_vel) < yaw_acc * DT + 1e-3 &&
                        std::abs(plan.pitch_vel - previous.pitch_vel) < pitch_acc * DT + 1e-3,
                    "Armor handoff introduced a discontinuous velocity command");
            if (std::hypot(tools::limit_rad(plan.target_yaw - previous.target_yaw),
                           plan.target_pitch - previous.target_pitch) > 0.02 / distance) {
                ++switches;
                require(!plan.fire,
                        "Plate switch fired using feedback aligned to the previous plate");
            }
        }
        if (std::hypot(tools::limit_rad(plan.target_yaw - feedback.yaw),
                       plan.target_pitch - feedback.pitch) >= fire_thresh) {
            require(!plan.fire, "Smoothed control reference bypassed physical target alignment");
        }
        previous = plan;
    }
    std::cout << "distance=" << distance << " speed=" << speed << " controlled=" << controlled
              << "/300 longest_stop=" << longest_stop << " switches=" << switches
              << " max_error=" << largest_error << " max_command_step=" << largest_jump
              << " centered_error=" << centered_error << " mean_ms=" << total_ms / 300
              << " max_ms=" << largest_ms << '\n';
    require(controlled >= 285,
            "Normal rotating target lost more than five percent of control frames");
    require(longest_stop <= 5, "Normal rotating target stopped for over 50 ms");
    require(switches > 0, "Rotating sequence never exercised a physical plate handoff");
    require(centered_frames > 0, "Rotating sequence never checked normal plate tracking");
}

} // namespace

bool run_planner_regressions(const std::string& scenario, const std::string& config,
                             Clock::time_point t0) {
    if (scenario == "fire_without_feedback") {
        Planner planner(config);
        const auto plan = planner.plan(world_snapshot(t0), 22, t0);
        require(plan.control && !plan.fire,
                "Static target requested fire without actual gimbal feedback");
    } else if (scenario == "feedback_alignment") {
        Planner planner(config);
        const auto target = world_snapshot(t0);
        const auto reference = planner.plan(target, 22, t0);
        io::GimbalState feedback;
        feedback.received_at = t0;
        feedback.yaw = reference.target_yaw + 0.1f;
        feedback.pitch = reference.target_pitch;
        require(!planner.plan(target, 22, t0, feedback).fire,
                "Yaw-misaligned gimbal requested fire");
        feedback.yaw = reference.target_yaw;
        feedback.pitch = reference.target_pitch + 0.1f;
        require(!planner.plan(target, 22, t0, feedback).fire,
                "Pitch-misaligned gimbal requested fire");
        feedback.pitch = reference.target_pitch;
        feedback.received_at = t0 - 1s;
        require(!planner.plan(target, 22, t0, feedback).fire, "Stale feedback requested fire");
        feedback.received_at = t0 + 1s;
        require(!planner.plan(target, 22, t0, feedback).fire, "Future feedback requested fire");
        feedback.received_at = t0;
        feedback.yaw = reference.target_yaw + 2 * CV_PI;
        require(planner.plan(target, 22, t0, feedback).fire, "Aligned wrapped yaw cannot fire");
        feedback.pitch = std::numeric_limits<float>::quiet_NaN();
        require(!planner.plan(target, 22, t0, feedback).fire, "Nonfinite feedback requested fire");
    } else if (scenario == "moving_intercept") {
        auto settings = load_planner_config(config);
        settings.low_speed_delay_time = settings.high_speed_delay_time = 0;
        Planner planner(settings);
        auto target = world_snapshot(t0);
        // 明确零处理延迟，使独立弹道参考只比较飞行时延。
        auto& state = target.state;
        state[target_state::X] = 8.2;
        state[target_state::Vx] = 3;
        state[target_state::Vy] = 3;
        const auto cfg = toml::parse_file(config);
        const auto position_at = [&](double time) -> Eigen::Vector3d {
            auto future = target;
            future = future.predicted(time);
            Eigen::Vector3d closest;
            double distance = std::numeric_limits<double>::infinity();
            for (const auto& armor : future.armor_xyza_list()) {
                if (armor.head<2>().norm() < distance) {
                    distance = armor.head<2>().norm();
                    closest = armor.head<3>();
                }
            }
            return closest;
        };
        double lo = 0, hi = 1;
        for (int i = 0; i < 80; ++i) {
            const double t = (lo + hi) * 0.5;
            const auto p = position_at(t);
            tools::Trajectory trajectory(22, p.head<2>().norm(), p.z());
            require(!trajectory.unsolvable, "Oracle fixture is unreachable");
            if (trajectory.fly_time > t)
                lo = t;
            else
                hi = t;
        }
        const auto hit = position_at((lo + hi) * 0.5);
        const double yaw = std::atan2(hit.y(), hit.x()) +
                           cfg["planner"]["yaw_offset"].value_or(0.0) * CV_PI / 180.0;
        tools::Trajectory trajectory(22, hit.head<2>().norm(), hit.z());
        const double pitch =
            -trajectory.pitch - cfg["planner"]["pitch_offset"].value_or(0.0) * CV_PI / 180.0;
        Plan plan{};
        for (int attempt = 0; attempt < 10; ++attempt) {
            plan = planner.plan(target, 22, t0);
            if (plan.control)
                break;
            require_stopped(plan);
        }
        const double yaw_error = std::abs(tools::limit_rad(plan.target_yaw - yaw));
        std::cout << "intercept yaw error: " << yaw_error * 180 / CV_PI << " deg\n";
        require(plan.control && yaw_error < 1e-5 && std::abs(plan.target_pitch - pitch) < 1e-5,
                "Moving intercept does not match the independent flight-time root");
    } else if (scenario == "planner_expiry") {
        Planner planner(config);
        std::optional<TargetSnapshot> target = world_snapshot(t0);
        require(planner.plan(target, 22, t0 + 10ms).control, "Fresh target cannot be controlled");
        require(!planner.plan(target, 22, t0 + 200ms).control,
                "Expired target still controls the gimbal");
        require(!planner.plan(target, 22, t0 - 1ms).control, "Future target was accepted");
    } else if (scenario == "planner_acceleration_bounds") {
        Planner planner(config);
        int controlled = 0;
        for (double w : {0., 1., 5., 10., 20., 30., 50., 100.}) {
            auto target = world_snapshot(t0);
            target.state[target_state::YawRate] = w;
            target.state[target_state::HeightDifference] = 0.1;
            const auto plan = planner.plan(target, 22, t0);
            if (w == 0)
                require(plan.control, "Stationary target did not produce control");
            if (!plan.control) {
                require_stopped(plan);
                continue;
            }
            ++controlled;
            require(std::isfinite(plan.yaw) && std::isfinite(plan.pitch), "Nonfinite MPC angles");
            require(std::abs(plan.yaw_acc) <= 50.0001f,
                    "Yaw acceleration exceeds configured bound");
            require(std::abs(plan.pitch_acc) <= 100.0001f,
                    "Pitch acceleration exceeds configured bound");
        }
        require(controlled > 0, "Acceleration bounds were never checked on an active plan");
    } else if (scenario == "mpc_objective") {
        // 将标量积分器的物理代价独立展开为二次型，检查 rho 不改变跟踪权重。
        // 终端参考设零，单独验证终端 Hessian。
        for (double rho : {0.2, 2.0, 20.0}) {
            constexpr int horizon = 20;
            Eigen::MatrixXd A = Eigen::MatrixXd::Constant(1, 1, 1.0);
            Eigen::MatrixXd B = Eigen::MatrixXd::Constant(1, 1, 1.0);
            Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(1, 1, 4.0);
            Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, 2.0);
            TinySolver* raw = nullptr;
            require(tiny_setup(&raw, A, B, Eigen::VectorXd::Zero(1), Q, R, rho, 1, 1, horizon, 0) ==
                        0,
                    "Could not set up objective fixture");
            auto cleanup = [](TinySolver* solver) {
                delete solver->solution;
                delete solver->cache;
                delete solver->settings;
                delete solver->work;
                delete solver;
            };
            std::unique_ptr<TinySolver, decltype(cleanup)> solver(raw, cleanup);
            tiny_set_bound_constraints(raw, Eigen::MatrixXd::Constant(1, horizon, -100),
                                       Eigen::MatrixXd::Constant(1, horizon, 100),
                                       Eigen::MatrixXd::Constant(1, horizon - 1, -100),
                                       Eigen::MatrixXd::Constant(1, horizon - 1, 100));
            raw->settings->max_iter = 3000;
            raw->settings->abs_pri_tol = raw->settings->abs_dua_tol = 1e-8;
            raw->work->Uref.setConstant(0.2);
            tiny_set_x0(raw, Eigen::VectorXd::Ones(1));
            update_linear_cost(raw);
            require(tiny_solve(raw) == 0, "Objective fixture did not converge");
            Eigen::MatrixXd hessian = 2.0 * Eigen::MatrixXd::Identity(horizon - 1, horizon - 1);
            Eigen::VectorXd gradient = Eigen::VectorXd::Constant(horizon - 1, -2.0 * 0.2);
            for (int time = 1; time < horizon; ++time) {
                Eigen::VectorXd response = Eigen::VectorXd::Zero(horizon - 1);
                response.head(time).setConstant(1.0);
                // Pinf 已含一次终端 ADMM 状态惩罚。
                const double weight = time == horizon - 1 ? raw->cache->Pinf(0, 0) - rho : 4.0;
                hessian += weight * response * response.transpose();
                gradient += weight * response;
            }
            const Eigen::VectorXd expected = -hessian.ldlt().solve(gradient);
            std::cout << "rho=" << rho << " objective error="
                      << (raw->solution->u.transpose() - expected).cwiseAbs().maxCoeff() << '\n';
            // 缓存的无穷时域 Riccati 递推在增益变化 1e-5 时停止，允许该数值误差。
            require((raw->solution->u.transpose() - expected).cwiseAbs().maxCoeff() < 2e-4,
                    "ADMM penalty changed the configured physical MPC objective");
        }
    } else if (scenario == "planner_visible_intercept") {
        Planner planner(config);
        auto target = world_snapshot(t0, 2);
        auto& state = target.state;
        state[target_state::Yaw] = CV_PI / 2;
        // 两块对置装甲此时均背向各自相机射线，最近装甲也不可见。
        require_stopped(planner.plan(target, 22, t0));
        state[target_state::Yaw] = 0;
        require(planner.plan(target, 22, t0).control, "Facing plate failed to restore control");
        state[target_state::Yaw] = 0.4;
        state[target_state::YawRate] = 2.0;
        // 指令时刻装甲可见，但后续 0.5 秒时域尾部经过两装甲的侧向空隙。
        Planner rotating(config);
        require(rotating.plan(target, 22, t0).control,
                "Invisible horizon tail stopped a visible two-plate interception");
    } else if (scenario == "planner_rotating_sequence") {
        for (double speed : {0.5, -0.5, 2.0, -2.0}) {
            rotating_sequence(config, t0, 1.0, speed);
        }
    } else if (scenario == "planner_rotating_geometry") {
        for (double distance : {3.0, 8.0}) {
            for (double speed : {2.0, -2.0, 5.0, -5.0}) {
                rotating_sequence(config, t0, distance, speed, CV_PI - 0.005, 0.08, 0.1);
            }
        }
    } else if (scenario == "planner_failed_solve" || scenario == "planner_solve_recovery") {
        // 以极低的 0.01 rad/s² 加速度约束制造真实不收敛，避免把正常旋转当失败。
        auto limited = toml::parse_file(config);
        limited["planner"].as_table()->insert_or_assign("max_yaw_acc", 0.01);
        limited["planner"].as_table()->insert_or_assign("max_pitch_acc", 0.01);
        const auto path =
            std::filesystem::temp_directory_path() /
            ("robocore-mpc-bounds-" + std::to_string(t0.time_since_epoch().count()) + ".toml");
        std::ofstream(path) << limited;
        Planner planner(path.string());
        std::filesystem::remove(path);
        require_stopped(planner.plan(std::optional{rotating_target(t0, 0.0)}, 22, t0));
        if (scenario == "planner_solve_recovery") {
            bool recovered = false;
            for (int i = 1; i <= 50; ++i) {
                const auto stamp = t0 + i * 10ms;
                const auto target = rotating_target(stamp, 0.0, 0.0);
                const auto plan = planner.plan(std::optional{target}, 22, stamp);
                if (!plan.control) {
                    require_stopped(plan);
                    continue;
                }
                require(std::hypot(tools::limit_rad(plan.yaw - plan.target_yaw),
                                   plan.pitch - plan.target_pitch) < 0.0035,
                        "Recovered solver retained the failed trajectory");
                require(!plan.fire, "Recovered solver fired without gimbal feedback");
                recovered = true;
                std::cout << "Recovered after " << i << " stationary frames\n";
                break;
            }
            require(recovered, "Planner did not recover from a failed solve within 50 frames");
        }
    } else if (scenario == "invalid_numeric") {
        Planner planner(config);
        const double nan = std::numeric_limits<double>::quiet_NaN();
        require(!planner.plan(TargetSnapshot{}, 22, t0).control,
                "Uninitialized target was accepted");
        require(!planner.plan(world_snapshot(t0), nan, t0).control,
                "NaN bullet speed was accepted");
        auto invalid = world_snapshot(t0);
        invalid.state[target_state::YawRate] = nan;
        require(!planner.plan(invalid, 22, t0).control, "NaN state was accepted");
        require(std::isnan(tools::limit_rad(std::numeric_limits<double>::infinity())),
                "Infinity was not rejected");
    } else if (scenario == "lost_target_fire") {
        Planner planner(config);
        auto target = world_snapshot(t0);
        target.observed = false;
        for (int i = 0; i < 20; ++i) {
            const auto plan = planner.plan(target, 22, t0);
            require(plan.control && !plan.fire, "A temporarily lost target remained fire eligible");
        }
    } else {
        return false;
    }
    return true;
}
