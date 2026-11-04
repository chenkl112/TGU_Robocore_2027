#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/planner/tinympc/admm.hpp"
#include "app/auto_aim/aimer.hpp"
#include "app/auto_aim/shooter.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/tracker.hpp"
#include "app/auto_aim/yolos/keypoints.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"
#include "tools/tomlpp.hpp"

#include <chrono>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>

using namespace std::chrono_literals;
using namespace app::auto_aim;
using Clock = std::chrono::steady_clock;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
Armor image_armor(Solver& solver, ArmorName name = ArmorName::three,
                  Eigen::Vector3d xyz = {3, 0, 0}) {
    Armor armor(0, 1.0f, {}, {{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    armor.name = name;
    armor.color = Color::red;
    armor.type = name == ArmorName::one ? ArmorType::big : ArmorType::small;
    armor.points = solver.reproject_armor(xyz, 0, armor.type);
    armor.center_norm = {0.5, 0.5};
    return armor;
}
Target world_target(Clock::time_point t, int armor_count = 4) {
    Armor armor(0, 1.0f, {}, {{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    armor.name = ArmorName::three;
    armor.type = ArmorType::small;
    armor.priority = ArmorPriority::first;
    armor.xyz_in_world = {0.8, 0, 0};
    armor.ypr_in_world = {0, 0, 0};
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
    Eigen::VectorXd covariance = Eigen::VectorXd::Ones(11);
    Target target(armor, t, 0.2, armor_count, covariance);
    target.observed = true;
    return target;
}

Target rotating_target(Clock::time_point t, double phase, double speed = 2.0) {
    auto target = world_target(t);
    auto& state = const_cast<tools::ExtendedKalmanFilter&>(target.ekf()).x;
    state[0] = 3.0;
    state[6] = phase;
    state[7] = speed;
    return target;
}

void require_stopped(const Plan& plan) {
    require(!plan.control && !plan.fire, "Failed MPC solve retained control or fire");
    const std::array values{plan.target_yaw, plan.target_pitch, plan.yaw, plan.yaw_vel,
                            plan.yaw_acc, plan.pitch, plan.pitch_vel, plan.pitch_acc};
    require(std::all_of(values.begin(), values.end(), [](float value) { return value == 0.0f; }),
            "Stopped MPC plan retained a nonzero or nonfinite command");
}

// Independent flight-time root for every physical plate. Unlike the planner's
// expanding bracket, these short-range fixtures all fit in the fixed [0, 2] s
// interval. This oracle checks the actual plate, not a smoothed aiming point.
Eigen::Vector2d intercept_reference(const Target& target, const toml::table& config,
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
            if (flight.fly_time > time) lo = time;
            else hi = time;
        }
        const auto armor = target.armor_xyza_list(delay + (lo + hi) * 0.5)[id];
        if (armor.x() * std::cos(armor.w()) + armor.y() * std::sin(armor.w()) <= 0 ||
            armor.head<2>().norm() >= closest) continue;
        closest = armor.head<2>().norm();
        if (facing_angle) *facing_angle = tools::limit_rad(armor.w() - std::atan2(armor.y(), armor.x()));
        const tools::Trajectory flight(22, closest, armor.z());
        reference << tools::limit_rad(std::atan2(armor.y(), armor.x()) +
                     config["planner"]["yaw_offset"].value_or(0.0) / 57.3),
                     -flight.pitch - config["planner"]["pitch_offset"].value_or(0.0) / 57.3;
    }
    require(std::isfinite(closest), "Interception oracle found no facing armor");
    return reference;
}

void rotating_sequence(const std::string& config_path, Clock::time_point start,
                       double distance, double speed, double bearing = 0,
                       double radius_difference = 0, double height_difference = 0) {
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
        auto& state = const_cast<tools::ExtendedKalmanFilter&>(target.ekf()).x;
        state[0] = distance * std::cos(bearing);
        state[2] = distance * std::sin(bearing);
        state[9] = radius_difference;
        state[10] = height_difference;
        // Previous measured alignment cannot authorize a shot at a new plate.
        io::GimbalState feedback;
        feedback.received_at = Clock::now();
        feedback.yaw = previous.target_yaw;
        feedback.pitch = previous.target_pitch;
        const auto began = Clock::now();
        const auto plan = planner.plan(std::optional{target}, 22, stamp, feedback);
        const double elapsed = std::chrono::duration<double, std::milli>(Clock::now() - began).count();
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
            require(error < fire_thresh, "Normal plate tracking remains inaccurate outside a handoff");
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
            // Check the sequence actually sent to the gimbal, not only each
            // independently optimized horizon's acceleration field.
            require(std::abs(yaw_step - previous.yaw_vel * DT) < yaw_acc * DT * DT + 1e-4 &&
                    std::abs(pitch_step - previous.pitch_vel * DT) < pitch_acc * DT * DT + 1e-4,
                    "Armor handoff introduced a discontinuous angle command");
            require(std::abs(plan.yaw_vel - previous.yaw_vel) < yaw_acc * DT + 1e-3 &&
                    std::abs(plan.pitch_vel - previous.pitch_vel) < pitch_acc * DT + 1e-3,
                    "Armor handoff introduced a discontinuous velocity command");
            if (std::hypot(tools::limit_rad(plan.target_yaw - previous.target_yaw),
                           plan.target_pitch - previous.target_pitch) > 0.02 / distance) {
                ++switches;
                require(!plan.fire, "Plate switch fired using feedback aligned to the previous plate");
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
    require(controlled >= 285, "Normal rotating target lost more than five percent of control frames");
    require(longest_stop <= 5, "Normal rotating target stopped for over 50 ms");
    require(switches > 0, "Rotating sequence never exercised a physical plate handoff");
    require(centered_frames > 0, "Rotating sequence never checked normal plate tracking");
}

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 3, "Expected scenario and config path");
        const std::string scenario = argv[1], config = argv[2];
        const auto t0 = Clock::now();
        if (scenario == "tracking_confirmation" || scenario == "priority_switch" || scenario == "association_gate") {
            Solver solver(config);
            Tracker tracker(config, solver);
            const auto first_name = scenario == "priority_switch" ? ArmorName::two : ArmorName::three;
            std::list<Target> targets;
            for (int i = 0; i < 5; ++i) {
                std::list<Armor> armors{image_armor(solver, first_name)};
                targets = tracker.track(armors, t0 + i * 10ms);
                require(i == 4 ? !targets.empty() : targets.empty(), "Target escaped detection confirmation");
            }
            require(tracker.state() == "tracking", "Tracker did not acquire the valid target");
            if (scenario == "priority_switch") {
                std::list<Armor> armors{image_armor(solver, ArmorName::three)};
                require(tracker.track(armors, t0 + 50ms).empty(), "Switched target bypassed confirmation");
                require(tracker.state() == "detecting", "Switched target did not restart detection");
            } else if (scenario == "association_gate") {
                const auto before = targets.front().ekf_x();
                std::list<Armor> armors{image_armor(solver, ArmorName::three, {3, 2, 0})};
                targets = tracker.track(armors, t0 + 50ms);
                require(tracker.state() == "temp_lost", "A distant same-ID target was merged into the track");
                require(!targets.empty() && !targets.front().observed, "Unmatched target remains fire eligible");
                require((targets.front().ekf_x().head<3>() - before.head<3>()).norm() < 0.1,
                        "Distant same-ID observation moved the tracked target");
            } else {
                tracker.reset();
                std::list<Armor> armors{image_armor(solver)};
                require(tracker.track(armors, t0 + 50ms).empty(), "Reset preserved an acquired target");
            }
        } else if (scenario == "fire_without_feedback") {
            Planner planner(config);
            const auto plan = planner.plan(world_target(t0), 22);
            require(plan.control && !plan.fire, "Static target requested fire without actual gimbal feedback");
        } else if (scenario == "shooter_alignment") {
            const auto path = std::filesystem::temp_directory_path() /
                ("robocore-shooter-" + std::to_string(t0.time_since_epoch().count()) + ".toml");
            std::ifstream input(config);
            std::ostringstream contents;
            contents << input.rdbuf();
            auto text = contents.str();
            const auto marker = std::string{"auto_fire = false"};
            const auto position = text.find(marker, text.find("[shooter]"));
            require(position != std::string::npos, "Shooter fixture is missing auto_fire");
            text.replace(position, marker.size(), "auto_fire = true");
            std::ofstream(path) << text;
            Aimer aimer(path.string());
            Shooter shooter(path.string());
            aimer.debug_aim_point.valid = true;
            const std::list<Target> targets{world_target(t0)};
            io::Command command;
            command.control = true;
            require(!shooter.shoot(command, aimer, targets, {0, 0.1, 0}),
                    "Shooter fired with a pitch-misaligned gimbal");
            require(!shooter.shoot(command, aimer, targets, {0.1, 0, 0}),
                    "Shooter fired with a yaw-misaligned gimbal");
            require(shooter.shoot(command, aimer, targets, {0, 0, 0}),
                    "Shooter rejected aligned yaw/pitch feedback");
            std::filesystem::remove(path);
        } else if (scenario == "feedback_alignment") {
            Planner planner(config);
            const auto target = world_target(t0);
            const auto reference = planner.plan(target, 22);
            io::GimbalState feedback;
            feedback.received_at = Clock::now();
            feedback.yaw = reference.target_yaw + 0.1f;
            feedback.pitch = reference.target_pitch;
            require(!planner.plan(target, 22, feedback).fire, "Yaw-misaligned gimbal requested fire");
            feedback.yaw = reference.target_yaw;
            feedback.pitch = reference.target_pitch + 0.1f;
            require(!planner.plan(target, 22, feedback).fire, "Pitch-misaligned gimbal requested fire");
            feedback.pitch = reference.target_pitch;
            feedback.received_at = Clock::now() - 1s;
            require(!planner.plan(target, 22, feedback).fire, "Stale feedback requested fire");
            feedback.received_at = Clock::now() + 1s;
            require(!planner.plan(target, 22, feedback).fire, "Future feedback requested fire");
            feedback.received_at = Clock::now();
            feedback.yaw = reference.target_yaw + 2 * CV_PI;
            require(planner.plan(target, 22, feedback).fire, "Aligned wrapped yaw cannot fire");
            feedback.pitch = std::numeric_limits<float>::quiet_NaN();
            require(!planner.plan(target, 22, feedback).fire, "Nonfinite feedback requested fire");
        } else if (scenario == "observation_expiry") {
            Solver solver(config);
            Tracker tracker(config, solver);
            Planner planner(config);
            std::list<Target> targets;
            for (int i = 0; i < 5; ++i) {
                std::list<Armor> armors{image_armor(solver)};
                targets = tracker.track(armors, t0 + i * 30ms);
            }
            require(!targets.empty(), "Failed to acquire expiry fixture");
            const auto observed_at = t0 + 120ms;
            for (int i = 1; i <= 22; ++i) {
                const auto now = observed_at + i * 30ms;
                // An out-of-gate same-ID observation is no fresher than no observation.
                std::list<Armor> armors;
                if (i % 2 == 0) armors.push_back(image_armor(solver, ArmorName::three, {3, 2, 0}));
                targets = tracker.track(armors, now);
                require(!targets.empty(), "Expiry fixture lost its predicted track before the count limit");
                require(targets.front().observation_timestamp() == observed_at,
                        "Rejected/empty frame changed the last observation timestamp");
                if (now - observed_at > 100ms) {
                    require(!planner.plan(std::optional{targets.front()}, 22, now).control,
                            "Empty/rejected observations refreshed the target age");
                }
            }
            const auto recovered_at = observed_at + 690ms;
            std::list<Armor> recovered{image_armor(solver)};
            targets = tracker.track(recovered, recovered_at);
            require(!targets.empty() && targets.front().observation_timestamp() == recovered_at &&
                    planner.plan(std::optional{targets.front()}, 22, recovered_at).control,
                    "A fresh accepted observation did not restore control");
        } else if (scenario == "moving_intercept") {
            Planner planner(config);
            auto target = world_target(t0);
            // Inject an exact initial condition for the independent ballistic oracle.
            auto& state = const_cast<tools::ExtendedKalmanFilter&>(target.ekf()).x;
            state[0] = 8.2; state[1] = 3; state[3] = 3;
            const auto cfg = toml::parse_file(config);
            const auto position_at = [&](double time) -> Eigen::Vector3d {
                auto future = target;
                future.predict(time);
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
                if (trajectory.fly_time > t) lo = t;
                else hi = t;
            }
            const auto hit = position_at((lo + hi) * 0.5);
            const double yaw = std::atan2(hit.y(), hit.x()) + cfg["planner"]["yaw_offset"].value_or(0.0) / 57.3;
            tools::Trajectory trajectory(22, hit.head<2>().norm(), hit.z());
            const double pitch = -trajectory.pitch - cfg["planner"]["pitch_offset"].value_or(0.0) / 57.3;
            Plan plan{};
            for (int attempt = 0; attempt < 10; ++attempt) {
                plan = planner.plan(target, 22);
                if (plan.control) break;
                require_stopped(plan);
            }
            const double yaw_error = std::abs(tools::limit_rad(plan.target_yaw - yaw));
            std::cout << "intercept yaw error: " << yaw_error * 180 / CV_PI << " deg\n";
            require(plan.control && yaw_error < 1e-5 && std::abs(plan.target_pitch - pitch) < 1e-5,
                    "Moving intercept does not match the independent flight-time root");
            state[5] = 1; state[7] = 7;
            for (double dt : {-0.5, 0., 0.1, 0.5}) {
                auto predicted = target;
                predicted.predict(dt);
                const auto reference = predicted.armor_xyza_list();
                const auto projected = target.armor_xyza_list(dt);
                for (std::size_t i = 0; i < reference.size(); ++i) {
                    require((reference[i] - projected[i]).norm() < 1e-12,
                            "Lightweight interception kinematics differ from EKF prediction");
                }
            }
        } else if (scenario == "planner_expiry") {
            Planner planner(config);
            std::optional<Target> target = world_target(t0);
            require(planner.plan(target, 22, t0 + 10ms).control, "Fresh target cannot be controlled");
            require(!planner.plan(target, 22, t0 + 200ms).control, "Expired target still controls the gimbal");
            require(!planner.plan(target, 22, t0 - 1ms).control, "Future target was accepted");
        } else if (scenario == "planner_acceleration_bounds") {
            Planner planner(config);
            int controlled = 0;
            for (double w : {0., 1., 5., 10., 20., 30., 50., 100.}) {
                const auto plan = planner.plan(Target(1, w, 0.2, 0.1), 22);
                if (w == 0) require(plan.control, "Stationary target did not produce control");
                if (!plan.control) {
                    require_stopped(plan);
                    continue;
                }
                ++controlled;
                require(std::isfinite(plan.yaw) && std::isfinite(plan.pitch), "Nonfinite MPC angles");
                require(std::abs(plan.yaw_acc) <= 50.0001f, "Yaw acceleration exceeds configured bound");
                require(std::abs(plan.pitch_acc) <= 100.0001f, "Pitch acceleration exceeds configured bound");
            }
            require(controlled > 0, "Acceleration bounds were never checked on an active plan");
        } else if (scenario == "mpc_objective") {
            // Independently condense a scalar integrator's physical cost into
            // a dense quadratic. Changing rho must not add it to the requested
            // tracking/input weights. Zero terminal reference also isolates the
            // library's terminal Hessian from its reference convention.
            for (double rho : {0.2, 2.0, 20.0}) {
                constexpr int horizon = 20;
                Eigen::MatrixXd A = Eigen::MatrixXd::Constant(1, 1, 1.0);
                Eigen::MatrixXd B = Eigen::MatrixXd::Constant(1, 1, 1.0);
                Eigen::MatrixXd Q = Eigen::MatrixXd::Constant(1, 1, 4.0);
                Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, 2.0);
                TinySolver* raw = nullptr;
                require(tiny_setup(&raw, A, B, Eigen::VectorXd::Zero(1), Q, R,
                                   rho, 1, 1, horizon, 0) == 0, "Could not set up objective fixture");
                auto cleanup = [](TinySolver* solver) {
                    delete solver->solution; delete solver->cache; delete solver->settings;
                    delete solver->work; delete solver;
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
                    // Pinf includes the terminal ADMM state penalty once.
                    const double weight = time == horizon - 1 ? raw->cache->Pinf(0, 0) - rho : 4.0;
                    hessian += weight * response * response.transpose();
                    gradient += weight * response;
                }
                const Eigen::VectorXd expected = -hessian.ldlt().solve(gradient);
                std::cout << "rho=" << rho << " objective error="
                          << (raw->solution->u.transpose() - expected).cwiseAbs().maxCoeff() << '\n';
                // The cached infinite-horizon Riccati recursion itself stops at
                // a 1e-5 gain change; allow that numerical approximation here.
                require((raw->solution->u.transpose() - expected).cwiseAbs().maxCoeff() < 2e-4,
                        "ADMM penalty changed the configured physical MPC objective");
            }
        } else if (scenario == "planner_visible_intercept") {
            Planner planner(config);
            auto target = world_target(t0, 2);
            auto& state = const_cast<tools::ExtendedKalmanFilter&>(target.ekf()).x;
            state[6] = CV_PI / 2;
            // At this phase both opposing plates face away from their own
            // camera rays, even though one is necessarily the closest plate.
            require_stopped(planner.plan(target, 22));
            state[6] = 0;
            require(planner.plan(target, 22).control, "Facing plate failed to restore control");
            state[6] = 0.4;
            state[7] = 2.0;
            // The command-time plate is visible, but the +0.5 s horizon tail
            // crosses the side-on gap between the two opposing plates.
            Planner rotating(config);
            require(rotating.plan(target, 22).control,
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
            // Deliberately demand rotational tracking with only 0.01 rad/s^2.
            // This forces true constrained nonconvergence without treating a
            // valid, normally actuated rotating target as an expected failure.
            auto limited = toml::parse_file(config);
            limited["planner"].as_table()->insert_or_assign("max_yaw_acc", 0.01);
            limited["planner"].as_table()->insert_or_assign("max_pitch_acc", 0.01);
            const auto path = std::filesystem::temp_directory_path() /
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
            require(!planner.plan(Target{}, 22).control, "Uninitialized target was accepted");
            require(!planner.plan(Target(1, 0, 0.2, 0.1), nan).control, "NaN bullet speed was accepted");
            require(!planner.plan(Target(1, nan, 0.2, 0.1), 22).control, "NaN state was accepted");
            require(std::isnan(tools::limit_rad(std::numeric_limits<double>::infinity())), "Infinity was not rejected");
        } else if (scenario == "lost_target_fire") {
            Planner planner(config);
            auto target = world_target(t0);
            target.observed = false;
            for (int i = 0; i < 20; ++i) {
                const auto plan = planner.plan(target, 22);
                require(plan.control && !plan.fire, "A temporarily lost target remained fire eligible");
            }
        } else if (scenario == "pose_validation") {
            Solver solver(config);
            auto armor = image_armor(solver);
            require(solver.solve(armor), "Valid projected armor did not solve");
            armor.points.assign(4, {10, 10});
            require(!solver.solve(armor), "Degenerate armor pose was accepted");
            armor = image_armor(solver);
            armor.points[0].x = std::numeric_limits<float>::quiet_NaN();
            require(!solver.solve(armor), "Nonfinite armor pose was accepted");
            bool threw = false;
            try { solver.set_R_gimbal2world(Eigen::Quaterniond(0, 0, 0, 0)); }
            catch (const std::invalid_argument&) { threw = true; }
            require(threw, "Zero quaternion was accepted");
        } else if (scenario == "tilted_keypoints" || scenario == "perspective_keypoints") {
            auto cfg = toml::parse_file(config);
            auto& camera = *cfg["camera"].as_table();
            camera.insert_or_assign("R_camera2gimbal", toml::array{1.,0.,0.,0.,1.,0.,0.,0.,1.});
            camera.insert_or_assign("t_camera2gimbal", toml::array{0.,0.,0.});
            camera.insert_or_assign("camera_matrix", toml::array{800.,0.,640.,0.,800.,480.,0.,0.,1.});
            camera.insert_or_assign("distort_coeffs", toml::array{0.,0.,0.,0.,0.});
            const auto path = std::filesystem::temp_directory_path() /
                ("robocore-corners-" + std::to_string(t0.time_since_epoch().count()) + ".toml");
            std::ofstream(path) << cfg;
            Solver solver(path.string());
            std::filesystem::remove(path);
            const cv::Mat K = (cv::Mat_<double>(3,3) << 800,0,640,0,800,480,0,0,1);
            const Eigen::Matrix3d base{{0,-1,0},{0,0,-1},{1,0,0}};
            for (auto type : {ArmorType::big, ArmorType::small}) {
                const float width = type == ArmorType::big ? 0.230f : 0.135f;
                const std::vector<cv::Point3f> object{{0,width/2,0.028f},{0,-width/2,0.028f},
                                                     {0,-width/2,-0.028f},{0,width/2,-0.028f}};
                for (double roll : {20.,-20.,45.,-45.,70.,-70.,0.}) {
                    for (double slant : (scenario == "perspective_keypoints" ? std::vector<double>{70., 80.} : std::vector<double>{0., 35.})) {
                        const Eigen::Matrix3d R = Eigen::AngleAxisd(roll * CV_PI/180, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
                            Eigen::AngleAxisd(slant * CV_PI/180, Eigen::Vector3d::UnitY()).toRotationMatrix() * base;
                        cv::Mat rotation, rvec;
                        cv::eigen2cv(R, rotation); cv::Rodrigues(rotation, rvec);
                        std::vector<cv::Point2f> expected;
                        cv::projectPoints(object, rvec, cv::Vec3d(0.1,-0.1,3), K, cv::Mat(), expected);
                        // Model channels are LT, LB, RB, RT, regardless of image dimensions.
                        std::vector<cv::Point2f> corners{expected[0], expected[3], expected[2], expected[1]};
                        require(decode_yolov8_keypoints(corners), "Valid model keypoints rejected");
                        Armor armor(1, 1, {}, corners);
                        armor.type = type; armor.name = ArmorName::one;
                        require(solver.solve(armor), "Tilted armor PnP failed");
                        if (scenario == "perspective_keypoints" && roll == 20.) {
                            std::cout << "type=" << type << " slant=" << slant << " depth=" << armor.xyz_in_world.z() << '\n';
                        }
                        require((armor.xyz_in_world - Eigen::Vector3d(0.1,-0.1,3)).norm() < 1e-3,
                                "Model keypoint mapping changed the recovered position");
                        for (int i = 0; i < 4; ++i) {
                            require(cv::norm(corners[i] - expected[i]) < 1e-4,
                                    "Physical corner correspondence was permuted");
                        }
                    }
                }
            }
            std::vector<cv::Point2f> invalid{{0,0},{1,0},{2,0},{3,0}};
            require(!decode_yolov8_keypoints(invalid), "Collinear corners were accepted");
            invalid = {{0,0},{1,0},{1,1},{std::numeric_limits<float>::quiet_NaN(),1}};
            require(!decode_yolov8_keypoints(invalid), "Nonfinite corners were accepted");
        } else if (scenario == "calibration_validation") {
            const auto path = std::filesystem::temp_directory_path() /
                              ("robocore-calibration-" + std::to_string(t0.time_since_epoch().count()) + ".toml");
            std::ofstream(path) << "[camera]\n";
            bool threw = false;
            try { Solver solver(path.string()); }
            catch (const std::invalid_argument&) { threw = true; }
            std::filesystem::remove(path);
            require(threw, "Missing calibration silently became identity matrices");
        } else if (scenario == "camera_timeout") {
            io::USBCamera camera(std::string("/nonexistent/robocore-camera.avi"));
            cv::Mat image;
            Clock::time_point timestamp;
            const auto start = Clock::now();
            camera.read(image, timestamp);
            require(image.empty() && Clock::now() - start < 500ms, "Unavailable camera blocked the consumer");
            camera.stop();
            const auto stopped = Clock::now();
            camera.read(image, timestamp);
            require(image.empty() && Clock::now() - stopped < 50ms, "Stopped camera did not wake the consumer");
        } else if (scenario == "trajectory") {
            const auto nan = std::numeric_limits<double>::quiet_NaN();
            require(tools::Trajectory(nan, 3, 0).unsolvable, "NaN trajectory accepted");
            require(tools::Trajectory(22, 0, 0).unsolvable, "Zero-distance trajectory accepted");
            require(tools::Trajectory(1, 100, 5).unsolvable, "Unreachable trajectory accepted");
            for (const double h : {-0.5, 0., 0.5}) {
                tools::Trajectory trajectory(22, 3, h);
                require(!trajectory.unsolvable && trajectory.fly_time > 0, "Reachable trajectory failed");
                const double height = 22 * std::sin(trajectory.pitch) * trajectory.fly_time -
                                      0.5 * 9.7833 * trajectory.fly_time * trajectory.fly_time;
                require(std::abs(height - h) < 1e-9, "Trajectory does not hit the requested height");
            }
        } else throw std::runtime_error("Unknown scenario");
        std::cout << "PASS " << scenario << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
