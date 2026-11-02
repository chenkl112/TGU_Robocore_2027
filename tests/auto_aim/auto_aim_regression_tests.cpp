#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/tracker.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>

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
Target world_target(Clock::time_point t) {
    Armor armor(0, 1.0f, {}, {{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    armor.name = ArmorName::three;
    armor.type = ArmorType::small;
    armor.priority = ArmorPriority::first;
    armor.xyz_in_world = {0.8, 0, 0};
    armor.ypr_in_world = {0, 0, 0};
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
    Eigen::VectorXd covariance = Eigen::VectorXd::Ones(11);
    Target target(armor, t, 0.2, 4, covariance);
    target.observed = true;
    return target;
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
        } else if (scenario == "planner_expiry") {
            Planner planner(config);
            std::optional<Target> target = world_target(t0);
            require(planner.plan(target, 22, t0 + 10ms).control, "Fresh target cannot be controlled");
            require(!planner.plan(target, 22, t0 + 200ms).control, "Expired target still controls the gimbal");
            require(!planner.plan(target, 22, t0 - 1ms).control, "Future target was accepted");
        } else if (scenario == "planner_acceleration_bounds") {
            Planner planner(config);
            for (double w : {0., 1., 5., 10., 20., 30., 50., 100.}) {
                const auto plan = planner.plan(Target(1, w, 0.2, 0.1), 22);
                require(plan.control, "Finite rotating target did not produce control");
                require(std::isfinite(plan.yaw) && std::isfinite(plan.pitch), "Nonfinite MPC angles");
                require(std::abs(plan.yaw_acc) <= 50.0001f, "Yaw acceleration exceeds configured bound");
                require(std::abs(plan.pitch_acc) <= 100.0001f, "Pitch acceleration exceeds configured bound");
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
