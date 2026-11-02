#include "app/auto_aim/planner/planner.hpp"
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
        } else if (scenario == "fire_without_feedback") {
            Planner planner(config);
            const auto plan = planner.plan(world_target(t0), 22);
            require(plan.control && !plan.fire, "Static target requested fire without actual gimbal feedback");
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
            const auto plan = planner.plan(target, 22);
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
