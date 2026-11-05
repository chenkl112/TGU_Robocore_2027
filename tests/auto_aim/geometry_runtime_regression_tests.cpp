#include "app/auto_aim/aimer.hpp"
#include "app/auto_aim/shooter.hpp"
#include "app/auto_aim/yolos/keypoints.hpp"
#include "regression_fixtures.hpp"
#include "tools/tomlpp.hpp"
#include "tools/trajectory.hpp"
#ifdef ROBOCORE_TEST_CAMERA
#include "io/usbcamera/usbcamera.hpp"
#endif
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

using namespace auto_aim_test;

bool run_geometry_runtime_regressions(const std::string& scenario, const std::string& config,
                                      Clock::time_point t0) {
    if (scenario == "shooter_alignment") {
        const auto path =
            std::filesystem::temp_directory_path() /
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
    } else if (scenario == "pose_validation") {
        Solver solver(config);
        auto armor = image_armor(solver);
        require(solver.solve(armor), "Valid projected armor did not solve");
        armor.points.fill({10, 10});
        require(!solver.solve(armor), "Degenerate armor pose was accepted");
        armor = image_armor(solver);
        armor.points[0].x = std::numeric_limits<float>::quiet_NaN();
        require(!solver.solve(armor), "Nonfinite armor pose was accepted");
        bool threw = false;
        try {
            solver.set_R_gimbal2world(Eigen::Quaterniond(0, 0, 0, 0));
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        require(threw, "Zero quaternion was accepted");
    } else if (scenario == "tilted_keypoints" || scenario == "perspective_keypoints") {
        auto cfg = toml::parse_file(config);
        auto& camera = *cfg["camera"].as_table();
        camera.insert_or_assign("R_camera2gimbal", toml::array{1., 0., 0., 0., 1., 0., 0., 0., 1.});
        camera.insert_or_assign("t_camera2gimbal", toml::array{0., 0., 0.});
        camera.insert_or_assign("camera_matrix",
                                toml::array{800., 0., 640., 0., 800., 480., 0., 0., 1.});
        camera.insert_or_assign("distort_coeffs", toml::array{0., 0., 0., 0., 0.});
        const auto path =
            std::filesystem::temp_directory_path() /
            ("robocore-corners-" + std::to_string(t0.time_since_epoch().count()) + ".toml");
        std::ofstream(path) << cfg;
        Solver solver(path.string());
        std::filesystem::remove(path);
        const cv::Mat K = (cv::Mat_<double>(3, 3) << 800, 0, 640, 0, 800, 480, 0, 0, 1);
        const Eigen::Matrix3d base{{0, -1, 0}, {0, 0, -1}, {1, 0, 0}};
        for (auto type : {ArmorType::big, ArmorType::small}) {
            const float width = type == ArmorType::big ? 0.230f : 0.135f;
            const std::vector<cv::Point3f> object{{0, width / 2, 0.028f},
                                                  {0, -width / 2, 0.028f},
                                                  {0, -width / 2, -0.028f},
                                                  {0, width / 2, -0.028f}};
            for (double roll : {20., -20., 45., -45., 70., -70., 0.}) {
                for (double slant :
                     (scenario == "perspective_keypoints" ? std::vector<double>{70., 80.}
                                                          : std::vector<double>{0., 35.})) {
                    const Eigen::Matrix3d R =
                        Eigen::AngleAxisd(roll * CV_PI / 180, Eigen::Vector3d::UnitZ())
                            .toRotationMatrix() *
                        Eigen::AngleAxisd(slant * CV_PI / 180, Eigen::Vector3d::UnitY())
                            .toRotationMatrix() *
                        base;
                    cv::Mat rotation, rvec;
                    cv::eigen2cv(R, rotation);
                    cv::Rodrigues(rotation, rvec);
                    std::vector<cv::Point2f> expected;
                    cv::projectPoints(object, rvec, cv::Vec3d(0.1, -0.1, 3), K, cv::Mat(),
                                      expected);
                    // 模型通道固定为左上、左下、右下、右上，与输入图像尺寸无关。
                    std::vector<cv::Point2f> corners{expected[0], expected[3], expected[2],
                                                     expected[1]};
                    require(decode_yolov8_keypoints(corners), "Valid model keypoints rejected");
                    Armor armor(1, 1, {}, corners);
                    armor.type = type;
                    armor.name = ArmorName::one;
                    require(solver.solve(armor), "Tilted armor PnP failed");
                    if (scenario == "perspective_keypoints" && roll == 20.) {
                        std::cout << "type=" << type << " slant=" << slant
                                  << " depth=" << armor.xyz_in_world.z() << '\n';
                    }
                    require((armor.xyz_in_world - Eigen::Vector3d(0.1, -0.1, 3)).norm() < 1e-3,
                            "Model keypoint mapping changed the recovered position");
                    for (int i = 0; i < 4; ++i) {
                        require(cv::norm(corners[i] - expected[i]) < 1e-4,
                                "Physical corner correspondence was permuted");
                    }
                }
            }
        }
        std::vector<cv::Point2f> invalid{{0, 0}, {1, 0}, {2, 0}, {3, 0}};
        require(!decode_yolov8_keypoints(invalid), "Collinear corners were accepted");
        invalid = {{0, 0}, {1, 0}, {1, 1}, {std::numeric_limits<float>::quiet_NaN(), 1}};
        require(!decode_yolov8_keypoints(invalid), "Nonfinite corners were accepted");
    } else if (scenario == "calibration_validation") {
        const auto path =
            std::filesystem::temp_directory_path() /
            ("robocore-calibration-" + std::to_string(t0.time_since_epoch().count()) + ".toml");
        std::ofstream(path) << "[camera]\n";
        bool threw = false;
        try {
            Solver solver(path.string());
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        std::filesystem::remove(path);
        require(threw, "Missing calibration silently became identity matrices");
#ifdef ROBOCORE_TEST_CAMERA
    } else if (scenario == "camera_timeout") {
        io::USBCamera camera(std::string("/nonexistent/robocore-camera.avi"));
        cv::Mat image;
        Clock::time_point timestamp;
        const auto start = Clock::now();
        camera.read(image, timestamp);
        require(image.empty() && Clock::now() - start < 500ms,
                "Unavailable camera blocked the consumer");
        camera.stop();
        const auto stopped = Clock::now();
        camera.read(image, timestamp);
        require(image.empty() && Clock::now() - stopped < 50ms,
                "Stopped camera did not wake the consumer");
#endif
    } else if (scenario == "trajectory") {
        const auto nan = std::numeric_limits<double>::quiet_NaN();
        require(tools::Trajectory(nan, 3, 0).unsolvable, "NaN trajectory accepted");
        require(tools::Trajectory(22, 0, 0).unsolvable, "Zero-distance trajectory accepted");
        require(tools::Trajectory(1, 100, 5).unsolvable, "Unreachable trajectory accepted");
        for (const double h : {-0.5, 0., 0.5}) {
            tools::Trajectory trajectory(22, 3, h);
            require(!trajectory.unsolvable && trajectory.fly_time > 0,
                    "Reachable trajectory failed");
            const double height = 22 * std::sin(trajectory.pitch) * trajectory.fly_time -
                                  0.5 * 9.7833 * trajectory.fly_time * trajectory.fly_time;
            require(std::abs(height - h) < 1e-9, "Trajectory does not hit the requested height");
        }
    } else {
        return false;
    }
    return true;
}
