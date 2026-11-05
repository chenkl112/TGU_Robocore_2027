#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/tracker.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <list>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

using namespace std::chrono_literals;
using namespace app::auto_aim;
using Clock = std::chrono::steady_clock;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition)
        throw std::runtime_error(message);
}

Armor observation(Solver& solver, ArmorName name, ArmorType physical_type,
                  const Eigen::Vector3d& position, double yaw, Color color) {
    // 物理尺寸只用于生成测试图像；初始类型仍由解码器的类别映射给出。
    // 步兵进入 Solver 时保持 unknown，不能预先泄露正确尺寸。
    auto corners = solver.reproject_armor(position, yaw, physical_type);
    Armor armor(1, 1.0f, {}, std::move(corners));
    armor.name = name;
    armor.color = color;
    armor.type = armor_type_from_name(name);
    armor.center_norm = {0.5f, 0.5f};
    return armor;
}

void check_name_mapping() {
    require(armor_type_from_name(ArmorName::one) == ArmorType::big,
            "Hero did not retain its known large-armor type");
    require(armor_type_from_name(ArmorName::two) == ArmorType::small &&
                armor_type_from_name(ArmorName::sentry) == ArmorType::small,
            "Engineer or sentry did not retain its known small-armor type");
    for (auto name : {ArmorName::three, ArmorName::four, ArmorName::five}) {
        require(armor_type_from_name(name) == ArmorType::unknown,
                "Infantry size was confirmed before calibrated geometry was available");
    }
}

void first_observation(Solver& solver, Color color) {
    int accepted = 0, ambiguous = 0;
    const Eigen::Vector3d position{3, 0, 0};
    for (auto name : {ArmorName::three, ArmorName::four, ArmorName::five}) {
        for (auto physical_type : {ArmorType::big, ArmorType::small}) {
            for (double degrees : {0., 35., 45., 50., 70., 80.}) {
                auto armor = observation(solver, name, physical_type, position,
                                         degrees * CV_PI / 180.0, color);
                require(armor.type == ArmorType::unknown,
                        "Initial infantry observation bypassed type resolution");
                const bool solved = solver.solve_observation(armor);
                if (physical_type == ArmorType::big && degrees == 45.0) {
                    require(solved, "Clear 45-degree large armor was not acquired without a prior");
                }
                if (!solved) {
                    ++ambiguous;
                    continue;
                }
                ++accepted;
                const auto context = " at " + std::to_string(degrees) + " degrees";
                require(armor.type == physical_type,
                        "First observation selected the wrong physical dimensions" + context);
                require((armor.xyz_in_world - position).norm() < 0.01,
                        "Resolved armor retained the wrong-size distance" + context);
            }
        }
    }
    require(accepted > 0, "All initial observations were rejected");
    std::cout << "initial geometry: accepted=" << accepted << " ambiguous=" << ambiguous << '\n';
}

void ambiguity_and_prior(Solver& solver, Color color) {
    const Eigen::Vector3d far_position{10, 0, 0};
    // 加入重力方向约束后，45 度大装甲仍能排除小装甲安装模型；
    // 70 度时两种尺寸在配置的原图像素噪声范围内确实无法区分。
    const double yaw = 70.0 * CV_PI / 180.0;
    auto initial = observation(solver, ArmorName::three, ArmorType::big, far_position, yaw, color);
    require(!solver.solve_observation(initial),
            "Distant near-affine armor guessed a size without sufficient evidence");

    auto tracked = observation(solver, ArmorName::three, ArmorType::big, far_position, yaw, color);
    require(solver.solve_observation(tracked, ArmorType::big),
            "A matching tracked size could not resolve the distant ambiguous observation");
    require(tracked.type == ArmorType::big && (tracked.xyz_in_world - far_position).norm() < 0.02,
            "Tracked-size resolution changed the physical dimensions or depth");

    // 近距离透视提供清晰的尺寸证据；先验只能消解歧义，不能覆盖相反的明确证据。
    for (auto physical_type : {ArmorType::big, ArmorType::small}) {
        const auto wrong_prior =
            physical_type == ArmorType::big ? ArmorType::small : ArmorType::big;
        auto clear =
            observation(solver, ArmorName::four, physical_type, {1, 0, 0}, CV_PI / 4, color);
        require(solver.solve_observation(clear, wrong_prior),
                "Clear opposite-size observation was rejected solely because of its prior");
        require(clear.type == physical_type,
                "Tracked prior overrode clear evidence for the opposite armor size");
        require((clear.xyz_in_world - Eigen::Vector3d(1, 0, 0)).norm() < 0.01,
                "Clear opposite-size observation retained the prior's incorrect depth");
    }
}

void corner_noise(Armor& armor, int mask, float amplitude = 1.0f) {
    for (int coordinate = 0; coordinate < 8; ++coordinate) {
        const float delta = ((mask >> coordinate) & 1) ? amplitude : -amplitude;
        if (coordinate % 2)
            armor.points[coordinate / 2].y += delta;
        else
            armor.points[coordinate / 2].x += delta;
    }
}

void noisy_size_hypotheses(Solver& solver, Color color) {
    int accepted = 0, ambiguous = 0;
    for (double distance : {1., 3., 5.}) {
        for (auto type : {ArmorType::big, ArmorType::small}) {
            int near_accepted = 0;
            for (double degrees : {0., 45., 70.}) {
                for (int mask = 0; mask < 256; ++mask) {
                    const Eigen::Vector3d position{distance, 0, 0};
                    auto noisy = observation(solver, ArmorName::three, type, position,
                                             degrees * CV_PI / 180.0, color);
                    corner_noise(noisy, mask);
                    auto tracked = noisy;
                    const auto context = " at distance=" + std::to_string(distance) +
                                         " yaw=" + std::to_string(degrees) +
                                         " mask=" + std::to_string(mask);
                    if (solver.solve_observation(noisy)) {
                        require(noisy.type == type,
                                "Noisy corners selected the wrong size" + context);
                        require((noisy.xyz_in_world - position).norm() < 0.6,
                                "Noisy corners caused a large range error" + context);
                        ++accepted;
                        ++near_accepted;
                    } else {
                        ++ambiguous;
                    }
                    require(solver.solve_observation(tracked, type) && tracked.type == type,
                            "Correct tracked size failed on a noisy observation" + context);
                }
            }
            // 普通距离下两种尺寸都应能被识别；拒绝全部含噪观测不算修复初始化问题。
            if (distance <= 3)
                require(near_accepted >= 256,
                        "Noise policy prevented ordinary-range infantry acquisition");
        }
    }
    std::cout << "corner-noise sweep: accepted=" << accepted << " ambiguous=" << ambiguous
              << ", no incorrect size and all correct-prior updates accepted\n";
}

void noisy_acquisition_recovery(const std::string& config, Solver& solver, Color color,
                                int confirmations) {
    Tracker tracker(config, solver);
    const auto t0 = Clock::now();
    const Eigen::Vector3d position{3, 0, 0};
    for (int frame = 0; frame < confirmations + 12; ++frame) {
        auto armor = observation(solver, ArmorName::three, ArmorType::big, position, 0, color);
        if (frame < confirmations + 3)
            corner_noise(armor, 17);
        if (frame == 0) {
            auto first = armor;
            require(solver.solve_observation(first) && first.type == ArmorType::big,
                    "The review's mask17 fixture did not acquire the correct large size");
            require((first.xyz_in_world - position).norm() < 0.1,
                    "The review's mask17 fixture retained metre-scale range error");
        }
        const auto stamp = t0 + frame * 10ms;
        std::list<Armor> armors{std::move(armor)};
        const auto targets = tracker.track(armors, stamp);
        if (frame < confirmations - 1)
            continue;
        require(targets.size() == 1 && targets.front().observed &&
                    targets.front().observation_timestamp() == stamp,
                "Clean observations could not update the track acquired from noisy corners");
        require(targets.front().armor_type == ArmorType::big &&
                    targets.front().armor_xyza_list().size() == 2,
                "Noisy initialization poisoned the balanced-infantry model");
    }
}

Armor pitched_observation(Solver& solver, ArmorType physical_type, const Eigen::Vector3d& position,
                          double yaw, double pitch, double roll, Color color) {
    const auto rotation = tools::rotation_matrix({yaw, pitch, roll});
    const float width = physical_type == ArmorType::big ? 0.230f : 0.135f;
    const std::vector<Eigen::Vector3d> object{{0, width / 2, 0.028},
                                              {0, -width / 2, 0.028},
                                              {0, -width / 2, -0.028},
                                              {0, width / 2, -0.028}};
    std::vector<cv::Point3f> world_points;
    for (const auto& point : object) {
        const Eigen::Vector3d world = position + rotation * point;
        world_points.emplace_back(world.x(), world.y(), world.z());
    }
    Armor result(1, 1, {}, solver.world2pixel(world_points));
    result.name = ArmorName::three;
    result.type = ArmorType::unknown;
    result.color = color;
    return result;
}

void balance_pitch_and_gravity(Solver& solver, Color color) {
    for (const auto& orientation :
         {Eigen::Quaterniond::Identity(),
          Eigen::Quaterniond(Eigen::AngleAxisd(170 * CV_PI / 180, Eigen::Vector3d::UnitZ()) *
                             Eigen::AngleAxisd(12 * CV_PI / 180, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd(-25 * CV_PI / 180, Eigen::Vector3d::UnitX()))}) {
        solver.set_R_gimbal2world(orientation);
        const Eigen::Vector3d position = solver.R_gimbal2world() * Eigen::Vector3d(3, 0, 0);
        const double bearing = std::atan2(position.y(), position.x());
        for (double pitch : {-60., -30., -10., 0., 15., 30., 45., 60.}) {
            auto armor = pitched_observation(solver, ArmorType::big, position, bearing,
                                             pitch * CV_PI / 180, 0, color);
            corner_noise(armor, 17, 0.5f);
            require(solver.solve_observation(armor, ArmorType::big) && armor.type == ArmorType::big,
                    "Gravity-aware classification rejected a dynamically pitched balance plate");
            require((armor.xyz_in_world - position).norm() < 0.1,
                    "Balancing pitch or camera roll biased the estimated range");
        }
        for (auto type : {ArmorType::big, ArmorType::small}) {
            auto armor = pitched_observation(solver, type, position, bearing, CV_PI / 12, 0, color);
            corner_noise(armor, 17);
            require(solver.solve_observation(armor) && armor.type == type,
                    "Camera attitude changed an ordinary-range size decision");
        }
    }
    solver.set_R_gimbal2world(Eigen::Quaterniond::Identity());
}

void mounting_model_mismatch(Solver& solver, Color color) {
    int acquired = 0, uncertain = 0;
    for (double distance : {1., 3., 5.}) {
        for (double degrees : {0., 45., 70.}) {
            for (auto type : {ArmorType::big, ArmorType::small}) {
                for (const auto& [pitch, roll] :
                     std::vector<std::pair<double, double>>{{15, -5}, {15, 5}, {0, 0}, {30, 0}}) {
                    for (int mask = 0; mask < 256; ++mask) {
                        const Eigen::Vector3d position{distance, 0, 0};
                        auto armor =
                            pitched_observation(solver, type, position, degrees * CV_PI / 180,
                                                pitch * CV_PI / 180, roll * CV_PI / 180, color);
                        corner_noise(armor, mask);
                        const auto context =
                            " at distance=" + std::to_string(distance) +
                            " type=" + std::to_string(type) + " yaw=" + std::to_string(degrees) +
                            " pitch=" + std::to_string(pitch) + " roll=" + std::to_string(roll) +
                            " mask=" + std::to_string(mask);
                        if (!solver.solve_observation(armor)) {
                            ++uncertain;
                            continue;
                        }
                        ++acquired;
                        require(armor.type == type,
                                "Chassis tilt was mistaken for a different armor size" + context);
                    }
                }
            }
        }
    }
    std::cout << "mounting-model perturbations: accepted=" << acquired << " uncertain=" << uncertain
              << ", no incorrect size\n";
}

void size_configuration(const std::string& config) {
    const auto original = toml::parse_file(config);
    const auto path = std::filesystem::temp_directory_path() /
                      ("robocore-size-config-" +
                       std::to_string(Clock::now().time_since_epoch().count()) + ".toml");
    for (const auto& [key, value] : std::vector<std::pair<std::string, double>>{
             {"size_noise_floor", 0},
             {"size_noise_floor", -1},
             {"size_noise_floor", 4},
             {"size_noise_floor", std::numeric_limits<double>::infinity()},
             {"max_balance_pitch", 0},
             {"max_balance_pitch", 90},
             {"max_balance_pitch", std::numeric_limits<double>::quiet_NaN()}}) {
        auto invalid = original;
        invalid["solver"].as_table()->insert_or_assign(key, value);
        std::ofstream(path) << invalid;
        bool rejected = false;
        try {
            Solver solver(path.string());
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        std::filesystem::remove(path);
        require(rejected, "Invalid physical-size solver configuration was accepted: " + key);
    }
}

Eigen::Vector3d balance_plate(double yaw) {
    // 车体中心固定在 (3.2, 0, 0)，观测装甲绕半径 0.2 m 的圆运动。
    return {3.2 - 0.2 * std::cos(yaw), -0.2 * std::sin(yaw), 0};
}

std::list<Target> acquire_balance(Tracker& tracker, Solver& solver, Color color, int confirmations,
                                  Clock::time_point t0) {
    std::list<Target> targets;
    for (int i = 0; i < confirmations; ++i) {
        std::list<Armor> armors{
            observation(solver, ArmorName::three, ArmorType::big, balance_plate(0), 0, color)};
        targets = tracker.track(armors, t0 + i * 10ms);
    }
    require(targets.size() == 1 && targets.front().observed,
            "Could not acquire the balanced-infantry fixture through type resolution");
    require(targets.front().armor_type == ArmorType::big &&
                targets.front().armor_xyza_list().size() == 2,
            "Balanced infantry was initialized with the wrong size or armor count");
    return targets;
}

void continuous_balance(const std::string& config, Solver& solver, Color color, int confirmations) {
    Tracker tracker(config, solver);
    const auto t0 = Clock::now();
    acquire_balance(tracker, solver, color, confirmations, t0);
    bool crossed_ratio_threshold = false;
    for (int step = 1; step <= 40; ++step) {
        const double yaw = 2.0 * step * CV_PI / 180.0;
        const auto stamp = t0 + (confirmations + step - 1) * 10ms;
        auto armor =
            observation(solver, ArmorName::three, ArmorType::big, balance_plate(yaw), yaw, color);
        crossed_ratio_threshold = crossed_ratio_threshold || armor.ratio < 3.0;
        std::list<Armor> armors{std::move(armor)};
        const auto targets = tracker.track(armors, stamp);
        const auto context = " at " + std::to_string(step * 2) + " degrees";
        require(targets.size() == 1 && targets.front().observed,
                "Balanced infantry lost an accepted observation while rotating across ratio 3" +
                    context);
        const auto& target = targets.front();
        require(target.armor_type == ArmorType::big && target.armor_xyza_list().size() == 2,
                "Rotating balanced infantry changed size or switched to the four-armor model" +
                    context);
        require(target.observation_timestamp() == stamp,
                "Continuous balanced-infantry observation did not refresh its measurement time");
    }
    require(crossed_ratio_threshold, "Balance fixture never crossed the old ratio threshold");
    std::cout
        << "balanced infantry: 40 accepted rotating frames, large type and 2 plates preserved\n";
}

void distant_same_id(const std::string& config, Solver& solver, Color color, int confirmations) {
    Tracker tracker(config, solver);
    const auto t0 = Clock::now();
    const auto acquired = acquire_balance(tracker, solver, color, confirmations, t0);
    const auto last_observation = acquired.front().observation_timestamp();
    const auto stamp = t0 + confirmations * 10ms;
    std::list<Armor> far_armors{
        observation(solver, ArmorName::three, ArmorType::big, {3, 2, 0}, 0, color)};
    const auto targets = tracker.track(far_armors, stamp);
    require(targets.size() == 1 && !targets.front().observed,
            "A distant same-ID observation inherited the tracked type and entered the track");
    require(targets.front().observation_timestamp() == last_observation,
            "Rejected same-ID observation refreshed the accepted measurement timestamp");
    require(targets.front().armor_type == ArmorType::big &&
                targets.front().armor_xyza_list().size() == 2,
            "Rejected observation changed the existing target's physical model");
}

void ambiguous_priority(const std::string& config, Solver& solver, Color color, int confirmations) {
    Tracker tracker(config, solver);
    const auto t0 = Clock::now();
    for (int i = 0; i < confirmations + 5; ++i) {
        // 两种优先级模式中步兵都高于哨兵；未确定尺寸的步兵不能阻止初次建轨或破坏已有轨迹。
        std::list<Armor> armors{
            observation(solver, ArmorName::three, ArmorType::big, {10, 0, 0}, 70 * CV_PI / 180,
                        color),
            observation(solver, ArmorName::sentry, ArmorType::small, {3, 0, 0}, 0, color)};
        const auto stamp = t0 + i * 10ms;
        const auto targets = tracker.track(armors, stamp);
        if (i < confirmations - 1)
            continue;
        require(targets.size() == 1 && targets.front().name == ArmorName::sentry &&
                    targets.front().observed && targets.front().observation_timestamp() == stamp,
                "Ambiguous higher-priority infantry interrupted the valid sentry track");
    }
}

} // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected config path");
        const std::string config = argv[1];
        const auto settings = toml::parse_file(config);
        const auto enemy = settings["tracker"]["enemy_color"].value_or(std::string{"blue"});
        const auto color = enemy == "red" ? Color::red : Color::blue;
        const int confirmations = settings["tracker"]["min_detect_count"].value_or(3);
        require(confirmations > 0, "Fixture requires a positive detection-confirmation count");
        Solver solver(config);
        check_name_mapping();
        first_observation(solver, color);
        ambiguity_and_prior(solver, color);
        noisy_size_hypotheses(solver, color);
        noisy_acquisition_recovery(config, solver, color, confirmations);
        balance_pitch_and_gravity(solver, color);
        mounting_model_mismatch(solver, color);
        size_configuration(config);
        continuous_balance(config, solver, color, confirmations);
        distant_same_id(config, solver, color, confirmations);
        ambiguous_priority(config, solver, color, confirmations);
        std::cout << "PASS armor type resolution and tracking\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
