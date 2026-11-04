#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/tracker.hpp"
#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

#include <chrono>
#include <cmath>
#include <iostream>
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
    if (!condition) throw std::runtime_error(message);
}

Armor observation(Solver& solver, ArmorName name, ArmorType physical_type,
                  const Eigen::Vector3d& position, double yaw, Color color) {
    // Physical dimensions generate the image only. The decoder's actual name
    // mapping supplies the initial type, so infantry enters Solver as unknown.
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
    const double yaw = 45.0 * CV_PI / 180.0;
    auto initial = observation(solver, ArmorName::three, ArmorType::big,
                               far_position, yaw, color);
    require(!solver.solve_observation(initial),
            "Distant near-affine armor guessed a size without sufficient evidence");

    auto tracked = observation(solver, ArmorName::three, ArmorType::big,
                               far_position, yaw, color);
    require(solver.solve_observation(tracked, ArmorType::big),
            "A matching tracked size could not resolve the distant ambiguous observation");
    require(tracked.type == ArmorType::big &&
                (tracked.xyz_in_world - far_position).norm() < 0.02,
            "Tracked-size resolution changed the physical dimensions or depth");

    // Close-range perspective provides clear evidence. A prior is allowed to
    // resolve ambiguity, but cannot override a clearly different physical size.
    for (auto physical_type : {ArmorType::big, ArmorType::small}) {
        const auto wrong_prior = physical_type == ArmorType::big ? ArmorType::small : ArmorType::big;
        auto clear = observation(solver, ArmorName::four, physical_type, {1, 0, 0}, yaw, color);
        require(solver.solve_observation(clear, wrong_prior),
                "Clear opposite-size observation was rejected solely because of its prior");
        require(clear.type == physical_type,
                "Tracked prior overrode clear evidence for the opposite armor size");
        require((clear.xyz_in_world - Eigen::Vector3d(1, 0, 0)).norm() < 0.01,
                "Clear opposite-size observation retained the prior's incorrect depth");
    }
}

Eigen::Vector3d balance_plate(double yaw) {
    // The body stays at (3.2, 0, 0); the observed plate moves on its 0.2 m radius.
    return {3.2 - 0.2 * std::cos(yaw), -0.2 * std::sin(yaw), 0};
}

std::list<Target> acquire_balance(Tracker& tracker, Solver& solver, Color color,
                                 int confirmations, Clock::time_point t0) {
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

void continuous_balance(const std::string& config, Solver& solver, Color color,
                        int confirmations) {
    Tracker tracker(config, solver);
    const auto t0 = Clock::now();
    acquire_balance(tracker, solver, color, confirmations, t0);
    bool crossed_ratio_threshold = false;
    for (int step = 1; step <= 40; ++step) {
        const double yaw = 2.0 * step * CV_PI / 180.0;
        const auto stamp = t0 + (confirmations + step - 1) * 10ms;
        auto armor = observation(solver, ArmorName::three, ArmorType::big,
                                 balance_plate(yaw), yaw, color);
        crossed_ratio_threshold = crossed_ratio_threshold || armor.ratio < 3.0;
        std::list<Armor> armors{std::move(armor)};
        const auto targets = tracker.track(armors, stamp);
        const auto context = " at " + std::to_string(step * 2) + " degrees";
        require(targets.size() == 1 && targets.front().observed,
                "Balanced infantry lost an accepted observation while rotating across ratio 3" + context);
        const auto& target = targets.front();
        require(target.armor_type == ArmorType::big && target.armor_xyza_list().size() == 2,
                "Rotating balanced infantry changed size or switched to the four-armor model" + context);
        require(target.observation_timestamp() == stamp,
                "Continuous balanced-infantry observation did not refresh its measurement time");
    }
    require(crossed_ratio_threshold, "Balance fixture never crossed the old ratio threshold");
    std::cout << "balanced infantry: 40 accepted rotating frames, large type and 2 plates preserved\n";
}

void distant_same_id(const std::string& config, Solver& solver, Color color,
                     int confirmations) {
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

void ambiguous_priority(const std::string& config, Solver& solver, Color color,
                        int confirmations) {
    Tracker tracker(config, solver);
    const auto t0 = Clock::now();
    for (int i = 0; i < confirmations + 5; ++i) {
        // Infantry outranks sentry in both supported priority modes. An
        // unresolved size must neither starve initial acquisition nor break it.
        std::list<Armor> armors{
            observation(solver, ArmorName::three, ArmorType::big, {10, 0, 0}, CV_PI / 4, color),
            observation(solver, ArmorName::sentry, ArmorType::small, {3, 0, 0}, 0, color)};
        const auto stamp = t0 + i * 10ms;
        const auto targets = tracker.track(armors, stamp);
        if (i < confirmations - 1) continue;
        require(targets.size() == 1 && targets.front().name == ArmorName::sentry &&
                    targets.front().observed && targets.front().observation_timestamp() == stamp,
                "Ambiguous higher-priority infantry interrupted the valid sentry track");
    }
}

}  // namespace

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
