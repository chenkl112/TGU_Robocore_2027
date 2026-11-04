#include "app/auto_aim/aimer.hpp"
#include "app/auto_aim/shooter.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
using namespace app::auto_aim;

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct FireConfig {
    std::filesystem::path path;

    explicit FireConfig(const char* original) {
        auto config = toml::parse_file(original);
        auto& shooter = *config["shooter"].as_table();
        shooter.insert_or_assign("auto_fire", true);
        shooter.insert_or_assign("first_tolerance", 5.0);
        shooter.insert_or_assign("judge_distance", 3.0);
        path = std::filesystem::temp_directory_path() /
            ("robocore-shooter-current-" + std::to_string(
                std::chrono::steady_clock::now().time_since_epoch().count()) + ".toml");
        std::ofstream output(path);
        output << config;
        require(static_cast<bool>(output), "Cannot write shooter test config");
    }

    ~FireConfig() {
        std::error_code ignored;
        std::filesystem::remove(path, ignored);
    }
};

Target near_target() {
    Armor armor(1, 1.0f, {}, {{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    armor.name = ArmorName::three;
    armor.type = ArmorType::small;
    armor.xyz_in_world = {1.0, 0, 0};
    armor.ypr_in_world = {0, 0, 0};
    armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
    Target target(armor, std::chrono::steady_clock::now(), 0.2, 4,
                  Eigen::VectorXd::Ones(11));
    target.observed = true;
    return target;
}
}  // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected config path");
        FireConfig config(argv[1]);
        Aimer aimer(config.path.string());
        aimer.debug_aim_point.valid = true;
        const std::list<Target> targets{near_target()};

        for (int axis = 0; axis < 2; ++axis) {
            Shooter shooter(config.path.string());
            io::Command command;
            command.control = true;
            require(shooter.shoot(command, aimer, targets, {0, 0, 0}),
                    "Initially aligned command cannot fire");
            if (axis == 0) command.yaw = 0.15;
            else command.pitch = 0.15;
            // 8.59 degrees is below the stability threshold (10 degrees),
            // but above the actual alignment tolerance (5 degrees).
            require(!shooter.shoot(command, aimer, targets, {0, 0, 0}),
                    "Feedback aligned only to the previous command allowed fire");
            require(shooter.shoot(command, aimer, targets, {command.yaw, command.pitch, 0}),
                    "Feedback aligned to the current command could not fire");
        }

        {
            Shooter shooter(config.path.string());
            io::Command command;
            command.control = true;
            command.yaw = CV_PI - 0.02;
            shooter.shoot(command, aimer, targets, {command.yaw, 0, 0});
            require(shooter.shoot(command, aimer, targets, {command.yaw, 0, 0}),
                    "Stable command near pi could not fire");
            command.yaw = -CV_PI + 0.02;
            require(shooter.shoot(command, aimer, targets, {CV_PI + 0.02, 0, 0}),
                    "Equivalent wrapped yaw was rejected");
            command.yaw += 2 * CV_PI;
            require(shooter.shoot(command, aimer, targets, {-CV_PI + 0.02, 0, 0}),
                    "Adding a full revolution changed alignment or stability");
        }

        for (int axis = 0; axis < 2; ++axis) {
            Shooter shooter(config.path.string());
            io::Command command;
            command.control = true;
            if (axis == 0) command.yaw = 0.3;
            else command.pitch = 0.3;
            require(!shooter.shoot(command, aimer, targets, {command.yaw, command.pitch, 0}),
                    "Current alignment bypassed the command stability check");
            require(shooter.shoot(command, aimer, targets, {command.yaw, command.pitch, 0}),
                    "Stable aligned command did not recover after a jump");
        }

        {
            Shooter shooter(config.path.string());
            io::Command command;
            command.control = true;
            for (double invalid : {std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity()}) {
                require(!shooter.shoot(command, aimer, targets, {invalid, 0, 0}),
                        "Nonfinite yaw feedback allowed fire");
                require(!shooter.shoot(command, aimer, targets, {0, invalid, 0}),
                        "Nonfinite pitch feedback allowed fire");
            }
            require(shooter.shoot(command, aimer, targets, {0, 0, 0}),
                    "Valid aligned feedback did not recover after invalid feedback");
        }

        std::cout << "PASS current-command yaw/pitch alignment, wrap, stability and invalid feedback\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
