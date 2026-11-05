#pragma once

#include "app/auto_aim/config.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/target.hpp"
#include "tools/math_tools.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

namespace auto_aim_test {
using namespace std::chrono_literals;
using namespace app::auto_aim;
using Clock = std::chrono::steady_clock;

inline void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}
inline Armor image_armor(Solver& solver, ArmorName name = ArmorName::three,
                         Eigen::Vector3d xyz = {3, 0, 0}) {
    Armor armor(0, 1.0f, {}, {{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    armor.name = name;
    armor.color = Color::red;
    armor.type = name == ArmorName::one ? ArmorType::big : ArmorType::small;
    const auto projected = solver.reproject_armor(xyz, 0, armor.type);
    std::copy(projected.begin(), projected.end(), armor.points.begin());
    armor.center_norm = {0.5, 0.5};
    return armor;
}
inline Target world_target(Clock::time_point t, int armor_count = 4) {
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

inline TargetSnapshot world_snapshot(Clock::time_point t, int armor_count = 4) {
    return world_target(t, armor_count).snapshot();
}

} // namespace auto_aim_test
