#include "tracker.hpp"

#include <numeric>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

Tracker::Tracker(const std::string& config_path, Solver& solver)
    : solver_{solver},
      priority_mode_{PriorityMode::mode_one},
      detect_count_(0),
      temp_lost_count_(0),
      state_{"lost"},
      pre_state_{"lost"},
      last_timestamp_(std::chrono::steady_clock::now()) {
    auto config = toml::parse_file(config_path);

    const std::string enemy_color_str =
        config["tracker"]["enemy_color"].value_or(std::string{"blue"});
    if (enemy_color_str == "red") {
        enemy_color_ = Color::red;
    } else {
        if (enemy_color_str != "blue") {
            LOG_WARN(MODULE, "Invalid enemy_color {}, using blue", enemy_color_str);
        }
        enemy_color_ = Color::blue;
    }
    LOG_INFO(MODULE, "Enemy armor color: {}", COLORS[enemy_color_]);
    const int priority_mode = config["tracker"]["priority_mode"].value_or(1);
    if (priority_mode == static_cast<int>(PriorityMode::mode_two)) {
        priority_mode_ = PriorityMode::mode_two;
    } else if (priority_mode != static_cast<int>(PriorityMode::mode_one)) {
        LOG_WARN(MODULE, "Invalid priority_mode {}, using mode 1", priority_mode);
    }
    min_detect_count_ = config["tracker"]["min_detect_count"].value_or(3);
    max_temp_lost_count_ = config["tracker"]["max_temp_lost_count"].value_or(10);
    normal_temp_lost_count_ = max_temp_lost_count_;
}

std::string Tracker::state() const { return state_; }

std::list<Target> Tracker::track(
    std::list<Armor>& armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color) {
    auto dt = tools::delta_time(t, last_timestamp_);
    last_timestamp_ = t;

    if (state_ != "lost" && dt > 0.1) {
        LOG_WARN(MODULE, "Large dt: {:.3f}s", dt);
        state_ = "lost";
    }

    if (use_enemy_color) {
        armors.remove_if(
            [&](const auto_aim::Armor& a) { return a.color != enemy_color_; });
    }
    armors.remove_if([](const Armor& armor) { return !is_robot_target(armor.name); });

    for (auto& armor : armors) {
        armor.priority = armor_priority(armor.name, priority_mode_);
    }

    armors.sort([](const Armor& a, const Armor& b) {
        auto distance_1 = cv::norm(a.center_norm - cv::Point2f(0.5F, 0.5F));
        auto distance_2 = cv::norm(b.center_norm - cv::Point2f(0.5F, 0.5F));
        return distance_1 < distance_2;
    });

    armors.sort([](const auto_aim::Armor& a, const auto_aim::Armor& b) {
        return a.priority < b.priority;
    });

    bool found;
    if (state_ == "lost") {
        found = set_target(armors, t);
    } else if (
        state_ == "tracking" && !armors.empty() &&
        armors.front().priority < target_.priority) {
        found = set_target(armors, t);
        if (found) {
            LOG_INFO(
                MODULE, "Switch to higher-priority target {}",
                ARMOR_NAMES[armors.front().name]);
        }
    } else {
        found = update_target(armors, t);
    }

    state_machine(found);

    if (state_ != "lost" && target_.diverged()) {
        LOG_DEBUG(MODULE, "[Tracker] Target diverged!");
        state_ = "lost";
        return {};
    }

    if (
        std::accumulate(
            target_.ekf().recent_nis_failures.begin(),
            target_.ekf().recent_nis_failures.end(), 0) >=
        (0.4 * target_.ekf().window_size)) {
        LOG_DEBUG(MODULE, "[Target] Bad Converge Found!");
        state_ = "lost";
        return {};
    }

    if (state_ == "lost") return {};

    std::list<Target> targets = {target_};
    return targets;
}

void Tracker::state_machine(bool found) {
    if (state_ == "lost") {
        if (!found) return;
        state_ = "detecting";
        detect_count_ = 1;
    } else if (state_ == "detecting") {
        if (found) {
            detect_count_++;
            if (detect_count_ >= min_detect_count_) state_ = "tracking";
        } else {
            detect_count_ = 0;
            state_ = "lost";
        }
    } else if (state_ == "tracking") {
        if (found) return;
        temp_lost_count_ = 1;
        state_ = "temp_lost";
    } else if (state_ == "temp_lost") {
        if (found) {
            state_ = "tracking";
        } else {
            temp_lost_count_++;
            max_temp_lost_count_ = normal_temp_lost_count_;
            if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
        }
    }
}

bool Tracker::set_target(
    std::list<Armor>& armors, std::chrono::steady_clock::time_point t) {
    if (armors.empty()) return false;

    auto& armor = armors.front();
    if (!solver_.solve(armor)) return false;

    auto is_balance = (armor.type == ArmorType::big) &&
                      (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                       armor.name == ArmorName::five);

    if (is_balance) {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        target_ = Target(armor, t, 0.2, 2, P0_dig);
    } else {
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        target_ = Target(armor, t, 0.2, 4, P0_dig);
    }

    return true;
}

bool Tracker::update_target(
    std::list<Armor>& armors, std::chrono::steady_clock::time_point t) {
    target_.predict(t);

    int found_count = 0;
    for (const auto& armor : armors) {
        if (armor.name != target_.name || armor.type != target_.armor_type) continue;
        found_count++;
    }

    if (found_count == 0) return false;

    int solved_count = 0;
    for (auto& armor : armors) {
        if (armor.name != target_.name || armor.type != target_.armor_type) continue;

        if (!solver_.solve(armor)) continue;
        target_.update(armor);
        ++solved_count;
    }

    return solved_count > 0;
}

}  // namespace app::auto_aim
