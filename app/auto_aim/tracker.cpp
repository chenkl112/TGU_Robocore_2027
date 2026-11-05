#include "tracker.hpp"

#include <cmath>
#include <numeric>
#include <stdexcept>
#include <utility>

#include "config.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

Tracker::Tracker(const std::string& config_path, Solver& solver)
    : Tracker(load_tracker_config(config_path), solver) {}

Tracker::Tracker(const TrackerConfig& config, Solver& solver)
    : max_match_distance_(config.max_match_distance), solver_(solver),
      enemy_color_(config.enemy_color == "red" ? Color::red : Color::blue),
      priority_mode_(static_cast<PriorityMode>(config.priority_mode)),
      min_detect_count_(config.min_detect_count), max_temp_lost_count_(config.max_temp_lost_count) {
    validate(config);
    LOG_INFO(MODULE, "Enemy armor color: {}", COLORS[enemy_color_]);
}

const char* Tracker::state_name() const {
    switch (state_) {
    case TrackerState::lost:
        return "lost";
    case TrackerState::detecting:
        return "detecting";
    case TrackerState::tracking:
        return "tracking";
    case TrackerState::temporarily_lost:
        return "temp_lost";
    }
    return "unknown";
}

void Tracker::reset() {
    state_ = TrackerState::lost;
    detect_count_ = temp_lost_count_ = 0;
    target_ = Target{};
}

std::list<Target> Tracker::track(std::list<Armor>& armors, TargetClock::time_point t,
                                 bool use_enemy_color) {
    if (!process_frame(armors, t, use_enemy_color))
        return {};
    return {target_};
}

std::optional<TargetSnapshot>
Tracker::track_snapshot(std::list<Armor>& armors, TargetClock::time_point t, bool use_enemy_color) {
    if (!process_frame(armors, t, use_enemy_color))
        return std::nullopt;
    return target_.snapshot();
}

bool Tracker::process_frame(std::list<Armor>& armors, TargetClock::time_point t,
                            bool use_enemy_color) {
    const double dt = target_time::seconds_between(t, last_timestamp_);
    last_timestamp_ = t;

    if (state_ != TrackerState::lost && (dt <= 0 || dt > 0.1)) {
        LOG_WARN(MODULE, "Large dt: {:.3f}s", dt);
        state_ = TrackerState::lost;
    }

    if (use_enemy_color) {
        armors.remove_if([&](const auto_aim::Armor& a) { return a.color != enemy_color_; });
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

    armors.sort(
        [](const auto_aim::Armor& a, const auto_aim::Armor& b) { return a.priority < b.priority; });

    bool found;
    if (state_ == TrackerState::lost) {
        found = set_target(armors, t);
    } else if (state_ == TrackerState::tracking && !armors.empty() &&
               armors.front().priority < target_.priority) {
        found = set_target(armors, t, target_.priority);
        if (found) {
            state_ = TrackerState::lost; // 切换身份后必须重新通过连续检测确认。
            LOG_INFO(MODULE, "Switch to higher-priority target {}", ARMOR_NAMES[target_.name]);
        } else {
            // 高优先级候选解算失败时，继续关联本帧已有目标。
            found = update_target(armors, t);
        }
    } else {
        found = update_target(armors, t);
    }

    state_machine(found);

    if (state_ != TrackerState::lost && target_.diverged()) {
        LOG_DEBUG(MODULE, "[Tracker] Target diverged!");
        state_ = TrackerState::lost;
        return {};
    }

    if (std::accumulate(target_.ekf().recent_nis_failures.begin(),
                        target_.ekf().recent_nis_failures.end(),
                        0) >= (0.4 * target_.ekf().window_size)) {
        LOG_DEBUG(MODULE, "[Target] Bad Converge Found!");
        state_ = TrackerState::lost;
        return {};
    }

    if (state_ == TrackerState::lost || state_ == TrackerState::detecting)
        return {};
    target_.observed = found;

    return true;
}

void Tracker::state_machine(bool found) {
    if (state_ == TrackerState::lost) {
        if (!found)
            return;
        state_ = min_detect_count_ <= 1 ? TrackerState::tracking : TrackerState::detecting;
        detect_count_ = 1;
    } else if (state_ == TrackerState::detecting) {
        if (found) {
            detect_count_++;
            if (detect_count_ >= min_detect_count_)
                state_ = TrackerState::tracking;
        } else {
            detect_count_ = 0;
            state_ = TrackerState::lost;
        }
    } else if (state_ == TrackerState::tracking) {
        if (found)
            return;
        temp_lost_count_ = 1;
        state_ = max_temp_lost_count_ == 0 ? TrackerState::lost : TrackerState::temporarily_lost;
    } else if (state_ == TrackerState::temporarily_lost) {
        if (found) {
            state_ = TrackerState::tracking;
        } else {
            temp_lost_count_++;
            if (temp_lost_count_ > max_temp_lost_count_)
                state_ = TrackerState::lost;
        }
    }
}

bool Tracker::set_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t,
                         std::optional<ArmorPriority> higher_than) {
    for (auto& armor : armors) {
        if (higher_than && armor.priority >= *higher_than)
            break;
        const auto resolved = solver_.resolve_observation(armor);
        if (!resolved)
            continue;
        const auto& candidate = resolved->armor();
        const bool is_balance =
            candidate.type == ArmorType::big &&
            (candidate.name == ArmorName::three || candidate.name == ArmorName::four ||
             candidate.name == ArmorName::five);
        Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
        target_ = Target(candidate, t, 0.2, is_balance ? 2 : 4, P0_dig);
        armor = candidate;
        return true;
    }
    return false;
}

bool Tracker::update_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t) {
    target_.predict(t);

    int solved_count = 0;
    for (auto& armor : armors) {
        if (armor.name != target_.name)
            continue;
        const auto resolved = solver_.resolve_observation(armor, target_.armor_type);
        if (!resolved)
            continue;
        const auto& candidate = resolved->armor();
        if (candidate.type != target_.armor_type)
            continue;
        if (target_.update(candidate, max_match_distance_)) {
            armor = candidate;
            ++solved_count;
        }
    }

    return solved_count > 0;
}

} // namespace app::auto_aim
