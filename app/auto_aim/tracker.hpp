#ifndef TGU_ROBOCORE_2027_TRACKER_HPP
#define TGU_ROBOCORE_2027_TRACKER_HPP
#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <optional>
#include <string>

#include "armor.hpp"
#include "solver.hpp"
#include "target.hpp"

namespace app::auto_aim {

struct TrackerConfig;

enum class TrackerState { lost, detecting, tracking, temporarily_lost };

class Tracker {
  public:
    Tracker(const std::string& config_path, Solver& solver);
    Tracker(const TrackerConfig& config, Solver& solver);

    TrackerState state() const {
        return state_;
    }
    const char* state_name() const;
    void reset();

    std::list<Target> track(std::list<Armor>& armors, std::chrono::steady_clock::time_point t,
                            bool use_enemy_color = true);

    // 正式入口使用该值快照接口，避免把滤波器历史和协方差复制到控制线程。
    std::optional<TargetSnapshot> track_snapshot(std::list<Armor>& armors,
                                                 TargetClock::time_point t,
                                                 bool use_enemy_color = true);

  private:
    bool process_frame(std::list<Armor>& armors, TargetClock::time_point t, bool use_enemy_color);
    double max_match_distance_;
    Solver& solver_;
    Color enemy_color_;
    PriorityMode priority_mode_;
    int min_detect_count_;
    int max_temp_lost_count_;
    int detect_count_ = 0;
    int temp_lost_count_ = 0;
    TrackerState state_ = TrackerState::lost;
    Target target_;
    TargetClock::time_point last_timestamp_{};

    void state_machine(bool found);

    bool set_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t,
                    std::optional<ArmorPriority> higher_than = std::nullopt);

    bool update_target(std::list<Armor>& armors, std::chrono::steady_clock::time_point t);
};

} // namespace app::auto_aim

#endif // TGU_ROBOCORE_2027_TRACKER_HPP
