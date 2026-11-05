#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/tracker.hpp"
#include "regression_fixtures.hpp"

#include <list>
#include <string>

using namespace auto_aim_test;

bool run_tracker_regressions(const std::string& scenario, const std::string& config,
                             Clock::time_point t0) {
    if (scenario == "tracking_confirmation" || scenario == "priority_switch" ||
        scenario == "association_gate") {
        Solver solver(config);
        Tracker tracker(config, solver);
        const auto first_name = scenario == "priority_switch" ? ArmorName::two : ArmorName::three;
        std::list<Target> targets;
        for (int i = 0; i < 5; ++i) {
            std::list<Armor> armors{image_armor(solver, first_name)};
            targets = tracker.track(armors, t0 + i * 10ms);
            require(i == 4 ? !targets.empty() : targets.empty(),
                    "Target escaped detection confirmation");
        }
        require(tracker.state() == TrackerState::tracking,
                "Tracker did not acquire the valid target");
        if (scenario == "priority_switch") {
            std::list<Armor> armors{image_armor(solver, ArmorName::three)};
            require(tracker.track(armors, t0 + 50ms).empty(),
                    "Switched target bypassed confirmation");
            require(tracker.state() == TrackerState::detecting,
                    "Switched target did not restart detection");
        } else if (scenario == "association_gate") {
            const auto before = targets.front().ekf_x();
            std::list<Armor> armors{image_armor(solver, ArmorName::three, {3, 2, 0})};
            targets = tracker.track(armors, t0 + 50ms);
            require(tracker.state() == TrackerState::temporarily_lost,
                    "A distant same-ID target was merged into the track");
            require(!targets.empty() && !targets.front().observed,
                    "Unmatched target remains fire eligible");
            require((targets.front().ekf_x().head<3>() - before.head<3>()).norm() < 0.1,
                    "Distant same-ID observation moved the tracked target");
        } else {
            tracker.reset();
            std::list<Armor> armors{image_armor(solver)};
            require(tracker.track(armors, t0 + 50ms).empty(), "Reset preserved an acquired target");
        }
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
            // 被关联门限拒绝的同 ID 观测不能刷新最后观测时间。
            std::list<Armor> armors;
            if (i % 2 == 0)
                armors.push_back(image_armor(solver, ArmorName::three, {3, 2, 0}));
            targets = tracker.track(armors, now);
            require(!targets.empty(),
                    "Expiry fixture lost its predicted track before the count limit");
            require(targets.front().observation_timestamp() == observed_at,
                    "Rejected/empty frame changed the last observation timestamp");
            if (now - observed_at > 100ms) {
                require(!planner.plan(std::optional{targets.front().snapshot()}, 22, now).control,
                        "Empty/rejected observations refreshed the target age");
            }
        }
        const auto recovered_at = observed_at + 690ms;
        std::list<Armor> recovered{image_armor(solver)};
        targets = tracker.track(recovered, recovered_at);
        require(
            !targets.empty() && targets.front().observation_timestamp() == recovered_at &&
                planner.plan(std::optional{targets.front().snapshot()}, 22, recovered_at).control,
            "A fresh accepted observation did not restore control");
    } else {
        return false;
    }
    return true;
}
