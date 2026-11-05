#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/tracker.hpp"
#include "regression_fixtures.hpp"

#include <cmath>
#include <limits>
#include <list>
#include <string>

using namespace auto_aim_test;

namespace {
Target moving_target(Clock::time_point t0) {
    // 从两次有效量测建立非零速度，不通过可写 EKF 状态注入夹具。
    auto target = world_target(t0);
    target.predict(t0 + 10ms);
    Armor measurement(0, 1.0f, {}, {{0, 0}, {1, 0}, {1, 1}, {0, 1}});
    measurement.name = ArmorName::three;
    measurement.type = ArmorType::small;
    measurement.xyz_in_world = {0.81, 0.01, 0.01};
    measurement.ypr_in_world = {0.02, 0, 0};
    measurement.ypd_in_world = tools::xyz2ypd(measurement.xyz_in_world);
    require(target.update(measurement), "Motion fixture observation was rejected");
    require(std::abs(target.ekf_x()[target_state::YawRate]) > 1e-6,
            "Motion fixture did not establish angular velocity");
    return target;
}

void expect_reason(const Plan& plan, StopReason expected) {
    require(!plan.control && !plan.fire && plan.reason == expected,
            "Planner returned the wrong stop reason");
    require(plan.yaw == 0 && plan.pitch == 0 && plan.yaw_vel == 0 && plan.pitch_vel == 0 &&
                plan.yaw_acc == 0 && plan.pitch_acc == 0 && plan.target_yaw == 0 &&
                plan.target_pitch == 0,
            "Rejected plan retained an actuator command");
}
} // namespace

bool run_time_contract_regressions(const std::string& scenario, const std::string& config,
                                   Clock::time_point t0) {
    if (scenario == "prediction_time_contract") {
        const auto initial = moving_target(t0);
        const auto observed = initial.observation_timestamp();
        const auto start = initial.timestamp();
        auto relative = initial;
        auto absolute = initial;
        relative.predict(0.01);
        absolute.predict(start + 10ms);
        require(relative.timestamp() == absolute.timestamp(),
                "Relative prediction did not advance time");
        relative.predict(start + 20ms);
        absolute.predict(start + 20ms);
        require((relative.ekf_x() - absolute.ekf_x()).norm() < 1e-12,
                "Mixed relative/absolute prediction extrapolated the same interval twice");
        require((relative.ekf().P - absolute.ekf().P).norm() < 1e-12,
                "Equivalent prediction sequences changed covariance");
        require(relative.observation_timestamp() == observed,
                "Prediction refreshed observation age");

        const auto snapshot = initial.snapshot();
        for (double dt : {-0.5, 0.0, 0.01, 0.5}) {
            auto reference = initial;
            reference.predict(dt);
            const auto sample = snapshot.predicted(dt);
            require((reference.ekf_x() - sample.state).norm() < 1e-12,
                    "Snapshot kinematics differ from independently propagated EKF state");
            require(sample.timestamp() == reference.timestamp(), "Snapshot and EKF time differ");
            require(sample.observation_timestamp() == observed, "Snapshot changed observation age");
        }
        require(snapshot.timestamp() == start && (snapshot.state - initial.ekf_x()).norm() == 0,
                "Sampling mutated the source snapshot");
        bool rejected = false;
        try {
            relative.predict(std::numeric_limits<double>::quiet_NaN());
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        require(rejected, "Nonfinite prediction interval was accepted");
        for (const auto& [stamp, delta] :
             {std::pair{Clock::time_point{}, 1e300}, std::pair{Clock::time_point::max(), 1e-9},
              std::pair{Clock::time_point::min(), -1e-9}}) {
            rejected = false;
            try {
                (void)target_time::add_seconds(stamp, delta);
            } catch (const std::invalid_argument&) {
                rejected = true;
            }
            require(rejected, "Unrepresentable command deadline was accepted");
        }
        auto extreme = world_target(Clock::time_point::min());
        extreme.predict(Clock::time_point::max());
        require(extreme.timestamp() == Clock::time_point::max() && extreme.ekf_x().allFinite(),
                "Absolute prediction overflowed its timestamp difference");
    } else if (scenario == "planner_time_contract") {
        Planner direct(config), optional(config);
        auto target = world_snapshot(t0);
        expect_reason(direct.plan(std::nullopt, 22, t0), StopReason::no_target);
        expect_reason(direct.plan(TargetSnapshot{}, 22, t0), StopReason::invalid_target);
        expect_reason(direct.plan(target, 22, t0 - 1ms), StopReason::future_state);
        expect_reason(direct.plan(target, 22, t0 + 1s), StopReason::expired_state);
        target.state_time = t0 + 1s;
        expect_reason(direct.plan(target, 22, t0 + 1s), StopReason::expired_observation);
        expect_reason(optional.plan(std::optional{target}, 22, t0 + 1s),
                      StopReason::expired_observation);
        target = world_snapshot(t0);
        target.observation_time = t0 + 1ms;
        expect_reason(direct.plan(target, 22, t0), StopReason::future_observation);
        target = world_snapshot(Clock::time_point::min());
        expect_reason(direct.plan(target, 22, Clock::time_point::max()), StopReason::expired_state);
        target = world_snapshot(Clock::time_point::max());
        expect_reason(direct.plan(target, 22, Clock::time_point::min()), StopReason::future_state);
        expect_reason(direct.plan(target, 22, Clock::time_point::max()),
                      StopReason::invalid_target);
        target = world_snapshot(Clock::time_point::min());
        expect_reason(direct.plan(target, 22, Clock::time_point::min()),
                      StopReason::invalid_target);
        target = world_snapshot(t0);
        target.state[target_state::Radius] = 0;
        expect_reason(direct.plan(target, 22, t0), StopReason::invalid_target);
        target = world_snapshot(t0);
        target.state[target_state::RadiusDifference] = -0.3;
        expect_reason(direct.plan(target, 22, t0), StopReason::invalid_target);
        target = world_snapshot(t0);
        expect_reason(direct.plan(target, std::numeric_limits<double>::quiet_NaN(), t0),
                      StopReason::invalid_bullet_speed);
        const auto unwrapped = direct.plan(target, 22, t0);
        const auto wrapped = optional.plan(std::optional{target}, 22, t0);
        require(unwrapped.control && wrapped.control && unwrapped.yaw == wrapped.yaw &&
                    unwrapped.pitch == wrapped.pitch && unwrapped.reason == StopReason::none,
                "Wrapping a snapshot in optional changed planner behavior");

        // 使用离现在很远的录包时间，验证反馈判龄没有偷用真实机器时钟。
        const auto replay_time = Clock::time_point{} + 10s;
        target = world_snapshot(replay_time);
        io::GimbalState feedback;
        feedback.received_at = replay_time;
        const auto reference = direct.plan(target, 22, replay_time);
        require(reference.control, "Replay fixture did not produce control");
        feedback.yaw = reference.target_yaw;
        feedback.pitch = reference.target_pitch;
        require(direct.plan(target, 22, replay_time, feedback).fire,
                "Recorded aligned feedback was rejected against wall clock");
        feedback.received_at = replay_time + 1ms;
        require(!direct.plan(target, 22, replay_time, feedback).fire,
                "Future recorded feedback allowed fire");
        feedback.received_at = replay_time - 1s;
        require(!direct.plan(target, 22, replay_time, feedback).fire,
                "Stale recorded feedback allowed fire");
    } else if (scenario == "planner_stop_reasons") {
        Planner planner(config);
        auto target = world_snapshot(t0, 2);
        target.state[target_state::Yaw] = CV_PI / 2;
        expect_reason(planner.plan(target, 22, t0), StopReason::no_visible_armor);
        target = world_snapshot(t0);
        target.state[target_state::X] = 10000;
        expect_reason(planner.plan(target, 22, t0), StopReason::ballistic_unreachable);
        auto limited = load_planner_config(config);
        limited.yaw.max_acc = limited.pitch.max_acc = 0.01;
        Planner constrained(limited);
        target = world_snapshot(t0);
        target.state[target_state::X] = 3;
        target.state[target_state::YawRate] = 2;
        expect_reason(constrained.plan(target, 22, t0), StopReason::solver_not_converged);
    } else if (scenario == "tracker_snapshot") {
        Solver solver(config);
        auto settings = load_tracker_config(config);
        settings.min_detect_count = 1;
        settings.max_temp_lost_count = 0;
        Tracker tracker(settings, solver);
        std::list<Armor> armors{image_armor(solver)};
        const auto first = tracker.track_snapshot(armors, t0);
        require(first && first->observed && tracker.state() == TrackerState::tracking,
                "Snapshot tracker failed immediate acquisition");
        std::list<Armor> empty;
        require(!tracker.track_snapshot(empty, t0 + 10ms) && tracker.state() == TrackerState::lost,
                "Zero loss allowance retained a target for an extra frame");
        armors = {image_armor(solver)};
        const auto recovered = tracker.track_snapshot(armors, t0 + 20ms);
        require(recovered && recovered->observation_timestamp() == t0 + 20ms,
                "Snapshot tracker did not recover with a fresh observation");
        require(first->observation_timestamp() == t0 && first->timestamp() == t0,
                "A later tracking frame mutated a published snapshot");
        tracker.reset();
        require(!tracker.track_snapshot(empty, t0 + 30ms), "Reset retained a published target");
    } else {
        return false;
    }
    return true;
}
