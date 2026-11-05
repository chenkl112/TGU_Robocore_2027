#ifndef TGU_ROBOCORE_2027_TARGET_HPP
#define TGU_ROBOCORE_2027_TARGET_HPP
#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <type_traits>
#include <vector>

#include "armor.hpp"
#include "tools/extended_kalman_filter.hpp"

namespace app::auto_aim {

// 世界坐标系下的匀速、匀角速度模型；位置单位 m，角度单位 rad。
namespace target_state {
enum Index { X, Vx, Y, Vy, Z, Vz, Yaw, YawRate, Radius, RadiusDifference, HeightDifference, Count };
}
using TargetState = Eigen::Matrix<double, target_state::Count, 1>;
using TargetCovariance = Eigen::Matrix<double, target_state::Count, target_state::Count>;
using TargetClock = std::chrono::steady_clock;

namespace target_time {
// 外部秒数转为时钟刻度并相加；非有限值或超出时钟范围时抛出 invalid_argument。
TargetClock::time_point add_seconds(TargetClock::time_point timestamp, double seconds);

// 先取无符号刻度差，避免录包中的极端时间戳在 signed duration 相减时溢出。
inline double seconds_between(TargetClock::time_point later, TargetClock::time_point earlier) {
    using Unsigned = std::make_unsigned_t<TargetClock::duration::rep>;
    using Duration = std::chrono::duration<Unsigned, TargetClock::duration::period>;
    const auto end = later.time_since_epoch().count();
    const auto start = earlier.time_since_epoch().count();
    const bool forward = end >= start;
    const Unsigned ticks =
        forward ? Unsigned(end) - Unsigned(start) : Unsigned(start) - Unsigned(end);
    const double seconds = std::chrono::duration<double>(Duration(ticks)).count();
    return forward ? seconds : -seconds;
}
} // namespace target_time

// 跨线程传递的运动学值快照，不含 EKF 协方差、历史诊断或可变共享状态。
struct TargetSnapshot {
    TargetState state = TargetState::Zero();
    int armor_count = 0;
    bool observed = false;
    ArmorName name = ArmorName::not_armor;
    ArmorType armor_type = ArmorType::small;
    ArmorPriority priority = ArmorPriority::fifth;
    bool jumped = false;
    TargetClock::time_point state_time{};
    TargetClock::time_point observation_time{};

    auto timestamp() const {
        return state_time;
    }
    auto observation_timestamp() const {
        return observation_time;
    }
    bool valid() const;
    // 只生成匀速外推结果；允许负偏移用于规划时域，不改变原快照及最后观测时间。
    TargetSnapshot predicted(double dt) const;
    TargetSnapshot predicted(TargetClock::time_point time) const;
    std::vector<Eigen::Vector4d> armor_xyza_list(double dt = 0.0) const;
};

class Target {
  public:
    bool observed = false;
    ArmorName name = ArmorName::not_armor;
    ArmorType armor_type = ArmorType::small;
    ArmorPriority priority = ArmorPriority::fifth;
    bool jumped = false;

    Target() = default;
    Target(const Armor& armor, TargetClock::time_point t, double radius, int armor_num,
           Eigen::VectorXd P0_dig);

    // 两个接口均推进状态及其时间；最后观测时间只由成功接受的量测更新。
    void predict(TargetClock::time_point t);
    void predict(double dt);
    auto timestamp() const {
        return t_;
    }
    auto observation_timestamp() const {
        return last_observation_;
    }
    bool update(const Armor& armor, double max_match_distance = 0.5);

    TargetSnapshot snapshot() const;
    TargetState ekf_x() const;
    const tools::ExtendedKalmanFilter& ekf() const;
    std::vector<Eigen::Vector4d> armor_xyza_list(double dt = 0.0) const;
    bool diverged() const;

  private:
    int armor_num_ = 0;
    tools::ExtendedKalmanFilter ekf_;
    TargetClock::time_point t_{};
    TargetClock::time_point last_observation_{};

    void predict_state(double dt);
    void update_ypda(const Armor& armor, int id);
    Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd& x, int id) const;
    Eigen::MatrixXd h_jacobian(const Eigen::VectorXd& x, int id) const;
};

} // namespace app::auto_aim

#endif // TGU_ROBOCORE_2027_TARGET_HPP
