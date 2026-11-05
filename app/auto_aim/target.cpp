#include "target.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

Target::Target(const Armor& armor, std::chrono::steady_clock::time_point t, double radius,
               int armor_num, Eigen::VectorXd P0_dig)
    : name(armor.name), armor_type(armor.type), priority(armor.priority), armor_num_(armor_num),
      t_(t), last_observation_(t) {
    if (armor_num != 2 && armor_num != 4) {
        throw std::invalid_argument("Target requires two or four armor plates");
    }
    if (P0_dig.size() != target_state::Count || !P0_dig.allFinite() || (P0_dig.array() < 0).any() ||
        !std::isfinite(radius) || radius <= 0) {
        throw std::invalid_argument("Invalid target covariance or radius");
    }
    auto r = radius;
    const auto& xyz = armor.xyz_in_world;
    const auto& ypr = armor.ypr_in_world;

    auto center_x = xyz[0] + r * std::cos(ypr[0]);
    auto center_y = xyz[1] + r * std::sin(ypr[0]);
    auto center_z = xyz[2];

    // x vx y vy z vz a w r l h，其中 l=r2-r1，h=z2-z1。
    TargetState x0;
    x0 << center_x, 0, center_y, 0, center_z, 0, ypr[0], 0, r, 0, 0;
    const TargetCovariance P0 = P0_dig.asDiagonal();

    auto x_add = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a + b;
        c[target_state::Yaw] = tools::limit_rad(c[target_state::Yaw]);
        return c;
    };

    ekf_ = tools::ExtendedKalmanFilter(x0, P0, x_add);
}

void Target::predict(TargetClock::time_point t) {
    predict_state(target_time::seconds_between(t, t_));
    t_ = t;
}

TargetClock::time_point target_time::add_seconds(TargetClock::time_point timestamp,
                                                 double seconds) {
    if (!std::isfinite(seconds))
        throw std::invalid_argument("Timestamp offset must be finite");
    const auto max_delta = std::chrono::duration<double>(TargetClock::duration::max()).count();
    if (std::abs(seconds) >= max_delta)
        throw std::invalid_argument("Timestamp offset is too large");
    const auto offset =
        std::chrono::duration_cast<TargetClock::duration>(std::chrono::duration<double>(seconds));
    if ((offset > TargetClock::duration::zero() &&
         timestamp > TargetClock::time_point::max() - offset) ||
        (offset < TargetClock::duration::zero() &&
         timestamp < TargetClock::time_point::min() - offset)) {
        throw std::invalid_argument("Timestamp addition overflows");
    }
    return timestamp + offset;
}

void Target::predict(double dt) {
    predict(target_time::add_seconds(t_, dt));
}

void Target::predict_state(double dt) {
    if (ekf_.x.size() != target_state::Count) {
        throw std::logic_error("Cannot predict an uninitialized target");
    }
    TargetCovariance F = TargetCovariance::Identity();
    TargetCovariance Q = TargetCovariance::Zero();
    // 每个位置/速度对采用同一匀加速度噪声模型，转动轴使用独立方差。
    constexpr double translation_variance = 100;
    constexpr double rotation_variance = 400;
    const double a = dt * dt * dt * dt / 4;
    const double b = dt * dt * dt / 2;
    const double c = dt * dt;
    for (const auto position :
         {target_state::X, target_state::Y, target_state::Z, target_state::Yaw}) {
        F(position, position + 1) = dt;
        const double variance =
            position == target_state::Yaw ? rotation_variance : translation_variance;
        Q.block<2, 2>(position, position) << a * variance, b * variance, b * variance, c * variance;
    }

    auto f = [&](const Eigen::VectorXd& x) -> Eigen::VectorXd {
        Eigen::VectorXd x_prior = F * x;
        x_prior[target_state::Yaw] = tools::limit_rad(x_prior[target_state::Yaw]);
        return x_prior;
    };

    ekf_.predict(F, Q, f);
}

bool Target::update(const Armor& armor, double max_match_distance) {
    if (ekf_.x.size() != target_state::Count || !armor.xyz_in_world.allFinite() ||
        !armor.ypr_in_world.allFinite() || !armor.ypd_in_world.allFinite() || !ekf_.x.allFinite() ||
        !std::isfinite(max_match_distance) || max_match_distance <= 0)
        return false;
    int id = -1;
    auto min_angle_error = 1e10;
    const std::vector<Eigen::Vector4d>& xyza_list = armor_xyza_list();

    std::vector<std::pair<Eigen::Vector4d, int>> xyza_i_list;
    for (int i = 0; i < armor_num_; i++) {
        xyza_i_list.push_back({xyza_list[i], i});
    }

    std::sort(
        xyza_i_list.begin(), xyza_i_list.end(),
        [](const std::pair<Eigen::Vector4d, int>& a, const std::pair<Eigen::Vector4d, int>& b) {
            Eigen::Vector3d ypd1 = tools::xyz2ypd(a.first.head(3));
            Eigen::Vector3d ypd2 = tools::xyz2ypd(b.first.head(3));
            return ypd1[2] < ypd2[2];
        });

    const int candidate_count = std::min(3, armor_num_);
    for (int i = 0; i < candidate_count; i++) {
        const auto& xyza = xyza_i_list[i].first;
        if ((armor.xyz_in_world - xyza.head<3>()).norm() > max_match_distance)
            continue;
        Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
        auto angle_error = std::abs(tools::limit_rad(armor.ypr_in_world[0] - xyza[3])) +
                           std::abs(tools::limit_rad(armor.ypd_in_world[0] - ypd[0]));

        if (std::abs(angle_error) < std::abs(min_angle_error)) {
            id = xyza_i_list[i].second;
            min_angle_error = angle_error;
        }
    }

    if (id < 0)
        return false;
    if (id != 0)
        jumped = true;

    update_ypda(armor, id);
    last_observation_ = t_;
    return true;
}

void Target::update_ypda(const Armor& armor, int id) {
    Eigen::MatrixXd H = h_jacobian(ekf_.x, id);

    auto center_yaw = std::atan2(armor.xyz_in_world[1], armor.xyz_in_world[0]);
    auto delta_angle = tools::limit_rad(armor.ypr_in_world[0] - center_yaw);
    Eigen::VectorXd R_dig{{4e-3, 4e-3, log(std::abs(delta_angle) + 1) + 1,
                           log(std::abs(armor.ypd_in_world[2]) + 1) / 200 + 9e-2}};

    Eigen::MatrixXd R = R_dig.asDiagonal();

    auto h = [&](const Eigen::VectorXd& x) -> Eigen::Vector4d {
        Eigen::VectorXd xyz = h_armor_xyz(x, id);
        Eigen::VectorXd ypd = tools::xyz2ypd(xyz);
        auto angle = tools::limit_rad(x[target_state::Yaw] + id * 2 * CV_PI / armor_num_);
        return {ypd[0], ypd[1], ypd[2], angle};
    };

    auto z_subtract = [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) -> Eigen::VectorXd {
        Eigen::VectorXd c = a - b;
        c[0] = tools::limit_rad(c[0]);
        c[1] = tools::limit_rad(c[1]);
        c[3] = tools::limit_rad(c[3]);
        return c;
    };

    const auto& ypd = armor.ypd_in_world;
    const auto& ypr = armor.ypr_in_world;
    Eigen::VectorXd z{{ypd[0], ypd[1], ypd[2], ypr[0]}};

    ekf_.update(z, H, R, h, z_subtract);
}

TargetState Target::ekf_x() const {
    if (ekf_.x.size() != target_state::Count) {
        return TargetState::Constant(std::numeric_limits<double>::quiet_NaN());
    }
    return ekf_.x;
}

const tools::ExtendedKalmanFilter& Target::ekf() const {
    return ekf_;
}

TargetSnapshot Target::snapshot() const {
    return {ekf_x(), armor_num_, observed,         name, armor_type, priority,
            jumped,  t_,         last_observation_};
}

std::vector<Eigen::Vector4d> Target::armor_xyza_list(double dt) const {
    return snapshot().armor_xyza_list(dt);
}

bool TargetSnapshot::valid() const {
    const double radius = state[target_state::Radius];
    const double alternate_radius = radius + state[target_state::RadiusDifference];
    return (armor_count == 2 || armor_count == 4) && state.allFinite() && radius > 0.05 &&
           radius < 0.5 && alternate_radius > 0.05 && alternate_radius < 0.5;
}

TargetSnapshot TargetSnapshot::predicted(double dt) const {
    auto result = *this;
    result.state_time = target_time::add_seconds(state_time, dt);
    result.state[target_state::X] += state[target_state::Vx] * dt;
    result.state[target_state::Y] += state[target_state::Vy] * dt;
    result.state[target_state::Z] += state[target_state::Vz] * dt;
    result.state[target_state::Yaw] =
        tools::limit_rad(state[target_state::Yaw] + state[target_state::YawRate] * dt);
    return result;
}

TargetSnapshot TargetSnapshot::predicted(TargetClock::time_point time) const {
    auto result = predicted(target_time::seconds_between(time, state_time));
    result.state_time = time;
    return result;
}

std::vector<Eigen::Vector4d> TargetSnapshot::armor_xyza_list(double dt) const {
    // 纯几何采样允许诊断尚未收敛的半径；规划入口另外通过 valid() 拒绝非法模型。
    if ((armor_count != 2 && armor_count != 4) || !state.allFinite() || !std::isfinite(dt))
        return {};
    const auto projected = predicted(dt);
    const auto& x = projected.state;
    std::vector<Eigen::Vector4d> armors;
    armors.reserve(armor_count);
    for (int id = 0; id < armor_count; ++id) {
        const double angle = tools::limit_rad(x[target_state::Yaw] + id * 2 * CV_PI / armor_count);
        const bool alternate = armor_count == 4 && (id == 1 || id == 3);
        const double radius =
            x[target_state::Radius] + (alternate ? x[target_state::RadiusDifference] : 0);
        armors.push_back({x[target_state::X] - radius * std::cos(angle),
                          x[target_state::Y] - radius * std::sin(angle),
                          x[target_state::Z] + (alternate ? x[target_state::HeightDifference] : 0),
                          angle});
    }
    return armors;
}

bool Target::diverged() const {
    if (ekf_.x.size() != target_state::Count || !ekf_.P.allFinite())
        return true;
    if (snapshot().valid())
        return false;
    LOG_DEBUG(MODULE, "[Target] radius={:.3f}, radius_difference={:.3f}",
              ekf_.x[target_state::Radius], ekf_.x[target_state::RadiusDifference]);
    return true;
}

Eigen::Vector3d Target::h_armor_xyz(const Eigen::VectorXd& x, int id) const {
    auto angle = tools::limit_rad(x[target_state::Yaw] + id * 2 * CV_PI / armor_num_);
    const bool alternate = armor_num_ == 4 && (id == 1 || id == 3);

    const double r = x[target_state::Radius] + (alternate ? x[target_state::RadiusDifference] : 0);
    auto armor_x = x[target_state::X] - r * std::cos(angle);
    auto armor_y = x[target_state::Y] - r * std::sin(angle);
    auto armor_z =
        (alternate) ? x[target_state::Z] + x[target_state::HeightDifference] : x[target_state::Z];

    return {armor_x, armor_y, armor_z};
}

Eigen::MatrixXd Target::h_jacobian(const Eigen::VectorXd& x, int id) const {
    auto angle = tools::limit_rad(x[target_state::Yaw] + id * 2 * CV_PI / armor_num_);
    const bool alternate = armor_num_ == 4 && (id == 1 || id == 3);

    const double r = x[target_state::Radius] + (alternate ? x[target_state::RadiusDifference] : 0);
    auto dx_da = r * std::sin(angle);
    auto dy_da = -r * std::cos(angle);

    auto dx_dr = -std::cos(angle);
    auto dy_dr = -std::sin(angle);
    auto dx_dl = (alternate) ? -std::cos(angle) : 0.0;
    auto dy_dl = (alternate) ? -std::sin(angle) : 0.0;

    auto dz_dh = (alternate) ? 1.0 : 0.0;

    Eigen::MatrixXd H_armor_xyza{{1, 0, 0, 0, 0, 0, dx_da, 0, dx_dr, dx_dl, 0},
                                 {0, 0, 1, 0, 0, 0, dy_da, 0, dy_dr, dy_dl, 0},
                                 {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, dz_dh},
                                 {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0}};

    Eigen::VectorXd armor_xyz = h_armor_xyz(x, id);
    Eigen::MatrixXd H_armor_ypd = tools::xyz2ypd_jacobian(armor_xyz);

    Eigen::MatrixXd H_armor_ypda{{H_armor_ypd(0, 0), H_armor_ypd(0, 1), H_armor_ypd(0, 2), 0},
                                 {H_armor_ypd(1, 0), H_armor_ypd(1, 1), H_armor_ypd(1, 2), 0},
                                 {H_armor_ypd(2, 0), H_armor_ypd(2, 1), H_armor_ypd(2, 2), 0},
                                 {0, 0, 0, 1}};

    return H_armor_ypda * H_armor_xyza;
}

} // namespace app::auto_aim
