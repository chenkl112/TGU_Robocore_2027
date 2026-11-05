#include "planner.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include "../config.hpp"
#include "tinympc/admm.hpp"
#include "tinympc/tiny_api.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace app::auto_aim {
namespace {
// 固定迭代上限兼顾参考变化时的收敛和单帧计算量，保留跨帧热启动。
constexpr int MAX_SOLVER_ITERATIONS = 100;
constexpr double SOLVER_RHO = 30.0;
constexpr int SHOOT_OFFSET = 2;

bool facing_gimbal(const Eigen::Vector4d& armor) {
    return armor.x() * std::cos(armor.w()) + armor.y() * std::sin(armor.w()) > 0;
}

struct InterceptionFailure {
    StopReason reason;
};

Plan stopped(StopReason reason) {
    Plan result;
    result.reason = reason;
    return result;
}

struct SolverDeleter {
    void operator()(TinySolver* solver) const {
        if (!solver)
            return;
        delete solver->solution;
        delete solver->cache;
        delete solver->settings;
        delete solver->work;
        delete solver;
    }
};
} // namespace

const char* stop_reason_name(StopReason reason) {
    switch (reason) {
    case StopReason::none:
        return "none";
    case StopReason::no_target:
        return "no_target";
    case StopReason::invalid_target:
        return "invalid_target";
    case StopReason::future_state:
        return "future_state";
    case StopReason::expired_state:
        return "expired_state";
    case StopReason::future_observation:
        return "future_observation";
    case StopReason::expired_observation:
        return "expired_observation";
    case StopReason::invalid_bullet_speed:
        return "invalid_bullet_speed";
    case StopReason::no_visible_armor:
        return "no_visible_armor";
    case StopReason::ballistic_unreachable:
        return "ballistic_unreachable";
    case StopReason::solver_not_converged:
        return "solver_not_converged";
    case StopReason::invalid_solution:
        return "invalid_solution";
    }
    return "unknown";
}

// 一根轴独立拥有求解器及热启动状态，yaw/pitch 共用初始化与约束投影逻辑。
class Planner::AxisSolver {
  public:
    explicit AxisSolver(const MpcAxisConfig& config) {
        const Eigen::Matrix2d A{{1, DT}, {0, 1}};
        const Eigen::Vector2d B{0, DT};
        const Eigen::Vector2d Q{config.Q[0], config.Q[1]};
        const Eigen::Matrix<double, 1, 1> R = Eigen::Matrix<double, 1, 1>::Constant(config.R);
        TinySolver* raw = nullptr;
        const int status = tiny_setup(&raw, A, B, Eigen::Vector2d::Zero(), Q.asDiagonal(), R,
                                      SOLVER_RHO, 2, 1, HORIZON, 0);
        solver_.reset(raw);
        if (status != 0)
            throw std::runtime_error("Failed to initialize MPC axis solver");
        const int bounds =
            tiny_set_bound_constraints(raw, Eigen::MatrixXd::Constant(2, HORIZON, -1e17),
                                       Eigen::MatrixXd::Constant(2, HORIZON, 1e17),
                                       Eigen::MatrixXd::Constant(1, HORIZON - 1, -config.max_acc),
                                       Eigen::MatrixXd::Constant(1, HORIZON - 1, config.max_acc));
        if (bounds != 0)
            throw std::runtime_error("Failed to initialize MPC axis bounds");
        solver_->settings->max_iter = MAX_SOLVER_ITERATIONS;
    }

    bool solve(const Eigen::Matrix<double, 2, HORIZON>& reference) {
        tiny_set_x0(solver_.get(), reference.col(0));
        solver_->work->Xref = reference;
        update_linear_cost(solver_.get());
        return tiny_solve(solver_.get()) == 0;
    }

    bool finite_input() const {
        return solver_->solution->u.allFinite();
    }
    double acceleration(int time) const {
        return solver_->solution->u(0, time);
    }

    Eigen::Matrix<double, 2, HORIZON> rollout() const {
        // ADMM 内部输入在收敛前可能超限；只使用投影后的输入重建可发送轨迹。
        Eigen::Matrix<double, 2, HORIZON> state = solver_->work->x;
        for (int i = 1; i < HORIZON; ++i) {
            state.col(i) = solver_->work->Adyn * state.col(i - 1) +
                           solver_->work->Bdyn * solver_->solution->u.col(i - 1);
        }
        return state;
    }

  private:
    std::unique_ptr<TinySolver, SolverDeleter> solver_;
};

Planner::Planner(const std::string& config_path) : Planner(load_planner_config(config_path)) {}

Planner::Planner(const PlannerConfig& config)
    : max_target_age_(config.max_target_age), feedback_timeout_(config.feedback_timeout),
      yaw_offset_(config.yaw_offset), pitch_offset_(config.pitch_offset),
      fire_thresh_(config.fire_thresh), low_speed_delay_time_(config.low_speed_delay_time),
      high_speed_delay_time_(config.high_speed_delay_time), decision_speed_(config.decision_speed) {
    validate(config);
    yaw_solver_ = std::make_unique<AxisSolver>(config.yaw);
    pitch_solver_ = std::make_unique<AxisSolver>(config.pitch);
}

Planner::~Planner() = default;

Plan Planner::plan(const std::optional<TargetSnapshot>& target, double bullet_speed,
                   TargetClock::time_point now, std::optional<io::GimbalState> feedback) {
    debug_xyza.setZero();
    if (!target)
        return stopped(StopReason::no_target);
    if (!target->valid())
        return stopped(StopReason::invalid_target);
    const double age = target_time::seconds_between(now, target->timestamp());
    const double observation_age =
        target_time::seconds_between(now, target->observation_timestamp());
    if (age < 0)
        return stopped(StopReason::future_state);
    if (age > max_target_age_)
        return stopped(StopReason::expired_state);
    if (observation_age < 0)
        return stopped(StopReason::future_observation);
    if (observation_age > max_target_age_)
        return stopped(StopReason::expired_observation);
    if (!std::isfinite(bullet_speed))
        return stopped(StopReason::invalid_bullet_speed);
    if (bullet_speed < 10 || bullet_speed > 25)
        bullet_speed = 22;
    const double delay = std::abs(target->state[target_state::YawRate]) > decision_speed_
                             ? high_speed_delay_time_
                             : low_speed_delay_time_;
    // 规划只外推固定维度运动学状态，不推进 EKF 协方差或观测时间。
    try {
        return solve(target->predicted(age + delay), bullet_speed, now, feedback);
    } catch (const std::invalid_argument&) {
        // 极端录包时间若无法容纳预测时域，应停控而不是使整个控制线程退出。
        return stopped(StopReason::invalid_target);
    }
}

Plan Planner::solve(const TargetSnapshot& target, double bullet_speed, TargetClock::time_point now,
                    const std::optional<io::GimbalState>& feedback) {
    double yaw0;
    Trajectory traj;
    bool fire_visible = false;
    try {
        const auto interception = aim(target, bullet_speed);
        yaw0 = interception.angles(0);
        debug_xyza = interception.armor;
        traj = get_trajectory(target, yaw0, bullet_speed, fire_visible);
    } catch (const InterceptionFailure& failure) {
        return stopped(failure.reason);
    }
    if (!traj.allFinite() || !std::isfinite(yaw0))
        return stopped(StopReason::invalid_solution);

    // 两轴都执行一次，以保留原有热启动和恢复行为；不能用短路表达式跳过 pitch。
    const bool yaw_converged = yaw_solver_->solve(traj.topRows<2>());
    const bool pitch_converged = pitch_solver_->solve(traj.bottomRows<2>());
    if (!yaw_converged || !pitch_converged)
        return stopped(StopReason::solver_not_converged);
    if (!yaw_solver_->finite_input() || !pitch_solver_->finite_input()) {
        return stopped(StopReason::invalid_solution);
    }
    const auto yaw_x = yaw_solver_->rollout();
    const auto pitch_x = pitch_solver_->rollout();
    if (!yaw_x.allFinite() || !pitch_x.allFinite())
        return stopped(StopReason::invalid_solution);

    Plan plan;
    plan.control = true;
    plan.reason = StopReason::none;
    plan.target_yaw = tools::limit_rad(traj(0, HALF_HORIZON) + yaw0);
    plan.target_pitch = traj(2, HALF_HORIZON);
    plan.yaw = tools::limit_rad(yaw_x(0, HALF_HORIZON) + yaw0);
    plan.yaw_vel = yaw_x(1, HALF_HORIZON);
    plan.yaw_acc = yaw_solver_->acceleration(HALF_HORIZON);
    plan.pitch = pitch_x(0, HALF_HORIZON);
    plan.pitch_vel = pitch_x(1, HALF_HORIZON);
    plan.pitch_acc = pitch_solver_->acceleration(HALF_HORIZON);

    const bool aligned = feedback && feedback->received_at <= now &&
                         target_time::seconds_between(now, feedback->received_at) <=
                             std::chrono::duration<double>(feedback_timeout_).count() &&
                         std::isfinite(feedback->yaw) && std::isfinite(feedback->pitch) &&
                         std::hypot(tools::limit_rad(plan.target_yaw - feedback->yaw),
                                    plan.target_pitch - feedback->pitch) < fire_thresh_;
    plan.fire =
        aligned && target.observed && fire_visible &&
        std::hypot(traj(0, HALF_HORIZON + SHOOT_OFFSET) - yaw_x(0, HALF_HORIZON + SHOOT_OFFSET),
                   traj(2, HALF_HORIZON + SHOOT_OFFSET) - pitch_x(0, HALF_HORIZON + SHOOT_OFFSET)) <
            fire_thresh_;
    return plan;
}

Planner::Interception Planner::aim(const TargetSnapshot& target, double bullet_speed,
                                   std::optional<std::size_t> armor_id, bool require_visible) {
    // 对每块物理装甲分别求拦截根；异高装甲在求根中切换会造成飞行时间不连续。
    const auto armors = target.armor_xyza_list();
    double best_distance = std::numeric_limits<double>::infinity();
    bool best_facing = false;
    Interception interception;
    bool converged_trajectory = false;
    for (std::size_t id = 0; id < armors.size(); ++id) {
        if (armor_id && id != *armor_id)
            continue;
        Eigen::Vector4d armor;
        auto evaluate = [&](double time) {
            armor = target.armor_xyza_list(time)[id];
            if (!armor.allFinite())
                return tools::Trajectory(0, 0, 0);
            return tools::Trajectory(bullet_speed, armor.head<2>().norm(), armor.z());
        };
        const auto initial = evaluate(0);
        if (initial.unsolvable)
            continue;
        double lower = 0;
        double upper = initial.fly_time;
        auto trajectory = evaluate(upper);
        for (int i = 0; !trajectory.unsolvable && trajectory.fly_time > upper && i < 30; ++i) {
            upper *= 1.5;
            trajectory = evaluate(upper);
        }
        if (trajectory.unsolvable || trajectory.fly_time > upper)
            continue;
        bool converged = false;
        for (int i = 0; i < 50; ++i) {
            const double time = (lower + upper) * 0.5;
            trajectory = evaluate(time);
            if (trajectory.unsolvable)
                break;
            if (std::abs(trajectory.fly_time - time) < 1e-7) {
                converged = true;
                break;
            }
            if (trajectory.fly_time > time)
                lower = time;
            else
                upper = time;
        }
        if (!converged)
            continue;
        converged_trajectory = true;
        const bool facing = facing_gimbal(armor);
        if (!armor_id) {
            if ((require_visible && !facing) || (best_facing && !facing))
                continue;
            if (facing == best_facing && armor.head<2>().norm() >= best_distance)
                continue;
        } else if (armor.head<2>().norm() >= best_distance)
            continue;
        best_distance = armor.head<2>().norm();
        best_facing = facing;
        interception.armor = armor;
        interception.armor_id = id;
        interception.angles << tools::limit_rad(std::atan2(armor.y(), armor.x()) + yaw_offset_),
            -trajectory.pitch - pitch_offset_;
    }
    if (!std::isfinite(best_distance)) {
        throw InterceptionFailure{converged_trajectory ? StopReason::no_visible_armor
                                                       : StopReason::ballistic_unreachable};
    }
    return interception;
}

Trajectory Planner::get_trajectory(const TargetSnapshot& target, double yaw0, double bullet_speed,
                                   bool& fire_visible) {
    Trajectory traj;
    auto previous = target.predicted(-DT * (HALF_HORIZON + 1));
    auto current = previous.predicted(DT);
    // 两装甲目标在时域尾部可能全部背向云台。尾部仍参与平滑，实际拦截与
    // 发弹采样必须可见；存在可见装甲时始终优先它，保持中央参考点一致。
    auto before = aim(previous, bullet_speed, std::nullopt, false);
    auto interception = aim(current, bullet_speed, std::nullopt, false);
    double last_yaw = yaw0;
    double relative_yaw = 0;
    for (int i = 0; i < HORIZON; ++i) {
        auto next = current.predicted(DT);
        const auto after = aim(next, bullet_speed, std::nullopt, false);
        // 角速度使用同一块装甲的前后位置，避免把换板误认为瞬时运动。
        // 选中的装甲未变时复用邻近拦截根。
        const auto before_angles = before.armor_id == interception.armor_id
                                       ? before.angles
                                       : aim(previous, bullet_speed, interception.armor_id).angles;
        const auto after_angles = after.armor_id == interception.armor_id
                                      ? after.angles
                                      : aim(next, bullet_speed, interception.armor_id).angles;
        const auto& angles = interception.angles;
        relative_yaw += tools::limit_rad(angles.x() - last_yaw);
        traj.col(i) << relative_yaw,
            tools::limit_rad(after_angles.x() - before_angles.x()) / (2 * DT), angles.y(),
            (after_angles.y() - before_angles.y()) / (2 * DT);
        if (i == HALF_HORIZON + SHOOT_OFFSET)
            fire_visible = facing_gimbal(interception.armor);
        last_yaw = angles.x();
        previous = std::move(current);
        current = std::move(next);
        before = interception;
        interception = after;
    }
    return traj;
}

} // namespace app::auto_aim
