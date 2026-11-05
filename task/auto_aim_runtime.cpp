#include "auto_aim_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <exception>

#include "tools/logger.hpp"
#include "tools/periodic_loop.hpp"

namespace task {
namespace {
using Clock = app::auto_aim::TargetClock;
using namespace std::chrono_literals;
constexpr const char* MODULE = "AUTO_AIM_RUNTIME";
} // namespace

AutoAimRuntime::AutoAimRuntime(const app::auto_aim::AutoAimConfig& config)
    : config_(config), gimbal_(config.gimbal), camera_(config.camera),
      detector_(config.yolo, config.classifier), tracking_(config), planner_(config.planner) {}

AutoAimRuntime::~AutoAimRuntime() {
    stop();
    join();
}

void AutoAimRuntime::stop() {
    stopping_.request_stop();
    camera_.stop();
    detector_.stop();
}

void AutoAimRuntime::join() {
    if (control_thread_.joinable())
        control_thread_.join();
    if (capture_thread_.joinable())
        capture_thread_.join();
    if (tracking_thread_.joinable())
        tracking_thread_.join();
}

void AutoAimRuntime::record_failure(const char* worker, std::exception_ptr error) {
    failed_ = true;
    stop();
    try {
        std::rethrow_exception(error);
    } catch (const std::exception& failure) {
        LOG_ERROR(MODULE, "{} failed: {}", worker, failure.what());
    } catch (...) {
        LOG_ERROR(MODULE, "{} failed with an unknown exception", worker);
    }
}

int AutoAimRuntime::run(const std::atomic<bool>& interrupted) {
    interrupted_ = &interrupted;
    auto launch = [this](const char* name, auto method) {
        return std::jthread([this, name, method] {
            try {
                (this->*method)();
            } catch (...) {
                record_failure(name, std::current_exception());
            }
        });
    };
    capture_thread_ = launch("capture", &AutoAimRuntime::capture_loop);
    tracking_thread_ = launch("tracking", &AutoAimRuntime::tracking_loop);
    control_thread_ = launch("control", &AutoAimRuntime::control_loop);

    const auto token = stopping_.get_token();
    auto previous_mode = io::GimbalMode::IDLE;
    while (!token.stop_requested() && !interrupted.load(std::memory_order_relaxed)) {
        const auto feedback = gimbal_.snapshot();
        if (feedback.mode != previous_mode) {
            LOG_INFO(MODULE, "Switch to {}", gimbal_.str(feedback.mode));
            previous_mode = feedback.mode;
        }
        tools::wait_until(token, Clock::now() + 10ms);
    }
    stop();
    join();
    gimbal_.send(io::Command{});
    return failed_ ? 1 : 0;
}

void AutoAimRuntime::capture_loop() {
    const auto token = stopping_.get_token();
    while (!token.stop_requested()) {
        const auto feedback = gimbal_.snapshot();
        if (feedback.mode != io::GimbalMode::AUTO_AIM) {
            tools::wait_until(token, Clock::now() + 10ms);
            continue;
        }
        cv::Mat image;
        Clock::time_point timestamp;
        camera_.read(image, timestamp);
        if (!image.empty() && !detector_.push(std::move(image), timestamp, feedback.session))
            break;
    }
}

void AutoAimRuntime::tracking_loop() {
    const auto token = stopping_.get_token();
    std::uint64_t session = 0;
    while (auto frame = detector_.wait_pop(token)) {
        const auto feedback = gimbal_.snapshot();
        if (session != feedback.session) {
            tracking_.reset();
            session = feedback.session;
        }
        if (feedback.mode != io::GimbalMode::AUTO_AIM || frame->session != session ||
            frame->timestamp < feedback.session_started) {
            latest_.publish({});
            continue;
        }
        Eigen::Quaterniond orientation;
        try {
            orientation = gimbal_.q(frame->timestamp, token);
        } catch (const std::runtime_error&) {
            tracking_.reset();
            latest_.publish({});
            continue;
        }
        latest_.publish({tracking_.process(frame->armors, orientation, frame->timestamp), session});
    }
}

void AutoAimRuntime::control_loop() {
    const auto token = stopping_.get_token();
    const auto period = std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(app::auto_aim::DT));
    tools::PeriodicSchedule schedule(period);
    std::uint64_t cycles = 0;
    double max_compute_ms = 0, max_interval_ms = 0, max_age_ms = 0;
    std::optional<Clock::time_point> previous_send;
    auto last_reason = app::auto_aim::StopReason::no_target;
    while (!token.stop_requested()) {
        const auto started = Clock::now();
        const auto update = latest_.read();
        const auto feedback = gimbal_.snapshot();
        // 快照可能刚被生产者刷新；判龄基准必须在读取快照之后取得。
        const auto planning_now = Clock::now();
        app::auto_aim::Plan plan;
        if (feedback.mode == io::GimbalMode::AUTO_AIM && update.session == feedback.session) {
            plan = planner_.plan(update.target, feedback.state.bullet_speed, planning_now,
                                 feedback.state);
        }

        io::Command command;
        command.control = plan.control;
        command.shoot = config_.planner.auto_fire && plan.fire;
        command.yaw = plan.yaw;
        command.yaw_vel = plan.yaw_vel;
        command.yaw_acc = plan.yaw_acc;
        command.pitch = plan.pitch;
        command.pitch_vel = plan.pitch_vel;
        command.pitch_acc = plan.pitch_acc;
        if (update.target) {
            try {
                command.valid_until = app::auto_aim::target_time::add_seconds(
                    update.target->observation_time, config_.planner.max_target_age);
            } catch (const std::invalid_argument&) {
                command = {};
            }
            max_age_ms = std::max(max_age_ms, std::chrono::duration<double, std::milli>(
                                                  planning_now - update.target->observation_time)
                                                  .count());
        }
        const auto computed = Clock::now();
        max_compute_ms = std::max(
            max_compute_ms, std::chrono::duration<double, std::milli>(computed - started).count());
        if (token.stop_requested() || interrupted_->load(std::memory_order_relaxed))
            break;
        gimbal_.send(command, update.session);
        const auto sent = Clock::now();
        if (previous_send) {
            max_interval_ms =
                std::max(max_interval_ms,
                         std::chrono::duration<double, std::milli>(sent - *previous_send).count());
        }
        previous_send = sent;
        ++cycles;
        if (plan.reason != last_reason) {
            LOG_DEBUG(MODULE, "Planning status: {}", app::auto_aim::stop_reason_name(plan.reason));
            last_reason = plan.reason;
        }
        tools::wait_until(token, schedule.next(Clock::now()));
    }
    LOG_INFO(MODULE,
             "Control cycles={} missed={} max_compute_ms={:.3f} max_send_interval_ms={:.3f} "
             "max_target_age_ms={:.3f}",
             cycles, schedule.missed(), max_compute_ms, max_interval_ms, max_age_ms);
}

} // namespace task
