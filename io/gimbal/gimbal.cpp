#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

namespace io {

static constexpr const char* MODULE = "GIMBAL";

Gimbal::Gimbal(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    auto com_port = std::string{config["gimbal"]["com_port"].value_or("/dev/ttyUSB0")};
    auto baudrate = config["gimbal"]["baudrate"].value_or(115200);

    const int timeout_ms = config["gimbal"]["feedback_timeout_ms"].value_or(100);
    if (timeout_ms <= 0) throw std::runtime_error("gimbal.feedback_timeout_ms must be positive");
    feedback_timeout_ = std::chrono::milliseconds(timeout_ms);
    fire_thresh_ = config["planner"]["fire_thresh"].value_or(0.05);
    if (!std::isfinite(fire_thresh_) || fire_thresh_ < 0) {
        throw std::runtime_error("planner.fire_thresh must be finite and nonnegative");
    }

    if (!serial_.open(com_port, baudrate)) {
        LOG_ERROR(MODULE, "[Gimbal] Failed to open serial: {}", com_port);
        throw std::runtime_error("failed to open gimbal serial: " + com_port);
    }

    thread_ = std::thread(&Gimbal::read_thread, this);

    std::unique_lock<std::mutex> lock(mutex_);
    if (!first_packet_cv_.wait_for(lock, std::chrono::seconds(3), [this] {
            return first_packet_received_;
        })) {
        lock.unlock();
        quit_ = true;
        serial_.close();
        pose_cv_.notify_all();
        if (thread_.joinable()) thread_.join();
        LOG_ERROR(MODULE, "[Gimbal] Timed out waiting for the first valid packet");
        throw std::runtime_error("timed out waiting for the first valid gimbal packet");
    }
    LOG_INFO(MODULE, "[Gimbal] First q received.");
}

Gimbal::~Gimbal() {
    send(false, false, 0, 0, 0, 0, 0, 0);
    quit_ = true;
    serial_.close();
    pose_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
}

GimbalMode Gimbal::mode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return feedback_fresh_locked() ? mode_ : GimbalMode::IDLE;
}

GimbalState Gimbal::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

bool Gimbal::feedback_fresh_locked() const {
    return first_packet_received_ && serial_.is_open() &&
           std::chrono::steady_clock::now() - last_packet_ <= feedback_timeout_;
}

uint64_t Gimbal::session() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_;
}

std::chrono::steady_clock::time_point Gimbal::session_started() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return session_started_;
}

std::string Gimbal::str(GimbalMode m) const {
    switch (m) {
        case GimbalMode::IDLE: return "IDLE";
        case GimbalMode::AUTO_AIM: return "AUTO_AIM";
        case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
        case GimbalMode::BIG_BUFF: return "BIG_BUFF";
        default: return "INVALID";
    }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t) {
    std::unique_lock<std::mutex> lock(mutex_);
    // Keep history: consuming samples here used to lose bracketing poses when
    // several camera frames shared the same IMU interval.
    if (!pose_cv_.wait_for(lock, feedback_timeout_, [&] {
            return quit_ || !serial_.is_open() ||
                   (!poses_.empty() && std::get<1>(poses_.back()) >= t);
        }) || quit_ || !feedback_fresh_locked() || poses_.empty()) {
        throw std::runtime_error("gimbal pose unavailable or timed out");
    }
    if (t < std::get<1>(poses_.front())) {
        throw std::runtime_error("camera timestamp predates gimbal pose history");
    }
    auto upper = std::lower_bound(poses_.begin(), poses_.end(), t,
        [](const auto& pose, auto stamp) { return std::get<1>(pose) < stamp; });
    if (upper == poses_.begin() || std::get<1>(*upper) == t) return std::get<0>(*upper);
    const auto& [q_b, t_b] = *upper;
    const auto& [q_a, t_a] = *std::prev(upper);
    const double interval = std::chrono::duration<double>(t_b - t_a).count();
    if (t_b - t_a > feedback_timeout_) {
        throw std::runtime_error("gimbal pose interval crosses a feedback outage");
    }
    const double ratio = std::chrono::duration<double>(t - t_a).count() / interval;
    return q_a.slerp(ratio, q_b).normalized();
}

void Gimbal::send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc, std::optional<uint64_t> expected_session) {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool finite = std::isfinite(yaw) && std::isfinite(yaw_vel) && std::isfinite(yaw_acc) &&
                        std::isfinite(pitch) && std::isfinite(pitch_vel) && std::isfinite(pitch_acc);
    control = control && finite && feedback_fresh_locked() && mode_ == GimbalMode::AUTO_AIM &&
              (!expected_session || *expected_session == session_);
    VisionToGimbal packet{};
    // Recheck actual alignment under the same lock as the latest feedback.
    // Planner also checks the interception reference; neither check alone permits fire.
    const double yaw_error = std::remainder(static_cast<double>(yaw) - state_.yaw, 2 * std::acos(-1.0));
    fire = fire && std::hypot(yaw_error, static_cast<double>(pitch) - state_.pitch) < fire_thresh_;
    packet.mode = control ? (fire ? 2 : 1) : 0;
    if (control) {
        packet.yaw = yaw;
        packet.yaw_vel = yaw_vel;
        packet.yaw_acc = yaw_acc;
        packet.pitch = pitch;
        packet.pitch_vel = pitch_vel;
        packet.pitch_acc = pitch_acc;
    }
    packet.crc16 = tools::get_crc16(
        reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - sizeof(packet.crc16));
    serial_.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void Gimbal::read_thread() {
    LOG_INFO(MODULE, "[Gimbal] read_thread started.");

    serial_.recv<GimbalToVision>([this](const GimbalToVision& pkt) {
        auto t = std::chrono::steady_clock::now();
        Eigen::Quaterniond q(pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]);
        if (!std::isfinite(q.norm()) || q.norm() < 1e-6) {
            LOG_WARN(MODULE, "[Gimbal] Ignoring invalid quaternion");
            return;
        }
        if (!std::isfinite(pkt.yaw) || !std::isfinite(pkt.pitch) ||
            !std::isfinite(pkt.yaw_vel) || !std::isfinite(pkt.pitch_vel) ||
            !std::isfinite(pkt.bullet_speed)) return;
        q.normalize();
        std::lock_guard<std::mutex> lock(mutex_);
        poses_.push_back({q, t});
        while (poses_.size() > 1000) poses_.pop_front();
        pose_cv_.notify_all();
        const auto previous_mode = mode_;
        const bool resumed = !first_packet_received_ || t - last_packet_ > feedback_timeout_;
        last_packet_ = t;
        state_.received_at = t;
        state_.yaw = pkt.yaw;
        state_.yaw_vel = pkt.yaw_vel;
        state_.pitch = pkt.pitch;
        state_.pitch_vel = pkt.pitch_vel;
        state_.bullet_speed = pkt.bullet_speed;
        state_.bullet_count = pkt.bullet_count;
        first_packet_received_ = true;
        first_packet_cv_.notify_all();

        switch (pkt.mode) {
            case 0: mode_ = GimbalMode::IDLE; break;
            case 1: mode_ = GimbalMode::AUTO_AIM; break;
            case 2: mode_ = GimbalMode::SMALL_BUFF; break;
            case 3: mode_ = GimbalMode::BIG_BUFF; break;
            default: mode_ = GimbalMode::IDLE; break;
        }
        if (resumed || previous_mode != mode_) {
            ++session_;
            session_started_ = t;
        }
    }, [](const GimbalToVision& pkt) {
        return tools::check_crc16(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    });

    while (!quit_ && serial_.is_open()) {
        serial_.spin_once();
    }

    pose_cv_.notify_all();
    LOG_INFO(MODULE, "[Gimbal] read_thread stopped.");
}

}  // namespace io
