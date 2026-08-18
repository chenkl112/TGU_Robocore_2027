#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"

namespace io {

static constexpr const char* MODULE = "GIMBAL";

Gimbal::Gimbal(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    auto com_port = std::string{config["gimbal"]["com_port"].value_or("/dev/ttyUSB0")};
    auto baudrate = config["gimbal"]["baudrate"].value_or(115200);

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
        if (thread_.joinable()) thread_.join();
        LOG_ERROR(MODULE, "[Gimbal] Timed out waiting for the first valid packet");
        throw std::runtime_error("timed out waiting for the first valid gimbal packet");
    }
    LOG_INFO(MODULE, "[Gimbal] First q received.");
}

Gimbal::~Gimbal() {
    quit_ = true;
    serial_.close();
    if (thread_.joinable()) thread_.join();
}

GimbalMode Gimbal::mode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return mode_;
}

GimbalState Gimbal::state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
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
    while (true) {
        auto [q_a, t_a] = queue_.pop();
        auto [q_b, t_b] = queue_.front();
        if (t <= t_a) return q_a;
        if (t > t_b) continue;

        const double interval = tools::delta_time(t_b, t_a);
        const double elapsed = tools::delta_time(t, t_a);
        const double ratio = interval > 0.0 ? std::clamp(elapsed / interval, 0.0, 1.0) : 0.0;
        return q_a.slerp(ratio, q_b).normalized();
    }
}

void Gimbal::send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc) {
    tx_data_.mode = control ? (fire ? 2 : 1) : 0;
    tx_data_.yaw = yaw;
    tx_data_.yaw_vel = yaw_vel;
    tx_data_.yaw_acc = yaw_acc;
    tx_data_.pitch = pitch;
    tx_data_.pitch_vel = pitch_vel;
    tx_data_.pitch_acc = pitch_acc;
    tx_data_.crc16 = tools::get_crc16(
        reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

    serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_));
}

void Gimbal::read_thread() {
    LOG_INFO(MODULE, "[Gimbal] read_thread started.");

    serial_.recv<GimbalToVision>([this](const GimbalToVision& pkt) {
        if (!tools::check_crc16(
                reinterpret_cast<const uint8_t*>(&pkt), sizeof(GimbalToVision))) {
            return;
        }

        auto t = std::chrono::steady_clock::now();
        Eigen::Quaterniond q(pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]);
        if (!std::isfinite(q.norm()) || q.norm() < 1e-6) {
            LOG_WARN(MODULE, "[Gimbal] Ignoring invalid quaternion");
            return;
        }
        q.normalize();
        queue_.push({q, t});

        std::lock_guard<std::mutex> lock(mutex_);
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
    });

    while (!quit_) {
        serial_.spin_once();
    }

    LOG_INFO(MODULE, "[Gimbal] read_thread stopped.");
}

}  // namespace io
