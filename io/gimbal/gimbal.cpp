#include "gimbal.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/tomlpp.hpp"

namespace io {

static constexpr const char* MODULE = "GIMBAL";

namespace {
GimbalConfig read_legacy_config(const std::string& config_path) {
    const auto config = toml::parse_file(config_path);
    GimbalConfig result;
    result.port = std::string{config["gimbal"]["com_port"].value_or("/dev/ttyUSB0")};
    result.baudrate = config["gimbal"]["baudrate"].value_or(115200);
    result.feedback_timeout =
        std::chrono::milliseconds(config["gimbal"]["feedback_timeout_ms"].value_or(100));
    result.reconnect_delay =
        std::chrono::milliseconds(config["gimbal"]["reconnect_delay_ms"].value_or(250));
    result.fire_threshold = config["planner"]["fire_thresh"].value_or(0.05);
    return result;
}
} // namespace

Gimbal::Gimbal(const std::string& config_path) : Gimbal(read_legacy_config(config_path)) {}

Gimbal::Gimbal(const GimbalConfig& config)
    : com_port_(config.port), baudrate_(config.baudrate), fire_thresh_(config.fire_threshold),
      feedback_timeout_(config.feedback_timeout), reconnect_delay_(config.reconnect_delay) {
    validate(config);

    if (!serial_.open(com_port_, baudrate_)) {
        LOG_ERROR(MODULE, "[Gimbal] Failed to open serial: {}", com_port_);
        throw std::runtime_error("failed to open gimbal serial: " + com_port_);
    }

    thread_ = std::thread(&Gimbal::read_thread, this);

    std::unique_lock<std::mutex> lock(mutex_);
    if (!first_packet_cv_.wait_for(lock, std::chrono::seconds(3),
                                   [this] { return first_packet_received_; })) {
        lock.unlock();
        request_stop();
        if (thread_.joinable())
            thread_.join();
        LOG_ERROR(MODULE, "[Gimbal] Timed out waiting for the first valid packet");
        throw std::runtime_error("timed out waiting for the first valid gimbal packet");
    }
    LOG_INFO(MODULE, "[Gimbal] First q received.");
}

Gimbal::~Gimbal() {
    send(false, false, 0, 0, 0, 0, 0, 0);
    request_stop();
    if (thread_.joinable())
        thread_.join();
}

void Gimbal::request_stop() {
    {
        // 与条件变量的谓词共用同一把锁，避免谓词检查和进入等待之间漏掉退出通知。
        std::lock_guard<std::mutex> lock(mutex_);
        quit_ = true;
    }
    serial_.close();
    pose_cv_.notify_all();
    reconnect_cv_.notify_all();
}

GimbalSnapshot Gimbal::snapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    const bool fresh = feedback_fresh_locked();
    return {state_, fresh ? mode_ : GimbalMode::IDLE, session_, session_started_, fresh};
}

GimbalMode Gimbal::mode() const {
    return snapshot().mode;
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
    case GimbalMode::IDLE:
        return "IDLE";
    case GimbalMode::AUTO_AIM:
        return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF:
        return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:
        return "BIG_BUFF";
    default:
        return "INVALID";
    }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t, std::stop_token stop) {
    std::unique_lock<std::mutex> lock(mutex_);
    // 保留完整历史：多个相机帧可能共享同一 IMU 插值区间，查询不能消费样本。
    if (!pose_cv_.wait_for(lock, stop, feedback_timeout_,
                           [&] {
                               return quit_ || !serial_.is_open() ||
                                      (!poses_.empty() && std::get<1>(poses_.back()) >= t);
                           }) ||
        quit_ || stop.stop_requested() || !feedback_fresh_locked() || poses_.empty()) {
        throw std::runtime_error("gimbal pose unavailable or timed out");
    }
    if (t < std::get<1>(poses_.front())) {
        throw std::runtime_error("camera timestamp predates gimbal pose history");
    }
    auto upper =
        std::lower_bound(poses_.begin(), poses_.end(), t,
                         [](const auto& pose, auto stamp) { return std::get<1>(pose) < stamp; });
    // 等待可能由断连唤醒，且串口可在随后的新鲜度检查前重开；仍须确认存在右侧样本。
    if (upper == poses_.end())
        throw std::runtime_error("gimbal pose has no upper timestamp bound");
    if (upper == poses_.begin() || std::get<1>(*upper) == t)
        return std::get<0>(*upper);
    const auto& [q_b, t_b] = *upper;
    const auto& [q_a, t_a] = *std::prev(upper);
    const double interval = std::chrono::duration<double>(t_b - t_a).count();
    if (t_b - t_a > feedback_timeout_) {
        throw std::runtime_error("gimbal pose interval crosses a feedback outage");
    }
    const double ratio = std::chrono::duration<double>(t - t_a).count() / interval;
    return q_a.slerp(ratio, q_b).normalized();
}

VisionToGimbal Gimbal::packet_locked(const Command& command, uint64_t expected_session) const {
    // 线上使用 float；有限的 double 也可能在窄化后溢出，需在边界再次检查。
    const auto fits_wire = [](double value) {
        return std::isfinite(value) && std::abs(value) <= std::numeric_limits<float>::max();
    };
    const bool finite = fits_wire(command.yaw) && fits_wire(command.yaw_vel) &&
                        fits_wire(command.yaw_acc) && fits_wire(command.pitch) &&
                        fits_wire(command.pitch_vel) && fits_wire(command.pitch_acc);
    const bool command_fresh =
        !command.valid_until || std::chrono::steady_clock::now() < *command.valid_until;
    const bool control = command.control && finite && command_fresh && feedback_fresh_locked() &&
                         mode_ == GimbalMode::AUTO_AIM && expected_session == session_;
    VisionToGimbal packet{};
    if (control) {
        packet.yaw = static_cast<float>(command.yaw);
        packet.yaw_vel = static_cast<float>(command.yaw_vel);
        packet.yaw_acc = static_cast<float>(command.yaw_acc);
        packet.pitch = static_cast<float>(command.pitch);
        packet.pitch_vel = static_cast<float>(command.pitch_vel);
        packet.pitch_acc = static_cast<float>(command.pitch_acc);
        // 使用实际线上 float 值校验，避免 double 窄化把门限内的角度舍入到门限外。
        // 规划器检查拦截参考，发送边界检查最新实际反馈；两者同时满足才允许开火。
        const double yaw_error =
            std::remainder(static_cast<double>(packet.yaw) - state_.yaw, 2 * std::acos(-1.0));
        const bool aligned =
            std::hypot(yaw_error, static_cast<double>(packet.pitch) - state_.pitch) < fire_thresh_;
        packet.mode = command.shoot && aligned ? 2 : 1;
    }
    packet.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t*>(&packet),
                                    sizeof(packet) - sizeof(packet.crc16));
    return packet;
}

void Gimbal::send(const Command& command, std::optional<uint64_t> expected_session) {
    // 即便旧调用方未传会话，也在排队前固定会话，防止等待中发生模式切换后重用旧命令。
    const uint64_t requested_session = expected_session.value_or(session());
    VisionToGimbal packet{};
    serial_.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet),
                  [&](size_t sent, const Serial::WriteAttempt& attempt) {
                      // 只在重新验证和一次非阻塞 write 期间持锁；背压等待不阻塞反馈线程。
                      // 锁顺序固定为 Gimbal 状态锁 → Serial 状态锁，接收回调不持后者。
                      std::lock_guard<std::mutex> lock(mutex_);
                      const auto current = packet_locked(command, requested_session);
                      if (sent == 0) {
                          packet = current;
                      } else if ((packet.mode != 0 && current.mode == 0) ||
                                 (packet.mode == 2 && current.mode != 2)) {
                          // 已写出的前缀无法撤回，也不能中途改包；撤销尾部并关闭该串口会话。
                          return false;
                      }
                      attempt();
                      return true;
                  });
}

void Gimbal::send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
                  float pitch_vel, float pitch_acc, std::optional<uint64_t> expected_session) {
    send(Command{.control = control,
                 .shoot = fire,
                 .yaw = yaw,
                 .yaw_vel = yaw_vel,
                 .yaw_acc = yaw_acc,
                 .pitch = pitch,
                 .pitch_vel = pitch_vel,
                 .pitch_acc = pitch_acc},
         expected_session);
}

void Gimbal::read_thread() {
    LOG_INFO(MODULE, "[Gimbal] read_thread started.");

    serial_.recv<GimbalToVision>(
        [this](const GimbalToVision& pkt) {
            auto t = std::chrono::steady_clock::now();
            Eigen::Quaterniond q(pkt.q[0], pkt.q[1], pkt.q[2], pkt.q[3]);
            if (!std::isfinite(q.norm()) || q.norm() < 1e-6) {
                LOG_WARN(MODULE, "[Gimbal] Ignoring invalid quaternion");
                return;
            }
            if (!std::isfinite(pkt.yaw) || !std::isfinite(pkt.pitch) ||
                !std::isfinite(pkt.yaw_vel) || !std::isfinite(pkt.pitch_vel) ||
                !std::isfinite(pkt.bullet_speed))
                return;
            q.normalize();
            std::lock_guard<std::mutex> lock(mutex_);
            poses_.push_back({q, t});
            while (poses_.size() > 1000)
                poses_.pop_front();
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
            case 0:
                mode_ = GimbalMode::IDLE;
                break;
            case 1:
                mode_ = GimbalMode::AUTO_AIM;
                break;
            case 2:
                mode_ = GimbalMode::SMALL_BUFF;
                break;
            case 3:
                mode_ = GimbalMode::BIG_BUFF;
                break;
            default:
                mode_ = GimbalMode::IDLE;
                break;
            }
            if (resumed || previous_mode != mode_) {
                ++session_;
                session_started_ = t;
            }
        },
        [](const GimbalToVision& pkt) {
            return tools::check_crc16(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
        });

    auto next_reconnect = std::chrono::steady_clock::now();
    while (!quit_) {
        if (!serial_.is_open()) {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                mode_ = GimbalMode::IDLE;
                // 重连退避必须能被退出打断，配置较长延迟也不能延长析构等待。
                if (reconnect_cv_.wait_until(lock, next_reconnect,
                                             [this] { return quit_.load(); })) {
                    break;
                }
            }
            if (quit_)
                break;
            if (serial_.open(com_port_, baudrate_)) {
                LOG_INFO(MODULE, "[Gimbal] Reconnected serial: {}", com_port_);
            } else {
                LOG_WARN(MODULE, "[Gimbal] Serial reconnect failed: {}", com_port_);
            }
            next_reconnect = std::chrono::steady_clock::now() + reconnect_delay_;
            continue;
        }
        serial_.spin_once();
        if (!serial_.is_open()) {
            next_reconnect = std::chrono::steady_clock::now() + reconnect_delay_;
        }
    }

    pose_cv_.notify_all();
    LOG_INFO(MODULE, "[Gimbal] read_thread stopped.");
}

} // namespace io
