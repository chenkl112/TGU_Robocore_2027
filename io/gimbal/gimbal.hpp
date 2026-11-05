#ifndef TGU_ROBOCORE_2027_GIMBAL_HPP
#define TGU_ROBOCORE_2027_GIMBAL_HPP
#pragma once

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <tuple>

#include "command.hpp"
#include "device_config.hpp"
#include "serial/serial.hpp"

namespace io {

class Gimbal {
  public:
    explicit Gimbal(const GimbalConfig& config);
    explicit Gimbal(const std::string& config_path);
    ~Gimbal();

    GimbalSnapshot snapshot() const;
    GimbalMode mode() const;
    GimbalState state() const;
    uint64_t session() const;
    std::chrono::steady_clock::time_point session_started() const;
    std::string str(GimbalMode m) const;
    // 查询等待可由调用方取消；运行时退出无需等待配置的完整反馈超时时间。
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t, std::stop_token stop = {});

    // 指令绑定调用时的会话；发送排队期间退出自瞄、失联或恢复后，旧指令不能复活。
    void send(const Command& command, std::optional<uint64_t> expected_session = std::nullopt);

    void send(bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
              float pitch_vel, float pitch_acc,
              std::optional<uint64_t> expected_session = std::nullopt);

  private:
    Serial serial_;

    std::string com_port_;
    int baudrate_ = 115200;

    std::thread thread_;
    std::atomic<bool> quit_{false};
    mutable std::mutex mutex_;
    std::condition_variable first_packet_cv_;
    std::condition_variable reconnect_cv_;
    bool first_packet_received_ = false;

    double fire_thresh_ = 0.05;
    std::chrono::milliseconds feedback_timeout_{100};
    std::chrono::milliseconds reconnect_delay_{250};
    std::chrono::steady_clock::time_point last_packet_{};
    std::chrono::steady_clock::time_point session_started_{};
    uint64_t session_ = 0;
    bool feedback_fresh_locked() const;
    VisionToGimbal packet_locked(const Command& command, uint64_t expected_session) const;

    GimbalMode mode_ = GimbalMode::IDLE;
    GimbalState state_;
    std::deque<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> poses_;
    std::condition_variable_any pose_cv_;

    void read_thread();
    void request_stop();
};

} // namespace io

#endif // TGU_ROBOCORE_2027_GIMBAL_HPP
