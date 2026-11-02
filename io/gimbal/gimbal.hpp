#ifndef TGU_ROBOCORE_2027_GIMBAL_HPP
#define TGU_ROBOCORE_2027_GIMBAL_HPP
#pragma once

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <optional>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "command.hpp"
#include "serial/serial.hpp"

namespace io {

class Gimbal {
public:
    explicit Gimbal(const std::string& config_path);
    ~Gimbal();

    GimbalMode mode() const;
    GimbalState state() const;
    uint64_t session() const;
    std::chrono::steady_clock::time_point session_started() const;
    std::string str(GimbalMode m) const;
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

    void send(
        bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
        float pitch_vel, float pitch_acc,
        std::optional<uint64_t> expected_session = std::nullopt);

private:
    Serial serial_;

    std::thread thread_;
    std::atomic<bool> quit_{false};
    mutable std::mutex mutex_;
    std::condition_variable first_packet_cv_;
    bool first_packet_received_ = false;

    double fire_thresh_ = 0.05;
    std::chrono::milliseconds feedback_timeout_{100};
    std::chrono::steady_clock::time_point last_packet_{};
    std::chrono::steady_clock::time_point session_started_{};
    uint64_t session_ = 0;
    bool feedback_fresh_locked() const;

    GimbalMode mode_ = GimbalMode::IDLE;
    GimbalState state_;
    std::deque<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> poses_;
    std::condition_variable pose_cv_;

    void read_thread();
};

}  // namespace io

#endif  // TGU_ROBOCORE_2027_GIMBAL_HPP
