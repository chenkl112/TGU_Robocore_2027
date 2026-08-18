#ifndef TGU_ROBOCORE_2027_GIMBAL_HPP
#define TGU_ROBOCORE_2027_GIMBAL_HPP
#pragma once

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "command.hpp"
#include "serial/serial.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io {

class Gimbal {
public:
    explicit Gimbal(const std::string& config_path);
    ~Gimbal();

    GimbalMode mode() const;
    GimbalState state() const;
    std::string str(GimbalMode m) const;
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

    void send(
        bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch,
        float pitch_vel, float pitch_acc);

private:
    Serial serial_;

    std::thread thread_;
    std::atomic<bool> quit_{false};
    mutable std::mutex mutex_;
    std::condition_variable first_packet_cv_;
    bool first_packet_received_ = false;

    VisionToGimbal tx_data_{};

    GimbalMode mode_ = GimbalMode::IDLE;
    GimbalState state_;
    tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
        queue_{1000};

    void read_thread();
};

}  // namespace io

#endif  // TGU_ROBOCORE_2027_GIMBAL_HPP
