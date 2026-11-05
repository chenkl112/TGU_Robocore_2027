#ifndef TGU_ROBOCORE_2027_COMMAND_HPP
#define TGU_ROBOCORE_2027_COMMAND_HPP
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>

namespace io {

struct Command {
    // 角度为 rad，角速度为 rad/s，角加速度为 rad/s²；horizon_distance 为 m。
    // control=false 时其余字段仅供诊断，发送边界会生成全零停控包。
    bool control = false;
    bool shoot = false;
    double yaw = 0.0;
    double yaw_vel = 0.0;
    double yaw_acc = 0.0;
    double pitch = 0.0;
    double pitch_vel = 0.0;
    double pitch_acc = 0.0;
    double horizon_distance = 0.0;
    // 本机单调时钟的最后有效时刻，不在线上传输；排队期间过期的控制命令必须停控。
    std::optional<std::chrono::steady_clock::time_point> valid_until = std::nullopt;
};

struct __attribute__((packed)) GimbalToVision {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode;
    float q[4];
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count;
    uint16_t crc16;
};

struct __attribute__((packed)) VisionToGimbal {
    uint8_t head[2] = {'S', 'P'};
    uint8_t mode;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
    uint16_t crc16;
};

// 协议与下位机共用固定字节布局；新增诊断字段应放在非线上结构中。
static_assert(sizeof(GimbalToVision) == 43);
static_assert(sizeof(VisionToGimbal) == 29);

enum class GimbalMode { IDLE, AUTO_AIM, SMALL_BUFF, BIG_BUFF };

struct GimbalState {
    // 本机单调时钟的接收时间，不属于串口线上协议。
    std::chrono::steady_clock::time_point received_at{};
    float yaw = 0;
    float yaw_vel = 0;
    float pitch = 0;
    float pitch_vel = 0;
    float bullet_speed = 0;
    uint16_t bullet_count = 0;
};

// 一次加锁读取同一时刻的反馈与会话，避免分别查询时混用两个控制周期。
// 失联时保留最后一帧数值供诊断，但 mode 降为 IDLE，feedback_fresh 为 false。
struct GimbalSnapshot {
    GimbalState state;
    GimbalMode mode = GimbalMode::IDLE;
    uint64_t session = 0;
    std::chrono::steady_clock::time_point session_started{};
    bool feedback_fresh = false;
};

} // namespace io

#endif // TGU_ROBOCORE_2027_COMMAND_HPP
