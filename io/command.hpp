#ifndef TGU_ROBOCORE_2027_COMMAND_HPP
#define TGU_ROBOCORE_2027_COMMAND_HPP
#pragma once

#include <cstdint>

namespace io {

struct Command {
    bool control = false;
    bool shoot = false;
    double yaw = 0.0;
    double yaw_vel = 0.0;
    double yaw_acc = 0.0;
    double pitch = 0.0;
    double pitch_vel = 0.0;
    double pitch_acc = 0.0;
    double horizon_distance = 0.0;
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

enum class GimbalMode { IDLE, AUTO_AIM, SMALL_BUFF, BIG_BUFF };

struct GimbalState {
    float yaw = 0;
    float yaw_vel = 0;
    float pitch = 0;
    float pitch_vel = 0;
    float bullet_speed = 0;
    uint16_t bullet_count = 0;
};

}  // namespace io

#endif  // TGU_ROBOCORE_2027_COMMAND_HPP
