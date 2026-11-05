#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <variant>

namespace io {

// 设备配置只保存已解析的值；设备层无需知道应用配置文件的布局。
struct GimbalConfig {
    std::string port = "/dev/ttyUSB0";
    int baudrate = 115200;
    std::chrono::milliseconds feedback_timeout{100};
    std::chrono::milliseconds reconnect_delay{250};
    double fire_threshold = 0.05; // 云台实际角度与指令角度的最大误差，单位 rad。
};

inline void validate(const GimbalConfig& config) {
    if (config.port.empty())
        throw std::runtime_error("gimbal.com_port must not be empty");
    if (config.baudrate <= 0)
        throw std::runtime_error("gimbal.baudrate must be positive");
    if (config.feedback_timeout.count() <= 0) {
        throw std::runtime_error("gimbal.feedback_timeout_ms must be positive");
    }
    // 与 TOML 毫秒字段的 int32 范围一致，避免转为单调时钟的纳秒时发生整数溢出。
    if (config.feedback_timeout.count() > std::numeric_limits<std::int32_t>::max()) {
        throw std::runtime_error(
            "gimbal.feedback_timeout_ms exceeds the supported millisecond range");
    }
    if (config.reconnect_delay.count() <= 0) {
        throw std::runtime_error("gimbal.reconnect_delay_ms must be positive");
    }
    if (config.reconnect_delay.count() > std::numeric_limits<std::int32_t>::max()) {
        throw std::runtime_error(
            "gimbal.reconnect_delay_ms exceeds the supported millisecond range");
    }
    if (!std::isfinite(config.fire_threshold) || config.fire_threshold < 0) {
        throw std::runtime_error("planner.fire_thresh must be finite and nonnegative");
    }
}

struct CameraConfig {
    std::variant<int, std::string> source{0};
    int width = 0;
    int height = 0;
    double fps = 200.0;
};

inline void validate(const CameraConfig& config) {
    if (const auto index = std::get_if<int>(&config.source); index && *index < 0) {
        throw std::invalid_argument("camera.source: camera index must be nonnegative");
    }
    if (const auto path = std::get_if<std::string>(&config.source); path && path->empty()) {
        throw std::invalid_argument("camera.source: must not be empty");
    }
    if (config.width < 0)
        throw std::invalid_argument("camera.width must be nonnegative");
    if (config.height < 0)
        throw std::invalid_argument("camera.height must be nonnegative");
    if (!std::isfinite(config.fps))
        throw std::invalid_argument("camera.fps must be finite");
}

} // namespace io
