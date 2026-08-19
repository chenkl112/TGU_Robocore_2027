#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <list>
#include <optional>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "app/auto_aim/aimer.hpp"
#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/shooter.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/tracker.hpp"
#include "app/auto_aim/yolo.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/tomlpp.hpp"

namespace {

constexpr const char* MODULE = "GAZEBO_AUTO_AIM";
constexpr std::uint32_t MAX_FRAME_BYTES = 16U * 1024U * 1024U;
std::atomic<bool> quit{false};

#pragma pack(push, 1)
struct FrameHeader {
    char magic[4];
    std::uint32_t payload_size;
    double stamp;
    float yaw;
    float pitch;
    float yaw_vel;
    float pitch_vel;
    float bullet_speed;
    std::uint32_t width;
    std::uint32_t height;
};
#pragma pack(pop)

static_assert(sizeof(FrameHeader) == 44);

void on_signal(int) { quit = true; }

bool read_exact(int socket_fd, void* output, std::size_t size) {
    auto* bytes = static_cast<std::uint8_t*>(output);
    std::size_t received = 0;
    while (received < size && !quit) {
        const auto result = ::recv(socket_fd, bytes + received, size - received, 0);
        if (result <= 0) return false;
        received += static_cast<std::size_t>(result);
    }
    return received == size;
}

bool send_all(int socket_fd, const std::string& message) {
    std::size_t sent = 0;
    while (sent < message.size() && !quit) {
        const auto result = ::send(
            socket_fd, message.data() + sent, message.size() - sent, MSG_NOSIGNAL);
        if (result <= 0) return false;
        sent += static_cast<std::size_t>(result);
    }
    return sent == message.size();
}

int connect_bridge(const std::string& host, int port) {
    const int socket_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (socket_fd < 0) return -1;

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    if (::inet_pton(AF_INET, host.c_str(), &address.sin_addr) != 1 ||
        ::connect(socket_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        ::close(socket_fd);
        return -1;
    }
    int no_delay = 1;
    ::setsockopt(socket_fd, IPPROTO_TCP, TCP_NODELAY, &no_delay, sizeof(no_delay));
    return socket_fd;
}

std::string upper_state(const std::string& state) {
    if (state == "tracking") return "TRACKING";
    if (state == "detecting") return "DETECTING";
    if (state == "temp_lost") return "TEMP_LOST";
    return "SEARCHING";
}

void append_number(std::ostringstream& json, double value) {
    if (std::isfinite(value)) {
        json << value;
    } else {
        json << "null";
    }
}

void append_point_list(
    std::ostringstream& json, const std::vector<cv::Point2f>& points) {
    json << '[';
    for (std::size_t index = 0; index < points.size(); ++index) {
        if (index != 0) json << ',';
        json << '[' << points[index].x << ',' << points[index].y << ']';
    }
    json << ']';
}

struct DebugDetection {
    app::auto_aim::Armor armor;
    bool pose_valid = false;
};

std::string build_status(
    const std::list<app::auto_aim::Armor>& tracked_armors,
    const std::vector<DebugDetection>& detections, const std::list<app::auto_aim::Target>& targets,
    const app::auto_aim::Plan& plan, bool mpc_control, bool gimbal_control,
    const io::Command& legacy_command, bool legacy_fire, bool fire, double command_yaw,
    double command_pitch,
    const std::string& tracker_state,
    const FrameHeader& frame, double inference_ms, std::uint64_t frame_count) {
    const std::string state = fire ? "FIRING" : (mpc_control ? "LOCKED" : upper_state(tracker_state));

    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{\"source\":\"robocore\",\"pipeline\":"
         << "\"YOLOV8/Solver/Tracker/EKF/Aimer/Shooter/Planner/TinyMPC\""
         << ",\"state\":\"" << state << "\""
         << ",\"detected\":" << (!tracked_armors.empty() ? "true" : "false")
         << ",\"locked\":" << (mpc_control ? "true" : "false")
         << ",\"fire\":" << (fire ? "true" : "false")
         << ",\"frame\":" << frame_count
         << ",\"image_size\":[" << frame.width << ',' << frame.height << ']'
         << ",\"network_input\":[640,480]"
         << ",\"inference_ms\":" << inference_ms
         << ",\"armor_count\":" << detections.size()
         << ",\"target_count\":" << targets.size()
         << ",\"tracker_state\":\"" << tracker_state << "\""
         << ",\"mpc_control\":" << (mpc_control ? "true" : "false")
         << ",\"gimbal_control\":" << (gimbal_control ? "true" : "false")
         << ",\"mpc_fire\":" << ((mpc_control && plan.fire) ? "true" : "false")
         << ",\"legacy_control\":" << (legacy_command.control ? "true" : "false")
         << ",\"legacy_fire\":" << (legacy_fire ? "true" : "false")
         << ",\"yaw\":" << command_yaw << ",\"pitch\":" << command_pitch
         << ",\"yaw_rate\":" << (mpc_control ? plan.yaw_vel : 0.0)
         << ",\"pitch_rate\":" << (mpc_control ? plan.pitch_vel : 0.0)
         << ",\"yaw_acc\":" << (mpc_control ? plan.yaw_acc : 0.0)
         << ",\"pitch_acc\":" << (mpc_control ? plan.pitch_acc : 0.0)
         << ",\"yaw_error\":" << tools::limit_rad(command_yaw - frame.yaw)
         << ",\"pitch_error\":" << command_pitch - frame.pitch;

    if (!tracked_armors.empty() && tracked_armors.front().points.size() == 4) {
        json << ",\"box\":";
        append_point_list(json, tracked_armors.front().points);
    } else {
        json << ",\"box\":null";
    }

    json << ",\"detections\":[";
    for (std::size_t index = 0; index < detections.size(); ++index) {
        if (index != 0) json << ',';
        const auto& detection = detections[index];
        const auto& armor = detection.armor;
        json << "{\"box\":";
        append_point_list(json, armor.points);
        json << ",\"center\":[" << armor.center.x << ',' << armor.center.y << ']'
             << ",\"confidence\":" << armor.confidence
             << ",\"name\":\"" << app::auto_aim::ARMOR_NAMES[armor.name] << "\""
             << ",\"priority\":" << static_cast<int>(armor.priority);
        if (detection.pose_valid) {
            json << ",\"translation\":[";
            append_number(json, armor.xyz_in_gimbal.x());
            json << ',';
            append_number(json, armor.xyz_in_gimbal.y());
            json << ',';
            append_number(json, armor.xyz_in_gimbal.z());
            json << "],\"rpy\":[";
            append_number(json, armor.ypr_in_gimbal[2] * 180.0 / CV_PI);
            json << ',';
            append_number(json, armor.ypr_in_gimbal[1] * 180.0 / CV_PI);
            json << ',';
            append_number(json, armor.ypr_in_gimbal[0] * 180.0 / CV_PI);
            json << ']';
        } else {
            json << ",\"translation\":null,\"rpy\":null";
        }
        json << '}';
    }
    json << "]}\n";
    return json.str();
}

double approach(double current, double target, double max_step) {
    return current + std::clamp(target - current, -max_step, max_step);
}

double approach_angle(double current, double target, double max_step) {
    return tools::limit_rad(
        current + std::clamp(tools::limit_rad(target - current), -max_step, max_step));
}

int run_connection(
    int socket_fd, const std::string& config_path, double fire_interval) {
    app::auto_aim::YOLO yolo(config_path, false);
    app::auto_aim::Solver solver(config_path);
    app::auto_aim::Tracker tracker(config_path, solver);
    app::auto_aim::Aimer aimer(config_path);
    app::auto_aim::Shooter shooter(config_path);
    app::auto_aim::Planner planner(config_path);
    auto config = toml::parse_file(config_path);
    const bool planner_auto_fire = config["planner"]["auto_fire"].value_or(false);
    const auto expected_width = config["camera"]["width"].value_or(640U);
    const auto expected_height = config["camera"]["height"].value_or(480U);
    const double max_yaw_step = config["simulation"]["max_yaw_step"].value_or(0.05);
    const double max_pitch_step = config["simulation"]["max_pitch_step"].value_or(0.03);
    const double max_fire_yaw_error =
        config["simulation"]["max_fire_yaw_error"].value_or(0.06);
    const double max_fire_pitch_error =
        config["simulation"]["max_fire_pitch_error"].value_or(0.06);
    const auto search_return_delay_frames =
        config["simulation"]["search_return_delay_frames"].value_or(20U);

    std::vector<std::uint8_t> pixels;
    std::uint64_t frame_count = 0;
    std::size_t previous_detection_count = 0;
    bool previous_mpc_control = false;
    std::uint64_t frames_without_mpc = 0;
    auto last_fire = std::chrono::steady_clock::time_point::min();
    LOG_INFO(MODULE, "Robocore full pipeline connected to Gazebo bridge");

    while (!quit) {
        FrameHeader frame{};
        if (!read_exact(socket_fd, &frame, sizeof(frame))) return 1;
        if (std::memcmp(frame.magic, "RCF1", 4) != 0 || frame.payload_size == 0 ||
            frame.payload_size > MAX_FRAME_BYTES || frame.width == 0 || frame.height == 0 ||
            frame.payload_size != frame.width * frame.height * 3U) {
            LOG_ERROR(MODULE, "Invalid frame packet from bridge");
            return 1;
        }
        if (frame.width != expected_width || frame.height != expected_height) {
            LOG_ERROR(
                MODULE, "Unexpected frame size {}x{}, expected {}x{}", frame.width,
                frame.height, expected_width, expected_height);
            return 1;
        }
        pixels.resize(frame.payload_size);
        if (!read_exact(socket_fd, pixels.data(), pixels.size())) return 1;
        cv::Mat image(
            static_cast<int>(frame.height), static_cast<int>(frame.width), CV_8UC3,
            pixels.data());
        const auto timestamp = std::chrono::steady_clock::now();
        const Eigen::Quaterniond orientation =
            Eigen::AngleAxisd(frame.yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(frame.pitch, Eigen::Vector3d::UnitY());
        solver.set_R_gimbal2world(orientation);

        const auto inference_started = std::chrono::steady_clock::now();
        auto armors = yolo.detect(image, static_cast<int>(frame_count));
        const double inference_ms = std::chrono::duration<double, std::milli>(
                                        std::chrono::steady_clock::now() - inference_started)
                                        .count();

        std::vector<DebugDetection> debug_detections;
        debug_detections.reserve(armors.size());
        for (const auto& armor : armors) {
            DebugDetection detection{armor, false};
            detection.pose_valid = solver.solve(detection.armor);
            debug_detections.push_back(std::move(detection));
        }

        auto targets = tracker.track(armors, timestamp);
        std::optional<app::auto_aim::Target> target;
        if (!targets.empty()) target = targets.front();
        const auto plan = planner.plan(target, frame.bullet_speed);
        const auto legacy_command = aimer.aim(targets, timestamp, frame.bullet_speed, false);
        const Eigen::Vector3d gimbal_position(frame.yaw, frame.pitch, 0.0);
        const bool legacy_fire = shooter.shoot(
            legacy_command, aimer, targets, gimbal_position);

        const auto now = std::chrono::steady_clock::now();
        const bool mpc_control = plan.control && tracker.state() == "tracking";
        if (mpc_control) {
            frames_without_mpc = 0;
        } else {
            ++frames_without_mpc;
        }

        double command_yaw = frame.yaw;
        double command_pitch = frame.pitch;
        if (mpc_control) {
            command_yaw = approach_angle(frame.yaw, plan.yaw, max_yaw_step);
            command_pitch = approach(frame.pitch, plan.pitch, max_pitch_step);
        } else if (frames_without_mpc >= search_return_delay_frames) {
            command_yaw = approach_angle(frame.yaw, 0.0, max_yaw_step);
            command_pitch = approach(frame.pitch, 0.0, max_pitch_step);
        }
        const bool search_control =
            !mpc_control && frames_without_mpc >= search_return_delay_frames &&
            (std::abs(frame.yaw) > 1e-4 || std::abs(frame.pitch) > 1e-4);
        const bool gimbal_control = mpc_control || search_control;

        const bool fire_ready =
            last_fire == std::chrono::steady_clock::time_point::min() ||
            std::chrono::duration<double>(now - last_fire).count() >= fire_interval;
        const bool gimbal_on_target =
            std::abs(tools::limit_rad(plan.yaw - frame.yaw)) <= max_fire_yaw_error &&
            std::abs(plan.pitch - frame.pitch) <= max_fire_pitch_error;
        const bool fire =
            planner_auto_fire && mpc_control && plan.fire && gimbal_on_target && fire_ready;
        if (fire) last_fire = now;

        if (debug_detections.size() != previous_detection_count ||
            mpc_control != previous_mpc_control) {
            if (!debug_detections.empty() && debug_detections.front().pose_valid) {
                const auto& xyz = debug_detections.front().armor.xyz_in_gimbal;
                LOG_INFO(
                    MODULE,
                    "transition frame={} detections={} tracker={} xyz_gimbal=({:.3f},"
                    "{:.3f},{:.3f}) target=({:.3f},{:.3f}) command=({:.3f},{:.3f}) "
                    "feedback=({:.3f},{:.3f})",
                    frame_count, debug_detections.size(), tracker.state(), xyz.x(), xyz.y(),
                    xyz.z(), plan.control ? plan.target_yaw : 0.0,
                    plan.control ? plan.target_pitch : 0.0, command_yaw, command_pitch,
                    frame.yaw, frame.pitch);
            } else {
                LOG_INFO(
                    MODULE, "transition frame={} detections={} tracker={} mpc={} feedback=({:.3f},"
                    "{:.3f})",
                    frame_count, debug_detections.size(), tracker.state(), plan.control,
                    frame.yaw, frame.pitch);
            }
            previous_detection_count = debug_detections.size();
            previous_mpc_control = mpc_control;
        }

        const auto status = build_status(
            armors, debug_detections, targets, plan, mpc_control, gimbal_control, legacy_command,
            legacy_fire, fire, command_yaw, command_pitch, tracker.state(), frame, inference_ms,
            frame_count);
        if (!send_all(socket_fd, status)) return 1;
        if (frame_count % 100 == 0) {
            LOG_INFO(
                MODULE, "frame={} detections={} tracker={} mpc={} fire={} inference={:.1f}ms",
                frame_count, armors.size(), tracker.state(), mpc_control, fire,
                inference_ms);
        }
        ++frame_count;
    }
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    const std::string keys =
        "{help h usage ? |                         | 输出参数说明 }"
        "{config c        | config/gazebo_auto_aim.toml | 仿真自瞄配置 }"
        "{host            | 127.0.0.1             | ROS 桥地址 }"
        "{port            | 5800                  | ROS 桥端口 }";
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    const auto config_path = cli.get<std::string>("config");
    const auto host = cli.get<std::string>("host");
    const int port = cli.get<int>("port");
    const auto config = toml::parse_file(config_path);
    const double fire_interval = config["simulation"]["fire_interval"].value_or(0.30);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    while (!quit) {
        const int socket_fd = connect_bridge(host, port);
        if (socket_fd < 0) {
            LOG_WARN(MODULE, "Waiting for Gazebo bridge at {}:{}", host, port);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            continue;
        }
        try {
            run_connection(socket_fd, config_path, fire_interval);
        } catch (const std::exception& error) {
            LOG_ERROR(MODULE, "Pipeline error: {}", error.what());
        }
        ::close(socket_fd);
        if (!quit) std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    return 0;
}
