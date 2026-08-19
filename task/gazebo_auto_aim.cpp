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
#include <cstdlib>
#include <cstring>
#include <filesystem>
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

class ControlPreview {
public:
    explicit ControlPreview(const toml::table& config) {
        show_window_ = config["simulation"]["show_recognition_window"].value_or(true) &&
                       std::getenv("DISPLAY") != nullptr;
        save_preview_ = config["simulation"]["save_recognition_preview"].value_or(true);
        preview_path_ = config["simulation"]["recognition_preview_path"].value_or(
            std::string{"output/robocore_recognition.png"});
        window_x_ = config["simulation"]["recognition_window_x"].value_or(1300);
        window_y_ = config["simulation"]["recognition_window_y"].value_or(40);
        if (save_preview_) {
            const auto parent = std::filesystem::path(preview_path_).parent_path();
            if (!parent.empty()) std::filesystem::create_directories(parent);
        }
    }

    void draw(
        const cv::Mat& image, const std::vector<DebugDetection>& detections,
        const std::list<app::auto_aim::Target>& targets, app::auto_aim::Solver& solver,
        const app::auto_aim::Planner& planner, const app::auto_aim::Plan& plan,
        const std::string& tracker_state, bool mpc_control, bool target_locked, bool fire,
        const FrameHeader& frame, double inference_ms, std::uint64_t frame_count) {
        cv::Mat view = image.clone();
        if (view.empty()) return;

        for (const auto& detection : detections) {
            if (detection.armor.points.size() != 4) continue;
            std::vector<cv::Point> points;
            points.reserve(4);
            for (const auto& point : detection.armor.points) points.emplace_back(point);
            cv::polylines(view, points, true, cv::Scalar(70, 255, 80), 2, cv::LINE_AA);
            cv::putText(
                view,
                app::auto_aim::ARMOR_NAMES[detection.armor.name] +
                    cv::format(" %.2f", detection.armor.confidence),
                points.front() + cv::Point(0, -7), cv::FONT_HERSHEY_SIMPLEX, 0.46,
                cv::Scalar(70, 255, 80), 1, cv::LINE_AA);
        }

        if (!targets.empty()) {
            const auto& target = targets.front();
            const auto armors = target.armor_xyza_list();
            // 与同济 auto_aim_debug_mpc.cpp 一致：Tracker/EKF 的每个 xyza
            // 直接交给 Solver 反投影，不在显示层重排装甲 ID 或修正几何。
            for (const auto& armor : armors) {
                const auto projected = solver.reproject_armor(
                    armor.head<3>(), armor[3], target.armor_type, target.name);
                if (!drawable_polygon(projected, view.size())) continue;
                std::vector<cv::Point> points;
                points.reserve(projected.size());
                for (const auto& point : projected) {
                    points.emplace_back(point);
                }
                cv::polylines(
                    view, points, true, cv::Scalar(255, 120, 20), 2, cv::LINE_AA);
            }

            if (mpc_control && planner.debug_xyza.allFinite()) {
                const auto aim_points = solver.reproject_armor(
                    planner.debug_xyza.head<3>(), planner.debug_xyza[3], target.armor_type,
                    target.name);
                if (drawable_polygon(aim_points, view.size())) {
                    std::vector<cv::Point> points;
                    points.reserve(aim_points.size());
                    for (const auto& point : aim_points) points.emplace_back(point);
                    cv::polylines(
                        view, points, true,
                        fire ? cv::Scalar(30, 30, 255) : cv::Scalar(0, 190, 255), 3,
                        cv::LINE_AA);
                }
            }
        }

        const std::string target_name =
            targets.empty() ? "-" : app::auto_aim::ARMOR_NAMES[targets.front().name];
        const double inference_fps = inference_ms > 0.0 ? 1000.0 / inference_ms : 0.0;
        std::string geometry = "geometry r1 -- r2 -- dz --";
        if (!targets.empty()) {
            const auto state = targets.front().ekf_x();
            geometry = cv::format(
                "geometry r1 %.3f r2 %.3f dz %+.3f m", state[8], state[8] + state[9],
                state[10]);
        }
        const std::vector<std::string> rows = {
            "ROBOCORE VISION / TRACKER EKF",
            "state " + upper_state(tracker_state),
            "target " + target_name,
            cv::format("detections %zu", detections.size()),
            "aim planner.debug_xyza",
            geometry,
            std::string{"locked "} + (target_locked ? "1" : "0"),
            std::string{"fire "} + (fire ? "1" : "0"),
            cv::format(
                "gimbal yaw %+6.2f pitch %+6.2f deg", frame.yaw * 180.0 / CV_PI,
                frame.pitch * 180.0 / CV_PI),
            cv::format(
                "inference %.1f ms  %.1f FPS  frame %llu", inference_ms, inference_fps,
                static_cast<unsigned long long>(frame_count)),
        };
        for (std::size_t index = 0; index < rows.size(); ++index) {
            cv::putText(
                view, rows[index], cv::Point(12, 23 + static_cast<int>(index) * 23),
                cv::FONT_HERSHEY_SIMPLEX, 0.46, cv::Scalar(235, 242, 245), 1,
                cv::LINE_AA);
        }

        const auto now = std::chrono::steady_clock::now();
        if (save_preview_ && !targets.empty() &&
            (last_save_ == std::chrono::steady_clock::time_point::min() ||
             std::chrono::duration<double>(now - last_save_).count() >= 0.20)) {
            cv::imwrite(preview_path_, view);
            last_save_ = now;
        }
        if (show_window_) {
            try {
                if (!window_initialized_) {
                    cv::namedWindow(window_name_, cv::WINDOW_NORMAL);
                    cv::resizeWindow(window_name_, view.cols, view.rows);
                    cv::moveWindow(window_name_, window_x_, window_y_);
                    window_initialized_ = true;
                }
                cv::imshow(window_name_, view);
                const int key = cv::waitKey(1) & 0xFF;
                if (key == 'q' || key == 27) {
                    cv::destroyWindow(window_name_);
                    show_window_ = false;
                }
            } catch (const cv::Exception& error) {
                LOG_WARN(MODULE, "Disable recognition window: {}", error.what());
                show_window_ = false;
            }
        }
    }

private:
    static bool finite_point(const cv::Point2f& point) {
        return std::isfinite(point.x) && std::isfinite(point.y);
    }

    static bool drawable_polygon(
        const std::vector<cv::Point2f>& points, const cv::Size& image_size) {
        if (points.size() != 4) return false;
        return std::all_of(points.begin(), points.end(), [&](const cv::Point2f& point) {
            return finite_point(point) && point.x > -image_size.width &&
                   point.x < image_size.width * 2 && point.y > -image_size.height &&
                   point.y < image_size.height * 2;
        });
    }

    const std::string window_name_{"Robocore Vision Tracker EKF"};
    std::string preview_path_;
    int window_x_{1300};
    int window_y_{40};
    bool show_window_{false};
    bool save_preview_{true};
    bool window_initialized_{false};
    std::chrono::steady_clock::time_point last_save_{
        std::chrono::steady_clock::time_point::min()};
};

std::string build_status(
    const std::list<app::auto_aim::Armor>& tracked_armors,
    const std::vector<DebugDetection>& detections, const std::list<app::auto_aim::Target>& targets,
    const app::auto_aim::Plan& plan, bool mpc_control, bool target_locked, bool gimbal_control,
    const io::Command& legacy_command, bool legacy_fire, bool fire, double command_yaw,
    double command_pitch,
    const std::string& tracker_state,
    const FrameHeader& frame, double inference_ms, std::uint64_t frame_count) {
    const std::string state =
        fire ? "FIRING" : (target_locked ? "LOCKED" : upper_state(tracker_state));
    const double inference_fps = inference_ms > 0.0 ? 1000.0 / inference_ms : 0.0;

    std::ostringstream json;
    json << std::fixed << std::setprecision(6);
    json << "{\"source\":\"robocore\",\"pipeline\":"
         << "\"YOLOV8/Solver/Tracker/EKF/Aimer/Shooter/Planner/TinyMPC\""
         << ",\"state\":\"" << state << "\""
         << ",\"detected\":" << (!tracked_armors.empty() ? "true" : "false")
         << ",\"locked\":" << (target_locked ? "true" : "false")
         << ",\"fire\":" << (fire ? "true" : "false")
         << ",\"frame\":" << frame_count
         << ",\"image_size\":[" << frame.width << ',' << frame.height << ']'
         << ",\"network_input\":[640,480]"
         << ",\"inference_ms\":" << inference_ms
         << ",\"inference_fps\":" << inference_fps
         << ",\"armor_count\":" << detections.size()
         << ",\"target_count\":" << targets.size()
         << ",\"tracker_state\":\"" << tracker_state << "\""
         << ",\"mpc_control\":" << (mpc_control ? "true" : "false")
         << ",\"gimbal_control\":" << (gimbal_control ? "true" : "false")
         << ",\"mpc_fire\":" << ((plan.control && plan.fire) ? "true" : "false")
         << ",\"legacy_control\":" << (legacy_command.control ? "true" : "false")
         << ",\"legacy_fire\":" << (legacy_fire ? "true" : "false")
         << ",\"yaw\":" << command_yaw << ",\"pitch\":" << command_pitch
         << ",\"yaw_rate\":" << (mpc_control ? plan.yaw_vel : 0.0)
         << ",\"pitch_rate\":" << (mpc_control ? plan.pitch_vel : 0.0)
         << ",\"yaw_acc\":" << (mpc_control ? plan.yaw_acc : 0.0)
         << ",\"pitch_acc\":" << (mpc_control ? plan.pitch_acc : 0.0)
         << ",\"yaw_error\":" << tools::limit_rad(command_yaw - frame.yaw)
         << ",\"pitch_error\":" << command_pitch - frame.pitch;

    json << ",\"vision_target\":";
    if (targets.empty()) {
        json << "null";
    } else {
        const auto& vision_target = targets.front();
        const auto state = vision_target.ekf_x();
        const auto predicted_armors = vision_target.armor_xyza_list();
        json << "{\"source\":\"tracker_ekf\",\"frame\":\"gimbal_origin_world_axes\""
             << ",\"name\":\"" << app::auto_aim::ARMOR_NAMES[vision_target.name] << "\""
             << ",\"center\":[";
        append_number(json, state[0]);
        json << ',';
        append_number(json, state[2]);
        json << ',';
        append_number(json, state[4]);
        json << "],\"velocity\":[";
        append_number(json, state[1]);
        json << ',';
        append_number(json, state[3]);
        json << ',';
        append_number(json, state[5]);
        json << "],\"yaw\":";
        append_number(json, state[6]);
        json << ",\"yaw_rate\":";
        append_number(json, state[7]);
        json << ",\"radius_1\":";
        append_number(json, state[8]);
        json << ",\"radius_2\":";
        append_number(json, state[8] + state[9]);
        json << ",\"height_difference\":";
        append_number(json, state[10]);
        json << ",\"last_armor_id\":" << vision_target.last_id << ",\"armors\":[";
        for (std::size_t index = 0; index < predicted_armors.size(); ++index) {
            if (index != 0) json << ',';
            const auto& armor = predicted_armors[index];
            json << "{\"id\":" << index << ",\"center\":[";
            append_number(json, armor[0]);
            json << ',';
            append_number(json, armor[1]);
            json << ',';
            append_number(json, armor[2]);
            json << "],\"yaw\":";
            append_number(json, armor[3]);
            json << '}';
        }
        json << "]}";
    }

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

int run_connection(
    int socket_fd, const std::string& config_path, double fire_interval) {
    app::auto_aim::YOLO yolo(config_path, false);
    app::auto_aim::Solver solver(config_path);
    app::auto_aim::Tracker tracker(config_path, solver);
    app::auto_aim::Aimer aimer(config_path);
    app::auto_aim::Shooter shooter(config_path);
    app::auto_aim::Planner planner(config_path);
    auto config = toml::parse_file(config_path);
    ControlPreview control_preview(config);
    const bool planner_auto_fire = config["planner"]["auto_fire"].value_or(false);
    const auto expected_width = config["camera"]["width"].value_or(640U);
    const auto expected_height = config["camera"]["height"].value_or(480U);
    const double max_fire_yaw_error =
        config["simulation"]["max_fire_yaw_error"].value_or(0.012);
    const double max_fire_pitch_error =
        config["simulation"]["max_fire_pitch_error"].value_or(0.012);
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
        const std::string tracker_state = tracker.state();
        const bool target_locked = plan.control && tracker_state == "tracking";
        // temp_lost 时 Tracker 仍持有有效 EKF 状态，继续预测跟枪和执行 Planner 决策。
        const bool mpc_control =
            plan.control && (tracker_state == "tracking" || tracker_state == "temp_lost");
        if (mpc_control) {
            frames_without_mpc = 0;
        } else if (tracker_state == "lost") {
            ++frames_without_mpc;
        } else {
            frames_without_mpc = 0;
        }

        double command_yaw = frame.yaw;
        double command_pitch = frame.pitch;
        if (mpc_control) {
            command_yaw = plan.yaw;
            command_pitch = plan.pitch;
        } else if (frames_without_mpc >= search_return_delay_frames) {
            command_yaw = 0.0;
            command_pitch = 0.0;
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
        // Planner 是唯一瞄准和开火决策源；适配层只确认枪管到位并限制实体弹丸射频。
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

        control_preview.draw(
            image, debug_detections, targets, solver, planner, plan, tracker_state,
            mpc_control, target_locked, fire, frame, inference_ms, frame_count);

        const auto status = build_status(
            armors, debug_detections, targets, plan, mpc_control, target_locked, gimbal_control,
            legacy_command, legacy_fire, fire, command_yaw, command_pitch, tracker_state, frame,
            inference_ms, frame_count);
        if (!send_all(socket_fd, status)) return 1;
        if (frame_count % 100 == 0) {
            LOG_INFO(
                MODULE,
                "frame={} detections={} tracker={} mpc={} fire={} inference={:.1f}ms/{:.1f}fps",
                frame_count, armors.size(), tracker.state(), mpc_control, fire,
                inference_ms, inference_ms > 0.0 ? 1000.0 / inference_ms : 0.0);
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
    const double fire_interval = config["simulation"]["fire_interval"].value_or(0.10);

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
