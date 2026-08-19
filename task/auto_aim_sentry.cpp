#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <opencv2/opencv.hpp>
#include <optional>
#include <string>
#include <thread>
#include <variant>

#include "app/auto_aim/multithread/mt_yolo.hpp"
#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/target.hpp"
#include "app/auto_aim/tracker.hpp"
#include "io/command.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"
#include "tools/tomlpp.hpp"

static constexpr const char* MODULE = "AUTO_AIM_SENTRY";

const std::string keys =
    "{help h usage ? |                       | 输出命令行参数说明 }"
    "{@config-path   | config/auto_aim.toml  | TOML 配置文件路径 }";

using namespace std::chrono_literals;

int run(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    auto config_path = cli.get<std::string>("@config-path");
    auto config = toml::parse_file(config_path);
    std::variant<int, std::string> camera_source{0};
    if (auto index = config["camera"]["source"].value<int64_t>()) {
        camera_source = static_cast<int>(*index);
    } else if (auto path = config["camera"]["source"].value<std::string>()) {
        camera_source = *path;
    }
    const int camera_width = config["camera"]["width"].value_or(0);
    const int camera_height = config["camera"]["height"].value_or(0);
    const double camera_fps = config["camera"]["fps"].value_or(0.0);

    io::Gimbal gimbal(config_path);
    io::USBCamera camera(camera_source, camera_width, camera_height, camera_fps);

    app::auto_aim::MultiThreadYOLO mt_yolo(config_path);
    app::auto_aim::Solver solver(config_path);
    app::auto_aim::Tracker tracker(config_path, solver);
    app::auto_aim::Planner planner(config_path);
    const bool auto_fire = config["planner"]["auto_fire"].value_or(false);

    tools::ThreadSafeQueue<std::optional<app::auto_aim::Target>, true> target_queue(1);
    target_queue.push(std::nullopt);

    std::atomic<bool> quit{false};

    // ===== 线程 1:采图 + 推 YOLO 队列 =====
    auto capture_thread = std::thread([&]() {
        while (!quit) {
            if (gimbal.mode() != io::GimbalMode::AUTO_AIM) {
                std::this_thread::sleep_for(50ms);
                continue;
            }

            cv::Mat img;
            std::chrono::steady_clock::time_point t;
            camera.read(img, t);
            if (img.empty()) {
                std::this_thread::sleep_for(2ms);
                continue;
            }
            mt_yolo.push(std::move(img), t);
        }
    });

    // ===== 线程 2:取 YOLO 结果 → Solver/Tracker → 推 target_queue =====
    auto track_thread = std::thread([&]() {
        while (!quit) {
            if (mt_yolo.empty()) {
                std::this_thread::sleep_for(1ms);
                continue;
            }
            auto [armors, t] = mt_yolo.pop();
            if (quit) break;

            auto q = gimbal.q(t);
            solver.set_R_gimbal2world(q);

            auto targets = tracker.track(armors, t);
            if (targets.empty()) {
                target_queue.push(std::nullopt);
            } else {
                target_queue.push(targets.front());
            }
        }
    });

    // ===== 线程 3:Planner/TinyMPC → Gimbal =====
    auto plan_thread = std::thread([&]() {
        while (!quit) {
            if (gimbal.mode() != io::GimbalMode::AUTO_AIM) {
                gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
                std::this_thread::sleep_for(50ms);
                continue;
            }
            if (target_queue.empty()) {
                std::this_thread::sleep_for(2ms);
                continue;
            }

            auto gs = gimbal.state();
            auto plan = planner.plan(target_queue.front(), gs.bullet_speed);

            gimbal.send(
                plan.control, auto_fire && plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc,
                plan.pitch, plan.pitch_vel, plan.pitch_acc);

            std::this_thread::sleep_for(10ms);
        }
    });

    // ===== 主线程:监听模式切换 =====
    auto last_mode = io::GimbalMode::IDLE;
    while (!quit) {
        auto mode = gimbal.mode();
        if (mode != last_mode) {
            LOG_INFO(MODULE, "Switch to {}", gimbal.str(mode));
            last_mode = mode;
        }
        std::this_thread::sleep_for(50ms);
    }

    quit = true;
    if (capture_thread.joinable()) capture_thread.join();
    if (track_thread.joinable()) track_thread.join();
    if (plan_thread.joinable()) plan_thread.join();
    gimbal.send(false, false, 0, 0, 0, 0, 0, 0);

    return 0;
}

int main(int argc, char* argv[]) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE, "Fatal error: {}", e.what());
        return 1;
    }
}
