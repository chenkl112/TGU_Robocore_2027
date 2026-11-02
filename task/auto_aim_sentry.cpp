#include <atomic>
#include <chrono>
#include <cstdint>
#include <csignal>
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

namespace {
static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> interrupted{false};
void stop_signal(int) { interrupted.store(true, std::memory_order_relaxed); }
struct TargetSnapshot {
    std::optional<app::auto_aim::Target> target;
    uint64_t session = 0;
};
}

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
    const double max_target_age = config["planner"]["max_target_age"].value_or(0.1);
    const bool auto_fire = config["planner"]["auto_fire"].value_or(false);

    tools::ThreadSafeQueue<TargetSnapshot, true> target_queue(1);
    target_queue.push({});
    std::atomic<bool> quit{false};
    std::atomic<bool> failed{false};
    auto protect = [&](auto action) {
        try {
            action();
        } catch (const std::exception& error) {
            failed = true;
            quit = true;
            LOG_ERROR(MODULE, "Worker failed: {}", error.what());
        } catch (...) {
            failed = true;
            quit = true;
            LOG_ERROR(MODULE, "Worker failed with an unknown exception");
        }
    };

    std::jthread capture_thread([&](std::stop_token stop) {
        protect([&] {
            while (!quit && !stop.stop_requested()) {
                if (gimbal.mode() != io::GimbalMode::AUTO_AIM) {
                    std::this_thread::sleep_for(10ms);
                    continue;
                }
                cv::Mat img;
                std::chrono::steady_clock::time_point t;
                camera.read(img, t);
                if (!img.empty()) mt_yolo.push(std::move(img), t);
            }
        });
    });

    std::jthread track_thread([&](std::stop_token stop) {
        protect([&] {
            uint64_t tracker_session = 0;
            while (!quit && !stop.stop_requested()) {
                if (mt_yolo.empty()) {
                    std::this_thread::sleep_for(1ms);
                    continue;
                }
                auto [armors, t] = mt_yolo.pop();
                const auto session = gimbal.session();
                if (session != tracker_session) {
                    tracker.reset();
                    tracker_session = session;
                }
                if (gimbal.mode() != io::GimbalMode::AUTO_AIM ||
                    t < gimbal.session_started()) {
                    target_queue.push({});
                    continue;
                }
                try {
                    solver.set_R_gimbal2world(gimbal.q(t));
                } catch (const std::runtime_error&) {
                    tracker.reset();
                    target_queue.push({});
                    continue;
                }
                auto targets = tracker.track(armors, t);
                target_queue.push({targets.empty() ? std::nullopt
                                                  : std::optional{targets.front()}, session});
            }
        });
    });

    std::jthread plan_thread([&](std::stop_token stop) {
        protect([&] {
            while (!quit && !stop.stop_requested()) {
                const auto snapshot = target_queue.front();
                const auto gs = gimbal.state();
                app::auto_aim::Plan plan{};
                if (gimbal.mode() == io::GimbalMode::AUTO_AIM &&
                    snapshot.session == gimbal.session()) {
                    plan = planner.plan(snapshot.target, gs.bullet_speed, std::chrono::steady_clock::now(), gs);
                }
                if (snapshot.target &&
                    std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                                  snapshot.target->observation_timestamp()).count() > max_target_age) {
                    plan = {};
                }
                if (quit || interrupted || stop.stop_requested()) break;
                gimbal.send(plan.control, auto_fire && plan.fire,
                            plan.yaw, plan.yaw_vel, plan.yaw_acc,
                            plan.pitch, plan.pitch_vel, plan.pitch_acc, snapshot.session);
                std::this_thread::sleep_for(10ms);
            }
        });
    });

    auto last_mode = io::GimbalMode::IDLE;
    while (!quit && !interrupted) {
        const auto mode = gimbal.mode();
        if (mode != last_mode) {
            LOG_INFO(MODULE, "Switch to {}", gimbal.str(mode));
            last_mode = mode;
        }
        std::this_thread::sleep_for(10ms);
    }

    quit = true;
    camera.stop();
    plan_thread.join();
    gimbal.send(false, false, 0, 0, 0, 0, 0, 0);
    capture_thread.join();
    track_thread.join();
    return failed ? 1 : 0;
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE, "Fatal error: {}", e.what());
        return 1;
    }
}
