#include <atomic>
#include <chrono>
#include <fstream>
#include <format>
#include <list>
#include <opencv2/opencv.hpp>
#include <thread>
#include <utility>

#include "app/auto_aim/aimer.hpp"
#include "app/auto_aim/shooter.hpp"
#include "app/auto_aim/solver.hpp"
#include "app/auto_aim/target.hpp"
#include "app/auto_aim/tracker.hpp"
#include "app/auto_aim/yolo.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/thread_safe_queue.hpp"

static constexpr const char* MODULE = "AUTO_AIM_TEST";

const std::string keys =
    "{help h usage ? |                        | 输出命令行参数说明 }"
    "{config-path c  | config/auto_aim.toml   | TOML 配置文件的路径}"
    "{start-index s  | 0                      | 视频起始帧下标    }"
    "{end-index e    | 0                      | 视频结束帧下标    }"
    "{@input-path    | assets/demo/demo       | avi 和 txt 文件的路径}";

using namespace std::chrono_literals;

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    auto input_path = cli.get<std::string>(0);
    auto config_path = cli.get<std::string>("config-path");
    auto start_index = cli.get<int>("start-index");
    auto end_index = cli.get<int>("end-index");

    auto video_path = std::format("{}.avi", input_path);
    auto text_path = std::format("{}.txt", input_path);
    cv::VideoCapture video(video_path);
    std::ifstream text(text_path);

    if (!video.isOpened()) {
        LOG_ERROR(MODULE, "Cannot open video: {}", video_path);
        return -1;
    }
    if (!text.is_open()) {
        LOG_ERROR(MODULE, "Cannot open text: {}", text_path);
        return -1;
    }

    app::auto_aim::YOLO yolo(config_path);
    app::auto_aim::Solver solver(config_path);
    app::auto_aim::Tracker tracker(config_path, solver);
    app::auto_aim::Aimer aimer(config_path);
    app::auto_aim::Shooter shooter(config_path);
    (void)shooter;

    using TimedTargets = std::pair<
        std::list<app::auto_aim::Target>, std::chrono::steady_clock::time_point>;
    tools::ThreadSafeQueue<TimedTargets, true> target_queue(1);
    target_queue.push({{}, std::chrono::steady_clock::now()});

    std::atomic<bool> quit{false};

    auto plan_thread = std::thread([&]() {
        constexpr double bullet_speed = 23.0;
        while (!quit) {
            if (target_queue.empty()) {
                std::this_thread::sleep_for(2ms);
                continue;
            }
            auto [targets, timestamp] = target_queue.front();
            auto command = aimer.aim(targets, timestamp, bullet_speed);

            if (command.control) {
                LOG_INFO(
                    MODULE, "[Aim] yaw={:.3f} pitch={:.3f}",
                    command.yaw * 57.3, command.pitch * 57.3);
            }
            std::this_thread::sleep_for(10ms);
        }
    });

    cv::Mat img;
    auto t0 = std::chrono::steady_clock::now();
    video.set(cv::CAP_PROP_POS_FRAMES, start_index);

    for (int frame_count = start_index;; frame_count++) {
        if (end_index > 0 && frame_count > end_index) break;

        video.read(img);
        if (img.empty()) break;

        double t, w, x, y, z;
        if (!(text >> t >> w >> x >> y >> z)) {
            LOG_ERROR(MODULE, "Pose stream ended or contains invalid data at frame {}", frame_count);
            break;
        }
        auto timestamp = t0 + std::chrono::microseconds(int(t * 1e6));

        solver.set_R_gimbal2world({w, x, y, z});

        auto armors = yolo.detect(img, frame_count);
        auto targets = tracker.track(armors, timestamp);
        target_queue.push({std::move(targets), timestamp});

        auto display = img.clone();
        tools::draw_text(display, std::format("[{}]", frame_count), {10, 30}, {255, 255, 255});

        LOG_INFO(MODULE, "frame={} armors={}", frame_count, armors.size());

        cv::imshow("auto_aim_test", display);
        int key = cv::waitKey(1);
        if (key == 27 || key == 'q') break;
    }

    quit = true;
    if (plan_thread.joinable()) plan_thread.join();

    return 0;
}
