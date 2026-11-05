#pragma once

#include <cstdint>
#include <exception>
#include <mutex>
#include <optional>
#include <stop_token>
#include <thread>

#include "../yolo.hpp"
#include "tools/thread_safe_queue.hpp"

namespace app::auto_aim {

struct DetectionFrame {
    std::list<Armor> armors;
    std::chrono::steady_clock::time_point timestamp{};
    std::uint64_t session = 0;
};

// 推理对象只由工作线程访问。输入/输出有界，帧始终携带采集时间和自瞄会话。
// 关闭会唤醒等待者；工作线程异常由 push/pop/wait_pop 转交给调用方。
class MultiThreadYOLO {
  public:
    explicit MultiThreadYOLO(const std::string& config_path, bool debug = false);
    MultiThreadYOLO(const YoloConfig& config, const ClassifierConfig& classifier,
                    bool debug = false);
    ~MultiThreadYOLO();

    // 图像缓冲区交给推理线程读取；结果返回前，调用方不得通过其他 Mat 别名修改像素。
    bool push(cv::Mat image, std::chrono::steady_clock::time_point timestamp,
              std::uint64_t session = 0);
    DetectionFrame pop();
    std::optional<DetectionFrame> wait_pop(std::stop_token stop);
    bool empty();
    void stop();

  private:
    struct InputFrame {
        cv::Mat image;
        std::chrono::steady_clock::time_point timestamp;
        std::uint64_t session;
    };
    explicit MultiThreadYOLO(const VisionConfig& config, bool debug);
    void worker(std::stop_token stop);
    void check_error();

    YOLO yolo_;
    tools::ThreadSafeQueue<InputFrame, true> input_{2};
    tools::ThreadSafeQueue<DetectionFrame, true> output_{2};
    std::mutex error_mutex_;
    std::exception_ptr error_;
    std::jthread thread_;
};

} // namespace app::auto_aim
