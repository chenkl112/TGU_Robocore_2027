#include "app/auto_aim/config.hpp"
#include "app/auto_aim/multithread/mt_yolo.hpp"
#include "app/auto_aim/pipeline.hpp"
#include "tools/logger.hpp"

#include <chrono>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <stop_token>
#include <thread>

namespace {
using namespace std::chrono_literals;
using namespace app::auto_aim;
using Clock = std::chrono::steady_clock;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

// 每个等待者有独立停止令牌；验证失败时也能结束线程，避免测试清理本身永久阻塞。
class Consumer {
  public:
    explicit Consumer(MultiThreadYOLO& worker)
        : result_(result_promise_.get_future()), entered_(entered_promise_.get_future()),
          thread_([this, &worker](std::stop_token stop) {
              entered_promise_.set_value();
              try {
                  result_promise_.set_value(worker.wait_pop(stop));
              } catch (...) {
                  result_promise_.set_exception(std::current_exception());
              }
          }) {
        entered_.wait();
    }

    void require_waiting() {
        require(result_.wait_for(20ms) == std::future_status::timeout,
                "Open empty worker did not block the consumer");
    }

    void cancel() {
        thread_.request_stop();
    }

    std::optional<DetectionFrame> take(std::chrono::milliseconds timeout = 5s) {
        if (result_.wait_for(timeout) != std::future_status::ready) {
            thread_.request_stop();
            throw std::runtime_error("Worker consumer did not finish before its deadline");
        }
        auto result = result_.get();
        thread_.join();
        return result;
    }

  private:
    std::promise<std::optional<DetectionFrame>> result_promise_;
    std::promise<void> entered_promise_;
    std::future<std::optional<DetectionFrame>> result_;
    std::future<void> entered_;
    std::jthread thread_;
};

void require_frame(const std::optional<DetectionFrame>& frame, Clock::time_point timestamp,
                   std::uint64_t session) {
    require(frame && frame->timestamp == timestamp && frame->session == session,
            "Inference changed a frame's timestamp or control session");
    require(frame->armors.empty(), "Blank lifecycle fixture produced an armor");
}

void worker_lifecycle(const AutoAimConfig& config) {
    auto worker = std::make_unique<MultiThreadYOLO>(config.yolo, config.classifier);
    const cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    const auto first_time = Clock::time_point{} + 3600s + 123ns;
    constexpr std::uint64_t first_session = 0x123456789abcdef0ULL;
    require(worker->push(image, first_time, first_session),
            "Running worker rejected its first frame");
    Consumer first(*worker);
    require_frame(first.take(), first_time, first_session);

    Consumer canceled(*worker);
    canceled.require_waiting();
    canceled.cancel();
    require(!canceled.take(1s), "Canceled wait returned a detection frame");
    // 取消的是消费者等待，不应关闭工作线程，也不能改变下一帧的会话。
    const auto second_time = first_time + 10ms;
    require(worker->push(image, second_time, first_session + 1),
            "Canceling a waiter stopped inference");
    Consumer second(*worker);
    require_frame(second.take(), second_time, first_session + 1);

    const auto queued_time = second_time + 10ms;
    require(worker->push(image, queued_time, first_session + 2), "Worker rejected a queued frame");
    const auto ready_deadline = Clock::now() + 5s;
    while (worker->empty() && Clock::now() < ready_deadline)
        std::this_thread::sleep_for(1ms);
    require(!worker->empty(), "Queued frame did not finish inference");
    std::stop_source stopped;
    stopped.request_stop();
    require(!worker->wait_pop(stopped.get_token()), "Pre-canceled consumer consumed a ready frame");
    Consumer queued(*worker);
    require_frame(queued.take(), queued_time, first_session + 2);

    Consumer closing(*worker);
    closing.require_waiting();
    const auto stop_started = Clock::now();
    worker->stop();
    worker->stop();
    require(!closing.take(1s), "Worker stop did not wake its waiting consumer");
    require(!worker->push(image, queued_time + 10ms, first_session + 3),
            "Stopped worker accepted a frame");
    worker.reset();
    require(Clock::now() - stop_started < 1s, "Repeated stop or idle destructor blocked");

    // 队列中仍有帧时也验证关闭；允许已开始的真实 CPU 推理结束，但不能继续无限等待输入。
    auto busy = std::make_unique<MultiThreadYOLO>(config.yolo, config.classifier);
    for (int frame = 0; frame < 8; ++frame) {
        require(busy->push(image, first_time + frame * 10ms, first_session),
                "Running worker rejected a queued frame before stopping");
    }
    const auto destroy_started = Clock::now();
    busy->stop();
    busy->stop();
    busy.reset();
    require(Clock::now() - destroy_started < 5s, "Queued-frame destructor did not finish");
}

void pipeline_rejected_inputs(const AutoAimConfig& config) {
    AimPipeline pipeline(config);
    const cv::Mat frame = cv::Mat::zeros(480, 640, CV_8UC3);
    const auto identity = Eigen::Quaterniond::Identity();
    const auto first = Clock::time_point{} + 3600s;
    require(!pipeline.step(frame, identity, first, 22).plan.control,
            "Blank pipeline controlled the gimbal");

    auto rejects = [&](const cv::Mat& input, const Eigen::Quaterniond& orientation,
                       Clock::time_point timestamp) {
        try {
            (void)pipeline.step(input, orientation, timestamp, 22);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    const auto second = first + 10ms;
    require(rejects(cv::Mat::zeros(480, 640, CV_8UC1), identity, second),
            "Pipeline accepted a non-BGR frame");
    require(!pipeline.step(frame, identity, second, 22).plan.control,
            "Rejected image consumed the next successful frame timestamp");
    const auto third = second + 10ms;
    require(rejects(frame, Eigen::Quaterniond(0, 0, 0, 0), third),
            "Pipeline accepted an invalid orientation");
    require(!pipeline.step(frame, identity, third, 22).plan.control,
            "Rejected pose consumed the next successful frame timestamp");
    require(rejects(frame, identity, third), "Pipeline accepted a repeated successful timestamp");
    require(rejects(frame, identity, first), "Pipeline accepted an out-of-order timestamp");
}
} // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected configuration path");
        auto config = load_auto_aim_config(argv[1]);
        config.yolo.device = "CPU";
        worker_lifecycle(config);
        pipeline_rejected_inputs(config);
        std::cout << "PASS worker metadata, cancellation, close, reuse, destruction and pipeline "
                     "retries\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
