#include "mt_yolo.hpp"

#include "tools/logger.hpp"

namespace app::auto_aim {
namespace {
constexpr const char* MODULE = "MT_YOLO";
}

MultiThreadYOLO::MultiThreadYOLO(const std::string& config_path, bool debug)
    : MultiThreadYOLO(load_vision_config(config_path), debug) {}

MultiThreadYOLO::MultiThreadYOLO(const VisionConfig& config, bool debug)
    : MultiThreadYOLO(config.yolo, config.classifier, debug) {}

MultiThreadYOLO::MultiThreadYOLO(const YoloConfig& config, const ClassifierConfig& classifier,
                                 bool debug)
    : yolo_(config, classifier, debug), thread_([this](std::stop_token stop) { worker(stop); }) {
    LOG_INFO(MODULE, "[MultiThreadYOLO] started.");
}

MultiThreadYOLO::~MultiThreadYOLO() {
    stop();
    if (thread_.joinable())
        thread_.join();
}

void MultiThreadYOLO::stop() {
    thread_.request_stop();
    input_.close();
    output_.close();
}

bool MultiThreadYOLO::push(cv::Mat image, std::chrono::steady_clock::time_point timestamp,
                           std::uint64_t session) {
    check_error();
    return input_.push({std::move(image), timestamp, session});
}

DetectionFrame MultiThreadYOLO::pop() {
    auto result = wait_pop({});
    if (!result)
        throw tools::QueueClosed();
    return std::move(*result);
}

std::optional<DetectionFrame> MultiThreadYOLO::wait_pop(std::stop_token stop) {
    check_error();
    auto result = output_.wait_pop(stop);
    check_error();
    return result;
}

bool MultiThreadYOLO::empty() {
    check_error();
    return output_.empty();
}

void MultiThreadYOLO::check_error() {
    std::lock_guard lock(error_mutex_);
    if (error_)
        std::rethrow_exception(error_);
}

void MultiThreadYOLO::worker(std::stop_token stop) {
    try {
        while (auto frame = input_.wait_pop(stop)) {
            if (stop.stop_requested())
                break;
            if (frame->image.empty())
                continue;
            auto armors = yolo_.detect(frame->image);
            if (!output_.push({std::move(armors), frame->timestamp, frame->session}))
                break;
        }
    } catch (...) {
        std::lock_guard lock(error_mutex_);
        error_ = std::current_exception();
    }
    // 包括失败路径：所有消费者都能结束等待，随后读取同一份异常状态。
    input_.close();
    output_.close();
    LOG_INFO(MODULE, "[MultiThreadYOLO] stopped.");
}

} // namespace app::auto_aim
