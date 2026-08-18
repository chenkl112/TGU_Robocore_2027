#include "mt_yolo.hpp"

#include "tools/logger.hpp"

namespace app::auto_aim {

static constexpr const char* MODULE = "MT_YOLO";

MultiThreadYOLO::MultiThreadYOLO(const std::string& config_path, bool debug)
    : yolo_(config_path, debug) {
    thread_ = std::thread(&MultiThreadYOLO::worker, this);
    LOG_INFO(MODULE, "[MultiThreadYOLO] started.");
}

MultiThreadYOLO::~MultiThreadYOLO() {
    quit_ = true;
    in_queue_.push({cv::Mat(), std::chrono::steady_clock::now()});
    if (thread_.joinable()) thread_.join();
}

void MultiThreadYOLO::push(cv::Mat img, std::chrono::steady_clock::time_point t) {
    in_queue_.push({std::move(img), t});
}

MultiThreadYOLO::FrameOut MultiThreadYOLO::pop() {
    return out_queue_.pop();
}

bool MultiThreadYOLO::empty() {
    return out_queue_.empty();
}

void MultiThreadYOLO::worker() {
    while (!quit_) {
        auto [img, t] = in_queue_.pop();
        if (quit_ || img.empty()) continue;

        auto armors = yolo_.detect(img);
        out_queue_.push({std::move(armors), t});
    }
    LOG_INFO(MODULE, "[MultiThreadYOLO] stopped.");
}

}  // namespace app::auto_aim
