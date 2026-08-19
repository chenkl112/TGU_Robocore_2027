#include "usbcamera.hpp"

#include <algorithm>

#include "tools/logger.hpp"

namespace io {

static constexpr const char* MODULE = "USB_CAM";
static constexpr double MAX_CAPTURE_FPS = 200.0;

static double normalize_fps(double fps) {
    return fps > 0.0 ? std::clamp(fps, 1.0, MAX_CAPTURE_FPS) : MAX_CAPTURE_FPS;
}

USBCamera::USBCamera(int device_index, int width, int height, double fps)
    : source_(device_index), width_(width), height_(height), fps_(normalize_fps(fps)) {
    open(source_, width_, height_, fps_);
    thread_ = std::thread(&USBCamera::capture_thread, this);
}

USBCamera::USBCamera(const std::string& source, int width, int height, double fps)
    : source_(source), width_(width), height_(height), fps_(normalize_fps(fps)) {
    open(source_, width_, height_, fps_);
    thread_ = std::thread(&USBCamera::capture_thread, this);
}

USBCamera::USBCamera(
    const std::variant<int, std::string>& source, int width, int height, double fps)
    : source_(source), width_(width), height_(height), fps_(normalize_fps(fps)) {
    open(source_, width_, height_, fps_);
    thread_ = std::thread(&USBCamera::capture_thread, this);
}

USBCamera::~USBCamera() {
    quit_ = true;
    frame_cv_.notify_all();
    if (thread_.joinable()) thread_.join();
    if (cap_.isOpened()) cap_.release();
}

void USBCamera::open(
    const std::variant<int, std::string>& src, int width, int height, double fps) {
    bool opened = false;
    if (std::holds_alternative<int>(src)) {
        opened = cap_.open(std::get<int>(src), cv::CAP_V4L2);
        if (!opened) opened = cap_.open(std::get<int>(src));
    } else {
        opened = cap_.open(std::get<std::string>(src));
    }

    if (!opened) {
        LOG_ERROR(MODULE, "[USBCamera] Failed to open source");
        ok_ = false;
        return;
    }

    if (width > 0) cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    if (height > 0) cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, normalize_fps(fps));
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    LOG_INFO(
        MODULE, "[USBCamera] opened {}x{} @{}fps", static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)), cap_.get(cv::CAP_PROP_FPS));
    ok_ = true;
}

void USBCamera::capture_thread() {
    while (!quit_) {
        if (!cap_.isOpened()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            open(source_, width_, height_, fps_);
            continue;
        }

        const auto capture_started = std::chrono::steady_clock::now();
        cv::Mat img;
        bool got = cap_.read(img);
        auto t = std::chrono::steady_clock::now();

        if (!got || img.empty()) {
            LOG_WARN(MODULE, "[USBCamera] read failed, reopening...");
            cap_.release();
            ok_ = false;
            continue;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            latest_img_ = std::move(img);
            latest_timestamp_ = t;
            ++frame_sequence_;
        }
        frame_cv_.notify_one();

        const auto frame_period = std::chrono::duration<double>(1.0 / fps_);
        const auto next_frame = capture_started +
                                std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                    frame_period);
        std::this_thread::sleep_until(next_frame);
    }
}

void USBCamera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    std::unique_lock<std::mutex> lock(mutex_);
    frame_cv_.wait(lock, [this] {
        return quit_ || frame_sequence_ != consumed_sequence_;
    });
    if (quit_) {
        img.release();
        return;
    }
    img = latest_img_.clone();
    timestamp = latest_timestamp_;
    consumed_sequence_ = frame_sequence_;
}

}  // namespace io
