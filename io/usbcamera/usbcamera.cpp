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
    : USBCamera(CameraConfig{device_index, width, height, fps}) {}

USBCamera::USBCamera(const std::string& source, int width, int height, double fps)
    : USBCamera(CameraConfig{source, width, height, fps}) {}

USBCamera::USBCamera(const std::variant<int, std::string>& source, int width, int height,
                     double fps)
    : USBCamera(CameraConfig{source, width, height, fps}) {}

USBCamera::USBCamera(const CameraConfig& config)
    : source_(config.source), width_(config.width), height_(config.height),
      fps_(normalize_fps(config.fps)) {
    validate(config);
    open(source_, width_, height_, fps_);
    thread_ = std::thread(&USBCamera::capture_thread, this);
}

USBCamera::~USBCamera() {
    stop();
    if (thread_.joinable())
        thread_.join();
    if (cap_.isOpened())
        cap_.release();
}

void USBCamera::stop() {
    quit_ = true;
    frame_cv_.notify_all();
}

void USBCamera::open(const std::variant<int, std::string>& src, int width, int height, double fps) {
    bool opened = false;
    if (std::holds_alternative<int>(src)) {
        opened = cap_.open(std::get<int>(src), cv::CAP_V4L2);
        if (!opened)
            opened = cap_.open(std::get<int>(src));
    } else {
        const auto& path = std::get<std::string>(src);
        if (path.starts_with("/dev/video")) {
            opened = cap_.open(path, cv::CAP_V4L2);
        } else {
            // 超时必须在打开时传给 FFmpeg，网络流的建立和读取都需要有界等待。
            opened = cap_.open(
                path, cv::CAP_FFMPEG,
                {cv::CAP_PROP_OPEN_TIMEOUT_MSEC, 1000, cv::CAP_PROP_READ_TIMEOUT_MSEC, 500});
        }
    }

    if (!opened) {
        LOG_ERROR(MODULE, "[USBCamera] Failed to open source");
        ok_ = false;
        return;
    }

    if (width > 0)
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
    if (height > 0)
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);
    cap_.set(cv::CAP_PROP_FPS, normalize_fps(fps));
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    LOG_INFO(MODULE, "[USBCamera] opened {}x{} @{}fps",
             static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH)),
             static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT)), cap_.get(cv::CAP_PROP_FPS));
    ok_ = true;
}

void USBCamera::capture_thread() {
    while (!quit_) {
        try {
            if (!cap_.isOpened()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                open(source_, width_, height_, fps_);
                continue;
            }

            const auto capture_started = std::chrono::steady_clock::now();
            cv::Mat img;
            bool got = false;
            if (cap_.getBackendName() == "V4L2") {
                std::vector<cv::VideoCapture> streams{cap_};
                std::vector<int> ready;
                if (!cv::VideoCapture::waitAny(streams, ready, 50'000'000))
                    continue;
                got = cap_.retrieve(img);
            } else {
                got = cap_.read(img);
            }
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
            const auto next_frame =
                capture_started +
                std::chrono::duration_cast<std::chrono::steady_clock::duration>(frame_period);
            std::this_thread::sleep_until(next_frame);
        } catch (const std::exception& error) {
            LOG_ERROR(MODULE, "Camera capture failed: {}", error.what());
            cap_.release();
            ok_ = false;
        }
    }
}

void USBCamera::read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp) {
    std::unique_lock<std::mutex> lock(mutex_);
    frame_cv_.wait_for(lock, std::chrono::milliseconds(50),
                       [this] { return quit_ || frame_sequence_ != consumed_sequence_; });
    if (quit_ || frame_sequence_ == consumed_sequence_) {
        img.release();
        return;
    }
    img = latest_img_.clone();
    timestamp = latest_timestamp_;
    consumed_sequence_ = frame_sequence_;
}

} // namespace io
