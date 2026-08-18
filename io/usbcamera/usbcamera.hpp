#ifndef TGU_ROBOCORE_2027_USBCAMERA_HPP
#define TGU_ROBOCORE_2027_USBCAMERA_HPP
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <string>
#include <thread>
#include <variant>

namespace io {

// 用法:
//   io::USBCamera cam(0);                               // 用 /dev/video0
//   io::USBCamera cam("/dev/video2");                   // 用指定设备路径
//   io::USBCamera cam("rtsp://192.168.1.10/stream");    // 网络流
//   io::USBCamera cam("test.mp4");                      // 视频文件
//
//   cv::Mat img;
//   std::chrono::steady_clock::time_point t;
//   cam.read(img, t);
class USBCamera {
public:
    explicit USBCamera(int device_index, int width = 0, int height = 0, double fps = 0.0);
    explicit USBCamera(
        const std::string& source, int width = 0, int height = 0, double fps = 0.0);
    explicit USBCamera(
        const std::variant<int, std::string>& source, int width = 0, int height = 0,
        double fps = 0.0);
    ~USBCamera();

    void read(cv::Mat& img, std::chrono::steady_clock::time_point& timestamp);
    bool ok() const { return ok_; }

private:
    void open(const std::variant<int, std::string>& src, int width, int height, double fps);
    void capture_thread();

    cv::VideoCapture cap_;
    std::variant<int, std::string> source_;
    int width_;
    int height_;
    double fps_;

    std::mutex mutex_;
    std::condition_variable frame_cv_;
    cv::Mat latest_img_;
    std::chrono::steady_clock::time_point latest_timestamp_;
    std::uint64_t frame_sequence_ = 0;
    std::uint64_t consumed_sequence_ = 0;

    std::thread thread_;
    std::atomic<bool> quit_{false};
    std::atomic<bool> ok_{false};
};

}  // namespace io

#endif  // TGU_ROBOCORE_2027_USBCAMERA_HPP
