#ifndef TGU_ROBOCORE_2027_MT_YOLO_HPP
#define TGU_ROBOCORE_2027_MT_YOLO_HPP
#pragma once

#include <atomic>
#include <chrono>
#include <list>
#include <opencv2/opencv.hpp>
#include <thread>
#include <tuple>

#include "../yolo.hpp"
#include "tools/thread_safe_queue.hpp"

namespace app::auto_aim {

// 把 YOLO 推理放进一个独立工作线程。
// 主线程不再被 detect() 阻塞,采图线程和检测线程并行流水线工作。
//
// 用法:
//   MultiThreadYOLO mt_yolo(config_path);
//   mt_yolo.push(img, t);                  // 喂一帧
//   auto [armors, t_out] = mt_yolo.pop();  // 取最新结果(阻塞)
class MultiThreadYOLO {
public:
    using FrameIn = std::tuple<cv::Mat, std::chrono::steady_clock::time_point>;
    using FrameOut = std::tuple<std::list<Armor>, std::chrono::steady_clock::time_point>;

    explicit MultiThreadYOLO(const std::string& config_path, bool debug = false);
    ~MultiThreadYOLO();

    void push(cv::Mat img, std::chrono::steady_clock::time_point t);
    FrameOut pop();
    bool empty();

private:
    void worker();

    YOLO yolo_;

    // PopWhenFull = true:输入队列满了就丢最老的,保证检测追的是最新画面
    tools::ThreadSafeQueue<FrameIn, true> in_queue_{2};
    tools::ThreadSafeQueue<FrameOut, true> out_queue_{2};

    std::thread thread_;
    std::atomic<bool> quit_{false};
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_MT_YOLO_HPP
