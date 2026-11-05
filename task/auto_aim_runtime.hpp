#pragma once

#include <atomic>
#include <cstdint>
#include <exception>
#include <optional>
#include <stop_token>
#include <thread>

#include "app/auto_aim/config.hpp"
#include "app/auto_aim/multithread/mt_yolo.hpp"
#include "app/auto_aim/pipeline.hpp"
#include "io/gimbal/gimbal.hpp"
#include "io/usbcamera/usbcamera.hpp"
#include "tools/latest_value.hpp"

namespace task {

// 运行时拥有设备和工作线程；跟踪器只由跟踪线程访问，规划器只由控制线程访问。
// 所有工作线程共用一个停止源，异常和进程信号都进入同一条退出路径。
class AutoAimRuntime {
  public:
    explicit AutoAimRuntime(const app::auto_aim::AutoAimConfig& config);
    ~AutoAimRuntime();
    int run(const std::atomic<bool>& interrupted);

  private:
    struct TrackingUpdate {
        std::optional<app::auto_aim::TargetSnapshot> target;
        std::uint64_t session = 0;
    };

    void capture_loop();
    void tracking_loop();
    void control_loop();
    void stop();
    void join();
    void record_failure(const char* worker, std::exception_ptr error);

    const app::auto_aim::AutoAimConfig config_;
    io::Gimbal gimbal_;
    io::USBCamera camera_;
    app::auto_aim::MultiThreadYOLO detector_;
    app::auto_aim::TrackingPipeline tracking_;
    app::auto_aim::Planner planner_;
    tools::LatestValue<TrackingUpdate> latest_;
    std::stop_source stopping_;
    std::atomic<bool> failed_{false};
    const std::atomic<bool>* interrupted_ = nullptr;
    std::jthread capture_thread_;
    std::jthread tracking_thread_;
    std::jthread control_thread_;
};

} // namespace task
