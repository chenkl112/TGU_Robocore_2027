#pragma once

#include <cstddef>
#include <list>
#include <optional>

#include "config.hpp"
#include "planner/planner.hpp"
#include "solver.hpp"
#include "tracker.hpp"
#include "yolo.hpp"

namespace app::auto_aim {

// 位姿解算和跟踪具有状态，只能由一个处理线程调用；实时和离线共用此流程。
class TrackingPipeline {
  public:
    explicit TrackingPipeline(const AutoAimConfig& config);
    std::optional<TargetSnapshot> process(std::list<Armor>& detections,
                                          const Eigen::Quaterniond& orientation,
                                          TargetClock::time_point timestamp);
    void reset();
    TrackerState state() const {
        return tracker_.state();
    }
    const char* state_name() const {
        return tracker_.state_name();
    }

  private:
    Solver solver_;
    Tracker tracker_;
};

struct PipelineResult {
    std::size_t detection_count = 0;
    std::optional<TargetSnapshot> target;
    TrackerState tracker_state = TrackerState::lost;
    Plan plan;
};

// 同步单步入口：每个记录帧只执行一次检测、跟踪和规划，不读取真实时钟。
// 反馈未提供时可以输出规划，但不能允许发弹。
class AimPipeline {
  public:
    explicit AimPipeline(const AutoAimConfig& config);
    PipelineResult step(const cv::Mat& frame, const Eigen::Quaterniond& orientation,
                        TargetClock::time_point timestamp, double bullet_speed,
                        std::optional<io::GimbalState> feedback = std::nullopt);

  private:
    YOLO detector_;
    TrackingPipeline tracking_;
    Planner planner_;
    std::optional<TargetClock::time_point> previous_timestamp_;
};

} // namespace app::auto_aim
