#include "pipeline.hpp"

#include <stdexcept>

namespace app::auto_aim {

TrackingPipeline::TrackingPipeline(const AutoAimConfig& config)
    : solver_(config.calibration, config.solver), tracker_(config.tracker, solver_) {}

std::optional<TargetSnapshot> TrackingPipeline::process(std::list<Armor>& detections,
                                                        const Eigen::Quaterniond& orientation,
                                                        TargetClock::time_point timestamp) {
    solver_.set_R_gimbal2world(orientation);
    return tracker_.track_snapshot(detections, timestamp);
}

void TrackingPipeline::reset() {
    tracker_.reset();
}

AimPipeline::AimPipeline(const AutoAimConfig& config)
    : detector_(config.yolo, config.classifier, false), tracking_(config),
      planner_(config.planner) {}

PipelineResult AimPipeline::step(const cv::Mat& frame, const Eigen::Quaterniond& orientation,
                                 TargetClock::time_point timestamp, double bullet_speed,
                                 std::optional<io::GimbalState> feedback) {
    if (previous_timestamp_ && timestamp <= *previous_timestamp_) {
        throw std::invalid_argument("frame timestamps must be strictly increasing");
    }
    auto detections = detector_.detect(frame);
    PipelineResult result;
    result.detection_count = detections.size();
    result.target = tracking_.process(detections, orientation, timestamp);
    result.tracker_state = tracking_.state();
    result.plan = planner_.plan(result.target, bullet_speed, timestamp, feedback);
    previous_timestamp_ = timestamp;
    return result;
}

} // namespace app::auto_aim
