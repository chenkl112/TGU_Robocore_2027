#pragma once

#include <cmath>
#include <utility>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace app::auto_aim {

// The bundled YOLOv8 pose head emits LT, LB, RB, RT. PnP expects LT, RT, RB, LB.
// Preserve channel identities: perspective can reverse projected edge lengths,
// and roll can interleave the image heights of endpoints from different bars.
inline bool decode_yolov8_keypoints(std::vector<cv::Point2f>& points) {
    if (points.size() != 4) return false;
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y)) return false;
    }
    // Validation must not reorder the points or discard the model's identities.
    if (!cv::isContourConvex(points) || std::abs(cv::contourArea(points)) < 1.0) return false;
    std::swap(points[1], points[3]);
    return true;
}

}  // namespace app::auto_aim
