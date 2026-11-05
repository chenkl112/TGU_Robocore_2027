#pragma once

#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <utility>
#include <vector>

namespace app::auto_aim {

// 模型输出为左上、左下、右下、右上；PnP 要求左上、右上、右下、左下。
// 必须保留通道对应的物理身份：透视会改变边长关系，侧倾会打乱不同灯条端点的图像高度。
inline bool decode_yolov8_keypoints(std::vector<cv::Point2f>& points) {
    if (points.size() != 4)
        return false;
    for (const auto& point : points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y))
            return false;
    }
    // 校验只排除退化轮廓，不能按图像位置重排物理角点。
    if (!cv::isContourConvex(points) || std::abs(cv::contourArea(points)) < 1.0)
        return false;
    std::swap(points[1], points[3]);
    return true;
}

} // namespace app::auto_aim
