#pragma once

#include "armor.hpp"
#include <algorithm>
#include <cmath>
#include <opencv2/core.hpp>

namespace app::auto_aim {

// 数字分类沿灯条向上下各扩展到灯条长度的 1.125 倍，保持原模型的训练裁剪方式。
// 四角必须是原图中的左上、右上、右下、左下，不能传入 ROI 局部坐标。
inline cv::Mat extract_armor_pattern(const cv::Mat& image, const ArmorCorners& points) {
    const auto tl = (points[0] + points[3]) / 2 - (points[3] - points[0]) * 1.125f;
    const auto bl = (points[0] + points[3]) / 2 + (points[3] - points[0]) * 1.125f;
    const auto tr = (points[2] + points[1]) / 2 - (points[2] - points[1]) * 1.125f;
    const auto br = (points[2] + points[1]) / 2 + (points[2] - points[1]) * 1.125f;
    for (const auto& p : {tl, bl, tr, br}) {
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            return {};
    }
    const int left = static_cast<int>(std::clamp(std::min(tl.x, bl.x), 0.f, float(image.cols)));
    const int top = static_cast<int>(std::clamp(std::min(tl.y, tr.y), 0.f, float(image.rows)));
    const int right = static_cast<int>(std::clamp(std::max(tr.x, br.x), 0.f, float(image.cols)));
    const int bottom = static_cast<int>(std::clamp(std::max(bl.y, br.y), 0.f, float(image.rows)));
    if (right <= left || bottom <= top)
        return {};
    return image(cv::Rect(left, top, right - left, bottom - top));
}

} // namespace app::auto_aim
