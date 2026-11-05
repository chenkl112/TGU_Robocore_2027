#pragma once

#include <algorithm>
#include <cstdint>
#include <opencv2/core.hpp>

namespace app::auto_aim {

// 负宽高表示延伸到当前图像边缘。保留配置值，让大小分辨率往返时都能恢复覆盖范围。
inline cv::Rect effective_yolo_roi(const cv::Rect& configured, bool enabled, cv::Size image) {
    if (image.empty())
        return {};
    if (!enabled)
        return {0, 0, image.width, image.height};
    using Wide = std::int64_t;
    const Wide right = configured.width < 0 ? image.width : Wide(configured.x) + configured.width;
    const Wide bottom =
        configured.height < 0 ? image.height : Wide(configured.y) + configured.height;
    const int left = std::clamp(configured.x, 0, image.width);
    const int top = std::clamp(configured.y, 0, image.height);
    const int clipped_right = static_cast<int>(std::clamp<Wide>(right, 0, image.width));
    const int clipped_bottom = static_cast<int>(std::clamp<Wide>(bottom, 0, image.height));
    if (clipped_right <= left || clipped_bottom <= top)
        return {};
    return {left, top, clipped_right - left, clipped_bottom - top};
}

} // namespace app::auto_aim
