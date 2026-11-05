#pragma once

#include <opencv2/imgproc.hpp>

namespace vision_fixture {

// 固定合成图用于真实 YOLO 和数字分类器的正例回归；不依赖相机、网络或额外图片文件。
inline cv::Mat hero_image() {
    cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
    cv::rectangle(image, {280, 223}, {360, 257}, {30, 30, 30}, -1);
    for (int x : {280, 360}) {
        cv::rectangle(image, {x - 4, 223}, {x + 4, 257}, {0, 0, 255}, -1);
        cv::rectangle(image, {x - 2, 225}, {x + 2, 255}, {255, 255, 255}, -1);
    }
    int baseline;
    const auto text = cv::getTextSize("1", cv::FONT_HERSHEY_COMPLEX, 1.4, 3, &baseline);
    cv::putText(image, "1", {320 - text.width / 2, 240 + text.height / 2}, cv::FONT_HERSHEY_COMPLEX,
                1.4, {255, 255, 255}, 3);
    return image;
}

} // namespace vision_fixture
