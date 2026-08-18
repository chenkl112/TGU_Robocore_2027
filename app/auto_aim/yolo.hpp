#ifndef TGU_ROBOCORE_2027_YOLO_HPP
#define TGU_ROBOCORE_2027_YOLO_HPP
#pragma once

#include <list>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"

namespace app::auto_aim {

class YOLOBase {
public:
    virtual ~YOLOBase() = default;
    virtual std::list<Armor> detect(const cv::Mat& img, int frame_count) = 0;
    virtual std::list<Armor> postprocess(
        double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count) = 0;
};

class YOLO {
public:
    YOLO(const std::string& config_path, bool debug = true);

    std::list<Armor> detect(const cv::Mat& img, int frame_count = -1);

    std::list<Armor> postprocess(
        double scale, cv::Mat& output, const cv::Mat& bgr_img, int frame_count);

private:
    std::unique_ptr<YOLOBase> yolo_;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_YOLO_HPP
