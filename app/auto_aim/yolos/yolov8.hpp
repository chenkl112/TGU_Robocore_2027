#ifndef TGU_ROBOCORE_2027_YOLOV8_HPP
#define TGU_ROBOCORE_2027_YOLOV8_HPP
#pragma once

#include <list>
#include <opencv2/opencv.hpp>
#include <openvino/openvino.hpp>
#if __has_include(<openvino/core/preprocess/pre_post_process.hpp>)
#include <openvino/core/preprocess/pre_post_process.hpp>
#else
#include <openvino/preprocess/pre_post_process.hpp>
#endif
#include <string>
#include <vector>

#include "armor.hpp"
#include "classifier.hpp"
#include "config.hpp"
#include "yolo.hpp"

namespace app::auto_aim {

class YOLOV8 : public YOLOBase {
  public:
    YOLOV8(const std::string& config_path, bool debug);
    YOLOV8(const YoloConfig& config, const ClassifierConfig& classifier, bool debug);
    YOLOV8(const YOLOV8&) = delete;
    YOLOV8& operator=(const YOLOV8&) = delete;

    std::list<Armor> detect(const cv::Mat& bgr_img, int frame_count) override;

    std::list<Armor> postprocess(double scale, const cv::Mat& output, const cv::Mat& bgr_img,
                                 int frame_count) override;

  private:
    explicit YOLOV8(const VisionConfig& config, bool debug);
    Classifier classifier_;

    std::string device_, model_path_;
    bool debug_, use_roi_;

    static constexpr std::size_t network_width_ = 640;
    static constexpr std::size_t network_height_ = 480;

    const int class_num_ = 2;
    const float nms_threshold_ = 0.3;
    const float score_threshold_ = 0.7;
    double min_confidence_;

    ov::Core core_;
    ov::CompiledModel compiled_model_;

    // 原始配置永久保留；每帧根据当前分辨率计算裁剪范围。
    cv::Rect configured_roi_;
    cv::Mat input_image_;
    ov::Tensor input_tensor_;
    // 一个 YOLOV8 实例只由一个检测线程使用，输入缓冲区和推理请求可复用。
    ov::InferRequest infer_request_;

    bool check_name(const Armor& armor) const;
    cv::Mat get_pattern(const cv::Mat& bgr_img, const Armor& armor) const;
    cv::Point2f get_center_norm(const cv::Mat& bgr_img, const cv::Point2f& center) const;
    std::list<Armor> parse(double scale, const cv::Mat& output, const cv::Mat& bgr_img,
                           int frame_count, cv::Point2f offset);
    void draw_detections(const cv::Mat& img, const std::list<Armor>& armors, int frame_count) const;
};

} // namespace app::auto_aim

#endif // TGU_ROBOCORE_2027_YOLOV8_HPP
