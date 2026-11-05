#ifndef TGU_ROBOCORE_2027_CLASSIFIER_HPP
#define TGU_ROBOCORE_2027_CLASSIFIER_HPP
#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"
#include "config.hpp"

namespace app::auto_aim {

class Classifier {
  public:
    explicit Classifier(const std::string& config_path);
    explicit Classifier(const ClassifierConfig& config);
    Classifier(const Classifier&) = delete;
    Classifier& operator=(const Classifier&) = delete;

    void classify(Armor& armor);

  private:
    cv::dnn::Net net_;
    cv::Mat input_ = cv::Mat::zeros(32, 32, CV_8UC1);
    cv::Mat blob_;
};

} // namespace app::auto_aim

#endif // TGU_ROBOCORE_2027_CLASSIFIER_HPP
