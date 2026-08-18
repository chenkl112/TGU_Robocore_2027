#ifndef TGU_ROBOCORE_2027_CLASSIFIER_HPP
#define TGU_ROBOCORE_2027_CLASSIFIER_HPP
#pragma once

#include <opencv2/opencv.hpp>
#include <string>

#include "armor.hpp"

namespace app::auto_aim {

class Classifier {
public:
    explicit Classifier(const std::string& config_path);

    void classify(Armor& armor);

private:
    cv::dnn::Net net_;
};

}  // namespace app::auto_aim

#endif  // TGU_ROBOCORE_2027_CLASSIFIER_HPP
