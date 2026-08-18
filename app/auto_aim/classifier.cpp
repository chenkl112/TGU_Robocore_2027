#include "classifier.hpp"

#include <algorithm>

#include "tools/tomlpp.hpp"

static constexpr const char* MODULE = "AUTO_AIM";

namespace app::auto_aim {

Classifier::Classifier(const std::string& config_path) {
    auto config = toml::parse_file(config_path);
    auto model = config["classifier"]["classify_model"].value_or("config/armor_classifier.onnx");

    net_ = cv::dnn::readNetFromONNX(model);
}

void Classifier::classify(Armor& armor) {
    if (armor.pattern.empty()) {
        armor.name = ArmorName::not_armor;
        return;
    }

    cv::Mat gray;
    cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

    auto input = cv::Mat(32, 32, CV_8UC1, cv::Scalar(0));
    auto x_scale = static_cast<double>(32) / gray.cols;
    auto y_scale = static_cast<double>(32) / gray.rows;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(gray.rows * scale);
    auto w = static_cast<int>(gray.cols * scale);

    if (h == 0 || w == 0) {
        armor.name = ArmorName::not_armor;
        return;
    }
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(gray, input(roi), {w, h});

    auto blob = cv::dnn::blobFromImage(input, 1.0 / 255.0, cv::Size(), cv::Scalar());

    net_.setInput(blob);
    cv::Mat outputs = net_.forward();

    float max = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::exp(outputs - max, outputs);
    float sum = cv::sum(outputs)[0];
    outputs /= sum;

    double confidence;
    cv::Point label_point;
    cv::minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &label_point);
    int label_id = label_point.x;

    armor.confidence = confidence;
    armor.name = static_cast<ArmorName>(label_id);
}

}  // namespace app::auto_aim
