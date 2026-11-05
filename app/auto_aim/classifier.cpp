#include "classifier.hpp"

#include <algorithm>

#include <stdexcept>

namespace app::auto_aim {

Classifier::Classifier(const std::string& config_path)
    : Classifier(load_classifier_config(config_path)) {}

Classifier::Classifier(const ClassifierConfig& config) {
    validate(config);
    net_ = cv::dnn::readNetFromONNX(config.model_path);
}

void Classifier::classify(Armor& armor) {
    armor.name = ArmorName::not_armor;
    armor.confidence = 0;
    if (armor.pattern.empty())
        return;

    if (armor.pattern.type() != CV_8UC3) {
        throw std::invalid_argument("Classifier expects an 8-bit BGR pattern");
    }
    cv::Mat gray;
    cv::cvtColor(armor.pattern, gray, cv::COLOR_BGR2GRAY);

    input_.setTo(0);
    auto x_scale = static_cast<double>(32) / gray.cols;
    auto y_scale = static_cast<double>(32) / gray.rows;
    auto scale = std::min(x_scale, y_scale);
    auto h = static_cast<int>(gray.rows * scale);
    auto w = static_cast<int>(gray.cols * scale);

    if (h == 0 || w == 0)
        return;
    auto roi = cv::Rect(0, 0, w, h);
    cv::resize(gray, input_(roi), {w, h});

    cv::dnn::blobFromImage(input_, blob_, 1.0 / 255.0, cv::Size(), cv::Scalar());

    net_.setInput(blob_);
    cv::Mat outputs = net_.forward();
    // 类别索引与 ArmorName 固定对应；异常模型输出不能静默转换成非法枚举。
    if (outputs.type() != CV_32F || outputs.total() != ARMOR_NAMES.size() ||
        !cv::checkRange(outputs)) {
        throw std::runtime_error("Classifier output must contain nine finite float32 logits");
    }

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

} // namespace app::auto_aim
