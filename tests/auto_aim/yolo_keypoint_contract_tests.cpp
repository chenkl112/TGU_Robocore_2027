#include "app/auto_aim/yolos/keypoints.hpp"
#include <openvino/openvino.hpp>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}

int main(int argc, char** argv) {
    try {
        require(argc == 2, "Expected the bundled YOLO model path");
        ov::Core core;
        auto model = core.compile_model(argv[1], "CPU");
        cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC3);
        cv::rectangle(image, {280, 223}, {360, 257}, {30, 30, 30}, -1);
        for (int x : {280, 360}) {
            cv::rectangle(image, {x - 4, 223}, {x + 4, 257}, {0, 0, 255}, -1);
            cv::rectangle(image, {x - 2, 225}, {x + 2, 255}, {255, 255, 255}, -1);
        }
        cv::putText(image, "3", {302, 256}, cv::FONT_HERSHEY_SIMPLEX, 1.4, {255,255,255}, 3);
        const std::vector<cv::Point2f> physical{{280,223}, {280,257}, {360,257}, {360,223}};
        for (double roll : {-30., -20., 0., 20., 30.}) {
            const auto rotation = cv::getRotationMatrix2D({320,240}, roll, 1);
            cv::Mat rotated;
            cv::warpAffine(image, rotated, rotation, image.size());
            std::vector<cv::Point2f> expected;
            cv::transform(physical, expected, rotation);
            auto blob = cv::dnn::blobFromImage(rotated, 1.0 / 255, {}, {}, true);
            auto request = model.create_infer_request();
            request.set_input_tensor(ov::Tensor(ov::element::f32, {1,3,480,640}, blob.ptr<float>()));
            request.infer();
            const auto output = request.get_output_tensor();
            require(output.get_shape() == ov::Shape({1,14,6300}), "Unexpected model contract");
            const auto* data = output.data<const float>();
            constexpr std::size_t count = 6300;
            std::size_t best = 0;
            float score = 0;
            for (std::size_t i = 0; i < count; ++i) {
                const float candidate = std::max(data[4 * count + i], data[5 * count + i]);
                if (candidate > score) { score = candidate; best = i; }
            }
            require(score > 0.8f, "Synthetic armor did not produce a confident detection");
            std::vector<cv::Point2f> corners;
            for (int channel = 0; channel < 4; ++channel) {
                corners.emplace_back(data[(6 + 2 * channel) * count + best],
                                     data[(7 + 2 * channel) * count + best]);
                int nearest = -1;
                double distance = std::numeric_limits<double>::infinity();
                for (int id = 0; id < 4; ++id) {
                    const double d = cv::norm(corners.back() - expected[id]);
                    if (d < distance) { nearest = id; distance = d; }
                }
                require(nearest == channel && distance < 13,
                        "Bundled model no longer emits LT, LB, RB, RT channels");
            }
            require(app::auto_aim::decode_yolov8_keypoints(corners), "Model corners were rejected");
            const std::array<int,4> pnp_order{0,3,2,1};
            for (int i = 0; i < 4; ++i) {
                require(cv::norm(corners[i] - expected[pnp_order[i]]) < 13,
                        "Decoder does not preserve the bundled model's physical corner identities");
            }
            std::cout << "roll=" << roll << " confidence=" << score << " semantic channels verified\n";
        }
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
