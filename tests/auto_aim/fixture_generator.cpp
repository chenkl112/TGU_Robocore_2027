#include <fstream>
#include <iomanip>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdexcept>
#include <string>

#include "vision_fixture.hpp"

int main(int argc, char** argv) {
    try {
        if (argc < 2 || argc > 3)
            throw std::invalid_argument("Expected recording prefix and optional positive mode");
        const std::string prefix = argv[1];
        const bool positive = argc == 3 && std::string(argv[2]) == "positive";
        if (argc == 3 && !positive)
            throw std::invalid_argument("Unknown fixture mode");
        const auto image =
            positive ? vision_fixture::hero_image() : cv::Mat::zeros(120, 160, CV_8UC3);
        // 正例使用无损编码，保证真实模型夹具的像素不被视频压缩改变。
        const int codec = positive ? cv::VideoWriter::fourcc('F', 'F', 'V', '1')
                                   : cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
        cv::VideoWriter video(prefix + ".avi", codec, 100, image.size());
        if (!video.isOpened())
            throw std::runtime_error("Could not open fixture video writer");
        std::ofstream poses(prefix + ".txt");
        if (!poses)
            throw std::runtime_error("Could not open fixture pose writer");
        poses << std::setprecision(17);
        const int frames = positive ? 60 : 400;
        for (int index = 0; index < frames; ++index) {
            video.write(image);
            // 原空白夹具前 10 帧四元数非法，跳帧必须同时跳过位姿记录才能成功。
            const bool valid_pose = positive || index >= 10;
            poses << (3600.0 + index * 0.01) << (valid_pose ? " 1 0 0 0\n" : " 0 0 0 0\n");
        }
        video.release();
        if (!poses.good())
            throw std::runtime_error("Could not finish fixture pose writer");
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Fixture generation failed: " << error.what() << '\n';
        return 1;
    }
}
