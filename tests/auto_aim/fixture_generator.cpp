#include <opencv2/opencv.hpp>
#include <fstream>
#include <stdexcept>
int main(int argc, char** argv) {
    if (argc != 2) return 1;
    const std::string prefix = argv[1];
    cv::VideoWriter video(prefix + ".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 100, {160, 120});
    if (!video.isOpened()) return 1;
    std::ofstream poses(prefix + ".txt");
    for (int i = 0; i < 400; ++i) {
        video.write(cv::Mat::zeros(120, 160, CV_8UC3));
        // Invalid poses before the seek point detect text/video misalignment.
        poses << (3600.0 + i * 0.01) << (i < 10 ? " 0 0 0 0\n" : " 1 0 0 0\n");
    }
    return poses.good() ? 0 : 1;
}
