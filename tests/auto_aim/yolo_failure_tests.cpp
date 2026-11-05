#include "app/auto_aim/multithread/mt_yolo.hpp"
#include "tools/logger.hpp"
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
using namespace std::chrono_literals;
int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        if (argc != 2)
            throw std::runtime_error("Expected config path");
        app::auto_aim::MultiThreadYOLO worker(argv[1]);
        const auto stamp = std::chrono::steady_clock::now();
        worker.push(cv::Mat::zeros(480, 640, CV_8UC3), stamp);
        const auto frame = worker.pop();
        if (frame.timestamp != stamp)
            throw std::runtime_error("Inference changed the frame timestamp");
        auto consumer = std::async(std::launch::async, [&] { return worker.pop(); });
        worker.push(cv::Mat::zeros(10, 10, CV_8UC1), std::chrono::steady_clock::now());
        if (consumer.wait_for(3s) != std::future_status::ready) {
            throw std::runtime_error("Failed worker did not wake the consumer");
        }
        try {
            consumer.get();
        } catch (const std::invalid_argument&) {
            std::cout << "PASS worker failure forwarded to consumer\n";
            return 0;
        }
        throw std::runtime_error("Inference failure was lost");
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
