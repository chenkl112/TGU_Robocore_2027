//
// Created by tgu on 2026/4/14.
//

#include "tools/logger.hpp"

int main() {
    tools::LoggerConfig cfg{
        .level = tools::LogLevel::Debug,
        .enable_console = true,
        .enable_file = false,
        .file_path = "logs.txt"
    };

    tools::Logger::instance().init(cfg);
    static constexpr const char* MODULE = "TEST";

    int a = 114514;
    float b = 1919.810;

    LOG_INFO(MODULE, "LOG_INFO: {}", a);
    LOG_DEBUG(MODULE, "LOG_DEBUG: {}", a);
    LOG_WARN(MODULE, "LOG_WARN: {}", b);
    LOG_ERROR(MODULE, "LOG_ERROR: {}", b);

    return 0;
}