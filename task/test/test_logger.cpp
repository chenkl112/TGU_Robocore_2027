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

    LOG_INFO("TEST_INFO", "LOG_INFO");
    LOG_DEBUG("TEST_DEBUG", "LOG_DEBUG");
    LOG_WARN("TEST_WARN", "LOG_WARN");
    LOG_ERROR("TEST_ERROR", "LOG_ERROR");

    return 0;
}