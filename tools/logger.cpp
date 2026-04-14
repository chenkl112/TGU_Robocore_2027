//
// Created by tgu on 2026/4/14.
//

#include "logger.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace tools {

Logger& Logger::instance() {
    static Logger inst;
    return inst;
}

Logger::~Logger() {
    if (ofs_.is_open()) {
        ofs_.close();
    }
}

void Logger::init(const LoggerConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    level_ = config.level;
    console_ = config.enable_console;
    file_ = config.enable_file;

    if (file_) {
        ofs_.open(config.file_path, std::ios::out | std::ios::app);
        if (!ofs_.is_open()) {
            file_ = false;
        }
    }
}

void Logger::log(LogLevel level,
                 const std::string& module,
                 const std::string& msg,
                 const char* file,
                 int line) {
    if (level < level_) return;

    std::lock_guard<std::mutex> lock(mutex_);

    std::string text = format(level, module, msg, file, line);

    if (console_) {
        (level >= LogLevel::Warn ? std::cerr : std::cout) << text << '\n';
    }

    if (file_ && ofs_.is_open()) {
        ofs_ << text << '\n';
    }
}

std::string Logger::format(LogLevel level,
                           const std::string& module,
                           const std::string& msg,
                           const char* file,
                           int line) const {
    using namespace std::chrono;

    auto now = system_clock::now();
    auto t = system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif

    std::ostringstream ss;

    ss << "[" << std::put_time(&tm, "%H:%M:%S") << "] ";

    switch (level) {
        case LogLevel::Debug: ss << "[DEBUG] "; break;
        case LogLevel::Info:  ss << "[INFO ] "; break;
        case LogLevel::Warn:  ss << "[WARN ] "; break;
        case LogLevel::Error: ss << "[ERROR] "; break;
        default: break;
    }

    // 提取文件名
    std::string f(file);
    auto pos = f.find_last_of("/\\");
    if (pos != std::string::npos) f = f.substr(pos + 1);

    ss << "[" << module << "] "
       << "[" << f << ":" << line << "] "
       << msg;

    return ss.str();
}

} // namespace tools