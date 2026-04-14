//
// Created by tgu on 2026/4/14.
//

#ifndef TGU_ROBOCORE_2027_LOGGER_HPP
#define TGU_ROBOCORE_2027_LOGGER_HPP

#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include <format>

namespace tools {

    enum class LogLevel {
        Debug = 0,
        Info,
        Warn,
        Error,
        Off
    };

    struct LoggerConfig {
        LogLevel level = LogLevel::Debug;
        bool enable_console = true;
        bool enable_file = false;
        std::string file_path = "log.txt";
    };

    class Logger {
    public:
        static Logger& instance();

        void init(const LoggerConfig& config);

        void log(LogLevel level,
                 const std::string& module,
                 const std::string& msg,
                 const char* file,
                 int line);

    private:
        Logger() = default;
        ~Logger();

        std::string format(LogLevel level,
                           const std::string& module,
                           const std::string& msg,
                           const char* file,
                           int line) const;

    private:
        LogLevel level_ = LogLevel::Debug;
        bool console_ = true;
        bool file_ = false;

        std::ofstream ofs_;
        std::mutex mutex_;
    };

} // namespace tools


#define LOG_INFO(module, fmt, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Info, module, \
std::format(fmt __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)

#ifdef NDEBUG
#define LOG_DEBUG(module, fmt, ...) ((void)0)
#else
#define LOG_DEBUG(module, fmt, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Debug, module, \
std::format(fmt __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)
#endif

#define LOG_WARN(module, fmt, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Warn, module, \
std::format(fmt __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)

#define LOG_ERROR(module, fmt, ...) \
::tools::Logger::instance().log( \
::tools::LogLevel::Error, module, \
std::format(fmt __VA_OPT__(,) __VA_ARGS__), \
__FILE__, __LINE__)

#endif //TGU_ROBOCORE_2027_LOGGER_HPP
