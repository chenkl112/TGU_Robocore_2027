//
// Created by tgu on 2026/4/14.
//

#ifndef TGU_ROBOCORE_2027_LOGGER_HPP
#define TGU_ROBOCORE_2027_LOGGER_HPP

#pragma once

#include <fstream>
#include <mutex>
#include <string>

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


#define LOG_DEBUG(module, msg) \
::tools::Logger::instance().log(::tools::LogLevel::Debug, module, msg, __FILE__, __LINE__)

#define LOG_INFO(module, msg) \
::tools::Logger::instance().log(::tools::LogLevel::Info, module, msg, __FILE__, __LINE__)

#define LOG_WARN(module, msg) \
::tools::Logger::instance().log(::tools::LogLevel::Warn, module, msg, __FILE__, __LINE__)

#define LOG_ERROR(module, msg) \
::tools::Logger::instance().log(::tools::LogLevel::Error, module, msg, __FILE__, __LINE__)

#endif //TGU_ROBOCORE_2027_LOGGER_HPP
