#include "tools/logger.hpp"

#include <array>
#include <barrier>
#include <filesystem>
#include <iostream>
#include <iterator>
#include <random>
#include <regex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <thread>
#include <vector>

namespace {
namespace fs = std::filesystem;
using tools::LogLevel;
using tools::Logger;
using tools::LoggerConfig;

void require(bool condition, const std::string &message) {
    // Unlike assert(), these checks also run in Release builds.
    if (!condition) throw std::runtime_error(message);
}

void stop_logging() {
    Logger::instance().init({.level = LogLevel::Off,
                             .enable_console = false,
                             .enable_file = false});
}

class TempLogs {
public:
    TempLogs() {
        std::random_device random;
        for (int attempt = 0; attempt < 100; ++attempt) {
            auto candidate = fs::temp_directory_path() /
                ("robocore-logger-" + std::to_string(random()) + "-" + std::to_string(random()));
            if (fs::create_directory(candidate)) {
                directory_ = std::move(candidate);
                return;
            }
        }
        throw std::runtime_error("Cannot create a unique temporary log directory");
    }

    ~TempLogs() {
        stop_logging();
        std::error_code ignored;
        fs::remove_all(directory_, ignored);
    }

    fs::path path(std::string_view name) const { return directory_ / name; }

    LoggerConfig config(std::string_view name) const {
        return {.level = LogLevel::Debug,
                .enable_console = false,
                .enable_file = true,
                .file_path = path(name).string()};
    }

private:
    fs::path directory_;
};

// Only installed while no worker threads exist; workers never redirect streams.
class CaptureConsole {
public:
    CaptureConsole() : old_out_(std::cout.rdbuf(out.rdbuf())),
                       old_err_(std::cerr.rdbuf(err.rdbuf())) {}
    ~CaptureConsole() {
        std::cout.rdbuf(old_out_);
        std::cerr.rdbuf(old_err_);
    }

    std::ostringstream out;
    std::ostringstream err;

private:
    std::streambuf *old_out_;
    std::streambuf *old_err_;
};

std::string read_file(const fs::path &path) {
    std::ifstream input(path, std::ios::binary);
    require(input.is_open(), "Cannot read " + path.string());
    std::string contents{std::istreambuf_iterator<char>(input), {}};
    require(!input.bad(), "Read failed for " + path.string());
    return contents;
}

std::vector<std::string> messages(const std::string &contents) {
    require(contents.empty() || contents.back() == '\n', "Incomplete final log line");
    static const std::regex line_pattern(
        R"(^\[[0-9]{2}:[0-9]{2}:[0-9]{2}\] \[(DEBUG|INFO |WARN |ERROR)\] \[INFRA\] \[(fixture\.cpp:42|logger_tests\.cpp:[0-9]+)\] (.*)$)");
    std::vector<std::string> result;
    std::istringstream lines(contents);
    std::string line;
    while (std::getline(lines, line)) {
        // Text-mode output on Windows uses CRLF.
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::smatch match;
        require(std::regex_match(line, match, line_pattern), "Malformed log line: " + line);
        result.push_back(match[3].str());
    }
    return result;
}

void expect_file(const fs::path &path, const std::vector<std::string> &expected) {
    require(messages(read_file(path)) == expected, "Unexpected records in " + path.string());
}

void emit(const std::string &message, LogLevel level = LogLevel::Info) {
    Logger::instance().log(level, "INFRA", message, "/virtual/path/fixture.cpp", 42);
}

void reinitialize() {
    TempLogs logs;
    const auto config = logs.config("same.log");
    Logger::instance().init(config);
    emit("before");
    Logger::instance().init(config);
    emit("after");
    Logger::instance().init(config);
    emit("again");
    stop_logging();
    expect_file(logs.path("same.log"), {"before", "after", "again"});
}

void switch_file() {
    TempLogs logs;
    Logger::instance().init(logs.config("a.log"));
    emit("only-a");
    Logger::instance().init(logs.config("b.log"));
    // Switching must flush and close A immediately.
    expect_file(logs.path("a.log"), {"only-a"});
    emit("only-b");
    stop_logging();
    expect_file(logs.path("a.log"), {"only-a"});
    expect_file(logs.path("b.log"), {"only-b"});
}

void toggle_file() {
    TempLogs logs;
    auto config = logs.config("toggle.log");
    Logger::instance().init(config);
    emit("enabled");
    config.enable_file = false;
    Logger::instance().init(config);
    expect_file(logs.path("toggle.log"), {"enabled"});
    const auto before = read_file(logs.path("toggle.log"));
    emit("disabled");
    require(read_file(logs.path("toggle.log")) == before, "Disabled log file changed");
    config.enable_file = true;
    Logger::instance().init(config);
    emit("reenabled");
    stop_logging();
    expect_file(logs.path("toggle.log"), {"enabled", "reenabled"});
}

void invalid_path() {
    TempLogs logs;
    CaptureConsole console;
    Logger::instance().init(logs.config("old.log"));
    emit("before-failure");
    auto invalid = logs.config("missing-parent/invalid.log");
    invalid.enable_console = true;
    Logger::instance().init(invalid);
    const auto diagnostic = console.err.str();
    const auto prefix = "Logger: cannot open log file '" + invalid.file_path + "': ";
    require(diagnostic.starts_with(prefix) && diagnostic.size() > prefix.size() + 1,
            "Missing file-open diagnostic or reason on stderr");
    emit("console-still-works");
    require(messages(console.out.str()) == std::vector<std::string>{"console-still-works"},
            "Console logging failed after invalid file path");
    expect_file(logs.path("old.log"), {"before-failure"});
    require(!fs::exists(logs.path("missing-parent/invalid.log")), "Invalid file unexpectedly exists");
    Logger::instance().init(logs.config("recovered.log"));
    emit("recovered");
    stop_logging();
    expect_file(logs.path("old.log"), {"before-failure"});
    expect_file(logs.path("recovered.log"), {"recovered"});
}

void levels() {
    TempLogs logs;
    constexpr std::array levels{LogLevel::Debug, LogLevel::Info, LogLevel::Warn,
                                LogLevel::Error, LogLevel::Off};
    constexpr std::array names{"DEBUG", "INFO ", "WARN ", "ERROR"};
    for (std::size_t threshold = 0; threshold < levels.size(); ++threshold) {
        CaptureConsole console;
        const auto name = std::to_string(threshold) + ".log";
        auto config = logs.config(name);
        config.level = levels[threshold];
        config.enable_console = true;
        Logger::instance().init(config);
        std::vector<std::string> expected, expected_out, expected_err;
        for (std::size_t severity = 0; severity < names.size(); ++severity) {
            const auto message = "level-" + std::to_string(severity);
            emit(message, levels[severity]);
            if (severity >= threshold) {
                expected.push_back(message);
                (severity < 2 ? expected_out : expected_err).push_back(message);
            }
        }
        stop_logging();
        expect_file(logs.path(name), expected);
        require(messages(console.out.str()) == expected_out, "Incorrect stdout filtering");
        require(messages(console.err.str()) == expected_err, "Incorrect stderr filtering");
        const auto content = read_file(logs.path(name));
        for (std::size_t severity = threshold; severity < names.size(); ++severity) {
            require(content.find("[" + std::string(names[severity]) + "]") != std::string::npos,
                    "Incorrect severity label");
        }
    }
}

void concurrent() {
    TempLogs logs;
    const auto config = logs.config("concurrent.log");
    Logger::instance().init(config);
    constexpr int writers = 6;
    constexpr int rounds = 150;
    constexpr int per_round = 3;
    std::barrier phase(writers + 1);
    {
        std::vector<std::jthread> threads;
        for (int writer = 0; writer < writers; ++writer) {
            threads.emplace_back([&, writer] {
                for (int round = 0; round < rounds; ++round) {
                    phase.arrive_and_wait();
                    for (int record = 0; record < per_round; ++record) {
                        emit(std::format("writer-{}-record-{}", writer, round * per_round + record));
                    }
                    phase.arrive_and_wait();
                }
            });
        }
        threads.emplace_back([&] {
            auto updated = config;
            for (int round = 0; round < rounds; ++round) {
                phase.arrive_and_wait();
                // Both thresholds accept every INFO record; changing the threshold
                // also exercises synchronization of configuration reads and writes.
                updated.level = round % 2 ? LogLevel::Debug : LogLevel::Info;
                Logger::instance().init(updated);
                phase.arrive_and_wait();
            }
        });
    } // Join every worker before closing or reading the file.
    stop_logging();
    const auto records = messages(read_file(logs.path("concurrent.log")));
    require(records.size() == writers * rounds * per_round, "Missing or extra concurrent records");
    std::set<std::string> remaining;
    for (int writer = 0; writer < writers; ++writer) {
        for (int record = 0; record < rounds * per_round; ++record) {
            remaining.insert(std::format("writer-{}-record-{}", writer, record));
        }
    }
    for (const auto &record : records) {
        require(remaining.erase(record) == 1, "Duplicate or unexpected record: " + record);
    }
    require(remaining.empty(), "Concurrent records are missing");
}

void debug_macro() {
    TempLogs logs;
    Logger::instance().init(logs.config("macros.log"));
    int evaluated = 0;
    LOG_DEBUG("INFRA", "debug-{}", ++evaluated);
    LOG_INFO("INFRA", "info-{}", 42);
    LOG_WARN("INFRA", "warn");
    LOG_ERROR("INFRA", "error");
    stop_logging();
#ifdef NDEBUG
    require(evaluated == 0, "Release LOG_DEBUG evaluated its arguments");
    expect_file(logs.path("macros.log"), {"info-42", "warn", "error"});
#else
    require(evaluated == 1, "Debug LOG_DEBUG did not evaluate its arguments once");
    expect_file(logs.path("macros.log"), {"debug-1", "info-42", "warn", "error"});
#endif
}
} // namespace

int main(int argc, char **argv) {
    try {
        require(argc == 2, "Usage: logger_tests <scenario>");
        const std::string_view scenario(argv[1]);
        if (scenario == "reinitialize") reinitialize();
        else if (scenario == "switch_file") switch_file();
        else if (scenario == "toggle_file") toggle_file();
        else if (scenario == "invalid_path") invalid_path();
        else if (scenario == "levels") levels();
        else if (scenario == "concurrent") concurrent();
        else if (scenario == "debug_macro") debug_macro();
        else throw std::runtime_error("Unknown scenario: " + std::string(scenario));
        std::cout << "PASS " << scenario << '\n';
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
