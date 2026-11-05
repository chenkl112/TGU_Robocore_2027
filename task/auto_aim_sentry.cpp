#include <atomic>
#include <csignal>
#include <exception>
#include <opencv2/core.hpp>

#include "app/auto_aim/config.hpp"
#include "auto_aim_runtime.hpp"
#include "tools/logger.hpp"

namespace {
constexpr const char* MODULE = "AUTO_AIM_SENTRY";
constexpr const char* KEYS = "{help h usage ? |                      | 输出命令行参数说明 }"
                             "{@config-path  | config/auto_aim.toml | TOML 配置文件路径 }";

static_assert(std::atomic<bool>::is_always_lock_free);
std::atomic<bool> interrupted{false};

// 信号处理函数只写无锁原子值，停止线程和发送停控由正常执行路径完成。
void stop_signal(int) {
    interrupted.store(true, std::memory_order_relaxed);
}
} // namespace

int main(int argc, char* argv[]) {
    std::signal(SIGINT, stop_signal);
    std::signal(SIGTERM, stop_signal);
    try {
        cv::CommandLineParser cli(argc, argv, KEYS);
        if (cli.has("help")) {
            cli.printMessage();
            return 0;
        }
        const auto path = cli.get<std::string>("@config-path");
        if (!cli.check()) {
            cli.printErrors();
            return 1;
        }
        const auto config = app::auto_aim::load_auto_aim_config(path);
        task::AutoAimRuntime runtime(config);
        return runtime.run(interrupted);
    } catch (const std::exception& error) {
        LOG_ERROR(MODULE, "Fatal error: {}", error.what());
        return 1;
    }
}
