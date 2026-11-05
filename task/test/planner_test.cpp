#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>

#include "app/auto_aim/config.hpp"
#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/target.hpp"
#include "tools/logger.hpp"

namespace {
constexpr const char* MODULE = "PLANNER_TEST";

const std::string keys = "{help h usage ? |                       | 输出命令行参数说明 }"
                         "{config-path c  | config/auto_aim.toml  | TOML 配置文件路径 }"
                         "{distance d     | 3.0                   | 目标中心距离，单位 m }"
                         "{angular-speed w| 5.0                   | 目标角速度，单位 rad/s }"
                         "{bullet-speed b | 22.0                  | 弹速，单位 m/s }";

int run(int argc, char* argv[]) {
    using namespace app::auto_aim;
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    const auto config_path = cli.get<std::string>("config-path");
    const double distance = cli.get<double>("distance");
    const double angular_speed = cli.get<double>("angular-speed");
    const double bullet_speed = cli.get<double>("bullet-speed");
    if (!cli.check() || !std::isfinite(distance) || distance <= 0 ||
        !std::isfinite(angular_speed) || !std::isfinite(bullet_speed) || bullet_speed <= 0) {
        throw std::invalid_argument("Invalid planner self-check options");
    }

    Planner planner(load_planner_config(config_path));
    TargetSnapshot target;
    target.state[target_state::X] = distance;
    target.state[target_state::YawRate] = angular_speed;
    target.state[target_state::Radius] = 0.2;
    target.state[target_state::HeightDifference] = 0.1;
    target.armor_count = 4;
    target.name = ArmorName::three;
    target.armor_type = ArmorType::small;
    target.observed = true;
    target.state_time = target.observation_time = TargetClock::time_point{};

    // 自检显式提供模拟观测和时钟。允许有限次热启动，但停控不能被当作自检成功。
    constexpr int max_attempts = 10;
    Plan plan;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
        plan = planner.plan(target, bullet_speed, target.state_time);
        if (plan.control)
            break;
    }
    const std::array outputs{plan.yaw,   plan.yaw_vel,   plan.yaw_acc,
                             plan.pitch, plan.pitch_vel, plan.pitch_acc};
    const bool finite = std::all_of(outputs.begin(), outputs.end(),
                                    [](float value) { return std::isfinite(value); });
    LOG_INFO(MODULE,
             "control={} fire={} yaw={:.6f} yaw_vel={:.6f} yaw_acc={:.6f} "
             "pitch={:.6f} pitch_vel={:.6f} pitch_acc={:.6f} reason={}",
             plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
             plan.pitch_vel, plan.pitch_acc, stop_reason_name(plan.reason));
    if (!plan.control || !finite) {
        LOG_ERROR(MODULE, "MPC self-check failed after {} attempts", max_attempts);
        return 1;
    }
    return 0;
}
} // namespace

int main(int argc, char* argv[]) {
    try {
        return run(argc, argv);
    } catch (const std::exception& error) {
        LOG_ERROR(MODULE, "Planner self-check failed: {}", error.what());
        return 1;
    }
}
