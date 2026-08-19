#include <algorithm>
#include <array>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <string>

#include "app/auto_aim/planner/planner.hpp"
#include "app/auto_aim/target.hpp"
#include "tools/logger.hpp"

static constexpr const char* MODULE = "PLANNER_TEST";

const std::string keys =
    "{help h usage ? |                       | 输出命令行参数说明 }"
    "{config-path c  | config/auto_aim.toml  | TOML 配置文件路径 }"
    "{distance d     | 3.0                   | 目标距离，单位 m }"
    "{angular-speed w| 5.0                   | 目标角速度，单位 rad/s }"
    "{bullet-speed b | 22.0                  | 弹速，单位 m/s }";

int main(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }

    const auto config_path = cli.get<std::string>("config-path");
    const auto distance = cli.get<double>("distance");
    const auto angular_speed = cli.get<double>("angular-speed");
    const auto bullet_speed = cli.get<double>("bullet-speed");

    app::auto_aim::Planner planner(config_path);
    app::auto_aim::Target target(distance, angular_speed, 0.2, 0.1);
    const auto plan = planner.plan(target, bullet_speed);

    const std::array outputs{
        plan.yaw, plan.yaw_vel, plan.yaw_acc,
        plan.pitch, plan.pitch_vel, plan.pitch_acc,
    };
    const bool finite = std::all_of(
        outputs.begin(), outputs.end(), [](float value) { return std::isfinite(value); });

    LOG_INFO(
        MODULE,
        "control={} fire={} yaw={:.6f} yaw_vel={:.6f} yaw_acc={:.6f} "
        "pitch={:.6f} pitch_vel={:.6f} pitch_acc={:.6f}",
        plan.control, plan.fire, plan.yaw, plan.yaw_vel, plan.yaw_acc, plan.pitch,
        plan.pitch_vel, plan.pitch_acc);

    if (!plan.control || !finite) {
        LOG_ERROR(MODULE, "MPC failed to produce a finite control plan");
        return 1;
    }
    return 0;
}
