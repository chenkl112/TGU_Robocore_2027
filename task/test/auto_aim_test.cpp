#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <format>
#include <fstream>
#include <iomanip>
#include <locale>
#include <memory>
#include <opencv2/opencv.hpp>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <unistd.h>
#include <vector>

#include "app/auto_aim/config.hpp"
#include "app/auto_aim/pipeline.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace {
constexpr const char* MODULE = "AUTO_AIM_TEST";
using namespace app::auto_aim;

const std::string keys =
    "{help h usage ? |                       | 输出命令行参数说明 }"
    "{config-path c  | config/auto_aim.toml  | TOML 配置文件路径 }"
    "{start-index s  | 0                     | 视频起始帧下标 }"
    "{end-index e    | 0                     | 视频结束帧下标，包含该帧；0 表示读到末尾 }"
    "{headless       | false                 | 不创建显示窗口 }"
    "{output         |                       | 每帧结果 CSV 文件路径 }"
    "{device         |                       | 覆盖推理设备，例如 CPU }"
    "{bullet-speed b | 23.0                  | 弹速，单位 m/s }"
    "{@input-path    | assets/demo/demo      | avi 和 txt 文件的共同路径前缀 }";

struct PoseRecord {
    double seconds;
    Eigen::Quaterniond orientation;
};

PoseRecord read_pose(std::istream& input, int frame) {
    std::string line;
    double seconds, w, x, y, z;
    if (!std::getline(input, line)) {
        throw std::runtime_error(std::format("Pose stream ended before frame {}", frame));
    }
    std::istringstream record(line);
    record.imbue(std::locale::classic());
    if (!(record >> seconds >> w >> x >> y >> z) || !(record >> std::ws).eof()) {
        throw std::runtime_error(std::format("Invalid pose record at frame {}", frame));
    }
    if (!std::isfinite(seconds) || seconds < 0 || seconds > 1e9) {
        throw std::runtime_error(std::format("Invalid recording timestamp at frame {}", frame));
    }
    return {seconds, {w, x, y, z}};
}

const char* state_name(TrackerState state) {
    switch (state) {
    case TrackerState::lost:
        return "lost";
    case TrackerState::detecting:
        return "detecting";
    case TrackerState::tracking:
        return "tracking";
    case TrackerState::temporarily_lost:
        return "temp_lost";
    }
    return "unknown";
}

void validate_output_path(const std::filesystem::path& output, const std::string& recording,
                          const std::string& config_path, const AutoAimConfig& config) {
    if (output.empty())
        return;
    std::vector<std::filesystem::path> inputs{recording + ".avi", recording + ".txt", config_path,
                                              config.yolo.model_path, config.classifier.model_path};
    auto weights = std::filesystem::path(config.yolo.model_path);
    if (weights.extension() == ".xml")
        inputs.push_back(weights.replace_extension(".bin"));
    const auto destination = std::filesystem::weakly_canonical(output);
    for (const auto& input : inputs) {
        // canonical 处理符号链接；equivalent 识别名字不同但共享 inode 的硬链接。
        // 文件系统访问异常直接向外传播，不能在无法确定输入身份时覆盖文件。
        const bool same_path = destination == std::filesystem::weakly_canonical(input);
        const bool same_file = std::filesystem::exists(output) && std::filesystem::exists(input) &&
                               std::filesystem::equivalent(output, input);
        if (same_path || same_file) {
            throw std::invalid_argument("Replay CSV must not overwrite an input file");
        }
    }
}

class ReplayCsv {
  public:
    explicit ReplayCsv(const std::filesystem::path& destination) : destination_(destination) {
        // 先写同目录临时文件；异常退出保留已有结果，成功后原子替换输出路径。
        auto pattern = destination.string() + ".tmp.XXXXXX";
        std::vector<char> name(pattern.begin(), pattern.end());
        name.push_back('\0');
        const int descriptor = ::mkstemp(name.data());
        if (descriptor < 0)
            throw std::system_error(errno, std::generic_category(), "Cannot create replay CSV");
        ::close(descriptor);
        pending_ = name.data();
        output_.open(pending_);
        if (!output_) {
            std::error_code ignored;
            std::filesystem::remove(pending_, ignored);
            throw std::runtime_error("Cannot open temporary replay CSV");
        }
        output_.imbue(std::locale::classic());
        output_ << std::setprecision(17);
    }

    ~ReplayCsv() {
        if (pending_.empty())
            return;
        output_.close();
        std::error_code ignored;
        std::filesystem::remove(pending_, ignored);
    }

    std::ostream& stream() {
        return output_;
    }

    void finish() {
        output_.flush();
        if (!output_)
            throw std::runtime_error("Could not finish replay CSV");
        output_.close();
        if (output_.fail())
            throw std::runtime_error("Could not close replay CSV");
        std::filesystem::rename(pending_, destination_);
        pending_.clear();
    }

  private:
    std::filesystem::path destination_, pending_;
    std::ofstream output_;
};

void write_csv_header(std::ostream& output) {
    output << "frame,time_seconds,detections,tracker_state,target_present,target_name,armor_count,"
              "observed,"
              "state_time_seconds,observation_time_seconds,x,vx,y,vy,z,vz,yaw_state,yaw_rate,"
              "radius,radius_difference,height_difference,control,fire,target_yaw,target_pitch,"
              "yaw,yaw_vel,yaw_acc,pitch,pitch_vel,pitch_acc,stop_reason\n";
}

void write_csv_row(std::ostream& output, int frame, double seconds, const PipelineResult& result) {
    const auto& target = result.target;
    output << frame << ',' << seconds << ',' << result.detection_count << ','
           << state_name(result.tracker_state) << ',' << bool(target) << ','
           << (target ? ARMOR_NAMES.at(target->name) : "none") << ','
           << (target ? target->armor_count : 0) << ',' << (target && target->observed) << ','
           << (target ? target_time::seconds_between(target->state_time, TargetClock::time_point{})
                      : 0)
           << ','
           << (target ? target_time::seconds_between(target->observation_time,
                                                     TargetClock::time_point{})
                      : 0);
    for (int index = 0; index < target_state::Count; ++index) {
        output << ',' << (target ? target->state[index] : 0);
    }
    const auto& plan = result.plan;
    output << ',' << plan.control << ',' << plan.fire << ',' << plan.target_yaw << ','
           << plan.target_pitch << ',' << plan.yaw << ',' << plan.yaw_vel << ',' << plan.yaw_acc
           << ',' << plan.pitch << ',' << plan.pitch_vel << ',' << plan.pitch_acc << ','
           << stop_reason_name(plan.reason) << '\n';
    if (!output)
        throw std::runtime_error("Could not write replay CSV");
}

int run(int argc, char* argv[]) {
    cv::CommandLineParser cli(argc, argv, keys);
    if (cli.has("help")) {
        cli.printMessage();
        return 0;
    }
    const auto input_path = cli.get<std::string>(0);
    const auto config_path = cli.get<std::string>("config-path");
    const int start_index = cli.get<int>("start-index");
    const int end_index = cli.get<int>("end-index");
    const bool headless = cli.get<bool>("headless");
    const auto output_path = cli.get<std::string>("output");
    const auto device = cli.get<std::string>("device");
    const double bullet_speed = cli.get<double>("bullet-speed");
    if (!cli.check() || start_index < 0 || end_index < 0 ||
        (end_index > 0 && end_index < start_index) || !std::isfinite(bullet_speed) ||
        bullet_speed <= 0) {
        throw std::invalid_argument("Invalid replay options or frame range");
    }

    cv::VideoCapture video(input_path + ".avi");
    std::ifstream poses(input_path + ".txt");
    if (!video.isOpened())
        throw std::runtime_error("Cannot open video: " + input_path + ".avi");
    if (!poses)
        throw std::runtime_error("Cannot open pose stream: " + input_path + ".txt");

    auto config = load_auto_aim_config(config_path);
    if (!device.empty())
        config.yolo.device = device;
    validate_output_path(output_path, input_path, config_path, config);

    if (start_index > 0 && !video.set(cv::CAP_PROP_POS_FRAMES, start_index)) {
        throw std::runtime_error(std::format("Cannot seek to frame {}", start_index));
    }
    // 视频跳帧时同步消费位姿记录；未处理帧的四元数不参与求解。
    for (int frame = 0; frame < start_index; ++frame)
        read_pose(poses, frame);

    // 模型加载成功后才创建输出；错误模型不能破坏已有 CSV 或配置引用的资源。
    AimPipeline pipeline(config);
    std::unique_ptr<ReplayCsv> output;
    if (!output_path.empty()) {
        output = std::make_unique<ReplayCsv>(output_path);
        write_csv_header(output->stream());
    }
    int processed = 0;
    for (int frame = start_index; end_index == 0 || frame <= end_index; ++frame) {
        cv::Mat image;
        if (!video.read(image) || image.empty()) {
            if (processed == 0)
                throw std::runtime_error("Replay contains no frame in the requested range");
            // 自然读到视频末尾时，位姿不应还有额外记录；显式结束帧不检查后续记录。
            if (!(poses >> std::ws).eof())
                throw std::runtime_error("Pose stream contains more records than video frames");
            break;
        }
        const auto pose = read_pose(poses, frame);
        // 固定录包时钟，不依赖实际执行速度；每个帧只推进一次跟踪和 MPC 热启动。
        const auto timestamp = target_time::add_seconds(TargetClock::time_point{}, pose.seconds);
        const auto result = pipeline.step(image, pose.orientation, timestamp, bullet_speed);
        if (output)
            write_csv_row(output->stream(), frame, pose.seconds, result);
        ++processed;
        LOG_INFO(MODULE, "frame={} armors={} state={} control={} reason={}", frame,
                 result.detection_count, state_name(result.tracker_state), result.plan.control,
                 stop_reason_name(result.plan.reason));
        if (!headless) {
            auto display = image.clone();
            tools::draw_text(display,
                             std::format("[{}] {}", frame, state_name(result.tracker_state)),
                             {10, 30}, {255, 255, 255});
            cv::imshow("auto_aim_test", display);
            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q')
                break;
        }
    }
    if (output)
        output->finish();
    return 0;
}
} // namespace

int main(int argc, char* argv[]) {
    try {
        return run(argc, argv);
    } catch (const std::exception& error) {
        LOG_ERROR(MODULE, "Replay failed: {}", error.what());
        return 1;
    }
}
