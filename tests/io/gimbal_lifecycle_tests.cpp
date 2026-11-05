#include "io/gimbal/gimbal.hpp"
#include "test_support.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"

#include <atomic>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <thread>

namespace {
using namespace test;

class Config {
  public:
    explicit Config(const std::string& port) {
        auto pattern = (std::filesystem::temp_directory_path() / "robocore-io-XXXXXX").string();
        std::vector<char> name(pattern.begin(), pattern.end());
        name.push_back('\0');
        require(::mkdtemp(name.data()) != nullptr, "Cannot create temporary config directory");
        directory_ = name.data();
        std::ofstream file(path());
        file << "[gimbal]\ncom_port = \"" << port << "\"\nbaudrate = 115200\n";
        require(file.good(), "Cannot write temporary config");
    }
    ~Config() {
        std::error_code ignored;
        std::filesystem::remove_all(directory_, ignored);
    }
    std::string path() const {
        return (directory_ / "test.toml").string();
    }

  private:
    std::filesystem::path directory_;
};

void silent_start() {
    Pty pty; // 对端保持打开，但不发送任何字节。
    Config config(pty.name());
    const auto start = Clock::now();
    bool timed_out = false;
    try {
        io::Gimbal device(config.path());
    } catch (const std::runtime_error& error) {
        timed_out =
            std::string(error.what()).find("first valid gimbal packet") != std::string::npos;
    }
    const auto elapsed = Clock::now() - start;
    require(timed_out, "Silent device did not report the first-packet timeout");
    require(elapsed >= 2900ms && elapsed < 4500ms, "Timeout cleanup blocked beyond its deadline");
    std::cout << "first-packet timeout including cleanup: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms\n";
}

void idle_shutdown() {
    Pty pty;
    Config config(pty.name());
    io::GimbalToVision packet{};
    packet.q[0] = 1.0f;
    packet.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - 2);
    std::atomic<bool> peer_failed{false};
    std::jthread peer([&](std::stop_token stop) {
        while (!stop.stop_requested()) {
            const auto count = ::write(pty.master(), &packet, sizeof(packet));
            if (count < 0 && errno != EAGAIN && errno != EINTR && errno != EIO) {
                peer_failed = true;
                break;
            }
            std::this_thread::sleep_for(10ms);
        }
    });
    auto device = std::make_unique<io::Gimbal>(config.path());
    peer.request_stop();
    peer.join();
    require(!peer_failed, "PTY peer failed during idle startup");
    require(device->mode() == io::GimbalMode::IDLE, "Expected an idle feedback fixture");
    std::this_thread::sleep_for(100ms); // 让接收线程进入无数据的等待状态。
    const auto start = Clock::now();
    device.reset();
    const auto elapsed = Clock::now() - start;
    require(elapsed < 1s, "Idle device destructor did not join the receiver promptly");
    std::cout << "idle shutdown: "
              << std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count() << " ms\n";
}
class FeedbackPeer {
  public:
    Pty pty;
    std::atomic<bool> active{true};
    std::atomic<uint8_t> mode{1};
    std::atomic<float> yaw{0};
    std::jthread sender;
    std::unique_ptr<io::Gimbal> device;

    explicit FeedbackPeer(bool prepend_false_header = false,
                          std::chrono::milliseconds reconnect_delay = 250ms,
                          std::chrono::milliseconds feedback_timeout = 100ms)
        : sender([&, prepend_false_header](std::stop_token stop) {
              while (!stop.stop_requested()) {
                  if (active) {
                      io::GimbalToVision packet{};
                      packet.mode = mode;
                      packet.yaw = yaw;
                      packet.q[0] = 1;
                      packet.bullet_speed = 22;
                      packet.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t*>(&packet),
                                                      sizeof(packet) - 2);
                      if (prepend_false_header) {
                          const auto count = ::write(pty.master(), packet.head, 2);
                          if (count != 2)
                              return;
                      }
                      const auto count = ::write(pty.master(), &packet, sizeof(packet));
                      if (count < 0 && errno != EAGAIN && errno != EINTR)
                          return;
                  }
                  std::this_thread::sleep_for(5ms);
              }
          }) {
        device = std::make_unique<io::Gimbal>(io::GimbalConfig{.port = pty.name(),
                                                               .feedback_timeout = feedback_timeout,
                                                               .reconnect_delay = reconnect_delay});
        std::this_thread::sleep_for(40ms);
    }
    ~FeedbackPeer() {
        sender.request_stop();
        if (sender.joinable())
            sender.join();
        device.reset();
    }
    io::VisionToGimbal command(float yaw = 0, std::optional<uint64_t> session = std::nullopt,
                               float pitch = 0) {
        return command(io::Command{.control = true, .shoot = true, .yaw = yaw, .pitch = pitch},
                       session);
    }
    io::VisionToGimbal command(const io::Command& command,
                               std::optional<uint64_t> session = std::nullopt) {
        device->send(command, session);
        io::VisionToGimbal result{};
        auto* bytes = reinterpret_cast<uint8_t*>(&result);
        size_t count = 0;
        const auto deadline = Clock::now() + 1s;
        while (count < sizeof(result) && Clock::now() < deadline) {
            const auto n = ::read(pty.master(), bytes + count, sizeof(result) - count);
            if (n > 0)
                count += static_cast<size_t>(n);
            else
                std::this_thread::sleep_for(1ms);
        }
        require(count == sizeof(result), "Missing outgoing gimbal command");
        require(tools::check_crc16(bytes, sizeof(result)), "Invalid outgoing CRC");
        return result;
    }
};

// 从另一个从端描述符填满同一 PTY 的发送缓冲区，保留主端到从端的反馈通路。
// 填充字节不包含帧头，之后可直接用生产解析器识别等待中的真实命令。
void fill_output(const Pty& pty) {
    const int fd = ::open(pty.name().c_str(), O_WRONLY | O_NONBLOCK | O_NOCTTY);
    require(fd >= 0, "Cannot open PTY output filler");
    const std::vector<uint8_t> filler(4096, 0);
    auto last_write = Clock::now();
    const auto deadline = last_write + 1s;
    size_t filled = 0;
    while (Clock::now() < deadline && Clock::now() - last_write < 15ms) {
        const auto count = ::write(fd, filler.data(), filler.size());
        if (count > 0) {
            filled += static_cast<size_t>(count);
            last_write = Clock::now();
        } else if (count < 0 && (errno == EAGAIN || errno == EINTR)) {
            std::this_thread::sleep_for(1ms);
        } else {
            ::close(fd);
            throw std::runtime_error("Cannot fill PTY output");
        }
    }
    ::close(fd);
    require(filled > 0 && Clock::now() < deadline, "PTY output did not reach stable backpressure");
}

io::VisionToGimbal drain_command(const Pty& pty) {
    io::StructParser<io::VisionToGimbal> parser([](const auto& packet) {
        return tools::check_crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
    });
    const auto deadline = Clock::now() + 1s;
    while (Clock::now() < deadline) {
        uint8_t bytes[4096];
        const auto count = ::read(pty.master(), bytes, sizeof(bytes));
        if (count > 0) {
            io::VisionToGimbal packet{};
            for (ssize_t i = 0; i < count; ++i) {
                if (parser.input(bytes[i], packet))
                    return packet;
            }
        } else {
            std::this_thread::sleep_for(1ms);
        }
    }
    throw std::runtime_error("Missing command after draining PTY backpressure");
}

void backpressure(const std::string& scenario) {
    FeedbackPeer peer;
    fill_output(peer.pty);
    const auto initial = peer.device->snapshot();
    std::atomic<bool> finished{false};
    std::jthread writer([&] {
        const auto expected_session = scenario == "backpressure_mode"
                                          ? std::nullopt
                                          : std::optional<uint64_t>{initial.session};
        io::Command command{.control = true, .shoot = true};
        if (scenario == "backpressure_deadline")
            command.valid_until = Clock::now() + 70ms;
        peer.device->send(command, expected_session);
        finished = true;
    });
    std::this_thread::sleep_for(25ms);
    require(!finished, "The command did not encounter real PTY backpressure");
    const auto query_started = Clock::now();
    const auto during = peer.device->snapshot();
    require(Clock::now() - query_started < 50ms, "Backpressure blocked the gimbal state lock");
    require(during.feedback_fresh && during.state.received_at > initial.state.received_at,
            "Backpressure prevented feedback from being received");

    if (scenario == "backpressure_mode") {
        peer.mode = 0;
        const auto deadline = Clock::now() + 70ms;
        while (peer.device->snapshot().mode != io::GimbalMode::IDLE && Clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
        }
        require(peer.device->snapshot().mode == io::GimbalMode::IDLE,
                "Mode change was blocked by output backpressure");
        peer.mode = 1;
        const auto resume_deadline = Clock::now() + 70ms;
        while (peer.device->snapshot().mode != io::GimbalMode::AUTO_AIM &&
               Clock::now() < resume_deadline) {
            std::this_thread::sleep_for(1ms);
        }
        const auto resumed = peer.device->snapshot();
        require(resumed.mode == io::GimbalMode::AUTO_AIM && resumed.session != initial.session,
                "Mode cycle did not advance the session while the writer was waiting");
    } else if (scenario == "backpressure_timeout") {
        peer.active = false;
        std::this_thread::sleep_for(130ms);
        require(!peer.device->snapshot().feedback_fresh, "Silent feedback did not expire");
    } else if (scenario == "backpressure_deadline") {
        std::this_thread::sleep_for(80ms);
        require(peer.device->snapshot().feedback_fresh,
                "Command deadline fixture lost feedback freshness");
    } else {
        peer.yaw = 0.1f;
        const auto deadline = Clock::now() + 70ms;
        while (peer.device->snapshot().state.yaw != 0.1f && Clock::now() < deadline) {
            std::this_thread::sleep_for(1ms);
        }
        require(peer.device->snapshot().state.yaw == 0.1f, "Latest alignment feedback was lost");
    }

    require(!finished, "The blocked writer timed out before its safety state could be checked");
    const auto packet = drain_command(peer.pty);
    writer.join();
    const uint8_t expected_mode = scenario == "backpressure_alignment" ? 1 : 0;
    require(packet.mode == expected_mode,
            "A queued command retained stale mode, session, freshness, or firing permission");
    std::cout << "snapshot under backpressure: "
              << std::chrono::duration<double, std::milli>(during.state.received_at -
                                                           initial.state.received_at)
                     .count()
              << " ms of new feedback\n";
}

void reconnect() {
    Pty first;
    const auto link = std::filesystem::temp_directory_path() /
                      ("robocore-gimbal-link-" + std::to_string(::getpid()));
    std::error_code ignored;
    std::filesystem::remove(link, ignored);
    require(::symlink(first.name().c_str(), link.c_str()) == 0, "Cannot create reconnect symlink");
    Config config(link.string());
    std::atomic<bool> first_running{true};
    std::jthread first_sender([&](std::stop_token stop) {
        while (!stop.stop_requested() && first_running) {
            io::GimbalToVision packet{};
            packet.mode = 1;
            packet.q[0] = 1;
            packet.bullet_speed = 22;
            packet.crc16 =
                tools::get_crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - 2);
            const auto count = ::write(first.master(), &packet, sizeof(packet));
            if (count < 0 && errno != EAGAIN && errno != EINTR)
                break;
            std::this_thread::sleep_for(5ms);
        }
    });
    auto device = std::make_unique<io::Gimbal>(config.path());
    std::this_thread::sleep_for(40ms);
    const auto old_session = device->session();
    first_running = false;
    first_sender.request_stop();
    first_sender.join();
    first.disconnect();
    const auto idle_deadline = Clock::now() + 500ms;
    while (Clock::now() < idle_deadline && device->mode() != io::GimbalMode::IDLE) {
        std::this_thread::sleep_for(10ms);
    }
    require(device->mode() == io::GimbalMode::IDLE, "Disconnected gimbal stayed active");

    Pty second;
    std::filesystem::remove(link, ignored);
    require(::symlink(second.name().c_str(), link.c_str()) == 0,
            "Cannot replace reconnect symlink");
    std::atomic<bool> second_running{true};
    std::jthread second_sender([&](std::stop_token stop) {
        while (!stop.stop_requested() && second_running) {
            io::GimbalToVision packet{};
            packet.mode = 1;
            packet.q[0] = 1;
            packet.bullet_speed = 22;
            packet.crc16 =
                tools::get_crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - 2);
            const auto count = ::write(second.master(), &packet, sizeof(packet));
            if (count < 0 && errno != EAGAIN && errno != EINTR)
                break;
            std::this_thread::sleep_for(5ms);
        }
    });
    const auto deadline = Clock::now() + 2s;
    while (Clock::now() < deadline && device->mode() != io::GimbalMode::AUTO_AIM) {
        std::this_thread::sleep_for(10ms);
    }
    require(device->mode() == io::GimbalMode::AUTO_AIM,
            "Gimbal did not reconnect to the replacement port");
    require(device->session() != old_session, "Reconnect reused the old tracking session");
    second_running = false;
    second_sender.request_stop();
    second_sender.join();
    device.reset();
    std::filesystem::remove(link, ignored);
}

void feedback_timeout() {
    FeedbackPeer peer;
    require(peer.command().mode == 2, "Fresh AUTO_AIM fixture cannot fire");
    peer.active = false;
    std::this_thread::sleep_for(160ms);
    require(peer.device->mode() == io::GimbalMode::IDLE, "Silent feedback remains AUTO_AIM");
    const auto packet = peer.command();
    require(packet.mode == 0 && packet.yaw == 0, "Feedback outage did not disable output");
    const auto old_session = peer.device->session();
    peer.active = true;
    std::this_thread::sleep_for(30ms);
    require(peer.device->session() != old_session, "Feedback recovery reused the old session");
    require(peer.command(1, old_session).mode == 0, "Old plan survived feedback recovery");
}

void reconnect_shutdown() {
    FeedbackPeer peer(false, 5s);
    peer.sender.request_stop();
    peer.sender.join();
    peer.pty.disconnect();
    const auto deadline = Clock::now() + 500ms;
    while (peer.device->mode() != io::GimbalMode::IDLE && Clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    require(peer.device->mode() == io::GimbalMode::IDLE, "PTY disconnect was not observed");
    const auto start = Clock::now();
    peer.device.reset();
    require(Clock::now() - start < 500ms, "Shutdown waited for the configured reconnect delay");
}

void pose_history() {
    FeedbackPeer peer;
    const auto query = Clock::now() - 20ms;
    const auto first = peer.device->q(query);
    const auto repeated = peer.device->q(query);
    require(first.angularDistance(repeated) < 1e-12, "Repeated query consumed the pose bracket");
    peer.active = false;
    const auto start = Clock::now();
    bool threw = false;
    try {
        peer.device->q(Clock::now() + 200ms);
    } catch (const std::runtime_error&) {
        threw = true;
    }
    require(threw && Clock::now() - start < 500ms, "Missing future IMU pose blocked indefinitely");
}

void pose_cancel() {
    // 长反馈超时不能拖住运行时退出；持续反馈也不会满足远期位姿查询。
    FeedbackPeer peer(false, 250ms, 5s);
    const auto query = Clock::now() + 10s;
    std::promise<bool> result;
    auto completed = result.get_future();
    std::jthread waiter([&](std::stop_token stop) {
        try {
            peer.device->q(query, stop);
            result.set_value(false);
        } catch (const std::runtime_error&) {
            result.set_value(true);
        }
    });
    require(completed.wait_for(20ms) == std::future_status::timeout,
            "Future pose query did not wait for its missing sample");
    waiter.request_stop();
    require(completed.wait_for(500ms) == std::future_status::ready && completed.get(),
            "Canceled pose query waited for the full feedback timeout");
    waiter.join();
}

void config_duration_bounds() {
    const auto maximum = std::chrono::milliseconds(std::numeric_limits<std::int32_t>::max());
    io::GimbalConfig config;
    config.feedback_timeout = config.reconnect_delay = maximum;
    io::validate(config);
    const auto rejected = [](const io::GimbalConfig& invalid, const char* field) {
        try {
            io::validate(invalid);
        } catch (const std::runtime_error& error) {
            return std::string(error.what()).find(field) != std::string::npos;
        }
        return false;
    };
    config.feedback_timeout = maximum + 1ms;
    require(rejected(config, "gimbal.feedback_timeout_ms"),
            "Typed feedback timeout accepted overflow");
    config.feedback_timeout = maximum;
    config.reconnect_delay = maximum + 1ms;
    require(rejected(config, "gimbal.reconnect_delay_ms"),
            "Typed reconnect delay accepted overflow");
    config.reconnect_delay = std::chrono::milliseconds::max();
    require(rejected(config, "gimbal.reconnect_delay_ms"),
            "Typed reconnect delay accepted maximum duration");
}

void feedback_alignment() {
    FeedbackPeer peer(true); // 每个反馈包前先发送伪 SP 帧头。
    require(peer.command(0.1f).mode == 1, "Yaw misalignment reached the wire as fire");
    require(peer.command(0, std::nullopt, 0.1f).mode == 1,
            "Pitch misalignment reached the wire as fire");
    require(peer.command().mode == 2,
            "Aligned feedback could not fire after CRC resynchronization");
    require(peer.command(2 * std::acos(-1.0)).mode == 2, "Wrapped aligned yaw could not fire");
    const io::Command rounded_outside{
        .control = true, .shoot = true, .yaw = std::nextafter(0.05, 0.0)};
    require(peer.command(rounded_outside).mode == 1,
            "Alignment checked a double value that narrows outside the wire firing threshold");
}

void mode_session() {
    FeedbackPeer peer;
    const auto original = peer.device->session();
    peer.mode = 0;
    std::this_thread::sleep_for(20ms);
    require(peer.command().mode == 0, "IDLE accepted a control/fire command");
    peer.mode = 1;
    std::this_thread::sleep_for(20ms);
    require(peer.device->session() != original, "Mode cycle reused the old tracking session");
    require(peer.command(1, original).mode == 0, "Old plan survived an IDLE/AUTO_AIM mode cycle");
    require(peer.command(0, peer.device->session()).mode == 2,
            "New tracking session cannot control");
    require(peer.command(std::numeric_limits<float>::quiet_NaN()).mode == 0,
            "NaN control reached the wire");
    const io::Command huge{
        .control = true, .shoot = true, .yaw = std::numeric_limits<double>::max()};
    require(peer.command(huge).mode == 0, "Float narrowing overflow reached the wire");
}
} // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected a lifecycle scenario");
        const std::string scenario(argv[1]);
        if (scenario == "silent_start")
            silent_start();
        else if (scenario == "idle_shutdown")
            idle_shutdown();
        else if (scenario == "feedback_timeout")
            feedback_timeout();
        else if (scenario == "pose_history")
            pose_history();
        else if (scenario == "pose_cancel")
            pose_cancel();
        else if (scenario == "config_duration_bounds")
            config_duration_bounds();
        else if (scenario == "feedback_alignment")
            feedback_alignment();
        else if (scenario == "reconnect")
            reconnect();
        else if (scenario == "reconnect_shutdown")
            reconnect_shutdown();
        else if (scenario == "mode_session")
            mode_session();
        else if (scenario == "backpressure_mode" || scenario == "backpressure_timeout" ||
                 scenario == "backpressure_deadline" || scenario == "backpressure_alignment")
            backpressure(scenario);
        else
            throw std::runtime_error("Unknown scenario: " + scenario);
        std::cout << "PASS " << scenario << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
