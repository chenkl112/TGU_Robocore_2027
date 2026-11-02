#include "test_support.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <cmath>
#include <limits>
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
    std::string path() const { return (directory_ / "test.toml").string(); }
private:
    std::filesystem::path directory_;
};

void silent_start() {
    Pty pty;  // Keep the peer open, but send no bytes at all.
    Config config(pty.name());
    const auto start = Clock::now();
    bool timed_out = false;
    try {
        io::Gimbal device(config.path());
    } catch (const std::runtime_error& error) {
        timed_out = std::string(error.what()).find("first valid gimbal packet") != std::string::npos;
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
    std::this_thread::sleep_for(100ms);  // Let the reader enter a silent wait.
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
    Config config{pty.name()};
    std::atomic<bool> active{true};
    std::atomic<uint8_t> mode{1};
    std::jthread sender;
    std::unique_ptr<io::Gimbal> device;

    FeedbackPeer() : sender([&](std::stop_token stop) {
        while (!stop.stop_requested()) {
            if (active) {
                io::GimbalToVision packet{};
                packet.mode = mode;
                packet.q[0] = 1;
                packet.bullet_speed = 22;
                packet.crc16 = tools::get_crc16(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet) - 2);
                const auto count = ::write(pty.master(), &packet, sizeof(packet));
                if (count < 0 && errno != EAGAIN && errno != EINTR) return;
            }
            std::this_thread::sleep_for(5ms);
        }
    }) {
        device = std::make_unique<io::Gimbal>(config.path());
        std::this_thread::sleep_for(40ms);
    }
    ~FeedbackPeer() {
        sender.request_stop();
        sender.join();
        device.reset();
    }
    io::VisionToGimbal command(float yaw = 1, std::optional<uint64_t> session = std::nullopt) {
        device->send(true, true, yaw, 0, 0, 0, 0, 0, session);
        io::VisionToGimbal result{};
        auto* bytes = reinterpret_cast<uint8_t*>(&result);
        size_t count = 0;
        const auto deadline = Clock::now() + 1s;
        while (count < sizeof(result) && Clock::now() < deadline) {
            const auto n = ::read(pty.master(), bytes + count, sizeof(result) - count);
            if (n > 0) count += static_cast<size_t>(n);
            else std::this_thread::sleep_for(1ms);
        }
        require(count == sizeof(result), "Missing outgoing gimbal command");
        require(tools::check_crc16(bytes, sizeof(result)), "Invalid outgoing CRC");
        return result;
    }
};

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

void pose_history() {
    FeedbackPeer peer;
    const auto query = Clock::now() - 20ms;
    const auto first = peer.device->q(query);
    const auto repeated = peer.device->q(query);
    require(first.angularDistance(repeated) < 1e-12, "Repeated query consumed the pose bracket");
    peer.active = false;
    const auto start = Clock::now();
    bool threw = false;
    try { peer.device->q(Clock::now() + 200ms); }
    catch (const std::runtime_error&) { threw = true; }
    require(threw && Clock::now() - start < 500ms, "Missing future IMU pose blocked indefinitely");
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
    require(peer.command(1, peer.device->session()).mode == 2, "New tracking session cannot control");
    require(peer.command(std::numeric_limits<float>::quiet_NaN()).mode == 0, "NaN control reached the wire");
}
}  // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected a lifecycle scenario");
        const std::string scenario(argv[1]);
        if (scenario == "silent_start") silent_start();
        else if (scenario == "idle_shutdown") idle_shutdown();
        else if (scenario == "feedback_timeout") feedback_timeout();
        else if (scenario == "pose_history") pose_history();
        else if (scenario == "mode_session") mode_session();
        else throw std::runtime_error("Unknown scenario: " + scenario);
        std::cout << "PASS " << scenario << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
