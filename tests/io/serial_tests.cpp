#include "io/serial/serial.hpp"
#include "test_support.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"

#include <atomic>
#include <filesystem>
#include <iostream>
#include <latch>
#include <thread>

namespace {
using namespace test;

struct __attribute__((packed)) Packet {
    uint8_t head[2] = {'T', 'S'};
    uint32_t value = 42;
    uint16_t crc = 0;
};
Packet packet(uint32_t value = 42) {
    Packet result;
    result.value = value;
    result.crc = tools::get_crc16(reinterpret_cast<const uint8_t*>(&result), sizeof(result) - 2);
    return result;
}
const uint8_t* bytes(const Packet& value) {
    return reinterpret_cast<const uint8_t*>(&value);
}
void subscribe(io::Serial& serial, std::vector<uint32_t>& received) {
    serial.recv<Packet>(
        [&received](const Packet& value) { received.push_back(value.value); },
        [](const Packet& value) { return tools::check_crc16(bytes(value), sizeof(value)); });
}
void open(io::Serial& serial, const Pty& pty) {
    require(serial.open(pty.name(), 115200), "Cannot open test serial port");
}
void read_timeout() {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    const auto start = Clock::now();
    serial.spin_once();
    const auto elapsed = Clock::now() - start;
    require(elapsed >= 20ms && elapsed < 1s, "Silent read did not wait for a bounded interval");
    require(serial.is_open(), "Silent timeout closed a healthy port");
}
void close_read() {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    std::latch started(1);
    std::jthread reader([&] {
        started.count_down();
        serial.spin_once();
    });
    started.wait();
    std::this_thread::sleep_for(20ms);
    const auto start = Clock::now();
    serial.close();
    reader.join();
    require(Clock::now() - start < 1s, "Close did not release the pending read");
    require(!serial.is_open(), "Closed port still reports open");
    serial.close();
}
void fragmented(bool multiple_subscriptions) {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    std::vector<uint32_t> a, b;
    std::vector<int> invocation_counts;
    subscribe(serial, a);
    if (multiple_subscriptions) {
        subscribe(serial, b);
        serial.recv<Packet>([count = 0, &invocation_counts](const Packet&) mutable {
            invocation_counts.push_back(++count);
        });
    }
    const auto first = packet(17);
    pty.send(bytes(first), 4);
    serial.spin_once();
    require(a.empty() && b.empty(), "Half packet delivered prematurely");
    pty.send(bytes(first) + 4, sizeof(first) - 4);
    serial.spin_once();
    const auto second = packet(18);
    const auto third = packet(19);
    std::vector<uint8_t> joined(bytes(second), bytes(second) + sizeof(second));
    joined.insert(joined.end(), bytes(third), bytes(third) + sizeof(third));
    pty.send(joined.data(), joined.size());
    serial.spin_once();
    const std::vector<uint32_t> expected{17, 18, 19};
    require(a == expected, "Fragmented or coalesced packets were lost or duplicated");
    if (multiple_subscriptions) {
        require(b == expected, "Subscriptions shared parser state");
        require(invocation_counts == std::vector<int>({1, 2, 3}),
                "Stateful callback was copied or reset between receive batches");
    }
}
void false_header() {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    std::vector<uint32_t> received;
    subscribe(serial, received);
    const auto valid = packet(123);
    std::vector<uint8_t> stream{valid.head[0], valid.head[1]};
    stream.insert(stream.end(), bytes(valid), bytes(valid) + sizeof(valid));
    pty.send(stream.data(), stream.size());
    serial.spin_once();
    require(received == std::vector<uint32_t>{123},
            "False header swallowed the following valid CRC packet");
    auto corrupt = packet(456);
    corrupt.crc ^= 1;
    const auto second = packet(789);
    stream.assign(bytes(corrupt), bytes(corrupt) + sizeof(corrupt));
    stream.insert(stream.end(), {valid.head[0], valid.head[1], valid.head[0]});
    stream.insert(stream.end(), bytes(second), bytes(second) + sizeof(second));
    for (const auto byte : stream) {
        pty.send(&byte, 1);
        serial.spin_once();
    }
    require(received == std::vector<uint32_t>({123, 789}),
            "Fragmented corrupt/false headers lost or duplicated a frame");
}
void isolated_ports() {
    Pty pty_a, pty_b;
    io::Serial a, b;
    open(a, pty_a);
    open(b, pty_b);
    std::vector<uint32_t> received_a, received_b;
    subscribe(a, received_a);
    subscribe(b, received_b);
    const auto value = packet();
    pty_a.send(bytes(value), 4);
    a.spin_once();
    pty_b.send(bytes(value) + 4, sizeof(value) - 4);
    b.spin_once();
    require(received_a.empty() && received_b.empty(), "Different ports assembled one packet");
}
void reopen() {
    Pty old_port, new_port;
    io::Serial serial;
    std::vector<uint32_t> received;
    subscribe(serial, received);
    const auto value = packet();
    open(serial, old_port);
    old_port.send(bytes(value), 4);
    serial.spin_once();
    // open() 可直接替换现有端口，无需先显式 close()。
    open(serial, new_port);
    new_port.send(bytes(value) + 4, sizeof(value) - 4);
    serial.spin_once();
    require(received.empty(), "Reopening retained partial parser state");
    new_port.send(bytes(value), sizeof(value));
    serial.spin_once();
    require(received == std::vector<uint32_t>{42}, "Subscription did not survive reopening");
    serial.close();
    open(serial, new_port);
    new_port.send(bytes(value), sizeof(value));
    serial.spin_once();
    require(received == std::vector<uint32_t>({42, 42}), "Explicit close/reopen failed");
}
size_t fd_count() {
    return static_cast<size_t>(std::distance(std::filesystem::directory_iterator("/proc/self/fd"),
                                             std::filesystem::directory_iterator{}));
}
void failed_open() {
    Pty pty;
    io::Serial serial;
    const auto before = fd_count();
    // Boost 首次 open 时创建事件循环描述符；先预热再统计端口泄漏。
    open(serial, pty);
    serial.close();
    const auto warmed = fd_count();
    require(warmed >= before, "Unexpected descriptor count");
    for (int repeat = 0; repeat < 10; ++repeat) {
        require(!serial.open(pty.name(), -1), "Invalid baud rate unexpectedly accepted");
        require(!serial.is_open(), "Failed configuration left port logically open");
        require(fd_count() == warmed, "Failed configuration leaked an open descriptor");
    }
    require(!serial.open("/nonexistent-robocore-test-device", 115200), "Missing port opened");
    open(serial, pty);
    const auto value = packet();
    require(serial.send(value) == sizeof(value), "Write after failed-open recovery failed");
    require(pty.receive(sizeof(value)) ==
                std::vector<uint8_t>(bytes(value), bytes(value) + sizeof(value)),
            "Unexpected bytes after recovery");
}
void disconnect() {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    pty.disconnect();
    serial.spin_once();
    require(!serial.is_open(), "Disconnect was not detected");
    const auto start = Clock::now();
    serial.spin_once();
    require(Clock::now() - start >= 20ms, "Disconnected reader busy-loops");
    require(Clock::now() - start < 1s, "Disconnected reader blocks indefinitely");
    require(serial.send(packet()) == 0, "Write succeeded on disconnected port");
}
void write_timeout(bool close_pending) {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    const std::vector<uint8_t> large(4 * 1024 * 1024, 0x5a);
    size_t sent = 0;
    std::latch started(1);
    const auto start = Clock::now();
    std::jthread writer([&] {
        started.count_down();
        sent = serial.write(large.data(), large.size());
    });
    started.wait();
    if (close_pending) {
        std::this_thread::sleep_for(20ms);
        serial.close();
    }
    writer.join();
    const auto elapsed = Clock::now() - start;
    require(elapsed < 1s, "Backpressured write failed to return in bounded time");
    if (!close_pending)
        require(elapsed >= 200ms, "Write timed out too early");
    require(sent > 0 && sent < large.size(), "Expected an accurate partial write count");
    require(!serial.is_open(), "Incomplete write left a partial stream active");
}
void callback_close() {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    bool first = false, second = false;
    const auto response = packet(99);
    serial.recv<Packet>([&](const Packet&) {
        first = serial.send(response) == sizeof(response);
        serial.close();
    });
    serial.recv<Packet>([&](const Packet&) { second = true; });
    const auto value = packet();
    pty.send(bytes(value), sizeof(value));
    serial.spin_once();
    require(first && !second && !serial.is_open(), "Callback send/close semantics failed");
    require(pty.receive(sizeof(response)) ==
                std::vector<uint8_t>(bytes(response), bytes(response) + sizeof(response)),
            "Callback response was corrupted");
}
void guarded_cancel() {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    const std::vector<uint8_t> large(4 * 1024 * 1024, 0x5a);
    size_t guard_calls = 0;
    // 守卫在首段写出后撤销；返回值必须只包含已写字节，残帧连接必须失效。
    const size_t sent = serial.write(large.data(), large.size(),
                                     [&](size_t written, const io::Serial::WriteAttempt& attempt) {
                                         ++guard_calls;
                                         if (written > 0)
                                             return false;
                                         attempt();
                                         return true;
                                     });
    require(guard_calls >= 2 && sent > 0 && sent < large.size(),
            "Guard cancellation did not preserve the partial byte count");
    require(!serial.is_open(), "Canceled partial frame left its connection active");
    open(serial, pty);
    const auto canceled = serial.write(
        large.data(), large.size(), [](size_t, const io::Serial::WriteAttempt&) { return false; });
    require(canceled == 0 && serial.is_open(),
            "Canceling before the first byte closed a healthy port");
}
void queued_write() {
    Pty old_port, new_port;
    io::Serial serial;
    open(serial, old_port);
    const std::vector<uint8_t> large(4 * 1024 * 1024, 0x5a);
    std::latch started(1);
    std::jthread first([&] {
        started.count_down();
        serial.write(large.data(), large.size());
    });
    started.wait();
    // 收到一个字节即可证明第一个发送者已经开始写入并持有发送权。
    old_port.receive(1);
    std::latch queued(1);
    std::atomic<bool> finished{false};
    size_t second_sent = 0;
    std::jthread second([&] {
        queued.count_down();
        second_sent = serial.send(packet());
        finished = true;
    });
    queued.wait();
    std::this_thread::sleep_for(20ms);
    const bool was_waiting = !finished.load();
    serial.close();
    const bool reopened = serial.open(new_port.name(), 115200);
    first.join();
    second.join();
    require(was_waiting, "Second writer did not queue behind the backpressured sender");
    require(reopened && serial.is_open(), "Old writer closed the replacement connection");
    require(second_sent == 0, "Queued write migrated to a replacement connection");
    pollfd descriptor{new_port.master(), POLLIN, 0};
    require(::poll(&descriptor, 1, 0) == 0, "Replacement port received old session bytes");
}
void callback_throw() {
    Pty pty;
    io::Serial serial;
    open(serial, pty);
    serial.recv<Packet>([](const Packet&) { throw std::runtime_error("test callback failure"); });
    const auto value = packet();
    pty.send(bytes(value), sizeof(value));
    serial.spin_once();
    require(!serial.is_open(), "Callback exception did not close the failed session");
    open(serial, pty);
    require(serial.is_open(), "Callback failure prevented reopening");
}
void concurrent() {
    Pty a, b;
    io::Serial serial;
    open(serial, a);
    std::atomic<int> received{0};
    std::atomic<bool> stop{false};
    std::atomic<bool> peer_failed{false};
    serial.recv<Packet>([&](const Packet& value) {
        if (tools::check_crc16(bytes(value), sizeof(value)))
            ++received;
    });
    const auto value = packet();
    auto peer = [&](const Pty& pty) {
        uint8_t drain[4096];
        while (!stop.load()) {
            const auto read_count = ::read(pty.master(), drain, sizeof(drain));
            if (read_count < 0 && errno != EAGAIN && errno != EINTR && errno != EIO) {
                peer_failed = true;
                break;
            }
            const auto write_count = ::write(pty.master(), bytes(value), sizeof(value));
            if (write_count < 0 && errno != EAGAIN && errno != EINTR && errno != EIO) {
                peer_failed = true;
                break;
            }
            std::this_thread::sleep_for(1ms);
        }
    };
    std::jthread peer_a([&] { peer(a); }), peer_b([&] { peer(b); });
    std::jthread reader([&] {
        while (!stop.load())
            serial.spin_once();
    });
    std::jthread writer([&] {
        while (!stop.load()) {
            serial.send(value);
            std::this_thread::sleep_for(1ms);
        }
    });
    bool opened = true;
    for (int repeat = 0; repeat < 100; ++repeat) {
        serial.close();
        opened = serial.open((repeat % 2 ? a : b).name(), 115200) && opened;
        std::this_thread::sleep_for(2ms);
        // 注册新订阅不能使正在分派的回调列表失效。
        if (repeat == 50)
            serial.recv<Packet>([](const Packet&) {});
    }
    stop = true;
    writer.join();
    reader.join();
    peer_a.join();
    peer_b.join();
    require(opened && serial.is_open(), "Old operation closed a new serial session");
    require(!peer_failed, "PTY peer failed during concurrent I/O");
    require(received > 0, "Concurrent reader never received a valid packet");
    serial.close();
}
} // namespace

int main(int argc, char** argv) {
    tools::Logger::instance().init({.level = tools::LogLevel::Off, .enable_console = false});
    try {
        require(argc == 2, "Expected a test scenario");
        const std::string scenario(argv[1]);
        if (scenario == "read_timeout")
            read_timeout();
        else if (scenario == "close_read")
            close_read();
        else if (scenario == "fragmented")
            fragmented(false);
        else if (scenario == "subscriptions")
            fragmented(true);
        else if (scenario == "false_header")
            false_header();
        else if (scenario == "isolated_ports")
            isolated_ports();
        else if (scenario == "reopen")
            reopen();
        else if (scenario == "failed_open")
            failed_open();
        else if (scenario == "disconnect")
            disconnect();
        else if (scenario == "write_timeout")
            write_timeout(false);
        else if (scenario == "close_write")
            write_timeout(true);
        else if (scenario == "queued_write")
            queued_write();
        else if (scenario == "callback_close")
            callback_close();
        else if (scenario == "callback_throw")
            callback_throw();
        else if (scenario == "concurrent")
            concurrent();
        else if (scenario == "guarded_cancel")
            guarded_cancel();
        else
            throw std::runtime_error("Unknown scenario: " + scenario);
        std::cout << "PASS " << scenario << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
