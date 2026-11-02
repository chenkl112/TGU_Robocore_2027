#pragma once

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <unistd.h>
#include <vector>

namespace test {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

inline void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

class Pty {
public:
    Pty() {
        char name[128]{};
        require(::openpty(&master_, &slave_, name, nullptr, nullptr) == 0, "openpty failed");
        name_ = name;
        termios settings{};
        require(::tcgetattr(slave_, &settings) == 0, "Cannot read PTY settings");
        ::cfmakeraw(&settings);
        require(::tcsetattr(slave_, TCSANOW, &settings) == 0, "Cannot configure raw PTY");
        const int flags = ::fcntl(master_, F_GETFL);
        if (flags < 0 || ::fcntl(master_, F_SETFL, flags | O_NONBLOCK) < 0) {
            ::close(master_);
            ::close(slave_);
            throw std::runtime_error("Cannot make PTY master nonblocking");
        }
    }
    ~Pty() {
        if (master_ >= 0) ::close(master_);
        ::close(slave_);
    }
    Pty(const Pty&) = delete;
    Pty& operator=(const Pty&) = delete;
    const std::string& name() const { return name_; }
    int master() const { return master_; }
    void disconnect() {
        if (master_ >= 0) ::close(master_);
        master_ = -1;
    }
    void send(const uint8_t* data, size_t size) const {
        const auto deadline = Clock::now() + 2s;
        size_t sent = 0;
        while (sent < size && Clock::now() < deadline) {
            const auto count = ::write(master_, data + sent, size - sent);
            if (count > 0) sent += static_cast<size_t>(count);
            else {
                require(count < 0 && (errno == EAGAIN || errno == EINTR), "PTY write failed");
                pollfd fd{master_, POLLOUT, 0};
                ::poll(&fd, 1, 10);
            }
        }
        require(sent == size, "PTY write timed out");
    }
    std::vector<uint8_t> receive(size_t size) const {
        std::vector<uint8_t> result(size);
        const auto deadline = Clock::now() + 2s;
        size_t received = 0;
        while (received < size && Clock::now() < deadline) {
            const auto count = ::read(master_, result.data() + received, size - received);
            if (count > 0) received += static_cast<size_t>(count);
            else {
                require(count < 0 && (errno == EAGAIN || errno == EINTR), "PTY read failed");
                pollfd fd{master_, POLLIN, 0};
                ::poll(&fd, 1, 10);
            }
        }
        require(received == size, "PTY read timed out");
        return result;
    }
private:
    int master_ = -1;
    int slave_ = -1;
    std::string name_;
};
}  // namespace test
