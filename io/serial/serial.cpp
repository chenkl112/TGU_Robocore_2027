#include "serial.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include "tools/logger.hpp"

namespace io {
namespace {
using Clock = std::chrono::steady_clock;
constexpr auto read_wait = std::chrono::milliseconds(50);
constexpr auto write_timeout = std::chrono::milliseconds(250);
} // namespace

Serial::Serial() : serial_(io_) {}

Serial::~Serial() {
    close();
}

bool Serial::open(const std::string& device, int baudrate) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    close_locked();
    try {
        serial_.open(device);
        serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
        serial_.set_option(boost::asio::serial_port_base::character_size(8));
        serial_.set_option(
            boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial_.set_option(boost::asio::serial_port_base::stop_bits(
            boost::asio::serial_port_base::stop_bits::one));
        serial_.set_option(boost::asio::serial_port_base::flow_control(
            boost::asio::serial_port_base::flow_control::none));

        const int fd = serial_.native_handle();
        const int flags = ::fcntl(fd, F_GETFL);
        if (flags == -1 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) == -1) {
            throw std::system_error(errno, std::generic_category(), "set nonblocking serial I/O");
        }
        is_open_ = true;
        state_changed_.notify_all();
        LOG_INFO(MODULE, "{} open success", device);
        return true;
    } catch (const std::exception& error) {
        close_locked();
        LOG_ERROR(MODULE, "{} open failed: {}", device, error.what());
        return false;
    }
}

void Serial::close_locked() {
    is_open_ = false;
    ++session_;
    boost::system::error_code ignored;
    serial_.close(ignored);
    state_changed_.notify_all();
}

void Serial::close() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    close_locked();
}

void Serial::close_session(uint64_t session) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (session_ == session)
        close_locked();
}

bool Serial::is_open() const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return is_open_;
}

bool Serial::is_current_session(uint64_t session) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    return is_open_ && session_ == session;
}

bool Serial::wait_ready(int fd, uint64_t session, short events, int timeout_ms) {
    pollfd descriptor{fd, events, 0};
    const int result = ::poll(&descriptor, 1, timeout_ms);
    const int error = errno;
    std::lock_guard<std::mutex> lock(state_mutex_);
    // poll 等待时 close/open 可能复用文件描述符；旧操作不能触碰新连接。
    if (!is_open_ || session_ != session)
        return false;
    if ((result < 0 && error != EINTR) ||
        (result > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))) {
        close_locked();
        return false;
    }
    return result > 0 && (descriptor.revents & events);
}

size_t Serial::write(const uint8_t* data, size_t size, const WriteGuard& guard) {
    if (size == 0)
        return 0;
    int fd;
    uint64_t session;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_open_)
            return 0;
        fd = serial_.native_handle();
        session = session_;
    }
    // 先记录连接会话再排队，避免等待中的发送迁移到重连后的新端口。
    std::lock_guard<std::mutex> writer(write_mutex_);
    const auto deadline = Clock::now() + write_timeout;
    size_t sent = 0;
    while (sent < size) {
        bool retry = true;
        bool wrote = false;
        const WriteAttempt attempt = [&] {
            std::lock_guard<std::mutex> lock(state_mutex_);
            if (!is_open_ || session_ != session) {
                retry = false;
                return;
            }
            if (Clock::now() >= deadline) {
                close_locked();
                retry = false;
                return;
            }
            const auto count = ::write(fd, data + sent, size - sent);
            if (count > 0) {
                sent += static_cast<size_t>(count);
                wrote = true;
                return;
            }
            if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
                close_locked();
                retry = false;
            }
        };
        try {
            // 锁顺序是写入排队锁 → 上层守卫锁 → 串口状态锁；poll 不持后两者。
            if (guard) {
                if (!guard(sent, attempt)) {
                    if (sent != 0)
                        close_session(session);
                    return sent;
                }
            } else {
                attempt();
            }
        } catch (...) {
            if (sent != 0)
                close_session(session);
            throw;
        }
        if (!retry)
            return sent;
        if (wrote)
            continue;
        const auto remaining =
            std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now());
        if (remaining.count() > 0) {
            wait_ready(fd, session, POLLOUT,
                       static_cast<int>(std::min(read_wait, remaining).count()));
        }
    }
    return sent;
}

void Serial::spin_once() {
    std::lock_guard<std::mutex> reader(read_mutex_);
    int fd;
    uint64_t session;
    {
        std::unique_lock<std::mutex> lock(state_mutex_);
        if (!is_open_) {
            // 断连后继续轮询的调用者也需要有界等待，避免忙循环占满 CPU。
            state_changed_.wait_for(lock, read_wait, [this] { return is_open_; });
            if (!is_open_)
                return;
        }
        fd = serial_.native_handle();
        session = session_;
    }
    if (!wait_ready(fd, session, POLLIN, static_cast<int>(read_wait.count())))
        return;

    uint8_t buffer[512];
    ssize_t count;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        if (!is_open_ || session_ != session)
            return;
        count = ::read(fd, buffer, sizeof(buffer));
        if (count <= 0) {
            if (count == 0 || (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
                close_locked();
            }
            return;
        }
    }

    decltype(callbacks_) callbacks;
    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callbacks = callbacks_;
    }
    // 回调执行时不持串口状态锁，因此可以发送或关闭串口；read_mutex_ 保证解析器串行。
    try {
        for (auto& callback : callbacks) {
            if (!is_current_session(session))
                break;
            callback(buffer, static_cast<size_t>(count), session);
        }
    } catch (const std::exception& error) {
        close_session(session);
        LOG_ERROR(MODULE, "receive callback failed: {}", error.what());
    } catch (...) {
        close_session(session);
        LOG_ERROR(MODULE, "receive callback failed with an unknown exception");
    }
}
} // namespace io
