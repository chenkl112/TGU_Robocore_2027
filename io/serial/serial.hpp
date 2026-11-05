//
// 由 Interweave 创建于 2026/4/10。
//

#ifndef TGU_ROBOCORE_2027_SERIAL_HPP
#define TGU_ROBOCORE_2027_SERIAL_HPP

#pragma once
#include <boost/asio.hpp>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>
#include <vector>

namespace io {

// ==================== RingBuffer ====================
template <size_t N> class RingBuffer {
  public:
    void push(uint8_t byte) {
        buffer_[write_] = byte;
        write_ = (write_ + 1) % N;

        if (size_ < N)
            size_++;
        else
            read_ = (read_ + 1) % N;
    }

    uint8_t at(size_t idx) const {
        return buffer_[(read_ + idx) % N];
    }

    void pop(size_t n) {
        read_ = (read_ + n) % N;
        size_ -= n;
    }

    size_t size() const {
        return size_;
    }

  private:
    uint8_t buffer_[N];
    size_t read_ = 0;
    size_t write_ = 0;
    size_t size_ = 0;
};

// ==================== Parser ====================
template <typename T, size_t N = 2048> class StructParser {
  public:
    static_assert(std::is_trivially_copyable_v<T>, "Serial packets must be trivially copyable");
    static_assert(sizeof(T) >= 2 && sizeof(T) <= N, "Packet must fit the parser buffer");

    explicit StructParser(std::function<bool(const T&)> validate = [](const T&) { return true; })
        : validate_(std::move(validate)) {
        T dummy{};

        std::memcpy(head_, &dummy, HEAD_SIZE);
    }

    void reset() {
        buffer_ = RingBuffer<N>{};
    }

    bool input(uint8_t byte, T& out) {
        buffer_.push(byte);

        while (buffer_.size() >= sizeof(T)) {

            // 匹配帧头（多字节）
            if (!match_head()) {
                buffer_.pop(1);
                continue;
            }

            if (buffer_.size() < sizeof(T))
                return false;

            // 拷贝数据
            uint8_t temp[sizeof(T)];
            for (size_t i = 0; i < sizeof(T); i++) {
                temp[i] = buffer_.at(i);
            }

            std::memcpy(&out, temp, sizeof(T));

            if (!validate_(out)) {
                // 只移走一个字节，合法帧可能从伪帧头候选的中间开始。
                buffer_.pop(1);
                continue;
            }
            buffer_.pop(sizeof(T));
            return true;
        }
        return false;
    }

  private:
    static constexpr size_t HEAD_SIZE = 2;

    bool match_head() {
        for (size_t i = 0; i < HEAD_SIZE; i++) {
            if (buffer_.at(i) != head_[i])
                return false;
        }
        return true;
    }

  private:
    std::function<bool(const T&)> validate_;
    RingBuffer<N> buffer_;
    uint8_t head_[HEAD_SIZE];
};

// ==================== Serial ====================
class Serial {
  public:
    Serial();
    ~Serial();

    bool open(const std::string& device, int baudrate);
    void close();
    bool is_open() const;

    using WriteAttempt = std::function<void()>;
    // 每次非阻塞写入前执行守卫。守卫可持上层状态锁，并同步调用一次 attempt。
    // 返回 false 表示撤销：已写部分帧时关闭本次连接，避免后续指令续接到残帧。
    // 守卫不得保存 attempt，也不得递归调用 write；串口锁在调用守卫前尚未持有。
    using WriteGuard = std::function<bool(size_t sent, const WriteAttempt& attempt)>;
    // 同连接按调用取得发送权的顺序串行写入；取得发送权后最多等待 250 ms。
    // 返回实际写入字节数，超时或不可恢复的错误会关闭本次连接。
    size_t write(const uint8_t* data, size_t size, const WriteGuard& guard = {});

    template <typename T> size_t send(const T& data) {
        return write(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
    }

    template <typename T>
    void recv(
        std::function<void(const T&)> cb,
        std::function<bool(const T&)> validate = [](const T&) { return true; }) {
        struct Receiver {
            StructParser<T> parser;
            uint64_t session = 0;
            std::function<void(const T&)> callback;
        };
        auto receiver = std::make_shared<Receiver>();
        receiver->callback = std::move(cb);
        receiver->parser = StructParser<T>{std::move(validate)};
        auto wrapper = [this, receiver](const uint8_t* data, size_t len, uint64_t session) {
            if (receiver->session != session) {
                receiver->parser.reset();
                receiver->session = session;
            }
            for (size_t i = 0; i < len; i++) {
                if (!is_current_session(session))
                    return;
                T pkt;
                if (receiver->parser.input(data[i], pkt)) {
                    receiver->callback(pkt);
                }
            }
        };

        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callbacks_.push_back(wrapper);
    }

    void spin_once();

  private:
    boost::asio::io_context io_;
    boost::asio::serial_port serial_;
    mutable std::mutex state_mutex_;
    std::condition_variable state_changed_;
    bool is_open_ = false;
    uint64_t session_ = 0;
    std::mutex read_mutex_;
    std::mutex write_mutex_;
    std::mutex callbacks_mutex_;
    std::vector<std::function<void(const uint8_t*, size_t, uint64_t)>> callbacks_;

    void close_locked();
    void close_session(uint64_t session);
    bool is_current_session(uint64_t session) const;
    bool wait_ready(int fd, uint64_t session, short events, int timeout_ms);
    static constexpr const char* MODULE = "SERIAL";
};

} // namespace io

#endif // TGU_ROBOCORE_2027_SERIAL_HPP
