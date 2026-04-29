//
// Created by Interweave on 2026/4/10.
//

#ifndef TGU_ROBOCORE_2027_SERIAL_HPP
#define TGU_ROBOCORE_2027_SERIAL_HPP

#pragma once
#include <boost/asio.hpp>
#include <functional>
#include <vector>
#include <cstring>

namespace io {

// ==================== RingBuffer ====================
template<size_t N>
class RingBuffer {
public:
    void push(uint8_t byte) {
        buffer_[write_] = byte;
        write_ = (write_ + 1) % N;

        if (size_ < N) size_++;
        else read_ = (read_ + 1) % N;
    }

    uint8_t at(size_t idx) const {
        return buffer_[(read_ + idx) % N];
    }

    void pop(size_t n) {
        read_ = (read_ + n) % N;
        size_ -= n;
    }

    size_t size() const { return size_; }

private:
    uint8_t buffer_[N];
    size_t read_ = 0;
    size_t write_ = 0;
    size_t size_ = 0;
};

// ==================== Parser ====================
template<typename T, size_t N = 2048>
class StructParser {
public:
    StructParser() {
        T dummy{};

        std::memcpy(head_, &dummy, HEAD_SIZE);
    }

    bool input(uint8_t byte, T& out) {
        buffer_.push(byte);

        while (buffer_.size() >= sizeof(T)) {

            // 匹配帧头（多字节）
            if (!match_head()) {
                buffer_.pop(1);
                continue;
            }

            if (buffer_.size() < sizeof(T)) return false;

            // 拷贝数据
            uint8_t temp[sizeof(T)];
            for (size_t i = 0; i < sizeof(T); i++) {
                temp[i] = buffer_.at(i);
            }

            std::memcpy(&out, temp, sizeof(T));

            buffer_.pop(sizeof(T));
            return true;
        }
        return false;
    }

private:
    static constexpr size_t HEAD_SIZE = 2;

    bool match_head() {
        for (size_t i = 0; i < HEAD_SIZE; i++) {
            if (buffer_.at(i) != head_[i]) return false;
        }
        return true;
    }

private:
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

    size_t write(const uint8_t* data, size_t size);

    template<typename T>
    size_t send(const T& data) {
        return write(reinterpret_cast<const uint8_t*>(&data), sizeof(T));
    }

    template<typename T>
    void recv(std::function<void(const T&)> cb) {

        auto wrapper = [this, cb](const uint8_t* data, size_t len) {
            for (size_t i = 0; i < len; i++) {
                T pkt;
                if (parser_<T>.input(data[i], pkt)) {
                    cb(pkt);
                }
            }
        };

        callbacks_.push_back(wrapper);
    }

    void spin_once();

private:
    boost::asio::io_context io_;
    boost::asio::serial_port serial_;
    bool is_open_;

    uint8_t rx_buf_[512];

    std::vector<std::function<void(const uint8_t*, size_t)>> callbacks_;

    template<typename T>
    static StructParser<T> parser_;
    static constexpr const char* MODULE = "SERIAL";
};

template<typename T>
StructParser<T> Serial::parser_;

} // namespace io

#endif //TGU_ROBOCORE_2027_SERIAL_HPP
