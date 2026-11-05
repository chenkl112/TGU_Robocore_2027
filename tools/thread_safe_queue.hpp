#pragma once

#include <condition_variable>
#include <cstddef>
#include <functional>
#include <mutex>
#include <optional>
#include <queue>
#include <stdexcept>
#include <stop_token>
#include <utility>

namespace tools {

class QueueClosed : public std::runtime_error {
  public:
    QueueClosed() : std::runtime_error("queue is closed") {}
};

// 有界消息队列：满时可丢弃最旧消息；关闭后拒绝写入，已有消息仍可被取完。
// 等待支持停止令牌，消费者不必依赖特殊数据或轮询来退出。
template <typename T, bool PopWhenFull = false> class ThreadSafeQueue {
  public:
    explicit ThreadSafeQueue(
        std::size_t capacity, std::function<void()> full_handler = [] {})
        : capacity_(capacity), full_handler_(std::move(full_handler)) {
        if (capacity == 0)
            throw std::invalid_argument("queue capacity must be positive");
    }

    bool push(T value) {
        std::unique_lock lock(mutex_);
        if (closed_)
            return false;
        if (queue_.size() == capacity_) {
            if constexpr (PopWhenFull) {
                queue_.pop();
            } else {
                // 用户回调可能访问队列，不能在持锁期间执行。
                lock.unlock();
                full_handler_();
                return false;
            }
        }
        queue_.push(std::move(value));
        condition_.notify_one();
        return true;
    }

    std::optional<T> wait_pop(std::stop_token stop = {}) {
        std::unique_lock lock(mutex_);
        condition_.wait(lock, stop, [this] { return closed_ || !queue_.empty(); });
        if (stop.stop_requested() || queue_.empty())
            return std::nullopt;
        T value = std::move(queue_.front());
        queue_.pop();
        return value;
    }

    T pop() {
        auto value = wait_pop();
        if (!value)
            throw QueueClosed();
        return std::move(*value);
    }

    void pop(T& value) {
        value = pop();
    }

    std::optional<T> try_pop() {
        std::lock_guard lock(mutex_);
        if (queue_.empty())
            return std::nullopt;
        T value = std::move(queue_.front());
        queue_.pop();
        return value;
    }

    bool empty() const {
        std::lock_guard lock(mutex_);
        return queue_.empty();
    }

    void close() {
        std::lock_guard lock(mutex_);
        closed_ = true;
        condition_.notify_all();
    }

    void clear() {
        std::lock_guard lock(mutex_);
        std::queue<T> empty;
        queue_.swap(empty);
    }

  private:
    std::queue<T> queue_;
    const std::size_t capacity_;
    std::function<void()> full_handler_;
    mutable std::mutex mutex_;
    std::condition_variable_any condition_;
    bool closed_ = false;
};

} // namespace tools
