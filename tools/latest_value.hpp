#pragma once

#include <mutex>
#include <utility>

namespace tools {

// 保存一个完整的最新值；读取返回独立副本，不等待生产者，也不消费当前值。
// 适用于跨线程传递小型状态快照，不应放入设备句柄或可变共享对象。
template <typename T> class LatestValue {
  public:
    explicit LatestValue(T initial = {}) : value_(std::move(initial)) {}

    void publish(T value) {
        std::lock_guard lock(mutex_);
        value_ = std::move(value);
    }

    T read() const {
        std::lock_guard lock(mutex_);
        return value_;
    }

  private:
    mutable std::mutex mutex_;
    T value_;
};

} // namespace tools
