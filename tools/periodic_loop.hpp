#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <stop_token>
#include <type_traits>

namespace tools {

// 截止时间以单调时钟推进；超期后跳过错过的周期，避免连续补发旧控制周期。
class PeriodicSchedule {
  public:
    using Clock = std::chrono::steady_clock;

    explicit PeriodicSchedule(Clock::duration period, Clock::time_point start = Clock::now())
        : period_(period), deadline_(start) {
        if (period <= Clock::duration::zero()) {
            throw std::invalid_argument("control period must be positive");
        }
    }

    Clock::time_point next(Clock::time_point now) {
        using Rep = Clock::duration::rep;
        using Unsigned = std::make_unsigned_t<Rep>;
        const auto period = period_.count();
        const auto maximum = std::numeric_limits<Rep>::max();
        auto next_tick = deadline_.time_since_epoch().count();
        if (next_tick > maximum - period) {
            throw std::overflow_error("control schedule exceeds clock range");
        }
        next_tick += period;
        const auto observed = now.time_since_epoch().count();
        if (next_tick < observed) {
            // 跨越时钟零点时，先做无符号差，避免极端时间戳的有符号减法溢出。
            const auto difference =
                static_cast<Unsigned>(observed) - static_cast<Unsigned>(next_tick);
            const auto skipped = difference / static_cast<Unsigned>(period) + 1;
            const auto advance =
                period - static_cast<Rep>(difference % static_cast<Unsigned>(period));
            if (observed > maximum - advance ||
                skipped > std::numeric_limits<std::uint64_t>::max() - missed_) {
                throw std::overflow_error("control schedule exceeds clock range");
            }
            next_tick = observed + advance;
            missed_ += skipped;
        }
        deadline_ = Clock::time_point(Clock::duration(next_tick));
        return deadline_;
    }

    std::uint64_t missed() const {
        return missed_;
    }

  private:
    Clock::duration period_;
    Clock::time_point deadline_;
    std::uint64_t missed_ = 0;
};

inline bool wait_until(std::stop_token stop, PeriodicSchedule::Clock::time_point deadline) {
    std::mutex mutex;
    std::condition_variable_any condition;
    std::unique_lock lock(mutex);
    condition.wait_until(lock, stop, deadline, [] { return false; });
    return !stop.stop_requested();
}

} // namespace tools
