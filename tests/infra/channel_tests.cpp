#include "tools/latest_value.hpp"
#include "tools/periodic_loop.hpp"
#include "tools/thread_safe_queue.hpp"

#include <array>
#include <atomic>
#include <barrier>
#include <chrono>
#include <cstdint>
#include <future>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using namespace std::chrono_literals;

void require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

void queue_close_and_stop() {
    tools::ThreadSafeQueue<int> queue(2);
    std::array<std::promise<void>, 2> entered;
    std::array<std::promise<std::optional<int>>, 2> results;
    std::array<std::future<std::optional<int>>, 2> futures{results[0].get_future(),
                                                           results[1].get_future()};
    std::vector<std::jthread> waiters;
    for (std::size_t i = 0; i < results.size(); ++i) {
        auto started = entered[i].get_future();
        waiters.emplace_back([&, i](std::stop_token stop) {
            entered[i].set_value();
            results[i].set_value(queue.wait_pop(stop));
        });
        started.wait();
        require(futures[i].wait_for(20ms) == std::future_status::timeout,
                "Empty open queue did not block its consumer");
    }
    queue.close();
    queue.close();
    for (auto& future : futures) {
        require(future.wait_for(1s) == std::future_status::ready && !future.get(),
                "Queue close did not wake every consumer");
    }
    require(!queue.push(42), "Closed queue accepted a write");

    tools::ThreadSafeQueue<int> cancellable(1);
    std::promise<void> entered_stop;
    std::promise<std::optional<int>> stopped_result;
    auto started = entered_stop.get_future();
    auto stopped = stopped_result.get_future();
    std::jthread waiter([&](std::stop_token stop) {
        entered_stop.set_value();
        stopped_result.set_value(cancellable.wait_pop(stop));
    });
    started.wait();
    require(stopped.wait_for(20ms) == std::future_status::timeout,
            "Cancellable wait returned before cancellation");
    waiter.request_stop();
    require(stopped.wait_for(1s) == std::future_status::ready && !stopped.get(),
            "Stop token did not wake the blocked queue consumer");
    require(cancellable.push(7), "Cancelling a consumer closed the queue");
    std::stop_source already_stopped;
    already_stopped.request_stop();
    require(!cancellable.wait_pop(already_stopped.get_token()),
            "Pre-stopped consumer unexpectedly consumed a message");
    require(cancellable.pop() == 7, "Cancelled wait discarded the queued message");
}

void queue_ownership_and_overflow() {
    bool invalid_capacity = false;
    try {
        tools::ThreadSafeQueue<int> invalid(0);
    } catch (const std::invalid_argument&) {
        invalid_capacity = true;
    }
    require(invalid_capacity, "Zero-capacity queue was accepted");

    tools::ThreadSafeQueue<int> draining(2);
    require(draining.push(1) && draining.push(2), "Queue lost messages before closing");
    draining.close();
    require(draining.pop() == 1 && draining.pop() == 2 && !draining.wait_pop(),
            "Closing the queue did not preserve FIFO draining");
    bool closed_exception = false;
    try {
        draining.pop();
    } catch (const tools::QueueClosed&) {
        closed_exception = true;
    }
    require(closed_exception, "Legacy pop did not report the closed and drained queue");

    tools::ThreadSafeQueue<int, true> newest(2);
    require(newest.push(1) && newest.push(2) && newest.push(3),
            "Drop-oldest queue rejected a frame");
    require(newest.pop() == 2 && newest.pop() == 3 && newest.empty(),
            "Full frame queue did not discard exactly the oldest frame");

    tools::ThreadSafeQueue<std::unique_ptr<int>> ownership(1);
    auto value = std::make_unique<int>(17);
    const auto* address = value.get();
    require(ownership.push(std::move(value)) && !value, "Queue did not take unique ownership");
    auto transferred = ownership.wait_pop();
    require(transferred && transferred->get() == address && **transferred == 17,
            "Queue copied or lost a uniquely owned message");

    // 满队列回调重新读写同一个队列，验证回调确实在锁外执行。
    tools::ThreadSafeQueue<int>* self = nullptr;
    int callbacks = 0;
    tools::ThreadSafeQueue<int> reentrant(1, [&] {
        ++callbacks;
        require(self->pop() == 10, "Full handler observed the wrong original message");
        require(self->push(30), "Full handler could not reenter queue push");
    });
    self = &reentrant;
    require(reentrant.push(10) && !reentrant.push(20),
            "Full queue did not reject the incoming message");
    require(callbacks == 1 && reentrant.pop() == 30,
            "Full callback ran more than once or lost its replacement message");
    require(reentrant.push(40), "Queue could not be reused after reentrant callback");
    reentrant.clear();
    require(reentrant.empty() && !reentrant.try_pop(), "Queue clear retained messages");
}

void latest_value_consistency() {
    struct Sample {
        std::uint64_t sequence;
        std::uint64_t inverse;
        std::array<std::uint64_t, 8> payload;
    };
    tools::LatestValue<Sample> latest({0, ~std::uint64_t{0}, {}});
    std::barrier begin(4);
    std::atomic<bool> done = false;
    std::atomic<bool> coherent = true;
    std::atomic<std::uint64_t> reads = 0;
    std::jthread writer([&] {
        begin.arrive_and_wait();
        for (std::uint64_t i = 1; i <= 20000; ++i) {
            Sample sample{i, ~i, {}};
            sample.payload.fill(i);
            latest.publish(sample);
        }
        done = true;
    });
    std::vector<std::jthread> readers;
    for (int reader = 0; reader < 3; ++reader) {
        readers.emplace_back([&] {
            begin.arrive_and_wait();
            do {
                const auto sample = latest.read();
                if (sample.inverse != ~sample.sequence)
                    coherent = false;
                for (const auto value : sample.payload) {
                    if (value != sample.sequence)
                        coherent = false;
                }
                ++reads;
            } while (!done.load());
        });
    }
    writer.join();
    for (auto& reader : readers)
        reader.join();
    require(coherent && reads > 0, "LatestValue exposed a partially published snapshot");
    auto copy = latest.read();
    copy.payload.fill(0);
    require(latest.read().sequence == 20000 && latest.read().payload[0] == 20000,
            "Reading consumed the latest value or returned mutable shared state");
}

void periodic_deadlines_and_stop() {
    using Clock = tools::PeriodicSchedule::Clock;
    const auto start = Clock::time_point{} + 1s;
    tools::PeriodicSchedule schedule(10ms, start);
    require(schedule.next(start) == start + 10ms, "First deadline was not one period after start");
    require(schedule.next(start + 10ms) == start + 20ms,
            "Periodic schedule drifted after on-time work");
    require(schedule.next(start + 35ms) == start + 40ms && schedule.missed() == 1,
            "Schedule did not skip the first missed deadline");
    require(schedule.next(start + 50ms) == start + 50ms && schedule.missed() == 1,
            "An exact deadline was incorrectly counted as missed");
    require(schedule.next(start + 95ms) == start + 100ms && schedule.missed() == 5,
            "Schedule attempted to catch up instead of skipping expired periods");
    bool invalid_period = false;
    try {
        tools::PeriodicSchedule invalid(Clock::duration::zero());
    } catch (const std::invalid_argument&) {
        invalid_period = true;
    }
    require(invalid_period, "Zero control period was accepted");
    // 不可表示的下一个截止时间应明确失败，且不能污染后续可表示的调度状态。
    tools::PeriodicSchedule boundary(Clock::duration{1}, Clock::time_point::min());
    bool overflow_rejected = false;
    try {
        boundary.next(Clock::time_point::max());
    } catch (const std::overflow_error&) {
        overflow_rejected = true;
    }
    require(overflow_rejected && boundary.missed() == 0 &&
                boundary.next(Clock::time_point::min()) ==
                    Clock::time_point::min() + Clock::duration{1},
            "Schedule overflow changed state or was not rejected");
    require(tools::wait_until({}, Clock::now() - 1ms), "Expired deadline did not finish normally");

    std::promise<void> entered;
    std::promise<bool> result;
    auto started = entered.get_future();
    auto stopped = result.get_future();
    std::jthread waiter([&](std::stop_token stop) {
        entered.set_value();
        result.set_value(tools::wait_until(stop, Clock::now() + 5s));
    });
    started.wait();
    require(stopped.wait_for(20ms) == std::future_status::timeout,
            "Periodic wait finished before its deadline or cancellation");
    waiter.request_stop();
    require(stopped.wait_for(1s) == std::future_status::ready && !stopped.get(),
            "Periodic wait did not stop promptly");
}
} // namespace

int main() {
    try {
        queue_close_and_stop();
        queue_ownership_and_overflow();
        latest_value_consistency();
        periodic_deadlines_and_stop();
        std::cout << "PASS queue lifecycle, ownership, reentry, latest snapshots and periodic "
                     "scheduling\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL " << error.what() << '\n';
        return 1;
    }
}
