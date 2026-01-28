#include <atomic>
#include <chrono>
#include <thread>
#include "Heartbeat.hpp"

class EthernetHBThread {
public:
    EthernetHBThread(HeartbeatLink& link,
                      std::chrono::milliseconds period = std::chrono::milliseconds(10))
        : link_(link), period_(period) {}

    void start() {
        if (running_.exchange(true)) return;
        worker_ = std::thread([this]{ run(); });
    }

    void stop() {
        if (!running_.exchange(false)) return;
        if (worker_.joinable()) worker_.join();
    }

    ~EthernetHBThread() { stop(); }

private:
    void run() {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();

        while (running_.load(std::memory_order_relaxed)) {
            next += period_;
            link_.send_heartbeat();
            std::this_thread::sleep_until(next);
        }
    }

    HeartbeatLink& link_;
    std::chrono::milliseconds period_;
    std::atomic<bool> running_{false};
    std::thread worker_;
};
