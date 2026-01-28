#pragma once

#include <chrono>
#include <functional>

class SimpleTimer {
public:
    using Clock = std::chrono::steady_clock;
    using TimePoint = std::chrono::time_point<Clock>;
    using Milliseconds = std::chrono::milliseconds;

    SimpleTimer() : active(false), expired(false), duration(0) {}

    /**
     * @brief Start the timer with a specific duration.
     * @param duration_ms The duration in milliseconds.
     * @param callback Optional function to call when timer expires (if using polling logic).
     */
    void start(int duration_ms, std::function<void()> callback = nullptr) {
        start_time = Clock::now();
        duration = Milliseconds(duration_ms);
        active = true;
        expired = false;
        on_expire = callback;
    }

    /**
     * @brief Check if the timer has finished.
     * * @return true if the timer was active AND the duration has passed.
     * @return false if the timer is not active OR time hasn't passed yet.
     */
    bool isExpired() {
        if (!active) return false;
        
        if (Clock::now() - start_time >= duration) {
            expired = true;
            // Optionally auto-deactivate? usually we leave it active until stopped or restarted
            // depending on if you want one-shot or continuous checking behavior.
            // For one-shot behavior that stays true until reset:
            return true;
        }
        return false;
    }
    
    /**
     * @brief Check if the timer is currently counting down.
     */
    bool isActive() const {
        return active;
    }

    /**
     * @brief Stop the timer manually. isExpired() will return false immediately.
     */
    void cancel() {
        active = false;
        expired = false;
    }

    /**
     * @brief Restart the timer with the previously set duration.
     */
    void restart() {
        if (duration.count() > 0) {
            start(static_cast<int>(duration.count()), on_expire);
        }
    }

    /**
     * @brief Get the remaining time in milliseconds.
     * @return Remaining ms, or 0 if expired/inactive.
     */
    long long getRemaining() {
        if (!active) return 0;
        auto now = Clock::now();
        auto elapsed = std::chrono::duration_cast<Milliseconds>(now - start_time);
        auto remaining = duration - elapsed;
        return (remaining.count() > 0) ? remaining.count() : 0;
    }

private:
    TimePoint start_time;
    Milliseconds duration;
    bool active;
    bool expired;
    std::function<void()> on_expire;
};