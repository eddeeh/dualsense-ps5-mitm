#pragma once

// How long the console went without an input report, in buckets a panel can
// draw.
//
// The relay's counters say how many reports were sent and how many were held;
// neither can show a hole. A 135 ms gap - the console going quiet while it
// streams haptic audio - moves the five-second totals by a few dozen reports
// out of sixteen hundred, which is inside their ordinary jitter. So the one
// failure a player can actually feel was, until this, visible only by walking a
// capture packet by packet.
//
// One bucket a second: the worst gap in it and how many crossed the threshold.
// A completed bucket is what gets published, so a reader sees a settled number
// rather than one still being accumulated, and the sequence number tells it
// when a new one is ready.

#include <chrono>
#include <cstdint>

class SendGaps {
public:
    using Clock = std::chrono::steady_clock;

    // A hundred milliseconds is what PERFORMANCE.md calls a stall, so the panel
    // and the capture analysis count the same events. At a 3 ms median it is
    // also rare enough that a nonzero count means something happened.
    static constexpr auto DEFAULT_THRESHOLD = std::chrono::milliseconds(100);
    static constexpr auto DEFAULT_BUCKET    = std::chrono::seconds(1);

    // Every report handed to the console link, in the order they go out.
    void on_sent(Clock::time_point now)
    {
        if (last_ != Clock::time_point{}) {
            const auto gap = now - last_;
            if (gap > cur_max_)
                cur_max_ = gap;
            if (gap > threshold_)
                cur_count_++;
        }
        last_ = now;

        if (bucket_start_ == Clock::time_point{}) {
            bucket_start_ = now;
            return;
        }
        if (now - bucket_start_ < bucket_)
            return;
        done_max_ = cur_max_;
        done_count_ = cur_count_;
        window_++;
        cur_max_ = Clock::duration::zero();
        cur_count_ = 0;
        bucket_start_ = now;
    }

    // Increments once per completed bucket; a reader redraws when it changes.
    uint32_t window() const { return window_; }
    uint32_t count() const { return done_count_; }
    uint32_t max_us() const
    {
        return uint32_t(std::chrono::duration_cast<std::chrono::microseconds>(done_max_).count());
    }

    void set_threshold(Clock::duration d) { threshold_ = d; }
    void set_bucket(Clock::duration d) { bucket_ = d; }

private:
    Clock::duration threshold_ = DEFAULT_THRESHOLD;
    Clock::duration bucket_ = DEFAULT_BUCKET;
    Clock::time_point last_{};
    Clock::time_point bucket_start_{};
    Clock::duration cur_max_ = Clock::duration::zero();
    uint32_t cur_count_ = 0;
    Clock::duration done_max_ = Clock::duration::zero();
    uint32_t done_count_ = 0;
    uint32_t window_ = 0;
};
