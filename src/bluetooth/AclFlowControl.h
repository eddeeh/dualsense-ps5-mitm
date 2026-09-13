#pragma once

// ACL flow control, lifted out of BluetoothHandler unchanged.
//
// On an HCI user channel the kernel does no flow control, so this is the only
// thing between us and a queue thousands of packets deep. It tracks how many
// ACL packets the controller still holds, per link, against the pool size it
// reported, and answers whether there is room for another - keeping the input
// stream shallow so queued reports are not felt as lag. The numbers behind the
// depth and the write-off threshold are with the code that uses them, below.

#include <cstdint>
#include <map>
#include <chrono>
#include <algorithm>
#include <iostream>

class AclFlowControl {
public:
    // Adapter index, only for log lines.
    void set_hci_id(uint32_t id) { hci_id_ = id; }

    // Pool size from Read Buffer Size. Zero means the controller has not
    // answered yet, which reads as "no metering".
    void set_max_packets(uint16_t n) { max_packets_ = n; }
    uint16_t max_packets() const { return max_packets_; }

    // A packet was written to `handle`.
    void on_sent(uint16_t handle) {
        in_flight_[handle]++;
        total_++;
    }

    // Number of Completed Packets: the controller freed `done` buffers on `handle`.
    void on_completed(uint16_t handle, int done) {
        auto it = in_flight_.find(handle);
        if (it == in_flight_.end())
            return;
        const int was = it->second;
        it->second = std::max(0, was - done);
        total_ -= was - it->second;
        last_completion_ = std::chrono::steady_clock::now();
    }

    // A link went away: write off whatever it still held.
    void on_disconnect(uint16_t handle) {
        if (auto it = in_flight_.find(handle); it != in_flight_.end()) {
            total_ -= it->second;
            in_flight_.erase(it);
        }
    }

    // A running total rather than a walk of the map - has_room() asks on every
    // report, and summing a one- or two-entry map was among the hottest symbols.
    int in_flight_total() const { return total_; }

    bool has_room() const {
        if (max_packets_ == 0)
            return true;  // controller never told us; nothing to meter against

        // Everything here assumes the controller gives its buffers back. When it
        // does not, the budget leaks a packet at a time until nothing can be sent,
        // on a link that otherwise looks perfectly healthy - silent and total. So
        // there is a way out: if we are blocked and nothing has completed for long
        // enough, the credits are written off.
        //
        // "Long enough" has to clear every legitimate delay, and on this link they
        // add up: the console sniffs the registration radio at 223.75 ms and the
        // controller batches its completion events on a 250 ms timer, so half a
        // second with nothing acknowledged is ordinary rather than broken. A 500 ms
        // threshold turned that into a feedback loop - write off two credits, send
        // two more into a link that delivers four packets a second, repeat every
        // half second - and the link died. Three seconds is comfortably past
        // anything the sniff and the batching can produce together, and still
        // catches a real stall long before a player notices; the genuine ones
        // observed ran to twenty seconds and more.
        if (total_ > 0) {
            const auto idle = std::chrono::steady_clock::now() - last_completion_;
            if (idle > write_off_after_) {
                std::cerr << "hci" << hci_id_ << " controller has not returned "
                          << total_ << " buffer(s) for "
                          << std::chrono::duration_cast<std::chrono::milliseconds>(idle).count()
                          << " ms - writing them off" << std::endl;
                in_flight_.clear();
                total_ = 0;
                last_completion_ = std::chrono::steady_clock::now();
                return true;
            }
        }
        // Two, and both neighbours are measured rather than reasoned.
        //
        // One is catastrophic. Held to a single packet in flight, the controller
        // stops acknowledging promptly and falls back to batching completions on
        // a 250 ms timer: median 250.0 ms between Number of Completed Packets
        // events, p90 250.9, so we send one and wait for that timer before the
        // next may go - 4 packets a second instead of 105, 248 ms between
        // transmits. The second slot is not there to avoid missing a poll, as an
        // earlier version of this comment guessed; it is there to keep the
        // completion pipeline from drying up.
        //
        // Those 250 ms are the depth-1 measurement, not a standing property of
        // the part - worth saying, because this comment read as though the timer
        // were always there. At depth 2 completions come back with the console's
        // polling: median 8.7 ms, p90 11.2, measured by replay_capture over
        // 616989 packets and three sessions, against the 8.0 ms recorded at the
        // time.
        //
        // More than two buys nothing *on the console link*, and the reason is
        // not the one this comment used to give. It said the console takes
        // roughly one packet per poll, so two already matches its rate; but
        // every completion returns the full budget of two, which is what a cap
        // of two looks like from the inside and tells us nothing about the
        // console's appetite. The real reason is that a stall there ends when
        // the console next opens its window, and no queue of ours moves that.
        //
        // Which is also why this is per link now rather than one constant for
        // both. On the pad link we are the master and the dynamics are not
        // comparable - see set_depth().
        return total_ < budget();
}

    // Two is the console link's number. The pad link's is three, and it was
    // measured: that controller returns credits two at a time every 20 ms
    // rather than one at a time every 10 ms, so the window in which has_room()
    // is false is twice as long, and the console's output - rumble, trigger
    // effects, haptic audio - is dropped rather than queued when it lands in
    // one. That cost 3.90% of the output stream against 0.13% before.
    //
    // Counting the console's output into 20 ms windows over a 50 minute
    // session says what each depth would cost: 3.68% at two, 0.25% at three,
    // 0.04% at four. The model's 3.68% against the counters' 3.90% is close
    // enough to trust the rest of it. Three takes out 93% of the loss for at
    // most one extra window of queueing; four would take out 99% for two, and
    // an effect delivered late is worth little, so three it is.
    void set_depth(int d) { depth_ = d; }

    // Only so the tests can reach the write-off above without waiting three
    // seconds for it. Nothing in the relay calls this: the threshold is tuned,
    // not configuration, and the comment on it says what tuning it costs.
    void set_write_off_after(std::chrono::steady_clock::duration d)
    {
        write_off_after_ = d;
    }

    // What has_room() actually meters against, which is not max_packets_. The
    // stats line used to print the pool size next to the in-flight count, so a
    // link pinned at its cap read as "2 of 8, plenty spare".
    int budget() const {
        // Before Read Buffer Size is answered there is nothing to meter
        // against and has_room() lets everything through, so report the depth
        // rather than the arithmetic: max_packets_ - 1 on an unsigned zero
        // promotes to -1, which the stats line duly printed as "acl 0/-1".
        if (max_packets_ == 0)
            return depth_;
        // One buffer is always kept free so an L2CAP response or an HCI-driven
        // procedure never finds the pool exhausted by the input stream.
        return std::min<int>(depth_, max_packets_ - 1);
}

private:
    uint32_t hci_id_ = 0;
    uint16_t max_packets_ = 0;
    int depth_ = 2;
    std::chrono::steady_clock::duration write_off_after_ = std::chrono::seconds(3);
    mutable std::map<uint16_t, int> in_flight_;
    mutable int total_ = 0;
    mutable std::chrono::steady_clock::time_point last_completion_ =
        std::chrono::steady_clock::now();
};
