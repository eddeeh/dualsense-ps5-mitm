#pragma once

// HCI commands, one at a time, the way the controller asks for them.
//
// Every Command Complete and Command Status carries Num_HCI_Command_Packets:
// how many commands the controller will accept right now. The ASUS BT400 says
// 1 in every answer it gives. The relay used to ignore that and write a whole
// setup at once - eleven commands inside a tenth of a millisecond - and the
// controller coped by quietly dropping some. Write Class of Device and Write
// Extended Inquiry Response never once got an answer, in any capture, so the
// console-facing adapter had in fact never been given the gamepad class it was
// supposed to present.
//
// It stopped coping once the relay began resetting the controller at start: a
// command from the other handler landed inside the 306 ms the BT400 takes to
// reset, and a few hundred milliseconds later the controller stopped answering
// anything, the call to the console included.
//
// So commands queue, and one goes out only while the controller has room for
// it. A reset holds the queue shut until the controller confirms it. A command
// the controller never answers is given up on after a timeout rather than
// wedging everything behind it.
//
// No I/O in here: the caller writes what next() hands back, and reports
// answers and writes, so the policy can be tested without a radio.

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>
#include <vector>

class HciCommandQueue {
public:
    using Clock = std::chrono::steady_clock;

    struct Command {
        uint16_t ogf = 0;
        uint16_t ocf = 0;
        std::vector<uint8_t> param;
        std::string what;
        bool quiet = false;         // a failure is expected and not worth a log line
    };

    // Two seconds is past anything a working controller takes - its slowest
    // answer seen here is the reset at 306 ms - and short enough that a
    // command lost inside a wedged one does not hold up a link key reply for
    // long.
    static constexpr auto DEFAULT_TIMEOUT = std::chrono::seconds(2);

    void push(Command c) { queue_.push_back(std::move(c)); }

    // The command to write now, if there is one and the controller has room.
    // Does not take it off the queue; written() does, once it is actually out.
    const Command *next() const
    {
        if (held_ || credits_ == 0 || queue_.empty())
            return nullptr;
        return &queue_.front();
    }

    // The command next() returned has been written.
    void written(Clock::time_point now)
    {
        if (queue_.empty())
            return;
        queue_.pop_front();
        if (credits_ > 0)
            credits_--;
        last_write_ = now;
        awaiting_ = true;
    }

    // The write failed: it never reached the controller, so it costs no room.
    void dropped()
    {
        if (!queue_.empty())
            queue_.pop_front();
    }

    // A Command Complete or Command Status - for any opcode, the no-op one
    // included. The count is the controller's allowance as of now, not a delta.
    void answered(uint8_t ncmd)
    {
        credits_ = ncmd;
        awaiting_ = false;
    }

    // Nothing goes out while held; for the length of a reset.
    void hold() { held_ = true; }
    void release() { held_ = false; }
    bool held() const { return held_; }

    // A write that has had no answer for longer than the timeout.
    bool overdue(Clock::time_point now) const
    {
        return awaiting_ && credits_ == 0 && now - last_write_ > timeout_;
    }

    // Stop waiting for it and let the queue move again.
    void give_up()
    {
        awaiting_ = false;
        credits_ = 1;
    }

    size_t queued() const { return queue_.size(); }
    uint8_t credits() const { return credits_; }
    void set_timeout(Clock::duration d) { timeout_ = d; }

private:
    std::deque<Command> queue_;
    uint8_t credits_ = 1;           // what a controller allows before it has said anything
    bool held_ = false;
    bool awaiting_ = false;
    Clock::time_point last_write_{};
    Clock::duration timeout_ = DEFAULT_TIMEOUT;
};
