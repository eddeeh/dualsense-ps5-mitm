// The HCI command queue: one command at a time while the controller says so.
#include "check.h"

#include "bluetooth/HciCommandQueue.h"

namespace {

using Clock = HciCommandQueue::Clock;
const Clock::time_point T0 = Clock::time_point{} + std::chrono::hours(1);

HciCommandQueue::Command cmd(uint16_t ocf)
{
    HciCommandQueue::Command c;
    c.ogf = 0x03;
    c.ocf = ocf;
    return c;
}

TEST(the_first_command_goes_straight_out)
{
    HciCommandQueue q;
    q.push(cmd(1));
    CHECK(q.next() != nullptr);
    CHECK_EQ(int(q.next()->ocf), 1);
}

TEST(the_second_waits_for_the_first_to_be_answered)
{
    // What the BT400 asks for in every answer it gives, and what the relay
    // used to ignore by writing eleven at once.
    HciCommandQueue q;
    q.push(cmd(1));
    q.push(cmd(2));
    q.written(T0);
    CHECK(q.next() == nullptr);
    q.answered(1);
    CHECK(q.next() != nullptr);
    CHECK_EQ(int(q.next()->ocf), 2);
}

TEST(order_is_kept)
{
    HciCommandQueue q;
    for (uint16_t i = 1; i <= 5; i++)
        q.push(cmd(i));
    for (uint16_t i = 1; i <= 5; i++) {
        CHECK(q.next() != nullptr);
        CHECK_EQ(int(q.next()->ocf), int(i));
        q.written(T0);
        q.answered(1);
    }
    CHECK(q.next() == nullptr);
}

TEST(an_allowance_of_nought_holds_everything_until_it_is_raised)
{
    // A controller may answer and still say it has no room. The answer does
    // not by itself mean the next command can go.
    HciCommandQueue q;
    q.push(cmd(1));
    q.push(cmd(2));
    q.written(T0);
    q.answered(0);
    CHECK(q.next() == nullptr);
    q.answered(1);
    CHECK(q.next() != nullptr);
}

TEST(a_larger_allowance_lets_several_go)
{
    HciCommandQueue q;
    q.push(cmd(1));
    q.push(cmd(2));
    q.push(cmd(3));
    q.answered(2);
    q.written(T0);
    CHECK(q.next() != nullptr);
    q.written(T0);
    CHECK(q.next() == nullptr);
}

TEST(nothing_goes_out_while_held_and_everything_after)
{
    // A reset: the other handler's command landed inside the BT400's 306 ms of
    // resetting and the controller stopped answering soon after.
    HciCommandQueue q;
    q.hold();
    q.push(cmd(0x1a));
    CHECK(q.next() == nullptr);
    q.answered(1);           // the reset's own Command Complete
    CHECK(q.next() == nullptr);
    q.release();
    CHECK(q.next() != nullptr);
    CHECK_EQ(int(q.next()->ocf), 0x1a);
}

TEST(an_unanswered_command_is_given_up_on)
{
    HciCommandQueue q;
    q.set_timeout(std::chrono::milliseconds(100));
    q.push(cmd(1));
    q.push(cmd(2));
    q.written(T0);
    CHECK(!q.overdue(T0 + std::chrono::milliseconds(50)));
    CHECK(q.overdue(T0 + std::chrono::milliseconds(150)));
    q.give_up();
    CHECK(!q.overdue(T0 + std::chrono::milliseconds(150)));
    CHECK(q.next() != nullptr);
    CHECK_EQ(int(q.next()->ocf), 2);
}

TEST(a_failed_write_costs_no_room)
{
    HciCommandQueue q;
    q.push(cmd(1));
    q.push(cmd(2));
    q.dropped();
    CHECK(q.next() != nullptr);
    CHECK_EQ(int(q.next()->ocf), 2);
    CHECK_EQ(int(q.credits()), 1);
}

}  // namespace
