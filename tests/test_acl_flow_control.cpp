// The ACL credit accounting.
//
// On an HCI user channel the kernel does no flow control, so this class is the
// only thing between the relay and a queue thousands of packets deep. It is
// pure state over integers, which makes it the one piece of the transport that
// can be tested without a radio.
#include "check.h"

#include "bluetooth/AclFlowControl.h"

#include <thread>

namespace {

AclFlowControl metered(uint16_t pool = 8)
{
    AclFlowControl a;
    a.set_max_packets(pool);
    return a;
}

TEST(without_a_pool_size_nothing_is_metered)
{
    // Zero means the controller has not answered Read Buffer Size yet. Refusing
    // to send until it does would stall the run rather than protect it.
    AclFlowControl a;
    CHECK_EQ(int(a.max_packets()), 0);
    CHECK(a.has_room());
    for (int i = 0; i < 100; i++)
        a.on_sent(1);
    CHECK(a.has_room());
}

TEST(the_default_budget_is_two_packets_in_flight)
{
    // Measured, and the neighbour below matters: at a single slot this
    // controller batches completions on a 250 ms timer, so one in flight means
    // four packets a second instead of a hundred. Two is the console link's
    // number; the pad link's is three and the test below says why.
    auto a = metered();
    CHECK(a.has_room());
    a.on_sent(1);
    CHECK(a.has_room());
    a.on_sent(1);
    CHECK(!a.has_room());
    CHECK_EQ(a.in_flight_total(), 2);
}

TEST(a_completion_frees_the_credit_again)
{
    auto a = metered();
    a.on_sent(1);
    a.on_sent(1);
    CHECK(!a.has_room());

    a.on_completed(1, 1);
    CHECK_EQ(a.in_flight_total(), 1);
    CHECK(a.has_room());

    a.on_completed(1, 1);
    CHECK_EQ(a.in_flight_total(), 0);
    CHECK(a.has_room());
}

TEST(completions_never_drive_the_count_below_zero)
{
    // The controller reports completions per link, and a completion can arrive
    // for a packet this object never counted - signalling used to be sent
    // without being accounted at all. It must not hand out credits that were
    // never spent.
    auto a = metered();
    a.on_sent(1);
    a.on_completed(1, 5);
    CHECK_EQ(a.in_flight_total(), 0);
    CHECK(a.has_room());
}

TEST(a_completion_for_an_unknown_link_is_ignored)
{
    auto a = metered();
    a.on_sent(1);
    a.on_completed(99, 1);
    CHECK_EQ(a.in_flight_total(), 1);
}

TEST(links_are_metered_against_one_shared_pool)
{
    // The pool is shared even though completions are reported per link, so the
    // running total is what the budget is measured against.
    auto a = metered();
    a.on_sent(1);
    a.on_sent(2);
    CHECK_EQ(a.in_flight_total(), 2);
    CHECK(!a.has_room());

    a.on_completed(2, 1);
    CHECK(a.has_room());
}

TEST(a_dropped_link_writes_off_what_it_held)
{
    // Whatever the controller flushed for a link that no longer exists is never
    // going to be reported complete. Leaving it counted leaks the budget a
    // packet at a time until nothing can be sent.
    auto a = metered();
    a.on_sent(1);
    a.on_sent(2);
    CHECK(!a.has_room());

    a.on_disconnect(2);
    CHECK_EQ(a.in_flight_total(), 1);
    CHECK(a.has_room());

    a.on_disconnect(1);
    CHECK_EQ(a.in_flight_total(), 0);
}

TEST(disconnecting_an_unknown_link_changes_nothing)
{
    auto a = metered();
    a.on_sent(1);
    a.on_disconnect(42);
    CHECK_EQ(a.in_flight_total(), 1);
}

TEST(a_pool_of_one_still_leaves_a_buffer_free)
{
    // budget = min(depth, max_packets - 1), so a one-packet controller can
    // never send: the reserved buffer is the whole pool. Odd, but it is what
    // the arithmetic says, and it is better than overrunning a tiny pool.
    auto a = metered(1);
    CHECK(!a.has_room());
}

TEST(the_depth_is_per_link_and_the_budget_follows_it)
{
    // Two links, two sets of dynamics. The console returns credits in pairs
    // every 5 ms and the pad's every 20 ms, so one constant starved the pad
    // link's output stream of 3.9% of its packets. The default stays at the
    // console's number; the pad side asks for three.
    auto console = metered();
    CHECK_EQ(console.budget(), 2);

    auto pad = metered();
    pad.set_depth(3);
    CHECK_EQ(pad.budget(), 3);

    pad.on_sent(1);
    pad.on_sent(1);
    CHECK(pad.has_room());      // where a depth of two would already be full
    pad.on_sent(1);
    CHECK(!pad.has_room());

    // And a credit back opens exactly one slot, not the whole budget.
    pad.on_completed(1, 1);
    CHECK(pad.has_room());
    pad.on_sent(1);
    CHECK(!pad.has_room());
}

TEST(an_unanswered_pool_size_reports_the_depth_rather_than_minus_one)
{
    // max_packets_ - 1 on an unsigned zero promotes to -1, and the stats line
    // printed it: "acl 0/-1". Nothing is metered until the controller answers,
    // so the depth is the honest thing to show.
    AclFlowControl a;
    CHECK_EQ(int(a.max_packets()), 0);
    CHECK_EQ(a.budget(), 2);
    CHECK(a.has_room());
}

TEST(a_small_pool_still_caps_a_raised_depth)
{
    // budget = min(depth, max_packets - 1): asking for more than the controller
    // has must not overrun it, and one buffer stays reserved either way.
    auto a = metered(3);
    a.set_depth(6);
    CHECK_EQ(a.budget(), 2);
}

TEST(credits_are_written_off_when_the_controller_stops_returning_them)
{
    // The escape hatch for a controller that takes buffers and never gives them
    // back: without it the budget leaks a packet at a time until nothing can be
    // sent, on a link that looks perfectly healthy. Three seconds in the relay,
    // shortened here so the test does not take three seconds.
    auto a = metered();
    a.set_write_off_after(std::chrono::milliseconds(1));
    a.on_sent(1);
    a.on_sent(1);
    CHECK(!a.has_room());
    CHECK_EQ(a.in_flight_total(), 2);

    // Still inside the window: nothing is forgiven yet.
    CHECK(!a.has_room());

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    CHECK(a.has_room());                 // written off
    CHECK_EQ(a.in_flight_total(), 0);

    // And the clock restarts with it, so the next stall is measured afresh
    // rather than firing again immediately.
    a.on_sent(1);
    a.on_sent(1);
    CHECK(!a.has_room());
}

TEST(a_link_that_keeps_completing_is_never_written_off)
{
    // The failure this guards against is the opposite one: a 500 ms threshold
    // once wrote off credits on a link that was merely slow, sent into it
    // anyway, and killed it. A completion has to reset the clock.
    auto a = metered();
    a.set_write_off_after(std::chrono::milliseconds(20));
    a.on_sent(1);
    a.on_sent(1);
    for (int i = 0; i < 5; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        a.on_completed(1, 1);
        a.on_sent(1);
    }
    // 25 ms of elapsed time, no window without a completion longer than 5 ms.
    CHECK_EQ(a.in_flight_total(), 2);
}

}  // namespace
