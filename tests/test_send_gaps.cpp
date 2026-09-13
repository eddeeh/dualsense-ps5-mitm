// The per-second view of how long the console waited between input reports.
#include "check.h"

#include "bluetooth/SendGaps.h"

namespace {

using Clock = SendGaps::Clock;
constexpr Clock::time_point T0{};

Clock::time_point at(int ms)
{
    return T0 + std::chrono::milliseconds(ms) + std::chrono::seconds(1);
}

SendGaps tracker()
{
    SendGaps g;
    g.set_bucket(std::chrono::milliseconds(100));
    g.set_threshold(std::chrono::milliseconds(20));
    return g;
}

TEST(nothing_is_reported_before_a_bucket_closes)
{
    auto g = tracker();
    g.on_sent(at(0));
    g.on_sent(at(5));
    CHECK_EQ(int(g.window()), 0);
    CHECK_EQ(int(g.count()), 0);
    CHECK_EQ(int(g.max_us()), 0);
}

TEST(the_first_send_is_not_a_gap)
{
    // There is nothing before it to measure from, and counting the wait since
    // the process started would report a huge gap on every fresh link.
    auto g = tracker();
    g.on_sent(at(0));
    g.on_sent(at(150));      // closes the bucket; only the 150 ms gap is in it
    CHECK_EQ(int(g.window()), 1);
    CHECK_EQ(int(g.count()), 1);
    CHECK_EQ(int(g.max_us()), 150000);
}

TEST(a_closed_bucket_reports_its_worst_gap_and_how_many_crossed)
{
    auto g = tracker();
    g.on_sent(at(0));
    g.on_sent(at(5));        // 5 ms   - under the threshold
    g.on_sent(at(35));       // 30 ms  - over
    g.on_sent(at(80));       // 45 ms  - over, and the worst
    g.on_sent(at(85));       // 5 ms
    g.on_sent(at(200));      // closes it; this 115 ms gap belongs to the next
    CHECK_EQ(int(g.window()), 1);
    CHECK_EQ(int(g.count()), 3);      // 30, 45 and the 115 that closed it
    CHECK_EQ(int(g.max_us()), 115000);
}

TEST(each_bucket_starts_clean)
{
    // A quiet second after a bad one must read as quiet, or the panel would
    // show a hole that has already passed.
    auto g = tracker();
    g.on_sent(at(0));
    g.on_sent(at(90));       // 90 ms gap
    g.on_sent(at(150));      // closes bucket 1
    CHECK_EQ(int(g.count()), 2);
    for (int t = 160; t <= 260; t += 10)
        g.on_sent(at(t));    // ten quiet milliseconds apart
    CHECK_EQ(int(g.window()), 2);
    CHECK_EQ(int(g.count()), 0);
    CHECK_EQ(int(g.max_us()), 10000);
}

TEST(the_window_number_is_what_says_a_bucket_is_new)
{
    auto g = tracker();
    for (int t = 0; t <= 500; t += 10)
        g.on_sent(at(t));
    CHECK_EQ(int(g.window()), 5);
}

TEST(a_silent_link_does_not_close_buckets_on_its_own)
{
    // Buckets roll on sends, so a link that stops entirely freezes its last
    // reading rather than reporting zeros. The panel says "pad silent" from the
    // feed's own timestamp for that case; this must not quietly contradict it.
    auto g = tracker();
    g.on_sent(at(0));
    g.on_sent(at(150));
    const uint32_t w = g.window();
    CHECK_EQ(int(w), 1);
    CHECK_EQ(int(g.max_us()), 150000);
}

}  // namespace
