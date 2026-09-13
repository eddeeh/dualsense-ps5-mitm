// The single-slot hold for an input report the console had no credit for.
//
// Small enough to read in one go, and worth testing anyway: it is the piece
// that decides what the console eventually sees when the link is busy, and its
// store() is a memcpy into a fixed buffer.
#include "check.h"

#include "bluetooth/HeldFrame.h"

#include <vector>

namespace {

std::vector<uint8_t> report(size_t n, uint8_t fill)
{
    return std::vector<uint8_t>(n, fill);
}

TEST(a_fresh_slot_holds_nothing)
{
    HeldFrame h;
    CHECK(!h.valid());
    CHECK_EQ(h.view().size(), size_t{0});
}

TEST(what_goes_in_comes_back_byte_for_byte)
{
    HeldFrame h;
    const auto r = report(1 + dualsense::INPUT_BT_SIZE, 0xa5);
    h.store(r);
    CHECK(h.valid());
    CHECK_EQ(h.view().size(), r.size());
    CHECK(std::equal(r.begin(), r.end(), h.view().begin()));
}

TEST(the_newest_report_replaces_the_held_one)
{
    // The whole point of one slot. If this ever queued instead, the console
    // would be handed stick positions in the order they went stale.
    HeldFrame h;
    h.store(report(10, 0x11));
    h.store(report(10, 0x22));
    CHECK_EQ(h.view().size(), size_t{10});
    for (uint8_t b : h.view())
        CHECK_EQ(int(b), 0x22);
}

TEST(a_shorter_report_does_not_leave_the_old_tail_behind)
{
    // len is what view() trusts, so a short store must not expose the bytes of
    // the longer one underneath it.
    HeldFrame h;
    h.store(report(20, 0xff));
    h.store(report(5, 0x01));
    CHECK_EQ(h.view().size(), size_t{5});
    for (uint8_t b : h.view())
        CHECK_EQ(int(b), 0x01);
}

TEST(an_oversized_report_is_refused_rather_than_truncated)
{
    // Truncating would put a malformed report on the wire, which the console
    // reads as a controller misbehaving. Holding nothing is the safe answer,
    // and the bounds check lives here rather than at the call site.
    HeldFrame h;
    h.store(report(8, 0x77));
    h.store(report(h.capacity() + 1, 0xee));
    CHECK(!h.valid());
    CHECK_EQ(h.view().size(), size_t{0});
}

TEST(the_slot_fits_exactly_one_framed_bt_input_report)
{
    // HIDP header plus the report the pad sends. One byte less and every
    // report would be refused above; the relay would go silent and look like a
    // dead link.
    HeldFrame h;
    CHECK_EQ(h.capacity(), size_t{1} + dualsense::INPUT_BT_SIZE);
    h.store(report(h.capacity(), 0x5a));
    CHECK(h.valid());
}

TEST(clearing_gives_the_slot_back)
{
    HeldFrame h;
    h.store(report(12, 0x33));
    CHECK(h.valid());
    h.clear();
    CHECK(!h.valid());
    CHECK_EQ(h.view().size(), size_t{0});
}

}  // namespace
