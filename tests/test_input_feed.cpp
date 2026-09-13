// The input feed ps5padlog-view reads.
//
// Two halves: decoding a report body into buttons, sticks and touch, checked
// against reports the pad actually sent; and the shared page itself - what the
// relay writes a viewer can read back, a change of buttons is one event and no
// change is none, and a ring that overflows says how much it lost.
#include "check.h"
#include "fixtures.h"

#include "input/InputFeed.h"
#include "dualsense/Transforms.h"

#include <algorithm>
#include <array>
#include <unistd.h>

using namespace inputfeed;

namespace {

// The link block the relay publishes alongside the body. These tests are about
// the body and the event ring, so only the flag the panel colours matters here.
inputfeed::Link live(bool on)
{
    inputfeed::Link l;
    l.console_live = on ? 1 : 0;
    return l;
}

std::array<uint8_t, BODY_LEN> body_of(const std::array<uint8_t, 79> &report)
{
    std::array<uint8_t, BODY_LEN> b{};
    std::copy_n(report.begin() + transforms::BT_BODY_OFF, BODY_LEN, b.begin());
    return b;
}

std::array<uint8_t, BODY_LEN> idle_body()
{
    return body_of(fixtures::input_report[0]);
}

TEST(a_resting_pad_holds_nothing)
{
    // The captured state reports: sticks near centre, d-pad 8, no touch.
    for (size_t i = 0; i < fixtures::INPUT_REPORTS; i++) {
        if (!fixtures::carries_state[i])
            continue;
        const PadState s = decode(body_of(fixtures::input_report[i]));
        CHECK_EQ(s.buttons, uint32_t(0));
        CHECK(s.lx > 118 && s.lx < 138);
        CHECK(s.ry > 118 && s.ry < 138);
        CHECK(!s.touch[0].active && !s.touch[1].active);
    }
}

TEST(the_opaque_frames_would_read_as_presses)
{
    // Why only state reports are published: every opaque frame in the fixtures
    // decodes to some button held.
    for (size_t i = 0; i < fixtures::INPUT_REPORTS; i++) {
        if (fixtures::carries_state[i])
            continue;
        CHECK(!transforms::carries_controller_state(fixtures::input_report[i]));
        CHECK(pack_buttons(body_of(fixtures::input_report[i])) != 0);
    }
}

TEST(the_dpad_unfolds_into_four_directions)
{
    auto b = idle_body();
    const auto with_hat = [&](uint8_t hat) {
        b[DPAD_OFF] = uint8_t((b[DPAD_OFF] & 0xf0) | hat);
        return pack_buttons(b);
    };
    CHECK_EQ(with_hat(0), uint32_t(UP));
    CHECK_EQ(with_hat(1), uint32_t(UP | RIGHT));
    CHECK_EQ(with_hat(4), uint32_t(DOWN));
    CHECK_EQ(with_hat(7), uint32_t(UP | LEFT));
    CHECK_EQ(with_hat(8), uint32_t(0));
    CHECK_EQ(with_hat(15), uint32_t(0));
}

TEST(each_button_byte_lands_on_its_bits)
{
    auto b = idle_body();
    b[DPAD_OFF] = 0x80 | 0x08;       // triangle, d-pad released
    CHECK_EQ(pack_buttons(b), uint32_t(TRIANGLE));
    b[DPAD_OFF] = 0x10 | 0x20 | 0x40 | 0x08;
    CHECK_EQ(pack_buttons(b), uint32_t(SQUARE | CROSS | CIRCLE));
    b[DPAD_OFF] = 0x08;
    b[BUTTONS2_OFF] = 0x01 | 0x80;   // L1, R3
    b[BUTTONS3_OFF] = 0x01 | 0x04 | 0x10;   // PS, mute, and a bit nobody defines
    CHECK_EQ(pack_buttons(b), uint32_t(L1 | R3 | PS | MUTE));
}

TEST(a_touch_point_decodes_to_twelve_bit_coordinates)
{
    auto b = idle_body();
    // x = 0x4b0 (1200), y = 0x21c (540), contact bit 7 clear.
    b[TOUCH_OFF] = 0x05;
    b[TOUCH_OFF + 1] = 0xb0;
    b[TOUCH_OFF + 2] = 0xc4;         // x hi nibble 4, y lo nibble c
    b[TOUCH_OFF + 3] = 0x21;
    const PadState s = decode(b);
    CHECK(s.touch[0].active);
    CHECK_EQ(int(s.touch[0].x), 1200);
    CHECK_EQ(int(s.touch[0].y), 540);
    CHECK(!s.touch[1].active);
}

// A feed file per test, removed afterwards.
struct TempFeed {
    std::string path = "/tmp/ds_input_test_" + std::to_string(getpid());
    ~TempFeed() { unlink(path.c_str()); }
};

TEST(what_the_relay_writes_a_viewer_reads)
{
    TempFeed tmp;
    Writer w;
    CHECK(w.open(tmp.path.c_str()));
    Reader r;
    CHECK(r.open(tmp.path.c_str()));
    CHECK(r.valid());
    CHECK(r.same_file(tmp.path.c_str()));

    auto b = idle_body();
    b[0] = 200;
    w.publish(b, live(true));
    const auto snap = r.latest();
    CHECK(snap.has_value());
    if (snap) {
        CHECK_EQ(int(snap->body[0]), 200);
        CHECK_EQ(snap->reports, uint64_t(1));
        CHECK(snap->console_live());
    }
}

TEST(only_a_change_of_buttons_is_an_event)
{
    TempFeed tmp;
    Writer w;
    CHECK(w.open(tmp.path.c_str()));
    Reader r;
    CHECK(r.open(tmp.path.c_str()));

    auto b = idle_body();
    w.publish(b, live(false));             // nothing held, as at start: no event
    b[0] = 10;                       // a stick moves: still no event
    w.publish(b, live(false));
    b[DPAD_OFF] = 0x80 | 0x08;       // triangle down
    w.publish(b, live(false));
    w.publish(b, live(false));             // still down
    b[DPAD_OFF] = 0x08;              // and up
    w.publish(b, live(false));

    uint64_t cursor = 0;
    std::vector<uint32_t> seen;
    const uint64_t lost = r.drain(cursor, [&](const Event &e) { seen.push_back(e.buttons); });
    CHECK_EQ(lost, uint64_t(0));
    CHECK_EQ(seen.size(), size_t(2));
    if (seen.size() == 2) {
        CHECK_EQ(seen[0], uint32_t(TRIANGLE));
        CHECK_EQ(seen[1], uint32_t(0));
    }
    CHECK_EQ(cursor, uint64_t(2));
}

TEST(a_viewer_that_falls_behind_is_told_how_much_it_lost)
{
    TempFeed tmp;
    Writer w;
    CHECK(w.open(tmp.path.c_str()));
    Reader r;
    CHECK(r.open(tmp.path.c_str()));

    auto b = idle_body();
    const uint64_t changes = EVENTS + 44;
    for (uint64_t i = 0; i < changes; i++) {
        b[DPAD_OFF] = uint8_t(i % 2 ? 0x08 : 0x28);   // cross down, up, down...
        w.publish(b, live(false));
    }
    uint64_t cursor = 0;
    size_t seen = 0;
    const uint64_t lost = r.drain(cursor, [&](const Event &) { seen++; });
    CHECK_EQ(lost, uint64_t(44));
    CHECK_EQ(seen, size_t(EVENTS));
    CHECK_EQ(cursor, changes);
}

TEST(a_relay_restart_starts_the_viewer_over)
{
    TempFeed tmp;
    Reader r;
    uint64_t cursor = 0;
    {
        Writer w;
        CHECK(w.open(tmp.path.c_str()));
        CHECK(r.open(tmp.path.c_str()));
        auto b = idle_body();
        for (int i = 0; i < 6; i++) {
            b[DPAD_OFF] = uint8_t(i % 2 ? 0x08 : 0x18);
            w.publish(b, live(false));
        }
        r.drain(cursor, [](const Event &) {});
        CHECK_EQ(cursor, uint64_t(6));
    }
    // A new run takes over the same file and zeroes it.
    Writer again;
    CHECK(again.open(tmp.path.c_str()));
    auto b = idle_body();
    b[DPAD_OFF] = 0x48;              // circle
    again.publish(b, live(false));
    size_t seen = 0;
    r.drain(cursor, [&](const Event &e) { seen++; CHECK_EQ(e.buttons, uint32_t(CIRCLE)); });
    CHECK_EQ(seen, size_t(1));
    CHECK_EQ(cursor, uint64_t(1));
}

}  // namespace
