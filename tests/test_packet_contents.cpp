// Bounds behaviour of the packet cursor.
//
// This is the class the whole parse path stands on: every HCI event and every
// ACL packet from the controller is read through it, and the lengths inside
// those bytes are the peer's claims rather than ours. Before the bounds checks
// went in, a wire-declared L2CAP length longer than the packet produced a span
// running off the end of the receive buffer, which was then relayed onward.
// Confirmed at the time with ASan against the old header.
#include "check.h"

#include "bluetooth/PacketContents.h"

#include <cstring>

namespace {

struct Big {
    uint8_t a[16];
};

TEST(packet_cursor_walks_to_the_end_and_stops)
{
    uint8_t buf[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    PacketContents p(buf, sizeof(buf));

    CHECK_EQ(p.remaining(), size_t{8});
    for (int i = 0; i < 8; i++) {
        const auto *b = p.get<uint8_t>();
        CHECK(b != nullptr);
        if (b)
            CHECK_EQ(int(*b), i + 1);
    }
    CHECK_EQ(p.remaining(), size_t{0});

    // Past the end every accessor refuses rather than running on.
    CHECK(p.get<uint8_t>() == nullptr);
    CHECK(p.bytes(1) == nullptr);
    CHECK(p.get(1) == nullptr);
}

TEST(structure_larger_than_the_packet_is_refused)
{
    uint8_t buf[8] = {};
    PacketContents p(buf, sizeof(buf));

    CHECK(p.get<Big>() == nullptr);
    // A refusal must not move the cursor, or the next read would be misaligned.
    CHECK_EQ(p.remaining(), size_t{8});
}

TEST(wire_declared_length_past_the_end_is_refused)
{
    // The handle_acl() case: two headers consumed, then a length the peer
    // declared. 0xffff is what a fragmented L2CAP payload can look like from
    // here, and fragments are not reassembled.
    uint8_t buf[8] = {};
    PacketContents p(buf, sizeof(buf));
    CHECK(p.get<uint32_t>() != nullptr);

    CHECK(p.get(0xffff) == nullptr);
    CHECK_EQ(p.remaining(), size_t{4});
}

TEST(partial_read_straddling_the_end_is_refused)
{
    uint8_t buf[8] = {};
    PacketContents p(buf, sizeof(buf));
    CHECK(p.get<uint8_t>() != nullptr);

    CHECK(p.bytes(7) != nullptr);          // exactly the rest
    CHECK_EQ(p.remaining(), size_t{0});
    CHECK(p.bytes(1) == nullptr);          // one past
}

TEST(empty_packet_yields_nothing)
{
    uint8_t buf[1] = {};
    PacketContents p(buf, 0);

    CHECK_EQ(p.remaining(), size_t{0});
    CHECK(p.get<uint8_t>() == nullptr);
    // Asking for nothing is not an error, and must not be reported as one.
    CHECK(p.bytes(0) != nullptr);
}

TEST(reads_return_the_bytes_that_are_there)
{
    uint8_t buf[6] = {0xa1, 0x31, 0xde, 0xad, 0xbe, 0xef};
    PacketContents p(buf, sizeof(buf));

    const auto *hdr = p.bytes(2);
    CHECK(hdr != nullptr);
    if (hdr) {
        CHECK_EQ(int(hdr[0]), 0xa1);
        CHECK_EQ(int(hdr[1]), 0x31);
    }
    const auto *rest = p.bytes(4);
    CHECK(rest != nullptr);
    if (rest)
        CHECK(std::memcmp(rest, "\xde\xad\xbe\xef", 4) == 0);
}

}  // namespace
