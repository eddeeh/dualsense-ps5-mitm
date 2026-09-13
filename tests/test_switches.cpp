// What is left of the DS_* switches, and bdaddr_t as a value.
//
// DS_FRESH_PAIRING is the only switch, and scripts/run.sh reads it. Every
// other DS_* name is one of the removed ones set out of habit, and the relay has
// to say it ignores it rather than look as if it had used it.
#include "check.h"

#include "common/BdAddr.h"
#include "common/Switches.h"

namespace {

std::vector<std::string> unknown(std::initializer_list<const char *> env)
{
    return unknown_switches({env.begin(), env.size()});
}

TEST(the_fresh_pairing_switch_is_known)
{
    CHECK(unknown({"DS_FRESH_PAIRING=1"}).empty());
}

TEST(a_removed_switch_is_reported_with_its_value)
{
    const auto u = unknown({"DS_AUTO_PS=0", "DS_FRESH_PAIRING=1", "DS_LINK_STATS=1"});
    CHECK_EQ(u.size(), size_t(2));
    if (u.size() == 2) {
        CHECK(u[0] == "DS_AUTO_PS=0");       // sorted
        CHECK(u[1] == "DS_LINK_STATS=1");
    }
}

TEST(everything_that_is_not_ds_is_ignored)
{
    CHECK(unknown({"PATH=/usr/bin", "SUDO_USER=ed", "XDS_THING=1"}).empty());
}

TEST(an_address_prints_the_way_ba2str_does)
{
    // Least significant byte first in memory, most significant first on screen.
    const bdaddr_t pad{{0x42, 0x79, 0xfe, 0x4c, 0x03, 0x88}};
    CHECK(std::format("{}", pad) == "88:03:4C:FE:79:42");
}

TEST(addresses_compare_as_values)
{
    const bdaddr_t a{{1, 2, 3, 4, 5, 6}};
    bdaddr_t b = a;
    CHECK(a == b);
    b.b[0] = 7;
    CHECK(a != b);
    CHECK(addr_is_set(a));
    CHECK(!addr_is_set(bdaddr_t{}));
}

}  // namespace
