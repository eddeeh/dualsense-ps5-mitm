#pragma once

// bdaddr_t as a value: compared with ==, printed with {}.
//
// BlueZ hands the address over as a bare six-byte struct, so every comparison
// was a memcmp of .b with a 6 next to it, and every log line with an address
// went through a char[18] and ba2str, or a to_addr_str() temporary. BlueZ's own
// BDADDR_ANY is an rvalue macro that bacmp() cannot take the address of in C++.

#include <bluetooth/bluetooth.h>

#include <cstring>
#include <format>

inline bool operator==(const bdaddr_t &a, const bdaddr_t &b)
{
    return std::memcmp(a.b, b.b, sizeof a.b) == 0;
}

// All zeros is how "not known yet" is written throughout.
inline bool addr_is_set(const bdaddr_t &a)
{
    return a != bdaddr_t{};
}

// Most significant byte first, upper case: what ba2str prints, and what every
// log and capture in this project shows.
template <>
struct std::formatter<bdaddr_t> {
    constexpr auto parse(std::format_parse_context &ctx) { return ctx.begin(); }

    auto format(const bdaddr_t &a, std::format_context &ctx) const
    {
        return std::format_to(ctx.out(), "{:02X}:{:02X}:{:02X}:{:02X}:{:02X}:{:02X}",
                              a.b[5], a.b[4], a.b[3], a.b[2], a.b[1], a.b[0]);
    }
};
