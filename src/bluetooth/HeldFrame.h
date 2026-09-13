#pragma once

// The one input report held back when the console's credits run out, lifted
// out of BluetoothHandler so it can be tested.
//
// Exactly one slot, and that is the design rather than a limitation: a queue
// would fill with reports that are already stale by the time a buffer comes
// free, which is the lag this relay exists to avoid. So a newer report
// overwrites the held one, and what finally goes out is the freshest thing the
// pad has said - measured at a median 65 us old, against the 4 ms it would be
// if it had waited its turn behind others.
//
// Bytes and length travel as one value because they used to be parallel members
// and drifted apart.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "dualsense/DualSense.h"

struct HeldFrame {
    // A BT input report plus the HIDP header byte in front of it.
    std::array<uint8_t, 1 + dualsense::INPUT_BT_SIZE> buf{};
    size_t len = 0;

    // Refuses rather than truncates, and keeps the check here rather than at
    // the call site. A short write would put a malformed report on the wire,
    // which is worse than holding nothing; an unchecked memcpy would be worse
    // than both, and it was one caller's memory away.
    void store(std::span<const uint8_t> r)
    {
        if (r.size() > buf.size()) {
            len = 0;
            return;
        }
        std::memcpy(buf.data(), r.data(), r.size());
        len = r.size();
    }

    void clear() { len = 0; }
    bool valid() const { return len != 0; }
    std::span<uint8_t> view() { return {buf.data(), len}; }
    std::span<const uint8_t> view() const { return {buf.data(), len}; }
    size_t capacity() const { return buf.size(); }
};
