#pragma once

// The pad's input, published for watching and logging from outside the relay -
// padlog. Read-only for everyone but the relay.
//
// Shared memory because it is written on the path that carries
// five hundred reports a second, and a copy into a mapped page costs nothing
// where a write() to a pipe or a socket would be the most expensive thing on it.
// The relay never waits on a reader and does not know whether there is one.
//
// Two parts:
//
//   the latest state  the newest state report's 63-byte body behind a sequence
//                     lock - the writer makes seq odd, copies, makes it even;
//                     a reader that sees it odd, or changed across its copy,
//                     tries again.
//   button events     a ring of every change in the buttons, stamped with the
//                     wall clock when the relay saw it. A viewer draws at 30 Hz
//                     and a tap can be shorter than a frame; the ring is what
//                     makes a log of presses complete.
//
// Only state reports are published. About one packet in ten on the interrupt
// channel is a frame of the pad's opaque stream under the same report id, and
// its "button" bytes are noise - fed through, they read as random presses.

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <new>
#include <optional>
#include <span>
#include <string>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace inputfeed {

inline constexpr uint32_t MAGIC   = 0x46495344;   // "DSIF"
inline constexpr uint32_t VERSION = 3;
inline constexpr const char *PATH = "/dev/shm/ds_input";
inline constexpr size_t BODY_LEN  = 63;
inline constexpr size_t EVENTS    = 256;

// ---------------------------------------------------------------------------
// The report body, as the pad lays it out (hid-playstation's
// dualsense_input_report; checked against the captured reports in
// tests/fixtures.h):
//
//   0-3   left X, left Y, right X, right Y   128 is centre
//   4-5   L2, R2 travel                      0 released, 255 fully pressed
//   7     d-pad in the low nibble            0 up, clockwise to 7, 8 released
//         square 0x10 cross 0x20 circle 0x40 triangle 0x80
//   8     L1 R1 L2 R2 Create Options L3 R3   bits 0-7
//   9     PS 0x01, touchpad click 0x02, mute 0x04
//   32-39 two touch points: [contact][x lo][x hi | y lo][y hi], contact bit 7
//         set when nothing is touching
//   52    battery: charging state high nibble, capacity low nibble
// ---------------------------------------------------------------------------

inline constexpr size_t DPAD_OFF    = 7;
inline constexpr size_t BUTTONS2_OFF = 8;
inline constexpr size_t BUTTONS3_OFF = 9;
inline constexpr size_t TOUCH_OFF   = 32;
inline constexpr size_t BATTERY_OFF = 52;

// Every button as one bit, the d-pad unfolded into four, so that any change in
// what is held is a change of this number.
enum Button : uint32_t {
    UP       = 1u << 0,
    RIGHT    = 1u << 1,
    DOWN     = 1u << 2,
    LEFT     = 1u << 3,
    SQUARE   = 1u << 4,
    CROSS    = 1u << 5,
    CIRCLE   = 1u << 6,
    TRIANGLE = 1u << 7,
    L1       = 1u << 8,
    R1       = 1u << 9,
    L2       = 1u << 10,
    R2       = 1u << 11,
    CREATE   = 1u << 12,
    OPTIONS  = 1u << 13,
    L3       = 1u << 14,
    R3       = 1u << 15,
    PS       = 1u << 16,
    TOUCHPAD = 1u << 17,
    MUTE     = 1u << 18,
};

inline constexpr size_t BUTTON_COUNT = 19;

constexpr uint32_t pack_buttons(std::span<const uint8_t, BODY_LEN> body)
{
    // Index is the hat value; 8 and anything above it is "released".
    static constexpr std::array<uint32_t, 8> hat = {
        UP, UP | RIGHT, RIGHT, DOWN | RIGHT, DOWN, DOWN | LEFT, LEFT, UP | LEFT,
    };
    const uint8_t dpad = body[DPAD_OFF] & 0x0f;
    return (dpad < hat.size() ? hat[dpad] : 0)
         | (body[DPAD_OFF] & 0xf0)
         | uint32_t(body[BUTTONS2_OFF]) << 8
         | uint32_t(body[BUTTONS3_OFF] & 0x07) << 16;
}

struct Touch {
    bool active = false;
    uint16_t x = 0;      // 0-1919
    uint16_t y = 0;      // 0-1079
};

struct PadState {
    uint32_t buttons = 0;
    uint8_t lx = 128, ly = 128, rx = 128, ry = 128;
    uint8_t l2 = 0, r2 = 0;
    std::array<Touch, 2> touch{};
    uint8_t battery = 0;
};

constexpr PadState decode(std::span<const uint8_t, BODY_LEN> body)
{
    PadState s;
    s.buttons = pack_buttons(body);
    s.lx = body[0];
    s.ly = body[1];
    s.rx = body[2];
    s.ry = body[3];
    s.l2 = body[4];
    s.r2 = body[5];
    for (size_t i = 0; i < s.touch.size(); i++) {
        const auto p = body.subspan(TOUCH_OFF + 4 * i, 4);
        s.touch[i].active = (p[0] & 0x80) == 0;
        s.touch[i].x = uint16_t(p[1] | (p[2] & 0x0f) << 8);
        s.touch[i].y = uint16_t((p[2] >> 4) | p[3] << 4);
    }
    s.battery = body[BATTERY_OFF];
    return s;
}

// ---------------------------------------------------------------------------
// The shared page
// ---------------------------------------------------------------------------

struct Event {
    uint64_t realtime_ns;    // CLOCK_REALTIME, for a log a person reads
    uint32_t buttons;        // everything held after the change
    uint32_t reserved;
};

// What the relay knows about the link that the pad's own bytes cannot say.
// Counters are cumulative and the reader turns them into rates; the gap fields
// are one completed one-second bucket, and `gap_window` says when a new one is
// ready. See SendGaps.h for why a hole needs its own measurement at all.
// What the relay is doing, in the relay's own words rather than guessed from
// counters, one step of the way from a pad in the hand to input on the console.
// The viewer turns each into a phrase and a hint, so a person watching knows
// whether to press something, plug something in, or wait.
enum Stage : uint8_t {
    STAGE_PAD_SEARCH = 0,       // paging the DualSense; it answers once PS is pressed
    STAGE_PAD_CONNECT = 1,      // it answered: authenticating, opening its channels
    STAGE_USB_WAIT = 2,         // the gadget is on the USB-C port, the PS5 has not configured it
    STAGE_USB_READ = 3,         // configured; the PS5 reads the controller, no input asked for yet
    STAGE_USB_PRESS = 4,        // the PS5 takes input over USB; pressing PS for it
    STAGE_CALLING = 5,          // handed over; calling the PS5 over Bluetooth
    STAGE_CONSOLE_CONNECT = 6,  // the PS5 answered; opening the controller's channels
    STAGE_RELAYING = 7,
};

struct Link {
    uint8_t stage = STAGE_PAD_SEARCH;
    uint8_t call_attempt = 0;       // in STAGE_CALLING: which call to the PS5 this is
    uint64_t console_sent = 0;      // input reports the console link accepted, ever
    uint64_t output_sent = 0;       // rumble and effects forwarded to the pad, ever
    uint64_t output_dropped = 0;    // ... dropped because the pad link was full
    uint32_t gap_window = 0;
    uint32_t gap_count = 0;         // gaps over 100 ms in that bucket
    uint32_t gap_max_us = 0;        // the worst one in it
    uint8_t console_live = 0;       // 1 while input goes to the console over BT
    // Straight from the controller. Quality is 0-255, higher better. RSSI on
    // BR/EDR is not dBm - it is the distance outside the golden receive power
    // range, so zero is the good answer.
    uint8_t link_quality = 0;
    int8_t link_rssi = 0;
    uint8_t afh_channels = 0;       // of 79 the link may hop on; fewer means coexistence
    uint8_t radio_read = 0;         // 1 once the controller has answered any of them
};

struct Shared {
    uint32_t magic;
    uint32_t version;
    std::atomic<uint64_t> seq;          // odd while the fields below are written
    uint64_t monotonic_ns;              // when the latest body arrived
    uint64_t reports;                   // state reports published, ever
    uint8_t body[BODY_LEN];
    Link link;
    std::atomic<uint64_t> event_head;   // events ever written; slot is head % EVENTS
    Event events[EVENTS];
};

inline uint64_t clock_ns(clockid_t clock)
{
    timespec ts{};
    clock_gettime(clock, &ts);
    return uint64_t(ts.tv_sec) * 1000000000ull + uint64_t(ts.tv_nsec);
}

// The relay's side. Not thread-safe and does not need to be: one handler, one
// io_context thread.
class Writer {
public:
    Writer() = default;
    Writer(const Writer &) = delete;
    Writer &operator=(const Writer &) = delete;
    ~Writer()
    {
        if (s_)
            munmap(s_, sizeof(Shared));
    }

    // Creates the file, or takes over the one a previous run left. False when
    // it cannot, and the relay goes on without a feed.
    bool open(const char *path = PATH)
    {
        const int fd = ::open(path, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
        if (fd < 0)
            return false;
        // fchmod as well: a run with a tighter umask must not leave a file the
        // viewer, which needs no privileges, cannot read.
        const bool sized = ftruncate(fd, sizeof(Shared)) == 0 && fchmod(fd, 0644) == 0;
        void *p = sized ? mmap(nullptr, sizeof(Shared), PROT_READ | PROT_WRITE,
                               MAP_SHARED, fd, 0)
                        : MAP_FAILED;
        ::close(fd);
        if (p == MAP_FAILED)
            return false;
        s_ = new (p) Shared{};
        s_->magic = MAGIC;
        s_->version = VERSION;
        return true;
    }

    bool is_open() const { return s_ != nullptr; }

    // One state report's body. On the report path: two clock reads, a copy and
    // a compare - no system call beyond the vDSO clock.
    // The link block on its own, without a report. The relay calls this on a
    // timer, because the steps before the pad connects - and the waits after
    // it - are exactly when there is no report to carry it, and exactly when
    // someone watching wants to know what is going on.
    void publish_link(const Link &link)
    {
        if (!s_)
            return;
        const uint64_t seq = s_->seq.load(std::memory_order_relaxed);
        s_->seq.store(seq + 1, std::memory_order_relaxed);
        s_->link = link;
        s_->seq.store(seq + 2, std::memory_order_relaxed);
    }

    void publish(std::span<const uint8_t, BODY_LEN> body, const Link &link)
    {
        if (!s_)
            return;
        const uint64_t seq = s_->seq.load(std::memory_order_relaxed);
        s_->seq.store(seq + 1, std::memory_order_relaxed);
        std::memcpy(s_->body, body.data(), BODY_LEN);
        s_->monotonic_ns = clock_ns(CLOCK_MONOTONIC);
        s_->reports++;
        s_->link = link;
        s_->seq.store(seq + 2, std::memory_order_relaxed);

        const uint32_t buttons = pack_buttons(body);
        if (buttons == last_buttons_)
            return;
        last_buttons_ = buttons;
        // The slot is filled before the head moves, so a reader never sees a
        // half-written event.
        const uint64_t head = s_->event_head.load(std::memory_order_relaxed);
        s_->events[head % EVENTS] = {clock_ns(CLOCK_REALTIME), buttons, 0};
        s_->event_head.store(head + 1, std::memory_order_relaxed);
    }

private:
    Shared *s_ = nullptr;
    uint32_t last_buttons_ = 0;
};

struct Snapshot {
    uint64_t monotonic_ns = 0;
    uint64_t reports = 0;
    Link link{};
    std::array<uint8_t, BODY_LEN> body{};

    bool console_live() const { return link.console_live != 0; }
};

// A viewer's side: read-only, and it never blocks the writer.
class Reader {
public:
    Reader() = default;
    Reader(const Reader &) = delete;
    Reader &operator=(const Reader &) = delete;
    ~Reader() { close(); }

    // False while the relay has not created the file yet.
    bool open(const char *path = PATH)
    {
        close();
        const int fd = ::open(path, O_RDONLY | O_CLOEXEC);
        if (fd < 0)
            return false;
        struct stat st{};
        const bool ok = fstat(fd, &st) == 0 && st.st_size >= off_t(sizeof(Shared));
        void *p = ok ? mmap(nullptr, sizeof(Shared), PROT_READ, MAP_SHARED, fd, 0)
                     : MAP_FAILED;
        ::close(fd);
        if (p == MAP_FAILED)
            return false;
        s_ = static_cast<const Shared *>(p);
        inode_ = st.st_ino;
        return true;
    }

    void close()
    {
        if (s_)
            munmap(const_cast<Shared *>(s_), sizeof(Shared));
        s_ = nullptr;
    }

    bool is_open() const { return s_ != nullptr; }
    bool valid() const { return s_ && s_->magic == MAGIC && s_->version == VERSION; }

    // Whether the file at path is still the one mapped - a relay that removed
    // and recreated it would otherwise be watched through a dead mapping.
    bool same_file(const char *path = PATH) const
    {
        struct stat st{};
        return s_ && stat(path, &st) == 0 && st.st_ino == inode_;
    }

    std::optional<Snapshot> latest() const
    {
        if (!valid())
            return std::nullopt;
        for (int attempt = 0; attempt < 16; attempt++) {
            const uint64_t before = s_->seq.load(std::memory_order_relaxed);
            if (before & 1)
                continue;
            Snapshot snap;
            std::memcpy(snap.body.data(), s_->body, BODY_LEN);
            snap.monotonic_ns = s_->monotonic_ns;
            snap.reports = s_->reports;
            snap.link = s_->link;
            if (s_->seq.load(std::memory_order_relaxed) == before)
                return snap;
        }
        return std::nullopt;
    }

    uint64_t event_head() const { return valid() ? s_->event_head.load() : 0; }

    // Events from `cursor` up to the head, oldest first. Returns how many were
    // lost because more than a ring's worth arrived since the last call.
    template <class F>
    uint64_t drain(uint64_t &cursor, F &&on_event) const
    {
        const uint64_t head = event_head();
        uint64_t lost = 0;
        if (head < cursor)                 // the relay restarted and reset the ring
            cursor = 0;
        if (head - cursor > EVENTS) {
            lost = head - cursor - EVENTS;
            cursor = head - EVENTS;
        }
        for (; cursor < head; cursor++)
            on_event(s_->events[cursor % EVENTS]);
        return lost;
    }

private:
    const Shared *s_ = nullptr;
    ino_t inode_ = 0;
};

}  // namespace inputfeed
