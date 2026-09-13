//
// ps5padlog-view - watch and log the pad's input while the relay runs.
//
//   build/ps5padlog-view                         live panel, redrawn in place
//   build/ps5padlog-view --log inputs.log        ... and every press and release to a file
//   build/ps5padlog-view > inputs.log            not a terminal: presses and releases only
//   build/ps5padlog-view --diag                  add the link numbers and the radio
//   build/ps5padlog-view --relay-log logs/run-X.log
//                                            also show the relay's latest log line
//
// Reads /dev/shm/ds_input, which the relay publishes (InputFeed.h). Needs no
// privileges, and can be started, stopped and started again while the relay
// runs. scripts/run.sh runs it in place of the relay's scrolling log.
//
// The panel is a fixed block of lines. Each frame moves the cursor back to the
// top of the block and rewrites it, the way a progress bar does, so the terminal
// never scrolls however much the pad does. The log is the other half: one line
// per button that goes down or comes up, stamped by the relay when it saw the
// change, so a tap shorter than a frame is still in it.
//
// By default the panel answers one question - is this working. The status is
// the relay's own account of which step it is on, from looking for the pad to
// relaying, with how long it has been there and, where there is something to do
// or a wait has gone on too long, what to do about it.
// Beside it, unlabelled, is one block per second of the longest the console went
// without fresh input: flat grey is smooth, a coloured spike is a stutter the
// player felt. That is the whole default view, because someone watching it
// while playing has no use for a number they would have to interpret.
//
// --diag adds the rest: the two rates, the gap count and worst, what the effects
// stream costs, and what the controller says about the radio itself.

#include "input/InputFeed.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <format>
#include <fstream>
#include <optional>
#include <print>
#include <ranges>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <signal.h>
#include <sys/ioctl.h>
#include <unistd.h>

using namespace inputfeed;
using namespace std::chrono_literals;

namespace {

volatile std::sig_atomic_t g_stop = 0;

void on_signal(int) { g_stop = 1; }

// ---------------------------------------------------------------------------
// What each button looks like
// ---------------------------------------------------------------------------

// 256-colour indices, which every terminal in use here understands.
constexpr int GREEN = 46, RED = 196, PINK = 213, BLUE = 75, WHITE = 15, GREY = 240,
              YELLOW = 220, CYAN = 80;

struct ButtonLook {
    Button bit;
    std::string_view glyph;     // what the panel and the log show
    std::string_view name;      // what the log also says, for grep
    int colour;
};

// Log order: face buttons, d-pad, shoulders, the small ones. The panel draws
// the d-pad first; see render().
constexpr std::array<ButtonLook, BUTTON_COUNT> looks = {{
    {TRIANGLE, "△", "triangle", GREEN},
    {CIRCLE,   "◯", "circle",   RED},
    {CROSS,    "✕", "cross",    BLUE},
    {SQUARE,   "▢", "square",   PINK},
    {UP,       "↑", "up",       WHITE},
    {RIGHT,    "→", "right",    WHITE},
    {DOWN,     "↓", "down",     WHITE},
    {LEFT,     "←", "left",     WHITE},
    {L1,       "L1", "L1",      WHITE},
    {R1,       "R1", "R1",      WHITE},
    {L2,       "L2", "L2",      WHITE},
    {R2,       "R2", "R2",      WHITE},
    {L3,       "L3", "L3",      WHITE},
    {R3,       "R3", "R3",      WHITE},
    {CREATE,   "Create", "create", WHITE},
    {OPTIONS,  "Options", "options", WHITE},
    {PS,       "PS", "ps",      WHITE},
    {TOUCHPAD, "Touch", "touchpad", WHITE},
    {MUTE,     "Mute", "mute",  WHITE},
}};

bool g_colour = true;
// Off by default. The panel someone leaves open while playing should say
// whether it is working; the numbers behind this are for the times it is not.
bool g_diag = false;

std::string paint(std::string_view text, int colour, bool bold = false)
{
    if (!g_colour)
        return std::string(text);
    return std::format("\x1b[{}38;5;{}m{}\x1b[0m", bold ? "1;" : "", colour, text);
}

// Characters on screen, not bytes: every glyph here is one column per code point.
size_t columns(std::string_view s)
{
    return size_t(std::ranges::count_if(s, [](char c) { return (uint8_t(c) & 0xc0) != 0x80; }));
}

// A button as the panel shows it: its colour when held, grey when not. Without
// colour, a released button is dots of the same width, or nothing would tell
// the two apart.
std::string button(const ButtonLook &b, uint32_t held)
{
    const bool down = (held & b.bit) != 0;
    if (!g_colour)
        return down ? std::string(b.glyph) : std::string(columns(b.glyph), '.');
    return paint(b.glyph, down ? b.colour : GREY, down);
}

const ButtonLook &look(Button bit)
{
    return *std::ranges::find(looks, bit, &ButtonLook::bit);
}

// Trigger travel as a bar of eighths, so a light touch still shows.
std::string bar(uint8_t value, int width = 20)
{
    static constexpr std::array<std::string_view, 9> eighths = {
        " ", "▏", "▎", "▍", "▌", "▋", "▊", "▉", "█"};
    const int total = value * width * 8 / 255;
    std::string s;
    for (int i = 0; i < width; i++) {
        const int cell = std::clamp(total - i * 8, 0, 8);
        s += eighths[cell];
    }
    return g_colour ? paint(s, value ? WHITE : GREY) : "[" + s + "]";
}

std::string stick(uint8_t x, uint8_t y)
{
    return std::format("x {:+4} y {:+4}", int(x) - 128, int(y) - 128);
}

// Three blocks, one per third of the charge, and none below 10% - blinking,
// because that is the band where a pad starts dropping its link and looks like
// a console refusing to play.
std::string battery(uint8_t status, bool blink_on)
{
    const int state = status >> 4;
    const int capacity = status & 0x0f;
    switch (state) {
    case 0xa: return paint("battery voltage out of range", YELLOW);
    case 0xb: return paint("battery temperature out of range", YELLOW);
    case 0xf: return paint("battery charging error", RED);
    default: break;
    }
    if (capacity > 10)
        return paint("battery ?", GREY);

    const int blocks = capacity == 0 ? 0 : capacity <= 3 ? 1 : capacity <= 6 ? 2 : 3;
    std::string cells;
    for (int i = 0; i < 3; i++)
        cells += i < blocks ? "▮" : "▯";
    if (blocks == 0 && !blink_on)
        cells = "   ";                  // the off half of the blink, same width
    const std::string_view label = state == 0x2 ? " full" : state == 0x1 ? " charging" : "";
    if (!g_colour)
        return "[" + cells + "]" + std::string(label);
    const int colour = blocks == 3 ? GREEN : blocks == 2 ? YELLOW : RED;
    return paint(cells, colour, true) + paint(label, GREY);
}

// ---------------------------------------------------------------------------
// The log
// ---------------------------------------------------------------------------

std::string wall_time(uint64_t realtime_ns)
{
    const time_t secs = time_t(realtime_ns / 1000000000ull);
    tm local{};
    localtime_r(&secs, &local);
    char date[32];
    strftime(date, sizeof(date), "%Y-%m-%d %H:%M:%S", &local);
    return std::format("{}.{:03}", date, realtime_ns / 1000000ull % 1000);
}

struct Log {
    std::FILE *file = nullptr;      // --log
    bool to_stdout = false;         // stdout is not a terminal

    void line(const std::string &text) const
    {
        if (file) {
            std::println(file, "{}", text);
            std::fflush(file);
        }
        if (to_stdout) {
            std::println("{}", text);
            std::fflush(stdout);
        }
    }
    bool active() const { return file || to_stdout; }
};

// ---------------------------------------------------------------------------
// The relay's own log, one line of it
// ---------------------------------------------------------------------------

struct RelayTail {
    std::string path;
    std::string last_event;     // the newest line that is not a statistics line
    std::string last_stats;     // the newest "USB 5s:" line

    void refresh()
    {
        if (path.empty())
            return;
        std::ifstream f(path, std::ios::binary | std::ios::ate);
        if (!f)
            return;
        const std::streamoff size = f.tellg();
        const std::streamoff from = std::max<std::streamoff>(0, size - 16384);
        f.seekg(from);
        std::string chunk(size_t(size - from), '\0');
        f.read(chunk.data(), std::streamsize(chunk.size()));
        std::vector<std::string_view> lines;
        for (auto part : chunk | std::views::split('\n'))
            if (!part.empty())
                lines.emplace_back(part.begin(), part.end());
        if (from > 0 && !lines.empty())
            lines.erase(lines.begin());     // the first one is probably cut
        const auto is_stats = [](std::string_view l) { return l.starts_with("USB 5s: "); };
        const auto rev = lines | std::views::reverse;
        if (const auto it = std::ranges::find_if(rev, is_stats); it != rev.end())
            last_stats = std::string(it->substr(8));
        if (const auto it = std::ranges::find_if_not(rev, is_stats); it != rev.end())
            last_event = std::string(*it);
    }
};

// ---------------------------------------------------------------------------
// The panel
// ---------------------------------------------------------------------------

struct Panel {
    uint32_t held = 0;
    std::deque<Button> recent;              // newest last
    std::optional<Snapshot> snap;
    double rate = 0;                        // state reports a second, from the pad
    double console_rate = 0;                // ... and how many of them the console took
    double out_rate = 0;                    // effects and rumble going the other way
    double out_loss = 0;                    // share of those the pad link had no room for
    std::deque<uint32_t> gap_history;       // worst gap per second, newest last
    uint32_t gap_window = 0;                // the bucket the last sample came from
    std::deque<uint32_t> gap_counts;        // over 100 ms, per second, alongside the maxima

    // Both read over the whole drawn window rather than the last bucket. A
    // count of nought beside three tall blocks reads as a contradiction, and
    // "worst 4 ms" one second after a 141 ms hole is worse than useless.
    uint32_t gap_count() const
    {
        uint32_t n = 0;
        for (uint32_t c : gap_counts)
            n += c;
        return n;
    }
    uint32_t gap_worst_us() const
    {
        uint32_t m = 0;
        for (uint32_t v : gap_history)
            m = std::max(m, v);
        return m;
    }
    std::string status;
    int status_colour = GREY;
    uint8_t stage = 0xff;                   // the last stage seen, to time how long it has lasted
    std::chrono::steady_clock::time_point stage_since{};
};

// What a person watching needs from each step: what is happening, and - only
// where there is something to do or it has gone on too long - what to do about
// it. The seconds are there so a wait that is working can be told from one that
// is stuck without opening the log.
struct Status {
    std::string text;
    int colour;
};

Status describe(const Snapshot &snap, std::chrono::seconds in_stage, bool pad_silent)
{
    // Stage, then the clock, then the hint - in that order because the panel
    // turns line wrap off, and on a narrow terminal it is the end that goes. The
    // clock is what says the relay is still alive; the hint can be lost.
    const auto s = in_stage.count();
    const auto say = [&](std::string what, int colour, const char *hint = nullptr) {
        what = "\u25cf " + what + std::format("  {}:{:02}", s / 60, s % 60);
        if (hint)
            what += std::string(" - ") + hint;
        return Status{what, colour};
    };
    switch (snap.link.stage) {
    case STAGE_PAD_SEARCH:
        return say("looking for the DualSense", YELLOW, "press PS on it");
    case STAGE_PAD_CONNECT:
        return say("DualSense found, connecting", WHITE);
    case STAGE_USB_WAIT:
        return s < 10 ? say("waiting for the PS5 to see the USB cable", WHITE)
                      : say("waiting for the PS5 to see the USB cable", YELLOW, "is it in the PS5?");
    case STAGE_USB_READ:
        return s < 20 ? say("PS5 is reading the controller over USB", WHITE)
                      : say("PS5 is reading the controller over USB", YELLOW,
                            "stuck? power the PS5 off fully");
    case STAGE_USB_PRESS:
        return say("PS5 takes input over USB, pressing PS for it", WHITE);
    case STAGE_CALLING:
        return say(std::format("calling the PS5 over Bluetooth, attempt {}", snap.link.call_attempt), WHITE);
    case STAGE_CONSOLE_CONNECT:
        return say("PS5 answered, opening the controller channels", WHITE);
    case STAGE_RELAYING:
        if (pad_silent)
            return {"\u25cf DualSense asleep - press PS to wake it", YELLOW};
        return {"\u25cf relaying", GREEN};
    }
    return say("working", WHITE);
}

// One block per second, tallest for anything at or over the scale. Flat and low
// is a healthy link; a spike is a hole, and a run of them is the burst that
// PERFORMANCE.md describes - which is the shape a single number cannot show.
std::string sparkline(const std::deque<uint32_t> &us)
{
    static constexpr std::string_view BLOCKS[] = {"\u2581","\u2582","\u2583","\u2584",
                                                  "\u2585","\u2586","\u2587","\u2588"};
    constexpr uint32_t FULL = 200000;       // 200 ms reads as off the top
    std::string out;
    for (uint32_t v : us) {
        const size_t i = size_t(std::min<uint64_t>(v, FULL) * 7 / FULL);
        // Coloured per block rather than per row: a person should see where the
        // bad second was without reading a number next to it.
        const int c = v >= 100000 ? RED : v >= 40000 ? YELLOW : GREY;
        out += paint(std::string(BLOCKS[i]), c);
    }
    return out;
}

void note_event(Panel &p, const Log &log, const Event &e)
{
    const uint32_t changed = e.buttons ^ p.held;
    const uint32_t went_down = changed & e.buttons;
    const uint32_t came_up = changed & p.held;
    for (size_t i = 0; i < looks.size(); i++) {
        const auto &b = looks[i];
        if (went_down & b.bit) {
            p.recent.push_back(b.bit);
            if (p.recent.size() > 64)
                p.recent.pop_front();
            if (log.active())
                log.line(std::format("{}  down  {}  {}", wall_time(e.realtime_ns), b.glyph, b.name));
        }
        if ((came_up & b.bit) && log.active())
            log.line(std::format("{}  up    {}  {}", wall_time(e.realtime_ns), b.glyph, b.name));
    }
    p.held = e.buttons;
}

// The newest press at the right end and older ones running off to the left, as
// many as fit. Line wrap is off while the panel is up, so a line longer than the
// terminal is cut at the right - which is where the newest press would be.
std::string recent_line(const std::deque<Button> &recent, int cols)
{
    const std::string label = "  recent ";
    const int budget = std::clamp(cols - int(label.size()) - 1, 0, 64);
    std::vector<const ButtonLook *> shown;
    int width = 0;
    for (auto it = recent.rbegin(); it != recent.rend(); ++it) {
        const auto &b = look(*it);
        const int w = int(columns(b.glyph)) + 1;
        if (width + w > budget)
            break;
        width += w;
        shown.push_back(&b);
    }
    // Newest on the left, pushing the rest right and off the far edge, so the
    // row moves the way it reads. It used to be right-aligned with the newest
    // at the right, which at a full row made every press shove the others
    // leftwards - against the direction the eye takes it in.
    std::string line = label;
    for (const auto *b : shown)
        line += " " + paint(b->glyph, b->colour, true);
    return line;
}

std::vector<std::string> render(const Panel &p, const RelayTail &tail, const std::string &log_path,
                                int cols, bool blink_on)
{
    PadState s;
    if (p.snap)
        s = decode(p.snap->body);

    std::vector<std::string> out;
    // The battery right after the title, where a narrow terminal still shows it.
    // One line, in the order it matters: battery right after the title where a
    // narrow terminal still shows it, then what the relay is doing, then how
    // smooth it is. The panel turns line wrap off, so on a short terminal the
    // tail is what gets cut - which is why the numbers --diag adds come last.
    std::string title = paint("DualSense", WHITE, true) + " "
                        // No battery until the pad has actually sent something: the
                        // stage arrives first now, and an empty body would draw as a
                        // flat, blinking battery on a pad that is merely not here yet.
                        + (p.snap && p.snap->reports ? battery(s.battery, blink_on) + "   " : "")
                        + paint(p.status, p.status_colour);
    if (p.snap && p.snap->console_live()) {
        if (!p.gap_history.empty())
            title += "   " + sparkline(p.gap_history);
        if (g_diag)
            title += std::format("   pad {:.0f}/s \u2192 console {:.0f}/s", p.rate, p.console_rate);
    }
    out.push_back(title);
    const uint32_t held = p.snap ? s.buttons : 0;

    auto group = [&](std::initializer_list<Button> bits) {
        std::string g;
        for (Button bit : bits) {
            if (!g.empty())
                g += ' ';
            g += button(look(bit), held);
        }
        return g;
    };
    // D-pad first, face buttons after it: the order they sit in on the pad,
    // left hand then right.
    out.push_back("  " + group({UP, RIGHT, DOWN, LEFT}) + "    "
                  + group({TRIANGLE, CIRCLE, CROSS, SQUARE}) + "    "
                  + group({L1, R1, L3, R3}) + "    "
                  + group({CREATE, OPTIONS, PS, TOUCHPAD, MUTE}));
    out.push_back(std::format("  L2 {} {:3}   R2 {} {:3}", bar(s.l2), s.l2, bar(s.r2), s.r2));
    out.push_back(std::format("  left  {}   right {}", stick(s.lx, s.ly), stick(s.rx, s.ry)));

    std::string touch = "  touch ";
    for (size_t i = 0; i < s.touch.size(); i++) {
        touch += i ? "   " : "";
        touch += s.touch[i].active ? std::format("{}: {:4},{:4}", i + 1, s.touch[i].x, s.touch[i].y)
                                   : std::format("{}: {:>9}", i + 1, "-");
    }
    out.push_back(touch);
    out.push_back(recent_line(p.recent, cols));

    // Everything with a number on it lives behind --diag: a panel someone
    // watches while playing should answer "is it working" and nothing else,
    // and the bar beside the status already does that.

    if (g_diag && p.snap && p.snap->console_live()) {
        std::string link = std::format("  gaps    {}/{}s", p.gap_count(),
                                       p.gap_history.size());
        if (p.gap_worst_us())
            link += std::format("  worst {} ms", (p.gap_worst_us() + 500) / 1000);
        out.push_back(link);
        std::string o = std::format("  effects {:.0f}/s   dropped {:.1f}%",
                                    p.out_rate, 100 * p.out_loss);
        if (p.snap->link.radio_read)
            o += std::format("   radio q{} rssi {} afh {}/79", p.snap->link.link_quality,
                             int(p.snap->link.link_rssi), p.snap->link.afh_channels);
        out.push_back(o);
    }

    if (g_diag && !tail.path.empty())
        out.push_back("  relay   " + paint(tail.last_event, GREY));
    if (g_diag && !log_path.empty())
        out.push_back("  log     " + paint(log_path, GREY));
    return out;
}

void usage()
{
    std::println("usage: ps5padlog-view [--diag] [--log FILE] [--relay-log FILE] [--relay-pid PID] [--fps N] [--no-colour] [--feed PATH]");
    std::println("  Shows the pad's input as the relay sees it, from {}.", PATH);
    std::println("  --log FILE        append every press and release to FILE");
    std::println("  --relay-log FILE  show the latest line of the relay's own log");
    std::println("  --relay-pid PID   stop when the relay exits, and show how its log ended");
    std::println("  --diag            add the link numbers, the radio and the relay's log line");
    std::println("  --fps N           redraws a second, default 30");
    std::println("  --feed PATH       read another feed file than {}", PATH);
    std::println("  Not on a terminal, it prints presses and releases instead of the panel.");
}

}  // namespace

int main(int argc, char **argv)
{
    std::string log_path;
    std::string feed_path = PATH;
    RelayTail tail;
    pid_t relay_pid = 0;
    int fps = 30;
    for (int i = 1; i < argc; i++) {
        const std::string_view arg = argv[i];
        const bool has_value = i + 1 < argc;
        if (arg == "--log" && has_value)
            log_path = argv[++i];
        else if (arg == "--feed" && has_value)
            feed_path = argv[++i];
        else if (arg == "--relay-log" && has_value)
            tail.path = argv[++i];
        else if (arg == "--relay-pid" && has_value)
            relay_pid = pid_t(std::atoi(argv[++i]));
        else if (arg == "--fps" && has_value)
            fps = std::clamp(std::atoi(argv[++i]), 1, 240);
        else if (arg == "--diag")
            g_diag = true;
        else if (arg == "--no-colour" || arg == "--no-color")
            g_colour = false;
        else {
            usage();
            return arg == "--help" || arg == "-h" ? 0 : 2;
        }
    }
    if (std::getenv("NO_COLOR"))
        g_colour = false;

    const bool tty = isatty(STDOUT_FILENO);
    if (!tty)
        g_colour = false;

    Log log;
    log.to_stdout = !tty;
    if (!log_path.empty()) {
        log.file = std::fopen(log_path.c_str(), "a");
        if (!log.file) {
            std::println(stderr, "cannot open {}: {}", log_path, std::strerror(errno));
            return 1;
        }
        log.line(std::format("# ps5padlog-view started {}", wall_time(clock_ns(CLOCK_REALTIME))));
    }

    struct sigaction sa{};
    sa.sa_handler = on_signal;
    sigemptyset(&sa.sa_mask);
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    sigaction(SIGHUP, &sa, nullptr);

    // Cursor hidden while the panel is up, and line wrap off: a line longer
    // than the terminal would otherwise wrap, and the block would no longer be
    // the number of lines the cursor moves back over.
    if (tty)
        std::print("\x1b[?25l\x1b[?7l");

    Reader reader;
    Panel panel;
    uint64_t cursor = 0;
    bool cursor_set = false;
    size_t drawn = 0;

    const auto frame = std::chrono::microseconds(1000000 / (tty ? fps : 100));
    auto next_open = std::chrono::steady_clock::now();
    auto next_tail = next_open;
    auto rate_since = next_open;
    auto next_relay_check = next_open;
    uint64_t rate_reports = 0;
    uint64_t rate_console = 0, rate_out = 0, rate_drop = 0;
    bool relay_gone = false;

    while (!g_stop) {
        const auto now = std::chrono::steady_clock::now();

        // A relay that could not start - an adapter it could not claim, say -
        // would otherwise leave the panel waiting for a feed that never comes.
        if (relay_pid > 0 && now >= next_relay_check) {
            next_relay_check = now + 500ms;
            if (kill(relay_pid, 0) != 0 && errno == ESRCH) {
                relay_gone = true;
                break;
            }
        }

        if ((!reader.valid() || !reader.same_file(feed_path.c_str())) && now >= next_open) {
            next_open = now + 500ms;
            if (reader.open(feed_path.c_str()) && reader.valid()) {
                // Only what happens from now on: the ring may hold a previous
                // session's presses, which belong to nobody watching.
                cursor = reader.event_head();
                cursor_set = true;
                if (const auto snap = reader.latest()) {
                    panel.held = pack_buttons(snap->body);
                    rate_reports = snap->reports;
                }
                rate_since = now;
            }
        }

        if (reader.valid() && cursor_set) {
            const uint64_t lost = reader.drain(cursor, [&](const Event &e) { note_event(panel, log, e); });
            if (lost && log.active())
                log.line(std::format("# {} button changes lost - the viewer fell behind", lost));

            panel.snap = reader.latest();
            if (panel.snap) {
                const auto age = std::chrono::nanoseconds(clock_ns(CLOCK_MONOTONIC) - panel.snap->monotonic_ns);
                const double since = std::chrono::duration<double>(now - rate_since).count();
                if (since >= 1.0) {
                    const auto &l = panel.snap->link;
                    panel.rate = double(panel.snap->reports - std::min(rate_reports, panel.snap->reports)) / since;
                    panel.console_rate = double(l.console_sent - std::min(rate_console, l.console_sent)) / since;
                    const uint64_t d_out = l.output_sent - std::min(rate_out, l.output_sent);
                    const uint64_t d_drop = l.output_dropped - std::min(rate_drop, l.output_dropped);
                    panel.out_rate = double(d_out) / since;
                    panel.out_loss = d_out + d_drop ? double(d_drop) / double(d_out + d_drop) : 0.0;
                    rate_reports = panel.snap->reports;
                    rate_console = l.console_sent;
                    rate_out = l.output_sent;
                    rate_drop = l.output_dropped;
                    rate_since = now;
                }
                // The relay publishes one completed bucket at a time and counts
                // them, so a new sample is exactly a change of that number.
                if (panel.snap->link.gap_window != panel.gap_window) {
                    panel.gap_window = panel.snap->link.gap_window;
                    panel.gap_history.push_back(panel.snap->link.gap_max_us);
                    panel.gap_counts.push_back(panel.snap->link.gap_count);
                    while (panel.gap_history.size() > 20) {
                        panel.gap_history.pop_front();
                        panel.gap_counts.pop_front();
                    }
                }
                if (panel.snap->link.stage != panel.stage) {
                    panel.stage = panel.snap->link.stage;
                    panel.stage_since = now;
                }
                const auto in_stage = std::chrono::duration_cast<std::chrono::seconds>(now - panel.stage_since);
                const Status st = describe(*panel.snap, in_stage, panel.snap->reports == 0 || age > 1s);
                panel.status = st.text;
                panel.status_colour = st.colour;
            }
        } else {
            panel.snap.reset();
            panel.status = "\u25cf starting the relay";
            panel.status_colour = YELLOW;
        }

        if (tty) {
            if (now >= next_tail) {
                tail.refresh();
                next_tail = now + 250ms;
            }
            winsize ws{};
            const int cols = ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col ? ws.ws_col : 80;
            const bool blink_on =
                std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()
                    / 500 % 2 == 0;
            const auto lines = render(panel, tail, log_path, cols, blink_on);
            std::string out;
            if (drawn)
                out += std::format("\x1b[{}A", drawn);
            for (const auto &line : lines)
                out += "\r\x1b[2K" + line + "\n";
            // The block can shrink when the relay goes away; clear what is left.
            for (size_t i = lines.size(); i < drawn; i++)
                out += "\r\x1b[2K\n";
            if (drawn > lines.size())
                out += std::format("\x1b[{}A", drawn - lines.size());
            std::fwrite(out.data(), 1, out.size(), stdout);
            std::fflush(stdout);
            drawn = lines.size();
        }

        std::this_thread::sleep_for(frame);
    }

    if (tty)
        std::print("\x1b[?7h\x1b[?25h");
    if (relay_gone) {
        std::println("\nThe relay has exited.");
        if (!tail.path.empty()) {
            std::ifstream f(tail.path);
            std::deque<std::string> last;
            for (std::string line; std::getline(f, line);) {
                last.push_back(std::move(line));
                if (last.size() > 15)
                    last.pop_front();
            }
            std::println("Last lines of {}:", tail.path);
            for (const auto &line : last)
                std::println("  {}", line);
        }
    }
    if (log.file) {
        log.line(std::format("# ps5padlog-view stopped {}", wall_time(clock_ns(CLOCK_REALTIME))));
        std::fclose(log.file);
    }
    return 0;
}
