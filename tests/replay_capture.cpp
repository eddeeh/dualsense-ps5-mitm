// Walks a whole btsnoop capture through the real parsing code.
//
//     cmake --build build --target replay_capture
//     ./build/replay_capture logs/capture-YYYYMMDD-HHMMSS.dump
//
// This is not a unit test and is not part of `tests`: captures are hundreds of
// megabytes and are not in the repository. It answers the questions a handful
// of embedded fixtures cannot.
//
//   - Do the bounds checks in PacketContents reject anything a real controller
//     sent? That is the risk the checks introduced, and eight fixtures cannot
//     retire it. Millions of packets can.
//   - Does the HasHID flag agree with the 0xd4 marker at the rate measured
//     before, on this capture?
//   - Does every checksum verify, on every report rather than on eight?
//   - How often does an L2CAP payload actually declare more bytes than arrived?
//     That is the fragmentation case the parser used to read past the end of,
//     so its real frequency is worth knowing rather than assuming.
//
// Exit status is non-zero if anything that should always hold did not.
#include "bluetooth/AclFlowControl.h"
#include "dualsense/DualSense.h"
#include "bluetooth/PacketContents.h"
#include "dualsense/Transforms.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/l2cap.h>

#include <cstdio>
#include <cstring>
#include <cstdint>
#include <algorithm>
#include <map>
#include <print>
#include <string>
#include <vector>

namespace {

struct Counts {
    uint64_t records = 0;
    uint64_t events = 0;
    uint64_t acl = 0;

    uint64_t input_reports = 0;
    uint64_t input_crc_ok = 0;
    uint64_t state_frames = 0;
    uint64_t opaque_frames = 0;
    uint64_t flag_marker_agree = 0;
    uint64_t flag_says_state_marker_says_opaque = 0;
    uint64_t flag_says_opaque_marker_says_state = 0;

    uint64_t feature_answers = 0;
    uint64_t feature_crc_ok = 0;
    uint64_t feature_size_known = 0;
    uint64_t feature_size_agrees = 0;

    uint64_t output_reports = 0;
    uint64_t output_crc_ok = 0;

    uint64_t l2cap_truncated = 0;      // declared more than arrived
    uint64_t event_short = 0;          // event smaller than its structure

    // ACL flow control, replayed through the real accounting.
    uint64_t acl_sent = 0;
    uint64_t acl_completed = 0;
    uint64_t completion_events = 0;
    uint64_t over_completions = 0;     // freed more than we had outstanding
    uint64_t negative_total = 0;       // the running total went below zero
    int      peak_in_flight = 0;
    uint16_t pool = 0;                 // from Read Buffer Size, if it is here
    std::vector<uint32_t> completion_gaps_us;   // for the median, not the mean
};

bool read_exact(std::FILE *f, void *p, size_t n)
{
    return std::fread(p, 1, n, f) == n;
}

uint32_t be32(const uint8_t *p)
{
    return uint32_t(p[0]) << 24 | uint32_t(p[1]) << 16 | uint32_t(p[2]) << 8 | p[3];
}

// Sizes the dispatch in handle_event() would consume for each event it handles.
// If a real capture ever carries one of these shorter than this, the bounds
// checks would drop it - which is exactly what this is here to find out.
size_t event_struct_size(uint8_t code)
{
    switch (code) {
    case EVT_CONN_COMPLETE:           return sizeof(evt_conn_complete);
    case EVT_LINK_KEY_REQ:            return sizeof(evt_link_key_req);
    case EVT_IO_CAPABILITY_REQUEST:   return sizeof(evt_io_capability_request);
    case EVT_USER_CONFIRM_REQUEST:    return sizeof(evt_user_confirm_request);
    case EVT_SIMPLE_PAIRING_COMPLETE: return sizeof(evt_simple_pairing_complete);
    case EVT_AUTH_COMPLETE:           return sizeof(evt_auth_complete);
    case EVT_ENCRYPT_CHANGE:          return sizeof(evt_encrypt_change);
    case EVT_ROLE_CHANGE:             return sizeof(evt_role_change);
    case EVT_QOS_SETUP_COMPLETE:      return sizeof(evt_qos_setup_complete);
    case EVT_MODE_CHANGE:             return sizeof(evt_mode_change);
    case EVT_CMD_COMPLETE:            return sizeof(evt_cmd_complete);
    case EVT_CMD_STATUS:              return sizeof(evt_cmd_status);
    case EVT_LINK_KEY_NOTIFY:         return sizeof(evt_link_key_notify);
    case EVT_CONN_REQUEST:            return sizeof(evt_conn_request);
    case EVT_DISCONN_COMPLETE:        return sizeof(evt_disconn_complete);
    default:                          return 0;   // not one this relay reads
    }
}

// Read Buffer Size tells the relay how many ACL packets the controller holds,
// and everything metered is metered against it. Worth taking from the capture
// rather than guessing, since it is right there in the command completion.
void note_buffer_size(PacketContents &pc, Counts &c)
{
    const auto *r = pc.bytes(7);
    if (r && r[0] == 0x00)
        c.pool = uint16_t(r[4] | (r[5] << 8));
}

uint32_t trailing_crc(const uint8_t *p, size_t n)
{
    return uint32_t(p[n - 4]) | uint32_t(p[n - 3]) << 8
         | uint32_t(p[n - 2]) << 16 | uint32_t(p[n - 1]) << 24;
}

void inspect_acl(const uint8_t *rec, size_t len, Counts &c,
                 std::map<uint8_t, uint64_t> &unknown_feature,
                 bool outbound, AclFlowControl &acl,
                 std::map<uint16_t, int> &outstanding)
{
    c.acl++;
    // The record is one H4 frame: [0x02][acl hdr][l2cap hdr][payload].
    PacketContents pc(const_cast<uint8_t *>(rec), len);
    if (!pc.get<uint8_t>())
        return;
    const auto *acl_hdr = pc.get<hci_acl_hdr>();
    const auto *l2 = pc.get<l2cap_hdr>();
    if (!acl_hdr || !l2)
        return;

    // Only what we put on the air is metered; the peer's packets cost us no
    // controller buffer. btsnoop records the direction, so this is exact.
    if (outbound) {
        const uint16_t h = uint16_t(btohs(acl_hdr->handle) & 0x0fff);
        acl.on_sent(h);
        outstanding[h]++;
        c.acl_sent++;
        c.peak_in_flight = std::max(c.peak_in_flight, acl.in_flight_total());
    }

    const uint16_t claimed = l2->len;
    const auto *payload = static_cast<const uint8_t *>(pc.get(claimed));
    if (!payload) {
        // Exactly the case handle_acl() now drops and used to read past.
        c.l2cap_truncated++;
        return;
    }
    if (claimed == 0)
        return;

    const std::span<const uint8_t> p{payload, claimed};

    if (p.size() == 1 + dualsense::INPUT_BT_SIZE
        && p[0] == dualsense::HIDP_DATA_INPUT && p[1] == dualsense::REPORT_BT) {
        c.input_reports++;
        if (transforms::input_crc_is_valid(p))
            c.input_crc_ok++;

        const bool by_flag = transforms::carries_controller_state(p);
        const bool by_marker = p[transforms::BT_BODY_OFF + 1] != 0xd4;
        by_flag ? c.state_frames++ : c.opaque_frames++;
        if (by_flag == by_marker)
            c.flag_marker_agree++;
        else if (by_flag)
            c.flag_says_state_marker_says_opaque++;
        else
            c.flag_says_opaque_marker_says_state++;
        return;
    }

    if (p.size() >= 6 && p[0] == dualsense::HIDP_DATA_FEATURE) {
        c.feature_answers++;
        if (dualsense::crc32_seeded(dualsense::CRC_SEED_FEATURE, &p[1],
                                    p.size() - 5)
            == trailing_crc(p.data(), p.size()))
            c.feature_crc_ok++;
        const size_t declared = dualsense::feature_report_size(p[1]);
        if (declared) {
            c.feature_size_known++;
            if (declared == p.size() - 1)
                c.feature_size_agrees++;
        } else {
            unknown_feature[p[1]]++;
        }
        return;
    }

    if (p.size() == 1 + dualsense::OUTPUT_BT_SIZE
        && p[0] == dualsense::HIDP_DATA_OUTPUT && p[1] == dualsense::REPORT_BT) {
        c.output_reports++;
        if (dualsense::crc32_seeded(dualsense::CRC_SEED_OUTPUT, &p[1],
                                    p.size() - 5)
            == trailing_crc(p.data(), p.size()))
            c.output_crc_ok++;
    }
}

void inspect_event(const uint8_t *rec, size_t len, Counts &c,
                   std::map<uint8_t, uint64_t> &short_events,
                   AclFlowControl &acl, std::map<uint16_t, int> &outstanding,
                   int64_t ts_us, int64_t &last_completion_us)
{
    c.events++;
    PacketContents pc(const_cast<uint8_t *>(rec), len);
    if (!pc.get<uint8_t>())
        return;
    const auto *hdr = pc.get<hci_event_hdr>();
    if (!hdr) {
        c.event_short++;
        return;
    }

    if (hdr->evt == EVT_CMD_COMPLETE) {
        const auto *cc = pc.get<evt_cmd_complete>();
        if (cc && btohs(cc->opcode)
                      == cmd_opcode_pack(OGF_INFO_PARAM, OCF_READ_BUFFER_SIZE)) {
            note_buffer_size(pc, c);
            if (c.pool)
                acl.set_max_packets(c.pool);
        }
        return;
    }

    if (hdr->evt == EVT_NUM_COMP_PKTS) {
        c.completion_events++;
        if (last_completion_us >= 0 && ts_us >= last_completion_us)
            c.completion_gaps_us.push_back(uint32_t(ts_us - last_completion_us));
        last_completion_us = ts_us;

        // Parsed the way handle_num_completed_packets() parses it.
        const uint8_t plen = hdr->plen;
        const auto *body = pc.bytes(plen);
        if (!body) {
            c.event_short++;
            short_events[hdr->evt]++;
            return;
        }
        const uint8_t handles = body[0];
        for (uint8_t i = 0; i < handles; i++) {
            const size_t off = 1 + size_t(i) * 4;
            if (off + 4 > plen)
                break;
            const uint16_t h = uint16_t(body[off] | (body[off + 1] << 8));
            const int done = int(body[off + 2] | (body[off + 3] << 8));
            c.acl_completed += uint64_t(done);
            // A controller freeing more buffers than we ever counted for a link
            // means packets left without being accounted - which is exactly the
            // shape of the bug where L2CAP signalling was sent unmetered.
            if (done > outstanding[h])
                c.over_completions++;
            outstanding[h] = std::max(0, outstanding[h] - done);
            acl.on_completed(h, done);
        }
        if (acl.in_flight_total() < 0)
            c.negative_total++;
        return;
    }

    if (hdr->evt == EVT_DISCONN_COMPLETE) {
        const auto *d = pc.get<evt_disconn_complete>();
        if (d) {
            acl.on_disconnect(d->handle);
            outstanding.erase(d->handle);
        }
        return;
    }

    const size_t need = event_struct_size(hdr->evt);
    if (!need)
        return;                        // an event this relay does not read
    if (pc.remaining() < need) {
        c.event_short++;
        short_events[hdr->evt]++;
    }
}

}  // namespace

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::println(stderr, "usage: {} <capture.dump>", argv[0]);
        return 2;
    }
    std::FILE *f = std::fopen(argv[1], "rb");
    if (!f) {
        std::println(stderr, "{}: cannot open", argv[1]);
        return 2;
    }
    uint8_t hdr[16];
    if (!read_exact(f, hdr, sizeof(hdr)) || std::memcmp(hdr, "btsnoop\0", 8) != 0) {
        std::println(stderr, "{}: not a btsnoop capture", argv[1]);
        std::fclose(f);
        return 2;
    }

    Counts c;
    AclFlowControl acl;
    std::map<uint16_t, int> outstanding;
    int64_t last_completion_us = -1;
    std::map<uint8_t, uint64_t> short_events, unknown_feature;
    std::vector<uint8_t> rec;
    uint8_t rh[24];
    while (read_exact(f, rh, sizeof(rh))) {
        const uint32_t incl = be32(rh + 4);
        if (incl > 65535)
            break;
        rec.resize(incl);
        if (!read_exact(f, rec.data(), incl))
            break;
        c.records++;
        if (incl < 1)
            continue;
        // btsnoop flags bit 0: 0 = sent by the host, 1 = received.
        const bool outbound = (be32(rh + 8) & 1) == 0;
        const int64_t ts_us = int64_t(be32(rh + 16)) << 32 | be32(rh + 20);
        if (rec[0] == 0x04)
            inspect_event(rec.data(), incl, c, short_events, acl, outstanding,
                          ts_us, last_completion_us);
        else if (rec[0] == 0x02)
            inspect_acl(rec.data(), incl, c, unknown_feature, outbound, acl,
                        outstanding);
    }
    std::fclose(f);

    std::println("{}", argv[1]);
    std::println("  records {}  events {}  acl {}", c.records, c.events, c.acl);
    std::println("");
    std::println("  input reports      {:>8}   crc ok {}", c.input_reports,
                 c.input_crc_ok);
    std::println("    state / opaque   {:>8} / {}", c.state_frames, c.opaque_frames);
    std::println("    flag vs marker   {:>8} agree, {} state-not-marked, "
                 "{} marked-but-state",
                 c.flag_marker_agree, c.flag_says_state_marker_says_opaque,
                 c.flag_says_opaque_marker_says_state);
    std::println("  feature answers    {:>8}   crc ok {}   size known {} agree {}",
                 c.feature_answers, c.feature_crc_ok, c.feature_size_known,
                 c.feature_size_agrees);
    std::println("  output reports     {:>8}   crc ok {}", c.output_reports,
                 c.output_crc_ok);
    std::println("");
    std::println("  l2cap truncated    {:>8}   (declared more than arrived)",
                 c.l2cap_truncated);
    std::println("  events short       {:>8}", c.event_short);
    if (c.acl_sent || c.completion_events) {
        std::println("");
        std::println("  acl flow control (replayed through AclFlowControl)");
        std::println("    pool from Read Buffer Size  {:>8}", c.pool);
        std::println("    packets we sent             {:>8}", c.acl_sent);
        std::println("    buffers the controller freed{:>8}  in {} events",
                     c.acl_completed, c.completion_events);
        std::println("    still outstanding at the end{:>8}", acl.in_flight_total());
        std::println("    peak in flight              {:>8}", c.peak_in_flight);
        std::println("    completions for packets we never counted {}",
                     c.over_completions);
        if (!c.completion_gaps_us.empty()) {
            auto &g = c.completion_gaps_us;
            std::sort(g.begin(), g.end());
            auto pct = [&](double q) {
                return double(g[size_t(q * double(g.size() - 1))]) / 1000.0;
            };
            // The median, not the mean: AclFlowControl's note about this
            // controller batching completions quotes a median, so quote one
            // back. See tests/README.md for what these captures say about it.
            std::println("    completion gap: median {:.1f} ms  p10 {:.1f}  "
                         "p90 {:.1f}  p99 {:.1f}  max {:.1f}  (n={})",
                         pct(0.5), pct(0.1), pct(0.9), pct(0.99), pct(1.0),
                         g.size());
        }
    }
    for (auto [code, n] : short_events)
        std::println("      event 0x{:02x}: {}", code, n);
    for (auto [id, n] : unknown_feature)
        std::println("  feature id 0x{:02x} has no size in the table ({}x)", id, n);

    // What must hold.
    //
    // Checksums are judged as a rate, not as a count, and the two failure modes
    // are nowhere near each other. A wrong seed or a wrong offset fails every
    // report; radio noise fails a handful - measured on these captures, two out
    // of 1553762, which is one in 777 thousand. The pad keeps its own count of
    // exactly this in BtCrcFailCount, so it is a fact of the link rather than a
    // fault in the relay, and a report that arrives corrupt is dropped by the
    // console anyway. Anything above a tenth of a percent is arithmetic.
    auto crc_rate_ok = [](const char *what, uint64_t ok, uint64_t total) {
        if (total == 0 || ok == total)
            return true;
        const double bad_rate = double(total - ok) / double(total);
        std::println("  {} of {} {} failed their checksum ({:.6f}%){}",
                     total - ok, total, what, bad_rate * 100,
                     bad_rate > 0.001 ? "  <- too many to be the air" : "");
        return bad_rate <= 0.001;
    };

    int bad = 0;
    if (!crc_rate_ok("input reports", c.input_crc_ok, c.input_reports))
        bad = 1;
    if (!crc_rate_ok("feature answers", c.feature_crc_ok, c.feature_answers))
        bad = 1;
    if (!crc_rate_ok("output reports", c.output_crc_ok, c.output_reports))
        bad = 1;
    if (c.feature_size_agrees != c.feature_size_known) {
        std::println("FAIL feature_report_size() disagrees with the wire {}x",
                     c.feature_size_known - c.feature_size_agrees);
        bad = 1;
    }
    if (c.event_short) {
        std::println("FAIL {} events were shorter than the structure the relay "
                     "reads from them", c.event_short);
        bad = 1;
    }
    if (c.negative_total) {
        std::println("FAIL the in-flight total went below zero {}x",
                     c.negative_total);
        bad = 1;
    }
    if (c.pool && c.peak_in_flight > int(c.pool)) {
        std::println("FAIL peak in flight {} exceeded the controller pool {}",
                     c.peak_in_flight, c.pool);
        bad = 1;
    }
    std::println("{}", bad ? "\nFAILED" : "\nall invariants held");
    return bad;
}
