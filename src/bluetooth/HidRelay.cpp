//
// HidRelay.cpp - ps5padlog
//
// The relay itself: the pad's input to the console over Bluetooth and to the
// USB gadget, the console's output and feature requests back to the pad, and
// the gadget's calls asking the pad the same things over the cable.
//

#include "bluetooth/BluetoothHandler.h"
#include "dualsense/DualSense.h"
#include "dualsense/Transforms.h"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <format>
#include <iostream>
#include <print>
#include <string>
#include <vector>

namespace {

// The console's output reports carry a subtype in byte 3. This one names the
// radio it wants the controller to move to - the bytes that looked like an
// audio volume field are a Bluetooth address.
constexpr uint8_t OUTPUT_SUBTYPE_HANDOVER = 0x97;

// The relay presses PS for the console itself, in the reports the gadget sends
// it, so the pad need not be touched a second time. Measured to work from a
// second after input starts flowing, where the same press at 211 ms was ignored
// outright. The repeat is what makes it safe rather than the delay being
// exactly right, so it is not a single shot.
constexpr int AUTO_PS_DELAY_MS  = 1000;
constexpr int AUTO_PS_PERIOD_MS = 1500;

}  // namespace

// The pad's answer to a feature request. Two things happen to it on the way
// past: the controller address inside reports 0x09 and 0x0b is rewritten to
// the adapter the console will actually meet, and anything the console asked
// for is handed straight back to it.
void BluetoothHandler::handle_feature_answer_from_pad(std::span<uint8_t> report) {

    // A one-byte payload is a HIDP HANDSHAKE: the pad acknowledging or
    // refusing our last request. Naming the code makes the difference
    // between "wrong report for this transport" and "wrong contents"
    // visible in the log.
    if (report.size() == 1) {
        const uint8_t code = report[0] & 0x0f;
        log("HIDP handshake: {} (0x{:02x})",
            dualsense::handshake_name(code), report[0]);
    } else {
        std::string hex;
        for (size_t i = 0; i < std::min<size_t>(report.size(), 14); i++)
            hex += std::format("{:02x} ", report[i]);
        log("HID control rx [{}]: {}", report.size(), hex);
    }

    // Report 0x09 carries the pad's own address and the host it is
    // bonded to. If that host is not this adapter, no saved link key
    // will ever authenticate and the pad has to be paired again.
    if (report[0] == dualsense::HIDP_DATA_FEATURE
        && report[1] == dualsense::REPORT_PAIRING_INFO
        && report.size() >= 17) {
        bdaddr_t host{};
        memcpy(host.b, &report[11], 6);
        if (host == local_addr_)
            std::println("Pad is bonded to host {} (this adapter)", host);
        else
            std::println("Pad is bonded to host {} but we are {} - it will need pairing mode",
                         host, local_addr_);
    }

    // Report 0x0b is the same idea with history: four slots of [rank][host],
    // rank 0 being the host the pad attached to most recently. It keeps three,
    // and a fresh bond to our spoofed address takes a slot of its own rather
    // than updating the one already holding that address - so run this often
    // enough and the console falls off the end. The pad then looks broken when
    // it is plugged back into the PS5, and the cure is a re-pair, which is
    // worth knowing before it gets diagnosed as dead hardware.
    if (report[0] == dualsense::HIDP_DATA_FEATURE
        && report[1] == dualsense::REPORT_PAIRING_LIST
        && report.size() >= dualsense::PAIRING_LIST_OFFSET
                            + dualsense::PAIRING_LIST_SLOTS
                            * dualsense::PAIRING_LIST_STRIDE) {
        const bdaddr_t &console = other_ ? other_->console_addr_ : console_addr_;
        bool console_listed = false;
        std::string line;
        for (size_t i = 0; i < dualsense::PAIRING_LIST_SLOTS; i++) {
            const uint8_t *entry = &report[dualsense::PAIRING_LIST_OFFSET
                                           + i * dualsense::PAIRING_LIST_STRIDE];
            bdaddr_t host{};
            memcpy(host.b, entry + 1, 6);
            if (!addr_is_set(host))
                continue;               // an unused slot is all zeros
            line += std::format(" [{}] {}", entry[0], host);
            if (addr_is_set(console) && host == console)
                console_listed = true;
        }
        std::cout << "Pad host table:" << line << std::endl;
        if (addr_is_set(console) && !console_listed) {
            std::println("Console {} has been pushed out of the pad's host table - using this "
                         "pad on the PS5 directly will need a re-pair", console);
        }
    }

    // The console reads the controller's Bluetooth address out of these
    // two reports and later expects a connection from it. Substituting
    // the PS5-facing adapter's address is what lets that connection
    // come from hardware we control.
    if (report[0] == dualsense::HIDP_DATA_FEATURE
        && (report[1] == dualsense::REPORT_PAIRING_INFO
            || report[1] == dualsense::REPORT_PAIRING_LIST)
        && report.size() >= 8
        && addr_is_set(ps5_side_addr_)) {
        memcpy(&report[2], ps5_side_addr_.b, 6);
        log("report 0x{:02x}: controller address rewritten to {}", report[1], ps5_side_addr_);

        // The host field matters just as much. Left alone it names our
        // pad-facing dongle, so the console reads it, sees a
        // controller bonded to somebody else and re-registers it on
        // every single connection - which is exactly what the log
        // shows. Once a console pairing has been captured, the pad is
        // presented as already bonded to the console.
    }

    // A feature answer the console asked for goes straight back to it.
    if (other_ && other_->hid_control_channel.dcid != 0)
        other_->l2cap_send_hid(other_->hid_control_channel, report[0],
                               &report[1], report.size() - 1);

    if (report[0] == dualsense::HIDP_DATA_FEATURE) {
        // Strip the HIDP header only. report[1] is the report ID, which
        // HID requires as the first byte of the GET_REPORT response on
        // the USB side, so everything from there on goes across as is.
        gadget.on_feature_report(&report[1], report.size() - 1);
    }
}

// The console's side of the control channel: feature reads and writes, and the
// HIDP handshakes that acknowledge them.
void BluetoothHandler::handle_feature_request_from_console(std::span<uint8_t> report) {
    // Console side: a GET/SET the console wants answered by the
    // controller. Only the real pad can answer it, so it is passed
    log("console control rx [{}]: {:02x} {:02x}", report.size(), report[0],
        report.size() > 1 ? report[1] : 0);

    // Asking for calibration or firmware info is exactly what makes a
    // real pad start sending the full 0x31 report.
    if (report.size() > 1 && !console_full_reports_
        && (report[1] == dualsense::REPORT_CALIBRATION
            || report[1] == dualsense::REPORT_FIRMWARE)) {
        console_full_reports_ = true;
        std::cout << "Console asked for report 0x" << std::hex << (int)report[1]
                  << std::dec << " - switching it to full 0x31 reports"
                  << std::endl;
    }
    if (other_ && other_->hid_control_channel.dcid != 0)
        other_->l2cap_send_hid(other_->hid_control_channel, report[0],
                               &report[1], report.size() - 1);
}

// Rumble, lightbar, trigger effects and the haptic audio stream, on their way
// to the controller.
void BluetoothHandler::relay_console_output_to_pad(std::span<uint8_t> report) {
    // Straight
    if (!console_output_seen_) {
        console_output_seen_ = true;
        std::cout << "Console is sending output reports - the controller is live"
                  << std::endl;
        quiet_the_radio();
    }

    // These carry an address. The console repeats an output report whose
    // body holds its own second radio - 2C:9E:00:26:39:AB where the link
    // runs on ...:AA - and then gives up. It is not chatter, it is where it
    // wants the controller to go; relaying it to the pad, which is not
    // talking to the console at all, threw the instruction away.
    if (report.size() >= 17 && report[1] == dualsense::REPORT_BT
        && report[3] == OUTPUT_SUBTYPE_HANDOVER) {
        bdaddr_t target{};
        memcpy(target.b, &report[10], 6);

        if (addr_is_set(target) && target != bt_addr_
            && !handover_seen_) {
            handover_seen_ = true;
            std::println("Console named its other radio {} in an output report", target);
        }
    }
    // Straight through to the pad, exactly as it arrived. This used to
    // run the console's output reports through the *input* relay by
    // copy-paste: it thinned them, built a cut-down input report out of
    // their bytes and sent that to the controller. So the pad was fed 54
    // nonsense input reports a second and never received one piece of
    // rumble, lightbar, trigger effect or audio the console sent - which
    // the console can tell, because a controller that ignores its output
    // is not a controller it will take input from.
    if (other_ && other_->hid_interrupt_channel.dcid != 0) {
        if (!other_->pad_output_seen_) {
            other_->pad_output_seen_ = true;
            std::cout << "Forwarding console output reports to the pad" << std::endl;
        }
        // Effects and audio are worth nothing late, so they are dropped
        // rather than queued when the pad's controller is full - the same
        // rule as the input stream, for the same reason.
        //
        // The opaque-frame test used to be asked here too, and was inert: it
        // wants a 79-byte pad report with 0xd4 in body[1], and these are the
        // console's own 0x31/0x36/0x39, whose byte in that position is 0x3f or
        // 0x06. It never once returned true.
        if (!other_->acl_.has_room()) {
            stats_.console_output_dropped++;
        } else {
            stats_.console_output_count++;
            other_->l2cap_send_data(other_->hid_interrupt_channel,
                                    report.data(), report.size());
        }
    }
}

// The pad's own battery, straight out of the report body. Worth a line in the
// log because a flat controller does not announce itself as one: it closes its
// L2CAP channels politely, one at a time, disconnects with "Remote User
// Terminated", comes back for shorter and shorter links, and finally stops
// answering pages at all. From the outside that is indistinguishable from a
// console refusing to play, and it cost an hour of looking at the wrong end of
// the link.
void BluetoothHandler::log_pad_battery(std::span<const uint8_t> report) {
    if (report.size() < transforms::BT_BODY_OFF + dualsense::STATUS_OFF + 1)
        return;
    const uint8_t status = report[transforms::BT_BODY_OFF + dualsense::STATUS_OFF];

    // The byte is not always the status, and the reason is now known: about one
    // packet in ten on this channel is not an input report at all but a frame of
    // the controller's opaque audio stream, sharing report id 0x31 and carrying
    // a valid CRC. Reading a status out of one gives a random byte.
    //
    // So a single sample is not evidence, and printing one produced a stream of
    // nonsense: "charging error (status 0x65)" between every two honest
    // readings.
    //
    // Two filters. First, the value has to be possible at all: the charging
    // nibble has six defined states and the capacity nibble never exceeds ten.
    const uint8_t capacity = status & 0x0f;
    const uint8_t charging = status >> 4;
    switch (charging) {
    case dualsense::STATUS_CHARGING_DISCHARGING:
    case dualsense::STATUS_CHARGING_CHARGING:
    case dualsense::STATUS_CHARGING_FULL:
    case dualsense::STATUS_CHARGING_VOLTAGE_ERR:
    case dualsense::STATUS_CHARGING_TEMP_ERR:
    case dualsense::STATUS_CHARGING_ERROR:
        break;
    default:
        return;   // not a status byte at all
    }
    if (capacity > 10)
        return;

    // Second, it has to hold still. Junk never repeats; a real change does.
    static uint8_t candidate = 0xff;
    static int seen = 0;
    static uint8_t reported = 0xff;

    if (status != candidate) {
        candidate = status;
        seen = 1;
        return;
    }
    if (++seen < 3 || status == reported)
        return;
    reported = status;

    switch (charging) {
    case dualsense::STATUS_CHARGING_FULL:
        printf("Pad battery: full\n");
        return;
    case dualsense::STATUS_CHARGING_VOLTAGE_ERR:
        printf("Pad battery: voltage out of range - charge level unknown\n");
        return;
    case dualsense::STATUS_CHARGING_TEMP_ERR:
        printf("Pad battery: temperature out of range - charge level unknown\n");
        return;
    case dualsense::STATUS_CHARGING_ERROR:
        printf("Pad battery: charging error\n");
        return;
    default:
        break;
    }

    const char *state = charging == dualsense::STATUS_CHARGING_CHARGING
                            ? "charging" : "on battery";
    if (capacity >= 10) {
        printf("Pad battery: 100%%, %s\n", state);
        return;
    }

    // A pad that actually runs out does not announce it - it closes its L2CAP
    // channels one at a time, disconnects with "Remote User Terminated", comes
    // back for shorter and shorter links and finally stops answering pages. That
    // reads as a hostile console from every other angle, so the bottom band is
    // worth pointing at even though 0 does not mean empty.
    printf("Pad battery: %u-%u%%, %s%s\n", capacity * 10u, capacity * 10u + 9u, state,
           capacity == 0 ? "  <- lowest band; if it starts dropping the link, charge it"
                         : "");
}

// An input report from the pad, forwarded to whichever transport the console is
// listening on. Early in a run that is both at once: the USB gadget is still up
// while the radio link is being brought up behind it.
void BluetoothHandler::relay_pad_input(std::span<uint8_t> report) {
    if (report.size() >= 1 + dualsense::INPUT_BT_SIZE
        && report[0] == dualsense::HIDP_DATA_INPUT && report[1] == dualsense::REPORT_BT) {
        log_pad_battery(report);
        // For ps5padlog-view. State reports only - the opaque stream's frames share
        // the report id and their button bytes would read as random presses.
        if (transforms::carries_controller_state(report))
            input_feed_.publish(
                report.subspan(transforms::BT_BODY_OFF).first<inputfeed::BODY_LEN>(),
                link_snapshot());
    }

    relay_pad_input_to_usb(report);
    relay_pad_input_to_console(report);
}

// The 63-byte report body is identical on both transports; only the header in
// front of it differs.
//   BT : [0xa1][0x31][seq_tag][body 63][pad 9][crc32 4]
//   USB: [0x01][body 63]
// The seq_tag is BT-only framing - copying from it shifts every axis and button
// by one byte.
void BluetoothHandler::relay_pad_input_to_usb(std::span<uint8_t> report) {
    // Nothing to build it for. Most of a session is spent here: the gadget is
    // detached the moment the console asks the controller to come over
    // Bluetooth, and it never comes back.
    if (!gadget.transport_open())
        return;

    if (report.size() < 1 + dualsense::INPUT_BODY_OFF_BT + dualsense::INPUT_BODY_LEN
        || report[0] != dualsense::HIDP_DATA_INPUT
        || report[1] != dualsense::REPORT_BT)
        return;

    // On the stack, and built by the same tested function the Bluetooth path
    // uses. This runs at the pad's full report rate; a heap allocation per
    // report showed up in the profile as time in the slab allocator, for eighty
    // bytes of known size.
    auto usb = transforms::make_usb_report(report);

    // The console issues the pair command because it sees PS pressed in the
    // reports arriving from us and for no other reason, 8 ms later. One press
    // at 200 ms was ignored, so this presses repeatedly until the console
    // answers rather than once.
    if (const auto live = gadget.input_flowing_since();
        live != std::chrono::steady_clock::time_point{} && !pair_command_seen_) {
        const auto since = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - live).count();
        // Long enough to survive a poll rate that drops four writes in five.
        constexpr int width_ms = 200;
        if (since >= AUTO_PS_DELAY_MS
            && (since - AUTO_PS_DELAY_MS) % AUTO_PS_PERIOD_MS < width_ms) {
            transforms::set_ps_on_usb_report(usb);
            // One line per attempt, not per report: the attempts are seconds
            // apart and the count is what a log has to show afterwards.
            const int attempt = 1 + int((since - AUTO_PS_DELAY_MS) / AUTO_PS_PERIOD_MS);
            if (attempt != auto_ps_attempt_) {
                auto_ps_attempt_ = attempt;
                log("pressing PS for the console, attempt {} at {} ms", attempt, since);
            }
        }
    }

    gadget.send_input(usb);
}

// Over Bluetooth, which is the path that matters once the console has moved the
// controller to its play radio.
void BluetoothHandler::relay_pad_input_to_console(std::span<uint8_t> report) {
    if (!other_ || other_->hid_interrupt_channel.dcid == 0)
        return;

    if (!other_->console_input_seen_) {
        other_->console_input_seen_ = true;
        std::cout << "Forwarding pad input reports to the console" << std::endl;
    }

    // No rate cap here any more, and the numbers are why. It used to thin the
    // stream to 250 Hz before the flow-control check, and the counters showed
    // exactly 3.9 reports thinned per report sent - which is not a limit, it is
    // the arithmetic of "everything inside the 4 ms window after a send". The
    // credit check below is the real limiter: the console frees a buffer about
    // every 8 ms and we send the freshest report we have at that moment.
    //
    // Dropping the cap also drops a steady_clock::now() from a path that runs
    // six hundred times a second, and removes the one case where it could hurt:
    // a credit freed early would have found the fresher report thinned away.
    // Split the two streams before spending anything on them. Measured, the
    // opaque frames were 13.3% of everything we sent the console - 2979 of
    // 22387 - and they are not controller state, so every credit one took was
    // a credit the freshest stick position did not get. Dropping them raises
    // the state rate from 171 to 197 per second and pulls the median gap
    // between sent reports, by the pad's own clock, from 5.64 ms to 4.89 ms.
    if (opaque_frame(report)) {
        other_->stats_.console_input_opaque_dropped++;
        return;
    }

    if (!other_->acl_.has_room()) {
        // Not queued - queued reports go stale behind each other, which is what
        // the depth limit exists to prevent. But not thrown away either.
        //
        // The pad delivers in bursts: its median gap between reports is under a
        // millisecond and its p90 is 8 ms, so when a controller buffer comes
        // free in one of those quiet stretches there is nothing new to put in
        // it and we wait for the pad instead. Measured, that costs a median of
        // 20 ms of free credits going unused, and it is why our gaps to the
        // console cover 25% of the time while the pad's own cover 13%.
        //
        // So exactly one report is held - always the newest - and sent the
        // instant a buffer returns. One slot, so nothing can go stale in it.
        other_->stats_.console_input_backpressure++;
        other_->held_.store(report);   // refuses an oversized one on its own
        return;
    }

    other_->held_.clear();          // superseded by the one going out now

    other_->note_input_sent();

    if (other_->console_full_reports_)
        send_full_report_to_console(report);
    else
        send_cut_down_report_to_console(report);
}

// The full 0x31 report, which is what the console gets once it has asked for
// calibration or firmware info - relayed exactly as the pad sent it.
void BluetoothHandler::send_full_report_to_console(std::span<uint8_t> report) {
    other_->l2cap_send_hid(other_->hid_interrupt_channel, report[0],
                           &report[1], report.size() - 1);
}

// Before the console has asked for anything it is still expecting the cut-down
// report a pad sends on a fresh link - the same fields, fewer of them, in the
// pre-0x31 order, and no checksum. Starting on 0x31 instead gets the link
// closed with "this device isn't supported".
void BluetoothHandler::send_cut_down_report_to_console(std::span<uint8_t> report) {
    const auto basic = transforms::make_cut_down_report(report);
    other_->l2cap_send_hid(other_->hid_interrupt_channel,
                           dualsense::HIDP_DATA_INPUT, basic.data(), basic.size());
}

// A buffer just came back. If the pad has not spoken since we last had to drop
// a report, this is the freshest thing we have and the alternative is silence.
// Telling the pad's two streams apart. The opaque frames share the report id
// and pass the same CRC, so the only handles on them are the 0xd4 marker in
// body[1] and the fact that they carry no sensor clock. Measured over 37080
// CRC-verified pad reports: marker plus clock discontinuity caught 4714 of the
// 4720 opaque frames, the six that slipped through had a timestamp that landed
// in the window by chance, and not one state report was misjudged.
bool BluetoothHandler::opaque_frame(std::span<const uint8_t> report) {
    // Anything that is not the right size is left alone rather than dropped:
    // this decision only exists to throw away microphone frames, and a report
    // we cannot parse is not one we should be silently discarding.
    if (report.size() != 1 + dualsense::INPUT_BT_SIZE)
        return false;
    return !transforms::carries_controller_state(report);
}

void BluetoothHandler::flush_held_report() {
    if (!held_.valid() || !acl_.has_room())
        return;
    if (hid_interrupt_channel.dcid == 0) {
        held_.clear();
        return;
    }

    // The send helpers live on the pad-facing handler, because that is the side
    // that relays: they put the report into `other_`'s channel, and from there
    // `other_` is this one. Calling them on `this` would send the report back to
    // the controller.
    if (!other_)
        return;

    const std::span<uint8_t> held = held_.view();
    held_.clear();
    note_input_sent();
    stats_.console_input_flushed++;

    if (console_full_reports_)
        other_->send_full_report_to_console(held);
    else
        other_->send_cut_down_report_to_console(held);
}

// ============================================================================
// USB gadget
// ============================================================================

boost::asio::awaitable<void> BluetoothHandler::start_usb_gadget() {
    // Pre-fill the feature cache before the console can ask for it. These three
    // describe fixed properties of the pad, so one fetch is enough; 0x20 also
    // switches the DualSense into full 0x31 report mode.
    l2cap_get_report(dualsense::REPORT_CALIBRATION);
    l2cap_get_report(dualsense::REPORT_PAIRING_INFO);
    l2cap_get_report(dualsense::REPORT_FIRMWARE);

    // Let the pad answer before the PS5 starts enumerating. Waiting on a timer
    // rather than sleeping keeps the HCI read loop running - the answers we are
    // waiting for arrive on it.
    co_await sleep_for(std::chrono::milliseconds(300));

    // On a reconnect the gadget is already enumerated on the PS5 and the pad
    // has just been re-primed above; bringing the gadget up a second time
    // would only make the console re-enumerate for no reason.
    if (usb_started_)
        co_return;
    usb_started_ = true;

    if (!gadget.enable_gadget()) {
        std::cerr << "USB gadget failed to come up" << std::endl;
        co_return;
    }

    boost::asio::co_spawn(ios, gadget.control_events(), boost::asio::detached);
    boost::asio::co_spawn(ios, gadget.endpoint_events(), boost::asio::detached);
    boost::asio::co_spawn(ios, gadget.report_stats(), boost::asio::detached);
    std::cout << "USB gadget live, relaying to PS5" << std::endl;
}

void BluetoothHandler::l2cap_get_report(uint8_t report_id) {
    l2cap_send_hid(hid_control_channel, dualsense::HIDP_GET_FEATURE, &report_id, 1);
}

// Re-frames a console feature report for Bluetooth and puts it on the pad's
// control channel.
void BluetoothHandler::l2cap_send_hid_feature_set(const uint8_t *report, size_t size) {
    const uint8_t report_id = report[0];

    // Over Bluetooth a feature report is always its full declared size with a
    // CRC32 in the last four bytes. Over USB the console sends only as much as
    // it needs and omits the checksum, so the report has to be re-framed here.
    size_t bt_size = dualsense::feature_report_size(report_id);
    if (bt_size == 0)
        bt_size = size + 4; // unknown report: assume payload plus checksum
    if (bt_size < 5)
        return;

    std::vector<uint8_t> buf(bt_size, 0);
    memcpy(buf.data(), report, std::min(size, bt_size - 4));
    dualsense::append_crc32(dualsense::CRC_SEED_SET_FEATURE, buf.data(), bt_size);

    l2cap_send_hid(hid_control_channel, dualsense::HIDP_SET_FEATURE, buf.data(), bt_size);
}

void BluetoothHandler::l2cap_send_output_report(const uint8_t *usb_report, size_t size) {
    // USB output report: [0x02][common 47][reserved]
    // BT  output report: [0x31][seq_tag][tag][common 47][reserved 24][crc32]
    // Only the framing differs; the 47-byte body is shared.
    if (size < 2)
        return;

    uint8_t bt[dualsense::OUTPUT_BT_SIZE] = {};
    bt[0] = dualsense::REPORT_BT;
    bt[1] = static_cast<uint8_t>(output_seq_ << 4);
    bt[2] = dualsense::OUTPUT_TAG;
    output_seq_ = (output_seq_ + 1) & 0x0f;

    const size_t common = std::min(size - 1, dualsense::OUTPUT_COMMON_SIZE);
    memcpy(&bt[3], &usb_report[1], common);

    dualsense::append_crc32(dualsense::CRC_SEED_OUTPUT, bt, sizeof(bt));
    l2cap_send_hid(hid_interrupt_channel, dualsense::HIDP_DATA_OUTPUT, bt, sizeof(bt));
}
