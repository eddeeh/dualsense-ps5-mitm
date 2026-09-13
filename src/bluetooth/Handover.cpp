//
// Handover.cpp - ps5padlog
//
// Moving the controller from the USB cable to the console's radio: the pairing
// data the console writes in report 0x0a, the pair command 0x08 that detaches
// the gadget and calls the console, the fast repaging after it, and easing the
// radio off once the console plays through its second link.
//

#include "bluetooth/BluetoothHandler.h"
#include "bluetooth/BluetoothHandlerInternal.h"
#include "dualsense/DualSense.h"

#include <array>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <print>

// Prints a report as it arrived from the console, before any re-framing.
static void dump_report(const char *tag, const uint8_t *report, size_t size) {
    printf("%s [%zu]:", tag, size);
    for (size_t i = 0; i < size && i < 32; i++)
        printf(" %02x", report[i]);
    if (size > 32)
        printf(" ...");
    printf("\n");
}

void BluetoothHandler::l2cap_send_feature_set(const uint8_t *report, size_t size) {
    if (size < 1)
        return;

    const uint8_t report_id = report[0];

    // Report 0x0a hands the controller a Bluetooth host address and link key to
    // pair with: the console's own, for the link it is about to expect. It is
    // absorbed for two reasons. The pad must not see it - it would repoint
    // itself at the console - and it could not act on it anyway, answering
    // ERR_INVALID_REPORT_ID over Bluetooth because it is already paired with
    // us. The console's status stage was acknowledged by the gadget regardless.
    // What the report carries is exactly what the PS5-facing side needs to
    // authenticate with, which is why it is captured here rather than dropped.
    if (report_id == dualsense::REPORT_PAIRING_SET) {
        dump_report("SET_REPORT 0x0a", report, size);
        if (size >= 23) {
            std::array<uint8_t, 16> key{};
            memcpy(key.data(), &report[7], 16);

            bool all_zero = true;
            for (auto b : key)
                if (b) { all_zero = false; break; }

            bdaddr_t host{};
            memcpy(host.b, &report[1], 6);

            if (all_zero) {
                // The console only issues a key the first time it meets a
                // controller; afterwards it writes zeros and expects the bond
                // it already has. Keeping the stored key is the whole point of
                // persisting it.
                std::println("  -> console {}, no new key{}", host,
                             console_pairing_seen_ ? " - keeping the saved one"
                                                   : " - and nothing saved yet");
                if (console_pairing_seen_)
                    console_addr_ = host;
            } else {
                console_addr_ = host;
                console_link_key_ = key;
                console_pairing_seen_ = true;
                std::print("  -> console {}, link key ", host);
                for (auto b : key)
                    std::print("{:02x}", b);
                std::print("\n");
                if (on_console_pairing_)
                    on_console_pairing_(console_addr_, console_link_key_);
            }
        }
        return;
    }

    // Report 0x08 is a command register, not a single command: the console has
    // been seen writing subcommands 0x01, 0x02 and 0x11 through it. Only 0x02
    // is known to be destructive - the pad acknowledged it and tore down all
    // three L2CAP channels 0 ms later (capture-20260905-090400, t=1.787s).
    // Swallowing the rest leaves the console retrying forever, waiting for a
    // state change that can only happen on the pad, so everything else goes
    // through - and 0x01, the request that the controller come to the console
    // over Bluetooth, is what starts the PS5-facing connection.
    if (report_id == dualsense::REPORT_TRANSPORT) {
        const uint8_t sub = size > 1 ? report[1] : 0;
        dump_report("SET_REPORT 0x08", report, size);

        if (sub == dualsense::TRANSPORT_UNPAIR) {
            printf("  -> unpair, absorbed\n");
            return;
        }

        // Absorbing this outright was a mistake: the pad has to actually see the
        // command for its state to change - only 0x02, which makes it drop the
        // link to us, stays absorbed.
        printf("  -> pair (0x%02x), relaying to the pad\n", sub);
        // Whatever produced it - the relay's own press or a hand on the pad -
        // the console has answered and there is nothing left to press for.
        if (!pair_command_seen_)
            log("pair command seen after {} synthetic press(es)", auto_ps_attempt_);
        pair_command_seen_ = true;
        l2cap_send_hid_feature_set(report, size);

        if (!console_pairing_seen_) {
            printf("  -> no pairing data captured yet, cannot reach the console\n");
            return;
        }

        // No console-facing radio - its adapter was missing or could not be
        // claimed - so there is nothing to hand the link over to. The gadget
        // stays attached; the console will not play through it, but detaching
        // would only take the controller off its screen as well.
        if (!other_) {
            std::cout << "  -> no console radio to hand over to, keeping USB attached"
                      << std::endl;
            return;
        }

        // Unplug, the way a real pad is unplugged at this point. The console
        // will not hold a Bluetooth link while the same controller sits on its
        // USB port - it answers our page with "limited resources" - so the
        // cable has to go before the radio link can exist. Deferred by a turn
        // of the event loop rather than done inline, because we are inside the
        // handler for the very report that asked for it.
        boost::asio::co_spawn(ios, [this]() -> boost::asio::awaitable<void> {
            co_await sleep_for(std::chrono::milliseconds(0));

            std::cout << "  -> detaching from USB" << std::endl;
            gadget.disable_gadget();

            // No pause before calling. There used to be 1.5 s here. With it the
            // console was reached 3.57 and 3.63 s after the pair command
            // (run-20260911-142535, -142753); without it 3.47 s, the first page
            // timing out and the immediate second one landing 0.9 s in
            // (run-20260911-143048), and no "limited resources" either way. The
            // console is ready when it is ready, and the pause was dead time.
            other_->connect_to_console(console_addr_, console_link_key_);
        }, boost::asio::detached);
        return;
    }

    l2cap_send_hid_feature_set(report, size);
}

void BluetoothHandler::connect_to_console(const bdaddr_t &addr,
                                         const std::array<uint8_t, 16> &key) {
    if (conn_side_ != EConnectionSide::to_playstation)
        return;
    if (connection_handle_ != 0)
        return;  // already on the console

    link_key_ = key;
    console_addr_ = addr;   // also what handle_conn_request() filters on
    bt_addr_ = addr;

    // From here the console may call us, so become reachable - and only now,
    // because until this moment the pad-facing adapter has been paging for this
    // very address and would find us instead of the controller.
    write_scan_enable(SCAN_PAGE_ONLY);
    log("now reachable - the pad is ours, the address is free");

    log("calling the console at {}", addr);
    handover_paging_since_ = std::chrono::steady_clock::now();
    write_page_timeout(HANDOVER_PAGE_TIMEOUT_SLOTS);
    send_connection_request(bt_addr_);
}

void BluetoothHandler::end_handover_paging() {
    if (handover_paging_since_ == std::chrono::steady_clock::time_point{})
        return;
    handover_paging_since_ = {};
    write_page_timeout(PAGE_TIMEOUT_SLOTS);
}

// Half of the input stream was not reaching the console on time, and neither
// reason was the console's doing.
//
// Page scan is set to a 180 ms window every 640 ms, which is deliberately
// aggressive so the console's call-back is never missed - but once it has
// called back, that is a quarter of the radio's time spent listening for a page
// that will not come. And the console leaves its registration link connected
// behind the play link, sniffed at 223.75 ms; every anchor point takes the
// radio off the link that matters. Measured together they cost 45.7 seconds of
// blocked transmit out of 96, in stalls of about 200 ms - which is exactly what
// the delay felt like in the hand.
//
// Neither is needed any more at this point: the console has called back, opened
// its channels and started sending output reports.
void BluetoothHandler::quiet_the_radio() {
    // Only once the console has actually moved us to its play radio. Doing this
    // on the registration link disables the very thing the call-back needs: the
    // console pages us from the second radio, finds nobody listening, and drops
    // the link fifteen seconds later - a clean twenty-one second loop that
    // never reaches a controller.
    if (!on_play_link_)
        return;

    // Not off - just no longer listening a quarter of the time. The standard
    // 11.25 ms window every 1.28 s keeps us findable if the console ever pages
    // us again, at about one percent of the radio instead of twenty-eight.
    write_page_scan_activity(PAGE_SCAN_INTERVAL_NORMAL, PAGE_SCAN_WINDOW_NORMAL);
    log("play link is up - page scan back to its normal duty cycle");

    // The registration link stays, and measured so: the console holds both links
    // as one presence. Closing it even well after the handover - play link
    // encrypted, channels open, output reports already flowing - makes the
    // console close the play link within milliseconds.
}
