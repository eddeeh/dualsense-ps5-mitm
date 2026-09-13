//
// HciCommands.cpp - ps5padlog
//
// The HCI commands BluetoothHandler sends, and the adapter settings built out
// of them: pairing, scan modes, the console-facing identity, paging the peer.
//

#include "bluetooth/BluetoothHandler.h"
#include "bluetooth/BluetoothHandlerInternal.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <print>
#include <vector>

static constexpr uint8_t HCI_AUTH_ENABLE_VAL = 0x01;

// DualSense Extended Inquiry Response (name = "DualSense Wireless Controller", class 0x002508)
static std::vector<uint8_t> extended_inquiry0 = {
    0x05, 0x03, 0x24, 0x11, 0x00, 0x12, 0x1e, 0x09, 0x44, 0x75, 0x61, 0x6c, 0x53, 0x65, 0x6e, 0x73,
    0x65, 0x20, 0x57, 0x69, 0x72, 0x65, 0x6c, 0x65, 0x73, 0x73, 0x20, 0x43, 0x6f, 0x6e, 0x74, 0x72,
    0x6f, 0x6c, 0x6c, 0x65, 0x72, 0x09, 0x10, 0x02, 0x00, 0x4c, 0x05, 0xe6, 0x0c, 0x00, 0x01, 0x00,
    // ... rest is zeros
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

namespace {

constexpr uint16_t LINK_POLICY_ROLE_SWITCH = 0x0001;
constexpr uint16_t LINK_POLICY_SNIFF       = 0x0004;

// Class of device: gamepad. The same three bytes appear inside feature reports
// 0x09 and 0x0b, which is where the console reads them from.
constexpr uint32_t CLASS_OF_DEVICE_GAMEPAD = 0x002508;

}  // namespace

bool BluetoothHandler::send_cmd(uint16_t ogf, uint16_t ocf, uint8_t plen,
                                const void *param, const char *what) {
    // Queued rather than written: see HciCommandQueue.h for what writing them
    // all at once did to the BT400. True means accepted, not yet answered -
    // which is all a write ever meant, since the answer comes as an event.
    HciCommandQueue::Command c;
    c.ogf = ogf;
    c.ocf = ocf;
    if (plen && param) {
        const auto *b = static_cast<const uint8_t *>(param);
        c.param.assign(b, b + plen);
    }
    c.what = what;
    c.quiet = suppress_cmd_errors_;
    cmd_queue_.push(std::move(c));
    pump_commands();
    return true;
}

void BluetoothHandler::pump_commands() {
    while (const auto *c = cmd_queue_.next()) {
        HciCommandQueue::Command cmd = *c;
        // hci_send_cmd takes a non-const parameter block it only ever reads.
        if (hci_send_cmd(bt_socket_.native_handle(), cmd.ogf, cmd.ocf,
                         uint8_t(cmd.param.size()),
                         cmd.param.data()) == 0) {
            cmd_queue_.written(std::chrono::steady_clock::now());
        } else {
            if (!cmd.quiet)
                logerr("{} failed: {}", cmd.what, strerror(errno));
            cmd_queue_.dropped();
        }
    }
}

// Four times a second, pad or no pad. Reports carry the link block too, but
// they only exist once the pad is talking, and every step before that is one
// where the viewer would otherwise have nothing to say.
boost::asio::awaitable<void> BluetoothHandler::publish_stage() {
    for (;;) {
        input_feed_.publish_link(link_snapshot());
        co_await sleep_for(std::chrono::milliseconds(250));
    }
}

// A controller that takes a command and never answers would otherwise stop the
// queue for good - and with it every link key reply and connection accept
// behind it. Checked often enough that the wait is the timeout and not this.
boost::asio::awaitable<void> BluetoothHandler::watch_commands() {
    for (;;) {
        co_await sleep_for(std::chrono::milliseconds(250));
        if (!cmd_queue_.overdue(std::chrono::steady_clock::now()))
            continue;
        logerr("a command had no answer in 2 s - moving on; {} still queued",
               cmd_queue_.queued());
        cmd_queue_.give_up();
        pump_commands();
    }
}

void BluetoothHandler::write_simple_pairing_enabled() {
    write_simple_pairing_mode_cp request = {.mode = 1};
    send_cmd(OGF_HOST_CTL, OCF_WRITE_SIMPLE_PAIRING_MODE,
             WRITE_SIMPLE_PAIRING_MODE_CP_SIZE, &request, "write simple pairing");
}

void BluetoothHandler::write_auth_enabled() {
    uint8_t auth_enabled = HCI_AUTH_ENABLE_VAL;
    send_cmd(OGF_HOST_CTL, OCF_WRITE_AUTH_ENABLE, sizeof(auth_enabled),
             &auth_enabled, "write auth enable");
}

void BluetoothHandler::write_event_mask() {
    set_event_mask_cp request = {
        .mask = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xbf, 0x3d}
    };
    send_cmd(OGF_HOST_CTL, OCF_SET_EVENT_MASK, SET_EVENT_MASK_CP_SIZE, &request,
             "set event mask");
}

void BluetoothHandler::write_class_of_device(uint32_t dev_class) {
    uint8_t dev_class_tmp[3] = {};
    memcpy(dev_class_tmp, &dev_class, 3);
    send_cmd(OGF_HOST_CTL, OCF_WRITE_CLASS_OF_DEV, sizeof(dev_class_tmp),
             dev_class_tmp, "write class of device");
}

void BluetoothHandler::write_extended_inquiry_result(std::vector<uint8_t> &&data) {
    write_ext_inquiry_response_cp eir;
    memset(&eir, 0, WRITE_EXT_INQUIRY_RESPONSE_CP_SIZE);
    eir.fec = 0;
    memcpy(eir.data, data.data(), std::min(data.size(), sizeof(eir.data)));
    send_cmd(OGF_HOST_CTL, OCF_WRITE_EXT_INQUIRY_RESPONSE,
             WRITE_EXT_INQUIRY_RESPONSE_CP_SIZE, &eir, "write EIR");
}

void BluetoothHandler::write_page_timeout(uint16_t slots) {
    uint16_t timeout = htobs(slots);
    send_cmd(OGF_HOST_CTL, OCF_WRITE_PAGE_TIMEOUT, sizeof(timeout), &timeout,
             "write page timeout");
}

void BluetoothHandler::write_page_scan_activity(uint16_t interval, uint16_t window) {
    write_page_activity_cp cp = {
        .interval = htobs(interval),
        .window = htobs(window),
    };
    send_cmd(OGF_HOST_CTL, OCF_WRITE_PAGE_ACTIVITY, WRITE_PAGE_ACTIVITY_CP_SIZE,
             &cp, "write page scan activity");
}

void BluetoothHandler::write_default_link_policy(uint16_t policy) {
    uint16_t value = htobs(policy);
    send_cmd(OGF_LINK_POLICY, OCF_WRITE_DEFAULT_LINK_POLICY, sizeof(value),
             &value, "write default link policy");
}

// Puts the controller back to power-on state before anything else is sent.
//
// On an HCI user channel the kernel skips its own initialisation, reset
// included - the channel exists so an application can own the controller, and
// owning it means resetting it. Nothing here did. So a controller kept whatever
// the last run left in its firmware, and the one that mattered was a live ACL
// link to the console: across a day of runs, all five that hung at
// "registering on the PS5" started with the console already streaming effects
// to us over Bluetooth on a handle this run never opened, and all eight that
// started without one registered. The console had its controller; it had no
// reason to poll the USB gadget for input. Unplugging the dongle fixed it
// because unplugging is a reset - this is the same thing without the hands.
//
// Waits for the Command Complete rather than sleeping a guess: a controller
// processing a reset discards the commands behind it, and every packet that
// arrives before the answer belongs to the state being thrown away, so it is
// dropped here rather than handed to code that would act on it.
boost::asio::awaitable<bool> BluetoothHandler::reset_controller() {
    // Written directly, with the queue shut behind it: nothing else may reach
    // the controller until it says the reset is done. Another handler's command
    // arriving inside the BT400's 306 ms of resetting is what left it silent.
    cmd_queue_.hold();
    if (hci_send_cmd(bt_socket_.native_handle(), OGF_HOST_CTL, OCF_RESET, 0, nullptr) != 0) {
        logerr("reset failed: {}", strerror(errno));
        cmd_queue_.release();
        pump_commands();
        co_return false;
    }

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    int discarded = 0;
    while (std::chrono::steady_clock::now() < deadline) {
        const ssize_t n = ::recv(bt_socket_.native_handle(), read_buffer_,
                                 sizeof(read_buffer_), MSG_DONTWAIT);
        if (n <= 0) {
            co_await sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        // H4 indicator, event code, length, commands allowed, opcode, status.
        if (n >= 7 && read_buffer_[0] == HCI_EVENT_PKT && read_buffer_[1] == EVT_CMD_COMPLETE
            && (read_buffer_[4] | (read_buffer_[5] << 8)) == cmd_opcode_pack(OGF_HOST_CTL, OCF_RESET)) {
            if (discarded)
                log("reset dropped {} packet(s) from what the last run left behind", discarded);
            if (read_buffer_[6] != 0)
                logerr("reset refused: status 0x{:02x}", read_buffer_[6]);
            cmd_queue_.answered(read_buffer_[3]);
            cmd_queue_.release();
            pump_commands();
            co_return read_buffer_[6] == 0;
        }
        discarded++;
    }
    logerr("did not complete a reset in 2 s - the controller has stopped answering. "
           "Unplug it, plug it back in and start again");
    cmd_queue_.release();
    pump_commands();
    co_return false;
}

// Asks the controller how the link is actually doing. Neither answer changes
// anything we do - they are here because "the connection is bad" had no
// measurement behind it, and these two separate a radio problem from a
// scheduling one. Quiet on failure: a controller that does not implement them
// should not fill the log every five seconds.
void BluetoothHandler::read_radio_quality() {
    if (connection_handle_ == 0)
        return;
    const uint16_t h = htobs(connection_handle_);
    suppress_cmd_errors_ = true;
    send_cmd(OGF_STATUS_PARAM, OCF_READ_LINK_QUALITY, sizeof(h), &h, "read link quality");
    send_cmd(OGF_STATUS_PARAM, OCF_READ_RSSI, sizeof(h), &h, "read rssi");
    // The hop set. Everything else about this link measures healthy while it
    // stalls, and a map with half its channels struck out is the one remaining
    // shape that looks like that: the radio is perfect on the channels it is
    // allowed, and there are not enough of them to keep a schedule.
    send_cmd(OGF_STATUS_PARAM, OCF_READ_AFH_MAP, sizeof(h), &h, "read afh map");
    suppress_cmd_errors_ = false;
}

void BluetoothHandler::write_scan_enable(uint8_t mode) {
    send_cmd(OGF_HOST_CTL, OCF_WRITE_SCAN_ENABLE, sizeof(mode), &mode,
             "write scan enable");
}

void BluetoothHandler::spoof_bd_addr(const bdaddr_t &new_addr) {
    // Vendor-specific HCI command to change BD_ADDR
    // This works on most USB BT adapters (CSR/Cambridge Silicon Radio)
    // OGF=0x3F (vendor), OCF=0x01 (CSR write BD_ADDR)
    uint8_t cmd_data[6];
    memcpy(cmd_data, new_addr.b, 6);

    // Try CSR vendor command first
    // Quiet on failure: the hint below is more use than a strerror line.
    suppress_cmd_errors_ = true;
    const bool sent = send_cmd(0x3f, 0x01, 6, cmd_data, "BD_ADDR spoof");
    suppress_cmd_errors_ = false;
    if (sent) {
        log("BD_ADDR spoofed to {}", new_addr);
    } else {
        logerr("BD_ADDR spoof failed (this is OK if using btmgmt)");
        std::println(stderr, "  Run manually: sudo btmgmt --index {} public-addr {}",
                     hci_device_id_, new_addr);
    }
}

void BluetoothHandler::apply_side_settings() {
    // Both bits are needed, and the second one is counter-intuitive.
    //
    // Role switch, because the console takes the master role once it has
    // authenticated and drops a controller that refuses.
    //
    // Sniff, because the console puts the registration link into sniff
    // immediately before it calls from its play radio - a controller that
    // refuses to be sniffed never gets that call. It was taken out once on the
    // theory that sniff is what the input lag was made of; it is not, the play
    // link is never sniffed, and removing it blocked the handover for days.
    write_default_link_policy(LINK_POLICY_ROLE_SWITCH | LINK_POLICY_SNIFF);

    if (conn_side_ == EConnectionSide::to_dualsense) {
        // The pad calls its stored host when PS is pressed, so this side has
        // to be listening. Applying the console-facing settings here - which
        // is what the reset recovery used to do regardless of side - turned
        // page scan off and left the pad blinking with nowhere to go.
        write_scan_enable(SCAN_PAGE_ONLY);
        return;
    }
    apply_console_side_identity();
}

void BluetoothHandler::apply_console_side_identity() {
    // Claim the pad's address before anything else touches the air. Broadcom
    // takes it through vendor command 0xfc01, which is what spoof_bd_addr()
    // sends; the read-back proves whether it stuck.
    if (addr_is_set(spoof_addr_)) {
        spoof_bd_addr(spoof_addr_);
        read_bd_addr();
    }
    write_class_of_device(CLASS_OF_DEVICE_GAMEPAD);
    write_extended_inquiry_result(std::vector<uint8_t>(extended_inquiry0));

    // Page scan only, never inquiry. Inquiry is discovery, and nothing here
    // discovers us: the console learns the controller's address from feature
    // reports 0x09 and 0x0b over USB and pages it directly. Across six captures
    // there is not one inquiry event - the matches a naive grep finds are the
    // event-mask listing at the top of btmon's output, not traffic.
    //
    // Connectable from the start, and this has now been established twice.
    //
    // The temptation is to hold it back: this adapter answers to the pad's own
    // address, which is exactly what the pad-facing adapter is paging for while
    // it looks for the controller, so our own page can land on our own second
    // radio - `refusing connection from <pad adapter>` in the log, and once it
    // left this radio wedged and answering nothing at all.
    //
    // Holding it back until the pair command fails, every time. The console
    // never sends a single Connect Request afterwards: it opens the
    // registration link, asks for report 0x20, puts the link into sniff, comes
    // back out and terminates it, and repeats that cycle forever. The likely
    // reason is that it probes the controller's address over the radio while it
    // still has it on USB - before the pair command - and a controller that was
    // unreachable then is not called back later.
    //
    // So the collision is tolerated in general, and closed precisely where it
    // bites - see send_connection_request(), which deafens this adapter for the
    // length of a page to the pad.
    //
    // Which is why this asks first. Both sides come up at once, and the pad
    // side pages immediately: without the check, that page's suppression is
    // undone here a moment later and the collision happens anyway, on the very
    // first attempt of the run.
    if (!scan_suppressed_)
        write_scan_enable(SCAN_PAGE_ONLY);
}

void BluetoothHandler::disconnect_link() {
    if (!bt_socket_.is_open())
        return;

    // Every one of them, not just the link we relay on. The console holds its
    // registration radio open behind the play radio, and a link we walk away
    // from stays valid to the peer for its full supervision timeout - during
    // which the next run's pages from this same address are ignored, which is
    // what a screen of PAGE TIMEOUT after a restart actually means.
    for (uint16_t handle : open_handles_) {
        disconnect_cp cp = {.handle = handle, .reason = DISCONNECT_REMOTE_USER};
        send_cmd(OGF_LINK_CTL, OCF_DISCONNECT, DISCONNECT_CP_SIZE, &cp,
                 "disconnect");
        log("closing link handle {}", handle);
    }
}

void BluetoothHandler::read_bd_addr() {
    send_cmd(OGF_INFO_PARAM, OCF_READ_BD_ADDR, 0, nullptr, "read bd_addr");
}

void BluetoothHandler::read_buffer_size() {
    send_cmd(OGF_INFO_PARAM, OCF_READ_BUFFER_SIZE, 0, nullptr,
             "read buffer size");
}

void BluetoothHandler::send_connection_request(const bdaddr_t &addr) {
    if (!addr_is_set(addr)) {
        logerr("no peer address yet, not paging");
        return;
    }

    log("creating connection to {}", addr);

    // The console-facing adapter answers to the pad's own address, so a page
    // for the pad can be answered by our own second radio - the log shows it as
    // `refusing connection from <this adapter>` on one side and
    // `REJECTED - UNACCEPTABLE BD_ADDR` on both, and after that the pad cannot
    // be reached at all.
    //
    // Turning that adapter's page scan off for good is the obvious fix and it
    // is wrong: the console never calls back afterwards, tested twice. So it is
    // silenced only for the length of this page - a page timeout is 1.28 s, and
    // the console's own call-back comes minutes later, long after this window
    // has closed. It is put back in handle_connection_complete(), on success
    // and on failure alike.
    if (conn_side_ == EConnectionSide::to_dualsense && other_
        && addr_is_set(other_->spoof_addr_)
        && addr == other_->spoof_addr_) {
        other_->write_scan_enable(SCAN_NONE);
        other_->scan_suppressed_ = true;
    }

    create_conn_cp conn_request = {
        .bdaddr = addr,
        .pkt_type = ACL_PTYPE_MASK,
        .pscan_rep_mode = 0x01,
        .pscan_mode = 0,
        .clock_offset = 0x00,
        // Allow_Role_Switch governs the switch during connection setup only;
        // the post-connection one is what the console asks for, and that is
        // permitted through the link policy instead. Setting this to 1 is
        // exactly when the console's answers turned into "rejected - limited
        // resources" on every attempt, so it stays off.
        .role_switch = 0x00
    };
    send_cmd(OGF_LINK_CTL, OCF_CREATE_CONN, sizeof(conn_request), &conn_request,
             "create connection");
}

void BluetoothHandler::request_authentication(uint16_t handle) {
    auth_requested_cp request = {.handle = handle};
    send_cmd(OGF_LINK_CTL, OCF_AUTH_REQUESTED, AUTH_REQUESTED_CP_SIZE, &request,
             "authentication requested");
}


void BluetoothHandler::request_encryption(uint16_t handle) {
    set_conn_encrypt_cp request = {.handle = handle, .encrypt = 0x01};
    if (send_cmd(OGF_LINK_CTL, OCF_SET_CONN_ENCRYPT, SET_CONN_ENCRYPT_CP_SIZE,
                 &request, "set connection encrypt"))
        log("requesting link encryption");
}
