//
// BluetoothHandler.cpp - dualsense-ps5-mitm
//
// Talks to a real DualSense over Bluetooth and relays it to the PS5 through the
// USB gadget: input reports outbound, GET_REPORT / SET_REPORT (including the
// console's authentication challenge) inbound.
//
// This file brings the adapter up, runs the read loop and makes the first split
// of what arrives. The rest of the class, by responsibility:
//   HciCommands.cpp  commands to the controller, and the adapter settings
//   HciEvents.cpp    events from it: connections, pairing, command results
//   L2cap.cpp        ACL reassembly, channel routing and setup, frame building
//   Sdp.cpp          the console's service query, answered by the pad
//   HidRelay.cpp     reports between the pad, the console and the USB gadget
//   Handover.cpp     from the USB cable to the console's radio
//

#include "bluetooth/BluetoothHandler.h"
#include "bluetooth/BluetoothHandlerInternal.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include <cerrno>
#include <cstring>
#include <iostream>
#include <string>

#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

// ============================================================================
// Constants
// ============================================================================

static constexpr auto use_nothrow_awaitable =
    boost::asio::as_tuple(boost::asio::use_awaitable);

// ============================================================================
// Constructor / Socket
// ============================================================================

BluetoothHandler::BluetoothHandler(boost::asio::io_context &ios, uint32_t hci_device_id,
                                   EConnectionSide conn_side,
                                   std::optional<std::array<uint8_t, 16>> link_key)
    : ios(ios),
      bt_socket_(ios),
      gadget(this, &ios),
      conn_side_(conn_side),
      link_key_(link_key),
      hci_device_id_(hci_device_id)
{
    acl_.set_hci_id(hci_device_id);
    // The pad link is ours to schedule - we page the pad and transmit when we
    // choose - and its controller hands credits back in pairs at half the rate
    // the console side does. Two is the console link's number; see set_depth().
    if (conn_side_ == EConnectionSide::to_dualsense)
        acl_.set_depth(3);
    // Read the adapter's own address before the user channel claims it - the
    // ioctl needs a device that is still visible to the kernel's own stack.
    // It is what the pad should have stored as its host, so having it here
    // turns "PIN OR KEY MISSING" into something the log can explain.
    int ctl = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
    if (ctl >= 0) {
        hci_dev_info di = {};
        di.dev_id = static_cast<uint16_t>(hci_device_id);
        if (ioctl(ctl, HCIGETDEVINFO, &di) == 0)
            local_addr_ = di.bdaddr;
        close(ctl);
    }

    int fd = open_socket(hci_device_id);
    if (fd < 0)
        return;
    bt_socket_.assign(fd);

    if (conn_side_ == EConnectionSide::to_dualsense) {
        if (!gadget.init_gadget())
            std::cerr << "USB gadget setup failed - the PS5 will not see a controller" << std::endl;

        // The pad's input for build/padlog. Nothing here depends on it, so
        // a failure is a line in the log and the relay carries on.
        if (input_feed_.open())
            log("input feed at {} - watch it with build/padlog", inputfeed::PATH);
        else
            logerr("input feed {} could not be created ({}) - padlog will have nothing to show",
                   inputfeed::PATH, strerror(errno));
    }
}

// HCI_CHANNEL_USER refuses to bind while the adapter is up - the kernel
// returns EBUSY. A previous run that was killed with Ctrl+C leaves it up, so
// take it down and try again rather than making the caller do it by hand.
static bool force_device_down(uint16_t hci_device_id) {
    int ctl = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
    if (ctl < 0)
        return false;
    const bool ok = ioctl(ctl, HCIDEVDOWN, static_cast<unsigned long>(hci_device_id)) == 0
                    || errno == EALREADY;
    close(ctl);
    return ok;
}

bool BluetoothHandler::set_ps5_side_adapter(uint16_t hci_index) {
    int ctl = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC, BTPROTO_HCI);
    if (ctl < 0)
        return false;

    hci_dev_info di = {};
    di.dev_id = hci_index;
    const bool ok = ioctl(ctl, HCIGETDEVINFO, &di) == 0;
    close(ctl);
    if (!ok) {
        std::cerr << "hci" << hci_index << " not readable, no PS5-side adapter" << std::endl;
        return false;
    }

    ps5_side_addr_ = di.bdaddr;
    std::println("PS5-side adapter: hci{} {} - the console will be told this is the controller",
                 hci_index, ps5_side_addr_);
    return true;
}

int BluetoothHandler::open_socket(uint16_t hci_device_id) {
    for (int attempt = 0; attempt < 10; ++attempt) {
        int hci_socket = socket(AF_BLUETOOTH, SOCK_RAW | SOCK_CLOEXEC | SOCK_NONBLOCK, BTPROTO_HCI);
        if (hci_socket < 0) {
            std::cerr << "Failed to create HCI socket: " << strerror(errno) << std::endl;
            return -1;
        }

        sockaddr_hci addr = {};
        addr.hci_family = AF_BLUETOOTH;
        addr.hci_dev = hci_device_id;
        addr.hci_channel = HCI_CHANNEL_USER;

        if (bind(hci_socket, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) == 0) {
            std::cout << "Successfully bound HCI_CHANNEL_USER to hci" << hci_device_id << std::endl;
            return hci_socket;
        }

        const int err = errno;
        close(hci_socket);

        if (err == EBUSY && attempt < 9) {
            if (attempt == 0)
                std::cout << "hci" << hci_device_id << " is up, taking it down" << std::endl;
            force_device_down(hci_device_id);
            usleep(300000);
            continue;
        }

        std::cerr << "FATAL: Failed to bind to hci" << hci_device_id
                  << ": " << strerror(err) << std::endl;
        std::cerr << "  -> Make sure bluetoothd is stopped: systemctl stop bluetooth" << std::endl;
        std::cerr << "  -> And rfkill is unblocked: rfkill unblock all" << std::endl;
        return -1;
    }
    return -1;
}

// ============================================================================
// Main coroutine
// ============================================================================

boost::asio::awaitable<void> BluetoothHandler::handle() {
    // First, before anything is configured: whatever the last run left in the
    // controller - above all a link the console is still using - goes. See
    // reset_controller() for what not doing this cost.
    boost::asio::co_spawn(ios, watch_commands(), boost::asio::detached);
    if (conn_side_ == EConnectionSide::to_dualsense)
        boost::asio::co_spawn(ios, publish_stage(), boost::asio::detached);
    co_await reset_controller();

    write_simple_pairing_enabled();
    write_event_mask();
    write_auth_enabled();

    // Pressing PS makes the pad page its stored host rather than wait to be
    // paged, so the adapter has to be listening. Without page scan the pad's
    // call goes nowhere and all we ever see is our own PAGE TIMEOUTs.
    write_page_scan_activity(PAGE_SCAN_INTERVAL_EAGER, PAGE_SCAN_WINDOW_EAGER);

    // Our own paging has to leave gaps in that listening: the controller
    // cannot answer a page while one of ours is outstanding. 1.28 s instead of
    // the default 5.12 s keeps the adapter reachable most of the time.
    write_page_timeout(PAGE_TIMEOUT_SLOTS);

    read_buffer_size();
    apply_side_settings();

    if (conn_side_ == EConnectionSide::to_dualsense) {
        send_connection_request(bt_addr_);
    } else {
        // Faces the console as the controller itself: a DualSense class of
        // device and its name in the inquiry response. Nothing is paged until
        // the console has handed us its address and link key over USB.
        log("standing by as the controller, waiting for the console's pairing data");

        // This adapter has been seen accepting commands and then answering
        // nothing at all - no events, not even command completions - which is
        // invisible until someone parses the capture. A read of its own
        // address is the cheapest liveness test there is.
        boost::asio::co_spawn(ios, [this]() -> boost::asio::awaitable<void> {
            co_await sleep_for(std::chrono::milliseconds(1500));
            if (!bd_addr_confirmed_) {
                std::cerr << "\n*** hci" << hci_device_id_
                          << " is not answering - it accepted our commands and sent"
                             " no events back.\n*** Unplug the console-facing dongle,"
                             " plug it back in and start again.\n" << std::endl;
            }
        }, boost::asio::detached);
    }

    boost::asio::co_spawn(ios, poll_radio_quality(), boost::asio::detached);

    // Main read loop.
    //
    // Every ACL packet the pad sends used to cost a full trip back through the
    // scheduler - an epoll wakeup, a coroutine resume and the executor copying
    // that goes with it - and at five hundred reports a second that was the
    // largest single cost in the profile, well above anything we actually do
    // with the bytes. So after each wakeup the socket is drained with plain
    // non-blocking reads.
    //
    // ::recv with MSG_DONTWAIT rather than the socket's own sync read, because
    // that one can decide to block, and a blocked read loop here is what
    // produces LMP timeouts and dropped links a moment later.
    //
    // The drain is bounded so a burst cannot starve the timers that drive
    // reconnects and the USB handover.
    constexpr int max_drain = 32;
    while (true) {
        auto [e1, bytes_read] = co_await bt_socket_.async_read_some(
            boost::asio::buffer(read_buffer_), use_nothrow_awaitable);
        if (e1) {
            logerr("read error: {}", e1.message());
            co_return;
        }
        handle_bluetooth_data(&read_buffer_[0], bytes_read);

        for (int drained = 0; drained < max_drain; drained++) {
            const ssize_t n = ::recv(bt_socket_.native_handle(), read_buffer_,
                                     sizeof(read_buffer_), MSG_DONTWAIT);
            if (n <= 0)
                break;   // EAGAIN: nothing more queued
            handle_bluetooth_data(&read_buffer_[0], static_cast<size_t>(n));
        }
    }
}

// ============================================================================
// Packet dispatch
// ============================================================================

void BluetoothHandler::handle_bluetooth_data(uint8_t *packet, size_t packet_size) {
    PacketContents packet_contents(packet, packet_size);
    const auto *packet_indicator = packet_contents.get<uint8_t>();
    if (!packet_indicator)
        return;

    switch (*packet_indicator) {
        case HCI_EVENT_PKT:
            handle_event(packet_contents);
            break;
        case HCI_ACLDATA_PKT:
            handle_acl(packet_contents);
            break;
        default:
            break;
    }
}

// ============================================================================
// Utility
// ============================================================================

boost::asio::awaitable<void> BluetoothHandler::sleep_for(std::chrono::milliseconds d) {
    boost::asio::steady_timer t(ios, d);
    co_await t.async_wait(boost::asio::use_awaitable);
}

std::string BluetoothHandler::relay_status() const {
    const BluetoothHandler *console = (conn_side_ == EConnectionSide::to_playstation)
                                          ? this : other_;
    if (!console)
        return "console side: none";

    std::string s = "console: ";
    if (console->connection_handle_ == 0)
        s += "not connected";
    else if (console->hid_interrupt_channel.dcid == 0)
        s += "linked, waiting for HID channels";
    else
        s += "relaying";

    s += ", reports sent " + std::to_string(console->stats_.console_input_count)
       + " held " + std::to_string(console->stats_.console_input_backpressure)
       + " flushed " + std::to_string(console->stats_.console_input_flushed)
       + " opaque " + std::to_string(console->stats_.console_input_opaque_dropped)
       + " | out " + std::to_string(console->stats_.console_output_count)
       + " held " + std::to_string(console->stats_.console_output_dropped)
       + " | acl " + std::to_string(console->acl_.in_flight_total()) + "/"
       + std::to_string(console->acl_.budget())
       + (console->link_sniffing_ ? " sniff" : "")
       + (console->radio_read_
              ? " | radio q" + std::to_string(console->link_quality_)
                    + " rssi " + std::to_string(console->link_rssi_)
                    + " afh " + std::to_string(console->afh_channels_) + "/79"
              : "");
    return s;
}
