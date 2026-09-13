#pragma once

#include <cstdint>
#include <cstring>
#include <print>
#include <format>
#include <optional>
#include <array>
#include <vector>
#include <functional>
#include <string>
#include <filesystem>
#include <chrono>
#include <map>
#include <set>
#include <span>
#include "dualsense/DualSense.h"
#include <boost/asio.hpp>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/l2cap.h>

#include "bluetooth/PacketContents.h"
#include "usb/IPadBackend.h"
#include "usb/USBHandler.h"
#include "bluetooth/AclFlowControl.h"
#include "bluetooth/HeldFrame.h"
#include "bluetooth/SendGaps.h"
#include "bluetooth/HciCommandQueue.h"
#include "input/InputFeed.h"
#include "common/BdAddr.h"

class BluetoothHandler : public IPadBackend {
public:
    enum class EConnectionSide {
        to_dualsense,
        to_playstation
    };

    BluetoothHandler(boost::asio::io_context &ios, uint32_t hci_device_id,
                     EConnectionSide conn_side,
                     std::optional<std::array<uint8_t, 16>> link_key = std::nullopt);

    boost::asio::awaitable<void> handle();

    // Closes the link before we exit. Leaving it open means the peer holds a
    // half-dead connection to this address and ignores the next run's pages
    // until its own supervision timeout expires.
    void disconnect_link();

    void set_other(BluetoothHandler *other) { other_ = other; }
    void set_on_link_key_saved(std::function<void(const std::array<uint8_t, 16>&)> cb) {
        on_link_key_saved_ = std::move(cb);
    }
    // Preload a console pairing captured on an earlier run. The console only
    // hands out a link key the first time it meets a controller, so without
    // this every later run authenticates with zeros.
    void set_console_pairing(const bdaddr_t &addr, const std::array<uint8_t, 16> &key) {
        console_addr_ = addr;
        console_link_key_ = key;
        console_pairing_seen_ = true;
    }
    void set_on_console_pairing(
        std::function<void(const bdaddr_t &, const std::array<uint8_t, 16> &)> cb) {
        on_console_pairing_ = std::move(cb);
    }

    // Called when the pad refuses the key we had saved, so the caller can drop
    // it from disk and let the next run start from a clean pairing.
    void set_on_link_key_rejected(std::function<void()> cb) {
        on_link_key_rejected_ = std::move(cb);
    }

    // One-line state of the console-facing link, for the periodic report. The
    // USB counters go to zero once the gadget is detached, which makes a run
    // that is progressing look like one that has died.
    std::string relay_status() const;

    // False when the HCI user channel could not be claimed - nothing this
    // object does afterwards can work.
    bool ok() const { return bt_socket_.is_open(); }

    // Set directly by main.cpp: the pad's address, which this side pages.
    bdaddr_t bt_addr_{};
    // Set directly by main.cpp too.
    // Address of the adapter that will face the PS5. The console learns the
    // controller's Bluetooth address only from feature reports 0x09 and 0x0b,
    // so handing it this one makes it expect a connection from an adapter we
    // actually own - no BD_ADDR spoofing, which Realtek parts do not support.
    bdaddr_t ps5_side_addr_{};

    // Set when the console's pair command (0x08) arrives, which is the whole
    // point of the PS press; the relay stops pressing once it has.
    bool pair_command_seen_ = false;
    int  auto_ps_attempt_ = 0;

    // Everything past here is the machinery, not the interface. It was all
    // public, which made the sixteen entry points above impossible to pick out
    // of a hundred and thirteen declarations. The relay reaches its peer
    // through `other_`, and a member function may touch another instance's
    // privates, so nothing had to be public for that.
private:
    // Every line this class logs names the adapter it came from, and that
    // prefix was written out by hand at 74 sites in three different idioms -
    // printf("hci%d "), std::cout << "hci" << id, std::cerr << the same.
    // One helper each way instead. std::format checks the placeholders against
    // the arguments at compile time, so a wrong one is a build error rather
    // than a mangled log line, which printf could not offer.
    template <class... A>
    void log(std::format_string<A...> fmt, A &&...a) const {
        std::println("hci{} {}", hci_device_id_,
                     std::format(fmt, std::forward<A>(a)...));
    }
    template <class... A>
    void logerr(std::format_string<A...> fmt, A &&...a) const {
        std::println(stderr, "hci{} {}", hci_device_id_,
                     std::format(fmt, std::forward<A>(a)...));
    }


    // L2CAP channel info
    struct HciChannel {
        uint16_t psm;
        uint16_t scid;
        uint16_t dcid = 0;
        uint16_t mtu = 185;
    };

    // Relay counters, printed by relay_status()/report_stats(). Grouped so the
    // state block below reads as state, not as a scoreboard.
    struct RelayStats {
        uint64_t console_input_count = 0;
        // Input reports dropped because the controller had no free buffer.
        uint64_t console_input_backpressure = 0;
        // Reports held and then sent once a buffer came free.
        uint64_t console_input_flushed = 0;
        // Opaque-stream frames recognised and not relayed.
        uint64_t console_input_opaque_dropped = 0;
        // The other direction: rumble/lightbar/trigger/audio from the console.
        uint64_t console_output_count = 0;
        uint64_t console_output_dropped = 0;
    };

    // Two SDP channels with distinct source CIDs, because both roles occur on
    // the same link: the peer opens hid_sdp_channel to browse us, and we open
    // sdp_client_channel to browse the pad. Sharing one struct made the second
    // remote CID overwrite the first.
    HciChannel hid_sdp_channel       = {.psm = 0x01, .scid = 0x7008};
    // The controller's whole service record has to arrive in one answer: the
    // console stalls with the HID channel in "authorization pending" when it
    // gets a chunked one and never asks for the continuation. 1000 is as large
    // as this can go - past the ACL MTU the pad would fragment, and incoming
    // fragments are not reassembled here.
    HciChannel sdp_client_channel    = {.psm = 0x01, .scid = 0x7009, .dcid = 0, .mtu = 1000};
    HciChannel hid_control_channel   = {.psm = 0x11, .scid = 0x7108};
    HciChannel hid_interrupt_channel = {.psm = 0x13, .scid = 0x720f, .dcid = 0, .mtu = 672};

    // Declaration order is construction order, and it has to match the
    // initialiser list or the compiler is right to complain.
    boost::asio::io_context &ios;
    boost::asio::posix::stream_descriptor bt_socket_;
    USBHandler gadget;

    EConnectionSide conn_side_;

    static int open_socket(uint16_t hci_device_id);

public:
    // Reads an adapter's address without claiming it, for the PS5-facing side.
    bool set_ps5_side_adapter(uint16_t hci_index);
private:

    // PS5 side: page the console and authenticate with the link key it wrote
    // over USB. Called once the console has asked the controller to pair.
    void connect_to_console(const bdaddr_t &addr, const std::array<uint8_t, 16> &key);

    // Every HCI command this class sends goes through here. Two reasons: the
    // socket handle stops being repeated at thirty-odd call sites, and a
    // command the kernel refuses gets a line instead of vanishing - only five
    // of those sites used to check the return, and a controller that starts
    // refusing commands usually goes on refusing the ones after it. `what`
    // names the command in that line. Returns true when the command was handed
    // to the kernel; quiet while suppress_cmd_errors_ is set, which is the
    // stale-link sweep deliberately firing at handles that may not exist.
    bool send_cmd(uint16_t ogf, uint16_t ocf, uint8_t plen, const void *param,
                  const char *what);

    // HCI command helpers
    void write_simple_pairing_enabled();
    void write_auth_enabled();
    void write_event_mask();
    void write_class_of_device(uint32_t dev_class);
    void write_extended_inquiry_result(std::vector<uint8_t> &&data);
    void write_scan_enable(uint8_t mode);
    void write_default_link_policy(uint16_t policy);
    void write_page_timeout(uint16_t slots);
    void write_page_scan_activity(uint16_t interval, uint16_t window);
    void read_radio_quality();
    boost::asio::awaitable<bool> reset_controller();
    void pump_commands();
    boost::asio::awaitable<void> watch_commands();
    boost::asio::awaitable<void> publish_stage();
    boost::asio::awaitable<void> poll_radio_quality();
    void spoof_bd_addr(const bdaddr_t &new_addr);
    void read_bd_addr();
    // Asks the controller how many ACL packets it can hold. Everything we put
    // on the air is metered against that answer.
    void read_buffer_size();
    // Once the console is playing through us, the radio has nothing left to do
    // but serve that one link: no page scan, no leftover registration link.
    void quiet_the_radio();
    // The controller hands buffers back with this event; without it we have no
    // idea how much of what we wrote is still queued.
    void handle_num_completed_packets(PacketContents &packet_contents, uint8_t plen);

    // Everything that makes this adapter look like the controller. Re-applied
    // after any reset, which wipes the spoofed address along with the rest.
    void apply_console_side_identity();

    // Scan modes and identity for whichever side this handler is, applied at
    // start and again after any reset - which clears all of it.
    void apply_side_settings();

public:
    // Address this adapter should claim before it does anything else. Set on
    // the console-facing side to the pad's own address, so the console meets a
    // controller it already knows instead of an unknown dongle.
    void set_spoof_addr(const bdaddr_t &addr) { spoof_addr_ = addr; }
private:

    void send_connection_request(const bdaddr_t &addr);
    void request_authentication(uint16_t handle);
    void request_encryption(uint16_t handle);

    // Event handlers
    void handle_bluetooth_data(uint8_t *packet, size_t packet_size);
    void handle_event(PacketContents &packet_contents);
    void handle_acl(PacketContents &packet_contents);

    // Routes one complete L2CAP PDU (its own 4-byte header first) to the
    // channel handlers below. Split out of handle_acl so a PDU reassembled
    // across several ACL packets reaches the same routing as one that arrived
    // whole.
    void route_l2cap_pdu(uint8_t *pdu, size_t total_len);

    // One per L2CAP channel and direction, called from handle_acl once it has
    // decided where a payload belongs. Each takes the L2CAP payload, which
    // starts at the HIDP transaction header.
    void handle_feature_answer_from_pad(std::span<uint8_t> report);
    void handle_feature_request_from_console(std::span<uint8_t> report);
    void relay_console_output_to_pad(std::span<uint8_t> report);
    void relay_pad_input(std::span<uint8_t> report);
    // Prints the controller's charge when it changes - see the note at the
    // definition for why that is worth a line.
    static void log_pad_battery(std::span<const uint8_t> report);
    void relay_pad_input_to_usb(std::span<uint8_t> report);
    void relay_pad_input_to_console(std::span<uint8_t> report);
    void send_full_report_to_console(std::span<uint8_t> report);
    void send_cut_down_report_to_console(std::span<uint8_t> report);
    // Sends whatever was held back, if a buffer has since come free.
    void flush_held_report();
    // True when a report is a frame of the pad's opaque stream rather than
    // controller state. Two signals, because neither alone is enough: the
    // 0xd4 marker can occur in a real report as the right stick's Y axis,
    // and a clock gap can be a genuine link stall. Requiring both means a
    // state report can never be dropped by this test.
    bool opaque_frame(std::span<const uint8_t> report);
    void handle_sdp_answer_from_pad(std::span<uint8_t> payload);
    void handle_sdp_query(std::span<uint8_t> payload);

    void handle_connection_complete(PacketContents &packet_contents);
    void handle_connection_failure(const evt_conn_complete &data);
    void handle_link_key_request(PacketContents &packet_contents);
    void handle_io_capability_request(PacketContents &packet_contents);
    void handle_user_confirm_request(PacketContents &packet_contents);
    void handle_simple_pairing_complete(PacketContents &packet_contents);
    void handle_auth_complete(PacketContents &packet_contents);
    void handle_encrypt_change(PacketContents &packet_contents);
    void handle_role_change(PacketContents &packet_contents);
    // Sniff or park on the link we relay on is felt as lag, so it is tracked
    // and logged rather than left to be inferred from a capture.
    void handle_mode_change(PacketContents &packet_contents);
    void handle_cmd_complete(PacketContents &packet_contents);
    void handle_cmd_status(PacketContents &packet_contents);
    void handle_link_key_notify(PacketContents &packet_contents);
    void handle_conn_request(PacketContents &packet_contents);
    void handle_disconnect_complete(PacketContents &packet_contents);

    // Forget the L2CAP channels and everything learned about the peer over
    // them. Both callers reach it the same way - the link went away, or the
    // peer moved to a different one - and channel state from the old link
    // addresses a combination that no longer exists.
    void reset_channels();

    // Re-page the peer after the link went away, at most one timer at a time.
    void schedule_reconnect(std::chrono::milliseconds delay);

    // Ends the fast repaging that follows the handover, putting the normal
    // page timeout back. Called on connection or once the window runs out.
    void end_handover_paging();

    // L2CAP handlers
    void handle_l2cap_cmd(PacketContents &packet_contents);
    void handle_conn_response(PacketContents &packet_contents);
    void handle_conf_response(PacketContents &packet_contents);
    void handle_conf_request(PacketContents &packet_contents);

    // L2CAP sending
    // Always on the link this handler holds; the handle used to be a
    // parameter that both callers filled with connection_handle_.
    void l2cap_send_cmd(uint8_t ident, uint8_t code, const void *data, uint16_t len);
    void l2cap_send_cmd_request(uint8_t code, void *data, uint16_t len);
    void l2cap_send_cmd_response(uint8_t ident, uint8_t code, void *data, uint16_t len);
    void l2cap_send_conn_request(uint16_t psm, uint16_t scid);
    void l2cap_send_configure_mtu_request(uint16_t dcid, uint16_t mtu_size);

    // HID over L2CAP
    void l2cap_send_data(const HciChannel &channel, const uint8_t *data, size_t len);
    void l2cap_send_framed(const HciChannel &channel,
                           const uint8_t *prefix, size_t prefix_len,
                           const uint8_t *data, size_t len);
    // The one place an ACL frame is built. Signalling reaches it with the
    // fixed CID and its command header as the prefix; everything else with a
    // channel's remote CID and, for HID, the HIDP header as the prefix.
    void l2cap_send_frame(uint16_t cid,
                          const uint8_t *prefix, size_t prefix_len,
                          const uint8_t *data, size_t len);
    void l2cap_send_hid(const HciChannel &channel, uint8_t hidp_header,
                        const uint8_t *data, size_t len);
    void handle_sdp_request(const uint8_t *request, size_t len);
public:
    // The three the USB gadget calls back into when the console asks the
    // controller something over the cable.
    void l2cap_get_report(uint8_t report_id);
    void l2cap_send_feature_set(const uint8_t *report, size_t size);
    void l2cap_send_output_report(const uint8_t *usb_report, size_t size);
private:
    void l2cap_send_hid_feature_set(const uint8_t *report, size_t size);

    // A delay inside a coroutine was a named steady_timer and an await on it,
    // two lines at each of nine sites that wanted nothing but to wait.
    boost::asio::awaitable<void> sleep_for(std::chrono::milliseconds d);

    // Brings the USB gadget up once the HID channels are open, without
    // blocking the event loop.
    boost::asio::awaitable<void> start_usb_gadget();

    HciChannel* get_channel_by_scid(uint16_t scid);
    HciChannel* get_channel_by_psm(uint16_t psm);
    static std::string get_error_message(uint8_t error_code);

    // State
    BluetoothHandler *other_ = nullptr;
    uint16_t connection_handle_ = 0;
    uint8_t cmd_request_ident = 1;
    // Identifier of the L2CAP request currently being handled; responses must
    // echo it back.
    uint8_t current_cmd_ident_ = 0;
    bool usb_started_ = false;
    bool reconnect_pending_ = false;
    // True when the pad called us rather than the other way round; it then
    // owns the L2CAP channel setup.
    bool incoming_link_ = false;
    bool accepted_incoming_ = false;
    bool console_output_seen_ = false;
    bool handover_seen_ = false;
    bool console_input_seen_ = false;
    int page_failures_ = 0;
    // When this side started calling the console after the handover; empty
    // outside that window. See HANDOVER_PAGE_WINDOW.
    std::chrono::steady_clock::time_point handover_paging_since_{};
    bool stale_link_reset_ = false;
    bool suppress_cmd_errors_ = false;
    RelayStats stats_;
    bool pad_output_seen_ = false;
    // The freshest input report we could not send because the controller had no
    // buffer free. Kept rather than thrown away so that the moment a buffer
    // comes back we have something current to put in it - see
    // flush_held_report(). Empty size means nothing held.
    HeldFrame held_;

    // Both live on the console-facing handler, with the counters they belong
    // next to. The gaps are the one thing the panel could not show before: a
    // hole is invisible in totals, see SendGaps.h.
    SendGaps gaps_;

    // Everything send_cmd() writes goes through here, so the controller is
    // never handed more commands than it has said it has room for.
    HciCommandQueue cmd_queue_;

    // What the controller says about the radio itself, which nothing here ever
    // asked before. Link quality is 0-255, higher is better. RSSI on BR/EDR is
    // not dBm: it is how far the receive power sits outside the golden range,
    // so zero is the good answer and either sign away from it is worse.
    uint8_t link_quality_ = 0;
    int8_t link_rssi_ = 0;
    uint8_t afh_channels_ = 0;   // of 79 the link is allowed to hop on
    bool afh_enabled_ = false;
    bool radio_read_ = false;

    // What the panel needs that the pad's own bytes cannot say. Read off the
    // console-facing handler, because that is where the link being described
    // actually is.
    // Latest first, so each test only has to rule out what comes after it.
    // Deliberately not console_pairing_seen_: that is set at start from the
    // saved pairing, so it says nothing about this run. The USB side's own
    // state does.
    inputfeed::Stage stage() const {
        if (other_ && other_->hid_interrupt_channel.dcid != 0)
            return inputfeed::STAGE_RELAYING;
        if (other_ && other_->connection_handle_ != 0)
            return inputfeed::STAGE_CONSOLE_CONNECT;
        if (pair_command_seen_)
            return inputfeed::STAGE_CALLING;
        if (connection_handle_ == 0)
            return inputfeed::STAGE_PAD_SEARCH;
        if (hid_interrupt_channel.dcid == 0)
            return inputfeed::STAGE_PAD_CONNECT;
        if (!gadget.endpoints_up())
            return inputfeed::STAGE_USB_WAIT;
        if (gadget.input_flowing_since() == std::chrono::steady_clock::time_point{})
            return inputfeed::STAGE_USB_READ;
        return inputfeed::STAGE_USB_PRESS;
    }

    inputfeed::Link link_snapshot() const {
        inputfeed::Link l;
        if (!other_)
            return l;
        l.console_live = other_->hid_interrupt_channel.dcid != 0;
        l.stage = stage();
        if (l.stage == inputfeed::STAGE_CALLING)
            l.call_attempt = uint8_t(std::min(other_->page_failures_ + 1, 255));
        l.console_sent = other_->stats_.console_input_count;
        l.output_sent = other_->stats_.console_output_count;
        l.output_dropped = other_->stats_.console_output_dropped;
        l.gap_window = other_->gaps_.window();
        l.gap_count = other_->gaps_.count();
        l.gap_max_us = other_->gaps_.max_us();
        l.link_quality = other_->link_quality_;
        l.link_rssi = other_->link_rssi_;
        l.radio_read = other_->radio_read_ ? 1 : 0;
        l.afh_channels = other_->afh_channels_;
        return l;
    }

    // One report accepted by the console link, from either path - straight
    // through or out of the held slot. Kept together so the gap between sends
    // cannot be measured from only half of them.
    void note_input_sent() {
        stats_.console_input_count++;
        gaps_.on_sent(SendGaps::Clock::now());
    }
    // The pad's input, published for build/ps5padlog-view. Only the pad-facing
    // side opens it; on the other it stays closed and publish() does nothing.
    inputfeed::Writer input_feed_;
    // ACL flow control: packets the controller still holds vs its pool.
    AclFlowControl acl_;
    // Every link this adapter currently holds. The console keeps its
    // registration radio connected after moving the controller to the play
    // one, so there are two, and only closing the one we relay on leaves the
    // other dangling - which the console honours until its own supervision
    // timeout, ignoring our pages from the same address in the meantime.
    std::set<uint16_t> open_handles_;
    // True once the console has moved us from the radio that registered the
    // controller to the one it plays through. Until then the aggressive page
    // scan is the whole point: that call-back is what we are waiting for.
    bool on_play_link_ = false;
    // Set while the pad-facing side is paging and this adapter has been made
    // deaf so it cannot answer that page itself.
    bool scan_suppressed_ = false;
    // Handles we have already asked the controller to drop because they belong
    // to a previous run - remembered so the log says it once, not per packet.
    std::set<uint16_t> ghost_handles_;
    // Reassembly of L2CAP PDUs that the controller delivers across several ACL
    // packets. An ACL start (PB=0b10) begins a PDU whose L2CAP length may exceed
    // what fits in one packet; the continuations (PB=0b01) carry the rest with
    // no header of their own. Without this the second fragment's payload bytes
    // were parsed as a fresh L2CAP header - garbage cid, absurd length - and the
    // whole PDU dropped, which is what stalled a fresh console enrolment at the
    // SDP browse. Keyed by ACL handle; a start clears any stale partial.
    struct AclReasm { std::vector<uint8_t> buf; size_t expected = 0; };
    std::map<uint16_t, AclReasm> acl_reasm_;
    // Set while a link is in sniff: the console only listens at the sniff
    // anchor points, so a full-rate stream would queue up behind them.
    bool link_sniffing_ = false;
    // Set once the console asks us for calibration or firmware info, which is
    // what makes a real pad start sending the full 0x31 report.
    bool console_full_reports_ = false;

    // The pad's answer to the console's service query, kept because the
    // console asks again on every link and the channel to the pad is not
    // always open at that moment - and an empty answer gets us rejected.
    std::vector<uint8_t> sdp_answer_cache_;

    bdaddr_t spoof_addr_{};
    bool bd_addr_confirmed_ = false;

    // Host address and link key the console wrote with SET_REPORT 0x0a. This
    // is what the PS5-side link will have to authenticate with.
    // The peer on the link we are currently relaying, taken from Connect
    // Complete. The play radio is a different address from the registration
    // one, and commands like Switch Role are addressed rather than handled.
    bdaddr_t link_peer_addr_{};

    bdaddr_t console_addr_{};
    std::array<uint8_t, 16> console_link_key_{};
    bool console_pairing_seen_ = false;

    // This adapter's own address: what the pad should be holding as its host.
    bdaddr_t local_addr_{};

    std::optional<std::array<uint8_t, 16>> link_key_;
    std::array<uint8_t, 16> negotiated_link_key_{};

    std::function<void(const std::array<uint8_t, 16>&)> on_link_key_saved_;
    std::function<void()> on_link_key_rejected_;
    std::function<void(const bdaddr_t &, const std::array<uint8_t, 16> &)> on_console_pairing_;

    uint8_t read_buffer_[4096]{};
    uint8_t write_buffer_[4096]{};

    // Sequence number for Bluetooth output reports (high nibble of seq_tag).
    uint8_t output_seq_ = 0;

    uint32_t hci_device_id_ = 0;
};
