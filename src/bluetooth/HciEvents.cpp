//
// HciEvents.cpp - dualsense-ps5-mitm
//
// Events from the controller: connections made, refused and lost, pairing and
// authentication, command status and completion, and the buffer counts the ACL
// flow control runs on.
//

#include "bluetooth/BluetoothHandler.h"
#include "bluetooth/BluetoothHandlerInternal.h"

#include <bit>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>

#include <cstdio>
#include <cstring>
#include <format>
#include <iostream>
#include <string>

namespace {

// General Bonding, no MITM protection - what the pad itself answers. Dedicated
// Bonding (0x02) makes it drop the key when the link goes, and the next run
// gets PIN OR KEY MISSING.
constexpr uint8_t AUTH_GENERAL_BONDING = 0x04;

// Every HCI event and command completion carries a status, and zero is the only
// one that means it worked.
constexpr uint8_t HCI_SUCCESS = 0x00;

// Link modes, as HCI_Mode_Change reports them.
constexpr uint8_t LINK_MODE_ACTIVE = 0x00;
constexpr uint8_t LINK_MODE_HOLD   = 0x01;
constexpr uint8_t LINK_MODE_SNIFF  = 0x02;

// HCI status codes come from BlueZ's own hci.h - HCI_PAGE_TIMEOUT,
// HCI_AUTHENTICATION_FAILURE, HCI_PIN_OR_KEY_MISSING, HCI_ACL_CONNECTION_EXISTS
// and HCI_OE_USER_ENDED_CONNECTION are all defined there. Redefining them here
// would only be a second opinion about what 0x04 means.

}  // namespace

void BluetoothHandler::handle_event(PacketContents &packet_contents) {
    const auto *event_header = packet_contents.get<hci_event_hdr>();
    if (!event_header)
        return;
    switch (event_header->evt) {
        case EVT_CONN_COMPLETE:
            handle_connection_complete(packet_contents);
            break;
        case EVT_LINK_KEY_REQ:
            handle_link_key_request(packet_contents);
            break;
        case EVT_IO_CAPABILITY_REQUEST:
            handle_io_capability_request(packet_contents);
            break;
        case EVT_USER_CONFIRM_REQUEST:
            handle_user_confirm_request(packet_contents);
            break;
        case EVT_SIMPLE_PAIRING_COMPLETE:
            handle_simple_pairing_complete(packet_contents);
            break;
        case EVT_AUTH_COMPLETE:
            handle_auth_complete(packet_contents);
            break;
        case EVT_ENCRYPT_CHANGE:
            handle_encrypt_change(packet_contents);
            break;
        case EVT_ROLE_CHANGE:
            handle_role_change(packet_contents);
            break;
        case EVT_NUM_COMP_PKTS:
            handle_num_completed_packets(packet_contents, event_header->plen);
            break;
        case EVT_MODE_CHANGE:
            handle_mode_change(packet_contents);
            break;
        case EVT_CMD_COMPLETE:
            handle_cmd_complete(packet_contents);
            break;
        case EVT_CMD_STATUS:
            handle_cmd_status(packet_contents);
            break;
        case EVT_LINK_KEY_NOTIFY:
            handle_link_key_notify(packet_contents);
            break;
        case EVT_CONN_REQUEST:
            handle_conn_request(packet_contents);
            break;
        case EVT_DISCONN_COMPLETE:
            handle_disconnect_complete(packet_contents);
            break;
        default:
            break;
    }
}

void BluetoothHandler::handle_connection_complete(PacketContents &packet_contents) {
    const auto *data = packet_contents.get<evt_conn_complete>();
    if (!data)
        return;   // short packet

    // A failure that follows a Connect Request we accepted closes that incoming
    // attempt, not our page, which may still be out. At startup the pad's calls
    // from before the run sit queued in the controller and each fails with
    // CONNECTION ACCEPT TIMEOUT; each used to switch the console-facing
    // adapter's page scan back on under our page for the pad's address, that
    // adapter answers to the pad's address, and the run died with UNACCEPTABLE
    // BD_ADDR (run-20260911-133245). Consumed here too, or the stale flag marks
    // the next link we dial as one the peer opened, and nobody authenticates it.
    const bool failed_incoming = data->status != HCI_SUCCESS && accepted_incoming_;
    if (failed_incoming)
        accepted_incoming_ = false;

    // Our page is over, one way or the other: the console-facing adapter can
    // listen again. See send_connection_request().
    if (!failed_incoming && conn_side_ == EConnectionSide::to_dualsense && other_
        && other_->scan_suppressed_) {
        other_->scan_suppressed_ = false;
        other_->write_scan_enable(SCAN_PAGE_ONLY);
    }

    if (data->status != HCI_SUCCESS) {
        handle_connection_failure(*data);
        return;
    }


    // The console moves the controller from the radio that registered it to
    // the one it plays through, and it opens the second link before dropping
    // the first. Taking the new handle while keeping channel state from the
    // old link sends everything into a combination that no longer exists: the
    // link dies, the console re-takes the controller, and the screen flickers
    // through the cycle.
    if (connection_handle_ != 0 && connection_handle_ != data->handle) {
        log("peer moved to another link, starting its channels over");

        // Both links stay up. The capture of the one run that reached a live
        // controller has the registration link on handle 0x0b and the play
        // radio on 0x0c at the same time, and it is on the second that the
        // console opens HID channels, reads calibration and firmware, and
        // starts streaming output reports. Closing the first was added later
        // on the theory that two links caused the LMP timeouts; it broke the
        // only sequence that ever worked.
        reset_channels();
        // The sniff the console put on the link it is leaving says nothing
        // about the one it is moving to.
        link_sniffing_ = false;
        on_play_link_ = true;
    }

    connection_handle_ = data->handle;
    link_peer_addr_ = data->bdaddr;
    open_handles_.insert(data->handle);
    page_failures_ = 0;
    end_handover_paging();

    // The pad-facing radio listens for a page with a 180 ms window every 640 ms,
    // because when the controller is asleep that call is the only way it comes
    // back and missing it costs a press. While the controller is connected it
    // will not page at all, and that window is 28% of the radio spent deaf to
    // the reports it is supposed to be receiving - the same tax that cost half
    // the transmit airtime on the console side until quiet_the_radio() eased it.
    //
    // The controller's own report stream has gaps over 20 ms across 13% of the
    // time, which is what our gaps to the console are now made of. This is the
    // most likely reason for them.
    if (conn_side_ == EConnectionSide::to_dualsense) {
        write_page_scan_activity(PAGE_SCAN_INTERVAL_NORMAL, PAGE_SCAN_WINDOW_NORMAL);
        log("pad is here - page scan back to its normal duty cycle");
    }
    incoming_link_ = accepted_incoming_;
    accepted_incoming_ = false;
    log("connected to {} (handle={})", data->bdaddr, connection_handle_);

    // On a link the peer opened, the peer drives authentication. Starting our
    // own at the same moment puts two LMP transactions in flight and the link
    // dies with an LMP response timeout - which is exactly how the console's
    // play radio kept dropping us.
    if (incoming_link_) {
        log("incoming link, letting the peer authenticate");
        return;
    }

    request_authentication(data->handle);
}

// A page that did not land. Which is routine on the console side: the PS5
// answers when it feels like it, and giving up was the wrong call - one run
// got in on the third attempt, another was still being ignored on the fifth.
void BluetoothHandler::handle_connection_failure(const evt_conn_complete &data) {
    logerr("connection to {} failed: {}", data.bdaddr, get_error_message(data.status));

    // A refused incoming connection also arrives here. Retrying it would
    // page whoever we last talked to - or nobody at all, when this side
    // has no peer yet - so only our own outgoing attempts are retried.
    if (!addr_is_set(bt_addr_) || data.bdaddr != bt_addr_)
        return;

    // The console also answers a page with "limited resources" while it is
    // busy or not ready to take the controller back, so the PS5 side
    // retries on any failure rather than only on a page timeout.
    //
    // UNACCEPTABLE BD_ADDR (BlueZ's HCI_REJECTED_PERSONAL) on the pad side is
    // our own console-facing adapter answering the page, because it claims the
    // pad's address and its page scan was still on: at startup the "scan off"
    // sits behind a dozen setup commands and reached its radio 83 ms into our
    // page (run-20260911-133743). Not retrying it ended the run there.
    if (data.status == HCI_PAGE_TIMEOUT || data.status == HCI_REJECTED_PERSONAL
        || conn_side_ == EConnectionSide::to_playstation) {
        // Retry on a timer; sleeping here would stall the HCI read loop and
        // overflow the socket buffer.
        if (conn_side_ == EConnectionSide::to_playstation) {
            page_failures_++;
            if (handover_paging_since_ != std::chrono::steady_clock::time_point{}) {
                if (std::chrono::steady_clock::now() - handover_paging_since_
                    >= HANDOVER_PAGE_WINDOW) {
                    end_handover_paging();
                    log("console has not answered {} calls since the handover - backing off",
                        page_failures_);
                } else if (data.status == HCI_PAGE_TIMEOUT) {
                    log("calling the console again at once (attempt {})",
                        page_failures_ + 1);
                    schedule_reconnect(std::chrono::milliseconds(0));
                    return;
                }
            }
            // Paging the console and listening for it are mutually
            // exclusive - the controller cannot answer a page while one of
            // ours is outstanding. Backing off hands it the initiative,
            // and page scan is already on.
            // The console answers when it feels like it - one run got in
            // on the third attempt, another was still being ignored on the
            // fifth. Giving up was the wrong call; this just slows down.
            if (page_failures_ == 6)
                std::cout << "Console has ignored " << page_failures_
                          << " calls - still trying, every 20 seconds"
                          << std::endl;
            schedule_reconnect(std::chrono::seconds(
                page_failures_ < 3 ? 3 : (page_failures_ < 6 ? 10 : 20)));
        } else {
            std::cout << "Retrying connection in 3 seconds (press PS on the pad)"
                      << std::endl;
            schedule_reconnect(std::chrono::seconds(3));
        }
    }
}

void BluetoothHandler::handle_link_key_request(PacketContents &packet_contents) {
    auto *data = packet_contents.get<evt_link_key_req>();
    if (!data)
        return;   // short packet

    if (link_key_.has_value()) {
        // We have a link key — reply with it
        link_key_reply_cp reply = {};
        reply.bdaddr = data->bdaddr;
        memcpy(reply.link_key, link_key_.value().data(), 16);
        send_cmd(OGF_LINK_CTL, OCF_LINK_KEY_REPLY, LINK_KEY_REPLY_CP_SIZE,
                 &reply, "link key reply");
        log("link key reply sent");
    } else {
        // No link key — trigger pairing
        // OCF_LINK_KEY_NEG_REPLY has no dedicated struct in BlueZ, just send the bdaddr
        bdaddr_t neg_reply_addr = data->bdaddr;
        send_cmd(OGF_LINK_CTL, OCF_LINK_KEY_NEG_REPLY, 6, &neg_reply_addr,
                 "link key negative reply");
        log("no link key, requesting pairing");
    }
}

void BluetoothHandler::handle_io_capability_request(PacketContents &packet_contents) {
    auto *data = packet_contents.get<evt_io_capability_request>();
    if (!data)
        return;   // short packet
    io_capability_reply_cp reply = {};
    reply.bdaddr = data->bdaddr;
    reply.capability = 0x03;       // NoInputNoOutput
    reply.oob_data = 0x00;
    // General Bonding, MITM not required. 0x02 is *Dedicated* Bonding, which
    // asks the pad to pair for this connection only - it answers the IO
    // Capability Response with 0x04 and drops the key once the link is gone,
    // so the next run is greeted with PIN OR KEY MISSING.
    reply.authentication = AUTH_GENERAL_BONDING;
    send_cmd(OGF_LINK_CTL, OCF_IO_CAPABILITY_REPLY, IO_CAPABILITY_REPLY_CP_SIZE,
             &reply, "io capability reply");
}

void BluetoothHandler::handle_user_confirm_request(PacketContents &packet_contents) {
    auto *data = packet_contents.get<evt_user_confirm_request>();
    if (!data)
        return;   // short packet
    user_confirm_reply_cp reply = {};
    reply.bdaddr = data->bdaddr;
    send_cmd(OGF_LINK_CTL, OCF_USER_CONFIRM_REPLY, USER_CONFIRM_REPLY_CP_SIZE,
             &reply, "user confirm reply");
}

void BluetoothHandler::handle_simple_pairing_complete(PacketContents &packet_contents) {
    auto *data = packet_contents.get<evt_simple_pairing_complete>();
    if (!data)
        return;   // short packet
    if (data->status != HCI_SUCCESS) {
        logerr("pairing failed: {}", get_error_message(data->status));
    } else {
        log("pairing complete");
    }
}

void BluetoothHandler::handle_auth_complete(PacketContents &packet_contents) {
    auto *data = packet_contents.get<evt_auth_complete>();
    if (!data)
        return;   // short packet
    if (data->status != HCI_SUCCESS) {
        logerr("auth failed: {}", get_error_message(data->status));

        // 0x05 / 0x06 mean the pad no longer holds the key we saved - it was
        // reset, re-paired to another host, or the bond never survived. The
        // key is worthless now, so drop it and pair again from scratch instead
        // of retrying it forever.
        if ((data->status == HCI_AUTHENTICATION_FAILURE
             || data->status == HCI_PIN_OR_KEY_MISSING) && link_key_.has_value()) {
            link_key_.reset();
            if (on_link_key_rejected_)
                on_link_key_rejected_();
            std::cout << "Saved link key rejected by the pad - discarding it" << std::endl;
            std::cout << "== put the DualSense in PAIRING MODE: hold PS + Create ~3s ==" << std::endl;
        }
        // The pad drops the link after a failed authentication; the reconnect
        // is driven from the disconnection event.
        return;
    }
    log("auth complete");

    // HID over Bluetooth runs in security mode 4, so the console will not
    // authorize PSM 0x11 on an unencrypted link. On the pad's link the pad is
    // master and turns encryption on by itself; the console leaves it to us,
    // and asking for the channel first earned a CONN_RSP of "authorization
    // pending" followed by a dropped ACL 65 ms later. The channels are opened
    // from handle_encrypt_change() instead.
    if (conn_side_ == EConnectionSide::to_playstation) {
        request_encryption(connection_handle_);
        return;
    }

    // Our own SDP channel to the pad, opened whichever way the link was
    // established: the console will ask us for the controller's service record
    // and only the pad can answer it. On an incoming link the pad opens its own
    // control and interrupt channels, so the chain stops here.
    l2cap_send_conn_request(sdp_client_channel.psm, sdp_client_channel.scid);

    if (incoming_link_)
        log("waiting for the pad to open its L2CAP channels");
}

// A command that fails at status time produces no completion event at all, so
// without this a rejected Create Connection looks exactly like a hung adapter.
void BluetoothHandler::handle_cmd_status(PacketContents &packet_contents) {
    const auto *data = packet_contents.get<evt_cmd_status>();
    if (!data)
        return;   // short packet
    // Room again, whatever the status turns out to be.
    cmd_queue_.answered(data->ncmd);
    pump_commands();
    if (data->status == HCI_SUCCESS)
        return;

    const uint16_t opcode = btohs(data->opcode);
    const bool create_conn = opcode == cmd_opcode_pack(OGF_LINK_CTL, OCF_CREATE_CONN);

    // Our page and the peer's crossed: the incoming one established the link
    // and ours was refused because it already exists. That is success, not a
    // fault - reporting it as one and resetting the controller used to kill
    // the connection that had just come up, leaving the pad blinking and
    // giving up.
    if (create_conn && data->status == HCI_ACL_CONNECTION_EXISTS
        && connection_handle_ != 0) {
        log("our page crossed an incoming one, already connected");
        return;
    }

    if (!suppress_cmd_errors_)
        logerr("command 0x{:x} refused by the controller: {}", opcode,
               get_error_message(data->status));

    if (!create_conn || data->status != HCI_ACL_CONNECTION_EXISTS)
        return;

    // Nothing connected and the controller still claims a link: it is one left
    // by a run that was killed without disconnecting.
    if (!stale_link_reset_) {
        stale_link_reset_ = true;

        // Not a reset: HCI Reset silences the Broadcom part outright, and the
        // adapter that holds these stale links is exactly that one. Sweeping
        // the low handles costs nothing - the ones that do not exist come back
        // as unknown identifiers, which is why this stays quiet.
        log("dropping links left by an earlier run");
        suppress_cmd_errors_ = true;
        for (uint16_t handle = 1; handle <= 12; handle++) {
            disconnect_cp cp = {.handle = handle, .reason = DISCONNECT_REMOTE_USER};
            send_cmd(OGF_LINK_CTL, OCF_DISCONNECT, DISCONNECT_CP_SIZE, &cp,
                     "disconnect");
        }

        boost::asio::co_spawn(ios, [this]() -> boost::asio::awaitable<void> {
            co_await sleep_for(std::chrono::seconds(2));
            suppress_cmd_errors_ = false;
        }, boost::asio::detached);

        schedule_reconnect(std::chrono::seconds(3));
    }
}

// Once every five seconds, which is the rate the stats line prints at, so the
// two numbers a reader compares were taken at about the same moment.
boost::asio::awaitable<void> BluetoothHandler::poll_radio_quality() {
    for (;;) {
        co_await sleep_for(std::chrono::seconds(5));
        read_radio_quality();
    }
}

void BluetoothHandler::handle_cmd_complete(PacketContents &packet_contents) {
    const auto *header = packet_contents.get<evt_cmd_complete>();
    if (!header)
        return;   // short packet
    // First, before any branch below returns: the controller has room again.
    cmd_queue_.answered(header->ncmd);
    pump_commands();

    // How many ACL packets the controller can hold. On a user channel the
    // kernel does no flow control at all, so this number is the only thing
    // standing between us and a queue thousands of packets deep - which is
    // exactly what killed every handover: the radio spent its whole time
    // draining a backlog and the console's second link timed out behind it.
    if (btohs(header->opcode) == cmd_opcode_pack(OGF_INFO_PARAM, OCF_READ_BUFFER_SIZE)) {
        const auto *r = packet_contents.bytes(7);
        if (r && r[0] == HCI_SUCCESS) {
            uint16_t acl_mtu = r[1] | (r[2] << 8);
            acl_.set_max_packets(r[4] | (r[5] << 8));
            log("controller buffers: {} ACL packets of {} bytes",
                acl_.max_packets(), acl_mtu);
        }
        return;
    }

    // status, handle, value - the value is the byte we came for.
    if (btohs(header->opcode) == cmd_opcode_pack(OGF_STATUS_PARAM, OCF_READ_LINK_QUALITY)) {
        const auto *r = packet_contents.bytes(4);
        if (r && r[0] == HCI_SUCCESS) {
            link_quality_ = r[3];
            radio_read_ = true;
        }
        return;
    }
    if (btohs(header->opcode) == cmd_opcode_pack(OGF_STATUS_PARAM, OCF_READ_RSSI)) {
        const auto *r = packet_contents.bytes(4);
        if (r && r[0] == HCI_SUCCESS) {
            link_rssi_ = int8_t(r[3]);
            radio_read_ = true;
        }
        return;
    }

    // status, handle, mode, then 10 bytes of bitmap - 79 channels, one bit
    // each. What matters is how many survive.
    if (btohs(header->opcode) == cmd_opcode_pack(OGF_STATUS_PARAM, OCF_READ_AFH_MAP)) {
        const auto *r = packet_contents.bytes(14);
        if (r && r[0] == HCI_SUCCESS) {
            afh_enabled_ = r[3] != 0;
            int n = 0;
            for (int i = 0; i < 10; i++)
                n += std::popcount(static_cast<unsigned>(r[4 + i]));
            afh_channels_ = uint8_t(n);
            radio_read_ = true;
        }
        return;
    }

    if (btohs(header->opcode) != cmd_opcode_pack(OGF_INFO_PARAM, OCF_READ_BD_ADDR))
        return;

    const auto *result = packet_contents.bytes(7);
    if (!result)
        return;
    if (result[0] != HCI_SUCCESS) {
        logerr("read_bd_addr failed: {}", get_error_message(result[0]));
        return;
    }

    bdaddr_t addr{};
    memcpy(addr.b, &result[1], 6);
    bd_addr_confirmed_ = true;
    if (addr_is_set(spoof_addr_) && addr != spoof_addr_)
        logerr("address is {}, the spoof to {} did not take", addr, spoof_addr_);
    else
        log("address is now {}", addr);
}

void BluetoothHandler::handle_role_change(PacketContents &packet_contents) {
    const auto *data = packet_contents.get<evt_role_change>();
    if (!data)
        return;   // short packet
    if (data->status != HCI_SUCCESS) {
        logerr("role switch refused: {}", get_error_message(data->status));
        return;
    }
    log("now {}", data->role ? "peripheral" : "central");
}

// Sniff or park on the input link is felt as lag, so it is worth seeing in the
// log rather than inferring from a capture.
void BluetoothHandler::handle_mode_change(PacketContents &packet_contents) {
    const auto *mode = packet_contents.get<evt_mode_change>();
    if (!mode)
        return;   // short packet
    if (mode->status != HCI_SUCCESS)
        return;

    // Only the link we are relaying on matters. The console leaves its
    // registration radio connected and sniffed after the handover, and
    // reporting that as "we are in sniff" reads as latency that is not there.
    if (mode->handle == connection_handle_)
        link_sniffing_ = mode->mode == LINK_MODE_SNIFF;

    log("link mode now {} (interval {} slots)",
        mode->mode == LINK_MODE_ACTIVE ? "active" :
        mode->mode == LINK_MODE_HOLD   ? "hold" :
        mode->mode == LINK_MODE_SNIFF  ? "sniff" : "park",
        btohs(mode->interval));
}

void BluetoothHandler::handle_encrypt_change(PacketContents &packet_contents) {
    const auto *data = packet_contents.get<evt_encrypt_change>();
    if (!data)
        return;   // short packet
    if (data->status != HCI_SUCCESS) {
        logerr("encryption failed: {}", get_error_message(data->status));
        return;
    }

    log("link encrypted ({})", data->encrypt ? "on" : "off");

    // On a link the pad opened, it authenticates and no Authentication
    // Complete reaches us - so the channel setup that used to hang off that
    // event never ran, and the console's first question found an empty cache.
    // Encryption Change arrives either way.
    if (conn_side_ == EConnectionSide::to_dualsense && data->encrypt
        && sdp_client_channel.dcid == 0) {
        log("opening our SDP channel to the pad");
        l2cap_send_conn_request(sdp_client_channel.psm, sdp_client_channel.scid);
    }

    // A pad that called us is central of its link - we accept as peripheral -
    // and against a real console it never is: the PS5 takes the central role
    // right after encryption, the same sequence it runs on our console-facing
    // radio. The link role is the one thing that separates the only two
    // hole-free sessions on record from all the others. Every captured session
    // in which the pad called us has the 135 ms holes on the console link, 0.2
    // to 0.8 a second. The two in which we paged the pad, and so were central
    // from the start, have none.
    if (conn_side_ == EConnectionSide::to_dualsense && data->encrypt && incoming_link_) {
        log("taking the central role on the pad's link, as the console would");
        request_central_role(link_peer_addr_);
    }

    if (conn_side_ != EConnectionSide::to_playstation || !data->encrypt)
        return;

    // The console is master now and opens the HID channels itself. Racing it
    // gives two channels on one PSM sharing a single HciChannel, and the
    // incoming remote CID overwrites ours. Wait, and only step in if it turns
    // out not to be coming.
    // Let the console open them. The pad-side captures show a real DualSense
    // opening its own channels, and that was taken as the thing to copy - but
    // those captures show it talking to a host that never opens any, which is
    // us. This console does open them, and in the one run that worked it did:
    // getting in first at 150 ms stopped it running its own sequence.
    //
    // And when it does not open them, stepping in does not help either: the
    // console had gone quiet after encryption, our L2CAP request fifteen
    // seconds later collided with whatever it was doing, and the link died with
    // 0x34, LMP Error Transaction Collision. A console that is not opening
    // channels is telling us something, and killing the link hides it.
    //
    // So the console is left to it.
}

void BluetoothHandler::handle_link_key_notify(PacketContents &packet_contents) {
    auto *data = packet_contents.get<evt_link_key_notify>();
    if (!data)
        return;   // short packet
    memcpy(negotiated_link_key_.data(), data->link_key, 16);

    log("link key captured: ");
    for (auto b : negotiated_link_key_) printf("%02x", b);
    printf("\n");

    // On the console side a key negotiated here replaces whatever the console
    // last wrote over USB: it is the bond that link actually holds now.
    if (conn_side_ == EConnectionSide::to_playstation) {
        console_link_key_ = negotiated_link_key_;
        console_pairing_seen_ = true;
        if (on_console_pairing_)
            on_console_pairing_(bt_addr_, negotiated_link_key_);
        return;
    }

    if (on_link_key_saved_) {
        on_link_key_saved_(negotiated_link_key_);
    }
}

void BluetoothHandler::handle_disconnect_complete(PacketContents &packet_contents) {
    const auto *data = packet_contents.get<evt_disconn_complete>();
    if (!data)
        return;   // short packet

    // Whatever link this was, it is gone: stop counting it as ours and write
    // off the buffers the controller flushed for it without reporting.
    open_handles_.erase(data->handle);
    acl_.on_disconnect(data->handle);
    ghost_handles_.erase(data->handle);
    acl_reasm_.erase(data->handle);

    if (data->handle != connection_handle_)
        return; // the other link, or a stale handle from a previous run

    log("disconnected: {}", get_error_message(data->reason));

    link_sniffing_ = false;
    // Whatever was waiting for a buffer belongs to a link that no longer exists.
    held_.clear();

    // The pad is gone, so its page is what brings it back and the eager window
    // earns its cost again.
    if (conn_side_ == EConnectionSide::to_dualsense)
        write_page_scan_activity(PAGE_SCAN_INTERVAL_EAGER, PAGE_SCAN_WINDOW_EAGER);

    // quiet_the_radio() eased the page scan off while the console was playing
    // through us. With that link gone the call-back matters again, so put the
    // aggressive window back and start over.
    if (conn_side_ == EConnectionSide::to_playstation) {
        on_play_link_ = false;
        write_page_scan_activity(PAGE_SCAN_INTERVAL_EAGER, PAGE_SCAN_WINDOW_EAGER);
        write_scan_enable(SCAN_PAGE_ONLY);
    }

    connection_handle_ = 0;
    incoming_link_ = false;
    reset_channels();

    // A console that closed the HID channels first and then the link is handing
    // the controller over to its play radio, and it calls back from there.
    // Paging the registration address again on top of that only gets in the
    // way, so this side waits and stays connectable instead.
    if (conn_side_ == EConnectionSide::to_playstation && data->reason == DISCONNECT_REMOTE_USER) {
        log("console let us go cleanly - waiting for it to call back");
        schedule_reconnect(std::chrono::seconds(20));
        return;
    }

    schedule_reconnect(std::chrono::seconds(3));
}

void BluetoothHandler::reset_channels() {
    hid_sdp_channel.dcid = 0;
    sdp_client_channel.dcid = 0;
    hid_control_channel.dcid = 0;
    hid_interrupt_channel.dcid = 0;
    console_full_reports_ = false;
    console_input_seen_ = false;
    console_output_seen_ = false;
}

void BluetoothHandler::schedule_reconnect(std::chrono::milliseconds delay) {
    if (reconnect_pending_)
        return;
    reconnect_pending_ = true;
    boost::asio::co_spawn(ios, [this, delay]() -> boost::asio::awaitable<void> {
        co_await sleep_for(delay);
        reconnect_pending_ = false;
        // The pad may have called us while this timer was running. Paging it
        // now earns ACL CONNECTION ALREADY EXISTS at best.
        if (connection_handle_ != 0)
            co_return;
        send_connection_request(bt_addr_);
    }, boost::asio::detached);
}

void BluetoothHandler::handle_conn_request(PacketContents &packet_contents) {
    auto *data = packet_contents.get<evt_conn_request>();
    if (!data)
        return;   // short packet
    // Each side talks to exactly one peer. The pad's host table holds both of
    // our adapters, so pressing PS can land it on the console-facing one,
    // which used to accept it, pair with it and then relay the pad to itself.
    const bdaddr_t &expected = (conn_side_ == EConnectionSide::to_dualsense)
                                   ? bt_addr_ : console_addr_;

    // The console has a second radio: it registers the controller from
    // 2C:9E:00:26:39:AA and then calls it from ...:AB, an address it announces
    // inside its output reports. Refusing that call - which this used to do -
    // turns away the console's own attempt to take the controller. Only the
    // last byte differs, so the match is on the other five.
    bool accept = addr_is_set(expected)
                  && memcmp(&data->bdaddr.b[1], &expected.b[1], 5) == 0;

    if (accept && data->bdaddr != expected)
        log("console calling from its other radio");

    if (!accept) {
        log("refusing connection from {} (this side belongs to {})", data->bdaddr,
            addr_is_set(expected) ? std::format("{}", expected) : "nobody yet");

        struct {
            bdaddr_t bdaddr;
            uint8_t reason;
        } __attribute__((packed)) reject = {data->bdaddr, 0x0f /* unacceptable BD_ADDR */};
        send_cmd(OGF_LINK_CTL, OCF_REJECT_CONN_REQ, sizeof(reject), &reject,
                 "reject connection");
        return;
    }

    log("incoming connection from {}", data->bdaddr);

    // The pad opens the HID channels when it is the one calling, so the
    // outgoing chain in handle_conf_request() must stay out of the way. The
    // flag is applied when the connection actually completes.
    accepted_incoming_ = true;

    accept_conn_req_cp reply = {};
    reply.bdaddr = data->bdaddr;
    reply.role = 0x01; // Remain peripheral
    send_cmd(OGF_LINK_CTL, OCF_ACCEPT_CONN_REQ, ACCEPT_CONN_REQ_CP_SIZE, &reply,
             "accept connection");
}

// The controller reports buffers back per link; the pool itself is shared, so
// the running total is what the budget is measured against.
void BluetoothHandler::handle_num_completed_packets(PacketContents &packet_contents,
                                                   uint8_t plen) {
    if (plen < 1)
        return;
    // The event header is already behind the cursor; what follows is the handle
    // count and then that many {handle, packets} pairs.
    const auto *body = packet_contents.bytes(plen);
    if (!body)
        return;
    const uint8_t num_handles = body[0];
    for (uint8_t i = 0; i < num_handles; i++) {
        const size_t off = 1 + i * 4;
        if (off + 4 > plen)
            break;
        const uint16_t handle = body[off] | (body[off + 1] << 8);
        const uint16_t done = body[off + 2] | (body[off + 3] << 8);
        acl_.on_completed(handle, static_cast<int>(done));
    }

    flush_held_report();
}

std::string BluetoothHandler::get_error_message(uint8_t error_code) {
    switch (error_code) {
        case 0x01: return "UNKNOWN HCI COMMAND";
        case 0x02: return "UNKNOWN CONNECTION IDENTIFIER";
        case 0x03: return "HARDWARE FAILURE";
        case 0x04: return "PAGE TIMEOUT";
        case 0x05: return "AUTHENTICATION FAILURE";
        case 0x06: return "PIN OR KEY MISSING";
        case 0x07: return "MEMORY CAPACITY EXCEEDED";
        case 0x08: return "CONNECTION TIMEOUT";
        case 0x09: return "CONNECTION LIMIT EXCEEDED";
        case 0x0A: return "MAX NUMBER OF CONNECTIONS EXCEEDED";
        case 0x0B: return "ACL CONNECTION ALREADY EXISTS";
        case 0x0C: return "COMMAND DISALLOWED";
        case 0x0D: return "REJECTED - LIMITED RESOURCES";
        case 0x0E: return "REJECTED - SECURITY";
        case 0x0F: return "REJECTED - UNACCEPTABLE BD_ADDR";
        case 0x10: return "CONNECTION ACCEPT TIMEOUT";
        case 0x11: return "UNSUPPORTED FEATURE OR PARAMETER";
        case 0x12: return "INVALID HCI COMMAND PARAMETERS";
        case 0x13: return "REMOTE USER TERMINATED CONNECTION";
        case 0x14: return "REMOTE DEVICE TERMINATED - LOW RESOURCES";
        case 0x15: return "REMOTE DEVICE TERMINATED - POWER OFF";
        case 0x16: return "CONNECTION TERMINATED BY LOCAL HOST";
        default:   return "UNKNOWN ERROR 0x" + std::to_string(error_code);
    }
}
