//
// L2cap.cpp - ps5padlog
//
// ACL in and out: reassembly of L2CAP PDUs that arrive in pieces, routing by
// channel, the signalling that opens and configures channels, and the one place
// an outgoing frame is built.
//

#include "bluetooth/BluetoothHandler.h"
#include "bluetooth/BluetoothHandlerInternal.h"
#include "dualsense/Transforms.h"

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/l2cap.h>

#include <cstring>
#include <iostream>

namespace {

// The fixed L2CAP channel every connection has, carrying CONNECT/CONFIG/
// DISCONNECT rather than data.
constexpr uint16_t L2CAP_CID_SIGNALLING = 0x0001;

// Packet-boundary flag on every ACL start we send: "flushable", 0b10. With the
// controller's default infinite flush timeout nothing is ever flushed, so for
// delivery this is the same as the other kind. The other kind is not free to use
// anyway: non-flushable is allowed only when both controllers advertise the
// Non-flushable Packet Boundary Flag LMP feature, and here neither does - two
// control packets marked that way stuck in the controller for good, taking the
// whole two-packet input budget with them.
constexpr uint16_t ACL_START_FLUSHABLE = 0x0002;

// Packet-boundary flag on an *incoming* ACL packet, bits 12-13 of the handle
// field. START opens a new L2CAP PDU (and carries its header); CONT continues
// the previous one with payload bytes only.
constexpr uint8_t ACL_PB_CONT  = 0x01;
constexpr uint8_t ACL_PB_START = 0x02;

// A basic-mode L2CAP PDU cannot exceed this, so a reassembly that would grow
// past it is a misread rather than a real payload - dropped instead of trusted.
constexpr size_t L2CAP_MAX_PDU = 4 + 65535;

}  // namespace

// ============================================================================
// ACL / L2CAP data handling
// ============================================================================

// The ACL header packs the 12-bit handle with PB/BC flags in the top nibble.
static inline uint16_t acl_handle_pack_handle(uint16_t packed) {
    return packed & 0x0fff;
}

void BluetoothHandler::handle_acl(PacketContents &packet_contents) {
    auto *acl_header = packet_contents.get<hci_acl_hdr>();
    if (!acl_header)
        return;

    // A link this process never opened. When a run is stopped without closing
    // its links, the controller keeps them and the peer keeps talking into
    // them: the next run starts up and finds the console still streaming output
    // reports on a handle it has no Connect Complete for. Our own L2CAP CIDs
    // are reused across runs, so the data matches a channel and gets relayed to
    // the pad as if it were live - while the console, believing the controller
    // is already connected over Bluetooth, never finishes the USB registration
    // and the run stalls with the pair command never sent.
    //
    // The discriminator is exact: a link we accepted or dialled has been
    // through handle_connection_complete and is in open_handles_. A ghost has
    // not. Close it and let the pairing start over.
    const uint16_t handle_field = btohs(acl_header->handle);
    const uint16_t acl_handle = acl_handle_pack_handle(handle_field);
    const uint8_t pb = (handle_field >> 12) & 0x3;
    if (!open_handles_.count(acl_handle)) {
        if (ghost_handles_.insert(acl_handle).second) {
            log("traffic on handle {}, which this run never opened - a link left behind by a previous one; closing it",
                acl_handle);
            disconnect_cp cp = {.handle = acl_handle, .reason = DISCONNECT_REMOTE_USER};
            send_cmd(OGF_LINK_CTL, OCF_DISCONNECT, DISCONNECT_CP_SIZE, &cp,
                     "disconnect");
        }
        return;
    }

    // The ACL payload actually delivered - what remains after the header. The
    // header's own length field is the peer's claim; this is the ground truth.
    const size_t avail = packet_contents.remaining();
    auto *acl_payload = reinterpret_cast<uint8_t *>(packet_contents.get(avail));
    if (!acl_payload || avail == 0)
        return;

    // A continuation carries payload bytes with no L2CAP header of its own:
    // append to the PDU this handle already has open, and route it once full.
    if (pb == ACL_PB_CONT) {
        auto it = acl_reasm_.find(acl_handle);
        if (it == acl_reasm_.end())
            return;   // nothing open to continue - stray fragment
        auto &r = it->second;
        r.buf.insert(r.buf.end(), acl_payload, acl_payload + avail);
        if (r.buf.size() < r.expected)
            return;   // still short
        route_l2cap_pdu(r.buf.data(), r.expected);
        acl_reasm_.erase(it);
        return;
    }

    // A start opens a fresh PDU; drop any partial left over from a lost one.
    acl_reasm_.erase(acl_handle);

    if (avail < sizeof(l2cap_hdr))
        return;   // too short even for the length field
    const auto *l2 = reinterpret_cast<const l2cap_hdr *>(acl_payload);
    const size_t expected = sizeof(l2cap_hdr) + l2->len;

    if (avail >= expected) {
        // Whole PDU in one packet, the common case - route it in place with no
        // copy.
        route_l2cap_pdu(acl_payload, expected);
        return;
    }
    if (expected > L2CAP_MAX_PDU) {
        log("L2CAP start on cid 0x{:04x} claims {} bytes - implausible, dropped",
            uint16_t(l2->cid), uint16_t(l2->len));
        return;
    }
    // Fragmented: hold the start and wait for the continuations.
    AclReasm r;
    r.expected = expected;
    r.buf.assign(acl_payload, acl_payload + avail);
    acl_reasm_[acl_handle] = std::move(r);
}

void BluetoothHandler::route_l2cap_pdu(uint8_t *pdu, size_t total_len) {
    PacketContents packet_contents(pdu, total_len);
    auto *l2cap_header = packet_contents.get<l2cap_hdr>();
    if (!l2cap_header)
        return;

    // Signalling carries L2CAP commands rather than a payload, so it keeps the
    // raw cursor; everything else is a HIDP transaction we hand on as bytes.
    if (l2cap_header->cid == L2CAP_CID_SIGNALLING) {
        handle_l2cap_cmd(packet_contents);
        return;
    }

    // Reassembly above guarantees the whole payload is present now; a failure
    // here would be a genuinely malformed PDU, not a fragment.
    auto *payload_bytes = reinterpret_cast<uint8_t *>(
        packet_contents.get(l2cap_header->len));
    if (!payload_bytes) {
        log("L2CAP payload on cid 0x{:04x} claims {} bytes, {} arrived - dropped",
            uint16_t(l2cap_header->cid), uint16_t(l2cap_header->len),
            packet_contents.remaining());
        return;
    }
    const std::span<uint8_t> payload{payload_bytes, l2cap_header->len};

    // Every handler below reads at least the HIDP transaction header or an SDP
    // pdu id out of byte 0. An empty payload is legal on the wire and there is
    // nothing any of them could do with one.
    if (payload.empty())
        return;

    // Which channel it arrived on, and which peer this handler faces, is the
    // whole routing decision. Both sides run the same code, so the side check
    // is what separates "the pad answered us" from "the console is asking".
    if (l2cap_header->cid == hid_control_channel.scid) {
        if (conn_side_ == EConnectionSide::to_dualsense)
            handle_feature_answer_from_pad(payload);
        else
            handle_feature_request_from_console(payload);

    } else if (l2cap_header->cid == hid_interrupt_channel.scid) {
        if (conn_side_ == EConnectionSide::to_playstation)
            relay_console_output_to_pad(payload);
        else
            relay_pad_input(payload);

    } else if (l2cap_header->cid == sdp_client_channel.scid) {
        handle_sdp_answer_from_pad(payload);

    } else if (l2cap_header->cid == hid_sdp_channel.scid) {
        handle_sdp_query(payload);
    }
}

// ============================================================================
// L2CAP Signaling
// ============================================================================

void BluetoothHandler::handle_l2cap_cmd(PacketContents &packet_contents) {
    auto *command_header = packet_contents.get<l2cap_cmd_hdr>();
    if (!command_header)
        return;   // short packet
    current_cmd_ident_ = command_header->ident;
    switch (command_header->code) {
        case L2CAP_CONN_RSP:
            handle_conn_response(packet_contents);
            break;
        case L2CAP_CONF_RSP:
            handle_conf_response(packet_contents);
            break;
        case L2CAP_CONF_REQ:
            handle_conf_request(packet_contents);
            break;
        case L2CAP_CONN_REQ:
            // The pad is opening a channel to us. This is the normal shape of a
            // reconnect: press PS, the pad calls its host and brings up HID
            // control and interrupt itself.
            {
                auto *req = packet_contents.get<l2cap_conn_req>();
                if (!req)
                    return;   // short packet
                HciChannel *channel = get_channel_by_psm(req->psm);
                if (!channel) {
                    l2cap_conn_rsp rsp = {.dcid = 0, .scid = req->scid,
                                          .result = 0x0002 /* PSM not supported */, .status = 0};
                    l2cap_send_cmd_response(command_header->ident, L2CAP_CONN_RSP, &rsp, sizeof(rsp));
                    break;
                }

                channel->dcid = req->scid;
                log("incoming L2CAP channel psm 0x{:02x} (dcid 0x{:04x})",
                    uint16_t(req->psm), uint16_t(req->scid));

                l2cap_conn_rsp rsp = {.dcid = channel->scid, .scid = req->scid,
                                      .result = 0, .status = 0};
                l2cap_send_cmd_response(command_header->ident, L2CAP_CONN_RSP, &rsp, sizeof(rsp));

                // Both ends configure independently; the peer's CONF_REQ is
                // answered in handle_conf_request(), this is our own half.
                l2cap_send_configure_mtu_request(channel->dcid, channel->mtu);
            }
            break;
        case L2CAP_DISCONN_REQ: {
            auto *disconn_request = packet_contents.get<l2cap_disconn_req>();
            if (!disconn_request)
                return;   // short packet
            log("peer closed L2CAP channel 0x{:04x}",
                uint16_t(disconn_request->dcid));
            l2cap_disconn_rsp rsp = {.dcid = disconn_request->dcid, .scid = disconn_request->scid};
            l2cap_send_cmd_response(command_header->ident, L2CAP_DISCONN_RSP, &rsp, L2CAP_DISCONN_RSP_SIZE);
            break;
        }
        default:
            break;
    }
}

void BluetoothHandler::l2cap_send_conn_request(uint16_t psm, uint16_t scid) {
    log("asking for L2CAP psm 0x{:02x}", psm);
    l2cap_conn_req req = {.psm = psm, .scid = scid};
    l2cap_send_cmd_request(L2CAP_CONN_REQ, &req, sizeof(req));
}

void BluetoothHandler::handle_conn_response(PacketContents &packet_contents) {
    auto *conn_response = packet_contents.get<l2cap_conn_rsp>();
    if (!conn_response)
        return;   // short packet

    // A peer that is still making up its mind answers PEND and is expected to
    // send a final response later; one that never does leaves the channel
    // half-open and invisible without this.
    log("L2CAP response for 0x{:04x}: {} (status {})", uint16_t(conn_response->scid),
        conn_response->result == L2CAP_CR_SUCCESS ? "open"
            : conn_response->result == L2CAP_CR_PEND ? "pending"
            : "refused",
        uint16_t(conn_response->status));

    if (conn_response->result == L2CAP_CR_SUCCESS) {
        HciChannel *channel = get_channel_by_scid(conn_response->scid);
        if (!channel) {
            std::cerr << "Unknown scid in conn response: 0x" << std::hex << conn_response->scid << std::dec << std::endl;
            return;
        }
        channel->dcid = conn_response->dcid;
        l2cap_send_configure_mtu_request(channel->dcid, channel->mtu);
    } else if (conn_response->result != L2CAP_CR_PEND) {
        std::cerr << "L2CAP connection failed, result=" << conn_response->result << std::endl;
    }
}

void BluetoothHandler::l2cap_send_configure_mtu_request(uint16_t dcid, uint16_t mtu_size) {
    uint8_t cmd[8] = {0};
    auto *req = reinterpret_cast<l2cap_conf_req *>(cmd);
    req->dcid = dcid;
    req->flags = 0x00;

    auto *opt = reinterpret_cast<l2cap_conf_opt *>(req->data);
    opt->type = L2CAP_CONF_MTU;
    opt->len = 0x02;
    *reinterpret_cast<uint16_t *>(opt->val) = mtu_size;

    l2cap_send_cmd_request(L2CAP_CONF_REQ, cmd, sizeof(cmd));
}

void BluetoothHandler::handle_conf_response(PacketContents &packet_contents) {
    auto *conf_response = packet_contents.get<l2cap_conf_rsp>();
    if (!conf_response)
        return;   // short packet

    // The interrupt channel is the last one to be configured.
    // The console asks for the controller's service record within a moment of
    // connecting, and answering it empty gets us rejected outright. Ask the pad
    // for it as soon as this channel is usable, so the answer is already in
    // hand whenever the question comes.
    if (conf_response->scid == sdp_client_channel.scid
        && conn_side_ == EConnectionSide::to_dualsense) {
        static const uint8_t browse[] = {
            0x06,                    // ServiceSearchAttributeRequest
            0x00, 0x01,              // transaction
            0x00, 0x0f,              // parameter length
            0x35, 0x03, 0x19, 0x01, 0x00,          // search pattern: L2CAP
            0x08, 0x00,                            // max attribute bytes
            0x35, 0x05, 0x0a, 0x00, 0x00, 0xff, 0xff,  // all attributes
            0x00,                    // no continuation
        };
        log("asking the pad for its service record");
        l2cap_send_data(sdp_client_channel, browse, sizeof(browse));
    }

    if (conf_response->scid != hid_interrupt_channel.scid)
        return;

    if (conn_side_ == EConnectionSide::to_dualsense) {
        std::cout << "All L2CAP channels open" << std::endl;
        boost::asio::co_spawn(ios, start_usb_gadget(), boost::asio::detached);
    } else {
        std::cout << "Console HID channels open - relaying the pad to the PS5" << std::endl;
    }
}

void BluetoothHandler::handle_conf_request(PacketContents &packet_contents) {
    auto *conf_request = packet_contents.get<l2cap_conf_req>();
    if (!conf_request)
        return;   // short packet
    HciChannel *channel = get_channel_by_scid(conf_request->dcid);
    if (!channel) {
        std::cerr << "Unknown dcid in conf request: 0x" << std::hex << conf_request->dcid << std::dec << std::endl;
        return;
    }

    // The option to echo back sits past the fixed header, and a CONF_REQ is
    // allowed to carry none. Reading it straight out of conf_request->data
    // took four bytes on trust: with an empty option list that is whatever
    // followed the packet in the receive buffer, echoed to the peer.
    const uint8_t *option = packet_contents.bytes(4);

    uint8_t cmd[10] = {0};
    auto *rsp = reinterpret_cast<l2cap_conf_rsp *>(cmd);
    rsp->scid = channel->dcid;
    rsp->flags = 0;
    rsp->result = 0;
    size_t rsp_len = sizeof(l2cap_conf_rsp);   // an empty option list is valid
    if (option) {
        memcpy(rsp->data, option, 4);
        rsp_len += 4;
    }

    l2cap_send_cmd_response(current_cmd_ident_, L2CAP_CONF_RSP, cmd,
                            static_cast<uint16_t>(rsp_len));

    // Chain: SDP config done → open HID control → control config done → open
    // interrupt. Only when we are the caller; on an incoming link the pad
    // opens its own channels and this would race with it.
    if (incoming_link_)
        return;

    // Towards the console the peer drives channel setup; chaining from here
    // would open a second channel on a PSM it has already opened.
    if (conn_side_ == EConnectionSide::to_playstation) {
        // Same reasoning: the console opens the interrupt channel itself right
        // after control. Only step in if it has not.
        if (channel == &hid_control_channel && hid_interrupt_channel.dcid == 0) {
            boost::asio::co_spawn(ios, [this, link = connection_handle_]()
                                           -> boost::asio::awaitable<void> {
                co_await sleep_for(std::chrono::seconds(10));
                if (connection_handle_ == link && hid_interrupt_channel.dcid == 0)
                    l2cap_send_conn_request(hid_interrupt_channel.psm,
                                            hid_interrupt_channel.scid);
            }, boost::asio::detached);
        }
        return;
    }

    if (channel == &sdp_client_channel) {
        l2cap_send_conn_request(hid_control_channel.psm, hid_control_channel.scid);
    } else if (channel == &hid_sdp_channel) {
        if (conn_side_ == EConnectionSide::to_playstation)
            l2cap_send_configure_mtu_request(channel->dcid, 672);
    } else if (channel == &hid_control_channel) {
        l2cap_send_conn_request(hid_interrupt_channel.psm, hid_interrupt_channel.scid);
    }
}

BluetoothHandler::HciChannel* BluetoothHandler::get_channel_by_psm(uint16_t psm) {
    if (psm == hid_control_channel.psm) return &hid_control_channel;
    if (psm == hid_interrupt_channel.psm) return &hid_interrupt_channel;
    if (psm == hid_sdp_channel.psm) return &hid_sdp_channel;
    return nullptr;
}

BluetoothHandler::HciChannel* BluetoothHandler::get_channel_by_scid(uint16_t scid) {
    if (scid == hid_control_channel.scid) return &hid_control_channel;
    if (scid == hid_interrupt_channel.scid) return &hid_interrupt_channel;
    if (scid == hid_sdp_channel.scid) return &hid_sdp_channel;
    if (scid == sdp_client_channel.scid) return &sdp_client_channel;
    return nullptr;
}

// ============================================================================
// Sending
// ============================================================================

void BluetoothHandler::l2cap_send_data(const HciChannel &channel,
                                      const uint8_t *data, size_t len) {
    l2cap_send_framed(channel, nullptr, 0, data, len);
}

// Builds the HCI and L2CAP headers and the payload in one pass into the
// member write buffer, with an optional prefix byte in front of the payload -
// which is how the HIDP header rides along without a second buffer.
void BluetoothHandler::l2cap_send_framed(const HciChannel &channel,
                                        const uint8_t *prefix, size_t prefix_len,
                                        const uint8_t *data, size_t len) {
    if (channel.dcid == 0) {
        std::cerr << "l2cap_send: channel psm 0x" << std::hex << channel.psm
                  << std::dec << " is not open" << std::endl;
        return;
    }
    l2cap_send_frame(channel.dcid, prefix, prefix_len, data, len);
}

// Signalling and data differ only in which CID they carry and what rides in
// front of the payload, so they share this. It used to be written twice, and
// the signalling copy did not tell the flow control it had spent a buffer.
void BluetoothHandler::l2cap_send_frame(uint16_t cid,
                                        const uint8_t *prefix, size_t prefix_len,
                                        const uint8_t *data, size_t len) {
    const size_t payload_len = prefix_len + len;
    const size_t total = 1 + HCI_ACL_HDR_SIZE + L2CAP_HDR_SIZE + payload_len;
    if (total > sizeof(write_buffer_)) {
        std::cerr << "l2cap_send: payload too large (" << payload_len << ")" << std::endl;
        return;
    }

    size_t off = 0;
    write_buffer_[off++] = HCI_ACLDATA_PKT;

    const hci_acl_hdr acl_hdr = {
        .handle = htobs(transforms::acl_header_handle(connection_handle_, ACL_START_FLUSHABLE)),
        .dlen = static_cast<uint16_t>(L2CAP_HDR_SIZE + payload_len)
    };
    memcpy(&write_buffer_[off], &acl_hdr, HCI_ACL_HDR_SIZE);
    off += HCI_ACL_HDR_SIZE;

    const l2cap_hdr hdr = {
        .len = static_cast<uint16_t>(payload_len),
        .cid = cid
    };
    memcpy(&write_buffer_[off], &hdr, L2CAP_HDR_SIZE);
    off += L2CAP_HDR_SIZE;

    if (prefix_len) {
        memcpy(&write_buffer_[off], prefix, prefix_len);
        off += prefix_len;
    }
    memcpy(&write_buffer_[off], data, len);
    off += len;

    boost::system::error_code ec;
    bt_socket_.write_some(boost::asio::buffer(write_buffer_, off), ec);
    if (ec) {
        std::cerr << "l2cap_send write error: " << ec.message() << std::endl;
        return;
    }
    acl_.on_sent(connection_handle_);
}

// The HIDP transaction header is one byte in front of the report. It used to be
// joined to the report in a freshly allocated vector, once per relayed report;
// now it is passed down and written straight into the frame that was going to
// be built anyway.
void BluetoothHandler::l2cap_send_hid(const HciChannel &channel, uint8_t hidp_header,
                                     const uint8_t *data, size_t len) {
    l2cap_send_framed(channel, &hidp_header, 1, data, len);
}

void BluetoothHandler::l2cap_send_cmd_request(uint8_t code, void *data, uint16_t len) {
    l2cap_send_cmd(cmd_request_ident++, code, data, len);
}

void BluetoothHandler::l2cap_send_cmd_response(uint8_t ident, uint8_t code, void *data, uint16_t len) {
    // L2CAP requires a response to carry the identifier of the request it
    // answers. The previous code ran an independent counter, which only lined
    // up by coincidence.
    l2cap_send_cmd(ident, code, data, len);
}

void BluetoothHandler::l2cap_send_cmd(uint8_t ident, uint8_t code,
                                      const void *data, uint16_t len) {
    // The command header is the prefix; the frame builder writes it in front
    // of the payload without a second buffer.
    const l2cap_cmd_hdr cmd_hdr = {.code = code, .ident = ident, .len = len};
    l2cap_send_frame(L2CAP_CID_SIGNALLING,
                     reinterpret_cast<const uint8_t *>(&cmd_hdr), L2CAP_CMD_HDR_SIZE,
                     static_cast<const uint8_t *>(data), len);
}
