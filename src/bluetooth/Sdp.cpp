//
// Sdp.cpp - ps5padlog
//
// Service discovery. The console asks the controller for its service record and
// only the pad can answer it, so the question is forwarded and the answer kept.
//

#include "bluetooth/BluetoothHandler.h"

#include <cstdint>
#include <iterator>
#include <vector>

namespace {

// SDP protocol data unit ids. A response is always the request id plus one,
// which is what the shared branch below relies on.
constexpr uint8_t SDP_ERROR_RESPONSE                  = 0x01;
constexpr uint8_t SDP_SERVICE_SEARCH_REQUEST          = 0x02;
constexpr uint8_t SDP_SERVICE_SEARCH_RESPONSE         = 0x03;
constexpr uint8_t SDP_SERVICE_ATTRIBUTE_REQUEST       = 0x04;
constexpr uint8_t SDP_SERVICE_SEARCH_ATTR_REQUEST     = 0x06;
constexpr uint8_t SDP_INVALID_REQUEST_SYNTAX          = 0x0003;

}  // namespace

// The pad's SDP answer: cached, because the console asks again on every link
// and the channel to the pad is not always open at that moment.
void BluetoothHandler::handle_sdp_answer_from_pad(std::span<uint8_t> payload) {
    // An answer from the pad's SDP server. It belongs to the console, which
    // asked the question; continuation state and transaction id pass
    // through untouched, so a chunked record reassembles at the far end.
    uint8_t *pdu = payload.data();
    // The continuation state sits at the end, but its length is only
    // knowable from the header: ParameterLength counts the byte count, the
    // attribute list and the continuation state together.
    int cont_len = -1;
    if (payload.size() >= 7) {
        const int param_len = (pdu[3] << 8) | pdu[4];
        const int list_len  = (pdu[5] << 8) | pdu[6];
        cont_len = param_len - 2 - list_len;
    }
    log("SDP answer pdu 0x{:02x} [{}] -> console ({})", pdu[0], payload.size(),
        cont_len == 1 ? "complete" : "truncated, console must continue");
    // Cache first, forward if there is anyone to forward to: the answer to
    // our own priming request arrives long before the console asks.
    if (other_) {
        other_->sdp_answer_cache_.assign(pdu, pdu + payload.size());
        if (other_->hid_sdp_channel.dcid != 0)
            other_->l2cap_send_data(other_->hid_sdp_channel, pdu, payload.size());
    }
}

// The console asking what services this controller offers.
void BluetoothHandler::handle_sdp_query(std::span<uint8_t> payload) {
    // The pad browses SDP before it opens the HID channels. The records
    // that used to be answered here were raw ACL frames with the
    // connection handle hardcoded to 1 and an L2CAP length of 672 that was
    // never filled - on any other handle the dongle answered with a
    // Hardware Error event and stopped responding. We are the host here,
    // not a controller advertising HID, so an empty but well formed answer
    // is both correct and enough.
    uint8_t *request = payload.data();

    // The console asking us for the controller's record: forward it to the
    // pad rather than inventing an answer. Anything else - the pad browsing
    // us, or the console asking before the pad link is up - still gets the
    // empty response, which the pad has always accepted.
    if (conn_side_ == EConnectionSide::to_playstation && other_
        && other_->sdp_client_channel.dcid != 0) {
        log("SDP request pdu 0x{:02x} [{}] -> pad", request[0], payload.size());
        other_->l2cap_send_data(other_->sdp_client_channel, request, payload.size());

    } else if (conn_side_ == EConnectionSide::to_playstation
               && sdp_answer_cache_.size() >= 3
               && request[0] == SDP_SERVICE_SEARCH_ATTR_REQUEST) {
        // The channel to the pad is not open just now, but the record does
        // not change. Only the transaction id has to be the one the console
        // used, or it discards the answer.
        std::vector<uint8_t> answer = sdp_answer_cache_;
        answer[1] = request[1];
        answer[2] = request[2];
        log("SDP request pdu 0x{:02x} [{}] -> answered from cache [{}]",
            request[0], payload.size(), answer.size());
        l2cap_send_data(hid_sdp_channel, answer.data(), answer.size());

    } else {
        handle_sdp_request(request, payload.size());
    }
}

// SDP is big endian and every response echoes the request's transaction id.
void BluetoothHandler::handle_sdp_request(const uint8_t *request, size_t len) {
    if (len < 5)
        return;

    const uint8_t pdu = request[0];
    const uint8_t txn_hi = request[1], txn_lo = request[2];

    // An empty result: a zero-length data element sequence, no continuation.
    static const uint8_t empty_list[] = {0x00, 0x02, 0x35, 0x00, 0x00};

    std::vector<uint8_t> rsp;
    switch (pdu) {
    case SDP_SERVICE_SEARCH_REQUEST:
        rsp = {SDP_SERVICE_SEARCH_RESPONSE, txn_hi, txn_lo, 0x00, 0x05,
               0x00, 0x00,   // TotalServiceRecordCount
               0x00, 0x00,   // CurrentServiceRecordCount
               0x00};        // no continuation
        break;
    case SDP_SERVICE_ATTRIBUTE_REQUEST:
    case SDP_SERVICE_SEARCH_ATTR_REQUEST:
        // Every SDP response id is its request id plus one.
        rsp = {static_cast<uint8_t>(pdu + 1), txn_hi, txn_lo, 0x00, 0x05};
        rsp.insert(rsp.end(), std::begin(empty_list), std::end(empty_list));
        break;
    default:   // ErrorResponse, invalid request syntax
        rsp = {SDP_ERROR_RESPONSE, txn_hi, txn_lo, 0x00, 0x02,
               0x00, SDP_INVALID_REQUEST_SYNTAX};
        break;
    }

    log("SDP request pdu 0x{:02x} -> empty response", pdu);
    l2cap_send_data(hid_sdp_channel, rsp.data(), rsp.size());
}
