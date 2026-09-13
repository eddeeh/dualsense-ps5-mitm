#pragma once
//
// DualSense wire-protocol constants shared by the USB and Bluetooth sides.
//
// Sizes, report IDs and CRC seeds are taken from the kernel's own DualSense
// driver (linux/drivers/hid/hid-playstation.c) and verified against the
// capture dumps in this directory.
//

#include <cstddef>
#include <cstdint>
#include <zlib.h>

namespace dualsense {

// HIDP transaction headers, first byte of every L2CAP HID payload.
inline constexpr uint8_t HIDP_DATA_INPUT   = 0xa1;
inline constexpr uint8_t HIDP_DATA_OUTPUT  = 0xa2;
inline constexpr uint8_t HIDP_DATA_FEATURE = 0xa3;
inline constexpr uint8_t HIDP_GET_FEATURE  = 0x43;
inline constexpr uint8_t HIDP_SET_OUTPUT   = 0x52;
inline constexpr uint8_t HIDP_SET_FEATURE  = 0x53;

// CRC32 seeds. The seed is the HIDP header byte of the transaction the report
// travels in, prepended to the data before the checksum is taken.
inline constexpr uint8_t CRC_SEED_INPUT       = 0xa1;
inline constexpr uint8_t CRC_SEED_OUTPUT      = 0xa2;
inline constexpr uint8_t CRC_SEED_FEATURE     = 0xa3;
inline constexpr uint8_t CRC_SEED_SET_FEATURE = 0x53;

// Report IDs.
inline constexpr uint8_t REPORT_INPUT_USB  = 0x01;
inline constexpr uint8_t REPORT_OUTPUT_USB = 0x02;
inline constexpr uint8_t REPORT_BT         = 0x31;  // input and output over BT

// Over Bluetooth the pad sends the cut-down report 0x01 until the host asks for
// calibration or firmware info, and only then switches to the full 0x31. A host
// that has asked for neither does not expect 0x31 and drops it.
//   0x01: [id][x][y][rx][ry][buttons0][buttons1][buttons2][z][rz]
inline constexpr uint8_t REPORT_INPUT_BT_BASIC = 0x01;
inline constexpr size_t  INPUT_BT_BASIC_SIZE   = 10;

inline constexpr uint8_t REPORT_CALIBRATION  = 0x05;
inline constexpr uint8_t REPORT_PAIRING_INFO = 0x09;  // pad MAC + current host
inline constexpr uint8_t REPORT_PAIRING_LIST = 0x0b;  // pad MAC + host table
inline constexpr uint8_t REPORT_PAIRING_SET  = 0x0a;  // console -> pad BT host+key

// Report 0x0b is the pad's host table: its own address, three class-of-device
// bytes, then four fixed slots of [rank][host MAC], little-endian, rank 0 being
// the host it attached to most recently. Only three are ever filled.
inline constexpr size_t PAIRING_LIST_OFFSET = 11;  // a3 + id + pad MAC + class
inline constexpr size_t PAIRING_LIST_SLOTS  = 4;
inline constexpr size_t PAIRING_LIST_STRIDE = 7;   // rank byte + 6-byte address

// Report 0x08 tells the pad which transport the console is talking to it on.
// A real DualSense receives it over USB and answers by dropping its Bluetooth
// link, which is precisely what happened here: in capture-20260905-090400 the
// pad acknowledged 0x08 with HANDSHAKE_SUCCESS at t=1.787s and 0 ms later sent
// L2CAP disconnect requests for all three channels, then closed the ACL link
// with "remote user terminated". The console is addressing the gadget, not the
// pad, so this one must never leave the proxy.
// Subcommands of the transport report 0x08, which is how the console tells the
// controller where to go. See the handler for what each one means in practice.
inline constexpr uint8_t TRANSPORT_PAIR   = 0x01;  // "come to me over Bluetooth"
inline constexpr uint8_t TRANSPORT_UNPAIR = 0x02;

inline constexpr uint8_t REPORT_TRANSPORT   = 0x08;
inline constexpr uint8_t REPORT_FIRMWARE     = 0x20;
inline constexpr uint8_t REPORT_AUTH_SET     = 0xf0;  // console -> pad challenge
inline constexpr uint8_t REPORT_AUTH_GET     = 0xf1;  // pad -> console signature
inline constexpr uint8_t REPORT_AUTH_STATUS  = 0xf2;

// HIDP handshake result codes, returned by the pad as a single byte on the
// control channel.
inline constexpr uint8_t HANDSHAKE_SUCCESS             = 0x00;
inline constexpr uint8_t HANDSHAKE_NOT_READY           = 0x01;
inline constexpr uint8_t HANDSHAKE_ERR_INVALID_REPORT_ID = 0x02;
inline constexpr uint8_t HANDSHAKE_ERR_UNSUPPORTED_REQ = 0x03;
inline constexpr uint8_t HANDSHAKE_ERR_INVALID_PARAM   = 0x04;
inline constexpr uint8_t HANDSHAKE_ERR_UNKNOWN         = 0x05;
inline constexpr uint8_t HANDSHAKE_ERR_FATAL           = 0x06;

inline const char *handshake_name(uint8_t code)
{
    switch (code) {
    case HANDSHAKE_SUCCESS:               return "SUCCESS";
    case HANDSHAKE_NOT_READY:             return "NOT_READY";
    case HANDSHAKE_ERR_INVALID_REPORT_ID: return "ERR_INVALID_REPORT_ID";
    case HANDSHAKE_ERR_UNSUPPORTED_REQ:   return "ERR_UNSUPPORTED_REQUEST";
    case HANDSHAKE_ERR_INVALID_PARAM:     return "ERR_INVALID_PARAMETER";
    case HANDSHAKE_ERR_UNKNOWN:           return "ERR_UNKNOWN";
    case HANDSHAKE_ERR_FATAL:             return "ERR_FATAL";
    default:                              return "?";
    }
}

// Declared size of each feature report, including the report ID byte, read off
// the USB HID report descriptor (the 0x95 report-count byte, plus one).
//
// Over Bluetooth a feature report is always this full size and its last four
// bytes are a CRC32. Over USB the console may send a short transfer and leave
// the checksum off - report 0x0A arrives as 23 bytes where the pad expects 27.
// Relaying such a report verbatim earns an ERR_INVALID_PARAMETER handshake.
inline size_t feature_report_size(uint8_t report_id)
{
    switch (report_id) {
    case 0x05: return 41;
    case 0x08: return 48;
    case 0x09: return 20;
    case 0x0a: return 27;
    case 0x0b: return 42;
    case 0x0c: return 42;
    case 0x20: return 64;
    case 0x21: return 5;
    case 0x22: return 64;
    case 0x80: return 64;
    case 0x81: return 64;
    case 0x82: return 10;
    case 0x83: return 64;
    case 0x84: return 64;
    case 0x85: return 3;
    case 0xa0: return 2;
    case 0xe0: return 64;
    case 0xf0: return 64;
    case 0xf1: return 64;
    case 0xf2: return 16;
    case 0xf4: return 64;
    case 0xf5: return 4;
    default:   return 0; // unknown report, caller falls back to size + CRC
    }
}

// Report sizes, including the report ID byte.
inline constexpr size_t INPUT_USB_SIZE  = 64;
inline constexpr size_t INPUT_BT_SIZE   = 78;
inline constexpr size_t OUTPUT_BT_SIZE  = 78;

// Output report as a Linux host sends it, which is what the pad accepts from
// us: [0x31][seq_tag][tag][common 47][reserved 24][crc32]. The console's own
// variant carries one extra byte before the common block, but this is the
// layout in hid-playstation.c and the pad honours it.
inline constexpr size_t  OUTPUT_COMMON_SIZE    = 47;
inline constexpr uint8_t OUTPUT_TAG            = 0x10;  // DS_OUTPUT_TAG
inline constexpr size_t  OUTPUT_COMMON_OFF     = 3;     // inside the body
inline constexpr size_t  OUTPUT_VALID_FLAG1    = OUTPUT_COMMON_OFF + 1;
inline constexpr size_t  OUTPUT_LIGHTBAR_RED   = OUTPUT_COMMON_OFF + 44;
inline constexpr uint8_t VALID_FLAG1_LIGHTBAR  = 0x04;  // BIT(2)

// The 63-byte input body is identical on both transports; only the header in
// front of it differs. USB: [id][body]. BT: [id][seq_tag][body][pad][crc32].
inline constexpr size_t INPUT_BODY_LEN    = 63;
inline constexpr size_t INPUT_BODY_OFF_USB = 1;
inline constexpr size_t INPUT_BODY_OFF_BT  = 2;

// Offset of the battery/charging byte inside the 63-byte body, counted off
// struct dualsense_input_report in hid-playstation.c: six axes, sequence
// number, four button bytes, four reserved, gyro and accel, the sensor
// timestamp, one reserved, two touch points and twelve reserved.
// Low nibble is the capacity, high nibble the charging state.
inline constexpr size_t STATUS_OFF = 52;
// The high nibble of the status byte, as the kernel's hid-playstation driver
// reads it. The last three are error states in which the capacity nibble means
// nothing at all.
inline constexpr uint8_t STATUS_CHARGING_DISCHARGING = 0x0;
inline constexpr uint8_t STATUS_CHARGING_CHARGING    = 0x1;
inline constexpr uint8_t STATUS_CHARGING_FULL        = 0x2;
inline constexpr uint8_t STATUS_CHARGING_VOLTAGE_ERR = 0xa;
inline constexpr uint8_t STATUS_CHARGING_TEMP_ERR    = 0xb;
inline constexpr uint8_t STATUS_CHARGING_ERROR       = 0xf;

// crc32(seed || data), matching crc32_le()/PS_*_CRC32_SEED in hid-playstation.c.
inline uint32_t crc32_seeded(uint8_t seed, const uint8_t *data, size_t len)
{
    uLong c = ::crc32(0L, Z_NULL, 0);
    c = ::crc32(c, &seed, 1);
    c = ::crc32(c, data, static_cast<uInt>(len));
    return static_cast<uint32_t>(c);
}

// Write the trailing little-endian CRC32 of a complete report in place.
inline void append_crc32(uint8_t seed, uint8_t *report, size_t total_len)
{
    const uint32_t crc = crc32_seeded(seed, report, total_len - 4);
    report[total_len - 4] = static_cast<uint8_t>(crc);
    report[total_len - 3] = static_cast<uint8_t>(crc >> 8);
    report[total_len - 2] = static_cast<uint8_t>(crc >> 16);
    report[total_len - 1] = static_cast<uint8_t>(crc >> 24);
}

// Feature reports that describe fixed properties of the pad and can be served
// from cache. Everything else (notably the authentication chain) must reach
// the real controller on every request.
inline bool is_cacheable_feature(uint8_t report_id)
{
    return report_id == REPORT_CALIBRATION ||
           report_id == REPORT_PAIRING_INFO ||
           report_id == REPORT_FIRMWARE;
}

} // namespace dualsense
