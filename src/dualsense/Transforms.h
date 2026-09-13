//
// The parts of the relay that are pure functions of bytes: no sockets, no
// state, nothing that needs a controller or a console to run. Kept apart so
// they can be reasoned about - and, if a test target is ever added, exercised -
// against reports captured off the wire, without a pad or a console present.
//
#ifndef PS5MITM_TRANSFORMS_H
#define PS5MITM_TRANSFORMS_H

#include <array>
#include <cstdint>
#include <cstring>
#include <span>

#include "dualsense/DualSense.h"

namespace transforms {

// ---------------------------------------------------------------------------
// HCI framing
// ---------------------------------------------------------------------------

// The 16-bit handle field of an HCI ACL header: 12 bits of handle, then the
// packet-boundary and broadcast flags.
//
// This is a function and not BlueZ's acl_handle_pack macro on purpose. The
// macro expands to ((h & 0x0fff) | (f << 12)), so passing a ternary as f binds
// looser than the shift and the flag lands in the handle unshifted - handle 12
// with flag 2 becomes 0x000e, which is handle 14. That cost an evening, which
// is why the shift lives here in one place with the mask spelled out.
constexpr uint16_t acl_header_handle(uint16_t handle, uint16_t boundary_flag)
{
    return static_cast<uint16_t>((handle & 0x0fff)
                                 | (static_cast<uint16_t>(boundary_flag & 0x3) << 12));
}

// ---------------------------------------------------------------------------
// Input reports
// ---------------------------------------------------------------------------

// Offset of the report body inside a Bluetooth payload that still carries its
// HIDP transaction header: [0xa1][0x31][seq_tag][body...].
inline constexpr size_t BT_BODY_OFF = 1 + dualsense::INPUT_BODY_OFF_BT;

// Face buttons and the d-pad hat share body byte 7: the hat in the low nibble,
// the four face buttons in the high one.
inline constexpr size_t FACE_BUTTON_OFF = 7;

// Third button byte, body[9]: PS in bit 0, the touchpad click in bit 1.
// Measured, the pad's button bytes go 08 00 00 -> 08 00 01 across a press and
// the console answers with the pair command 8 ms later, so this one bit is
// what the whole Bluetooth handover hangs on.
inline constexpr size_t  BUTTON_PS_OFF = 9;
inline constexpr uint8_t BUTTON_PS     = 0x01;

// About one report in ten on the interrupt channel is not controller state at
// all: the pad interleaves frames of an opaque, incompressible stream under the
// same report id, with a byte-wide counter in body[0] and 0xd4 fixed in body[1].
// The console switches it on when it takes the play link, and everything about
// it - the entropy, the frame rate, the microphone in the hardware - says audio.
//
// Telling the two apart used to key on that 0xd4 marker, which is not a proof:
// in a state report body[1] is the right stick's Y axis and 0xd4 is an ordinary
// value for it. The flags byte between the report id and the body is better -
// the documented layout of ReportIn31 names it, bit 0 HasHID, bit 1 HasMic,
// bits 4-7 a sequence number. Over 120000 captured frames the flag agreed with
// the marker heuristic on all but four, and three of those four were real
// controller state the heuristic threw away because the right stick's Y axis
// happened to sit at 0xd4.
inline constexpr size_t BT_FLAGS_OFF = 2;
inline constexpr uint8_t FLAG_HAS_HID = 0x01;

inline bool carries_controller_state(std::span<const uint8_t> payload)
{
    return payload.size() == 1 + dualsense::INPUT_BT_SIZE
        && (payload[BT_FLAGS_OFF] & FLAG_HAS_HID) != 0;
}

// Whether a payload's trailing checksum is the one it should be. Removed once
// when nothing called it; the tests call it now, which is the use the comment
// in this file's header always claimed for these functions.
inline bool input_crc_is_valid(std::span<const uint8_t> payload)
{
    if (payload.size() != 1 + dualsense::INPUT_BT_SIZE)
        return false;
    const size_t n = payload.size();
    const uint32_t theirs = (uint32_t)payload[n - 4]
                          | (uint32_t)payload[n - 3] << 8
                          | (uint32_t)payload[n - 2] << 16
                          | (uint32_t)payload[n - 1] << 24;
    return theirs == dualsense::crc32_seeded(dualsense::CRC_SEED_INPUT, &payload[1],
                                             dualsense::INPUT_BT_SIZE - 4);
}

// The cut-down report a pad sends before the host has asked it for anything.
// Same fields, fewer of them, in the pre-0x31 order, and no checksum.
//
// The third button byte is not a straight copy: here it carries PS and the
// touchpad in its low bits and an incrementing counter in bits 2-7, which lives
// in a separate field of the full report. Sending it as zero gives the host a
// controller whose counter never moves, and it ignores the stream.
inline std::array<uint8_t, dualsense::INPUT_BT_BASIC_SIZE>
make_cut_down_report(std::span<const uint8_t> payload)
{
    const uint8_t *b = &payload[BT_BODY_OFF];
    const uint8_t buttons2 = (b[9] & 0x03) | static_cast<uint8_t>((b[6] & 0x3f) << 2);
    const uint8_t buttons0 = b[FACE_BUTTON_OFF];

    return {
        dualsense::REPORT_INPUT_BT_BASIC,
        b[0], b[1], b[2], b[3],    // sticks
        buttons0, b[8], buttons2,  // buttons + counter
        b[4], b[5],                // triggers
    };
}

// The same 63-byte body, reframed for USB. Only the header in front of it
// differs; the seq_tag is Bluetooth-only framing, and copying from it shifts
// every axis and button by one byte.
//
// The charging nibble is forced, the capacity nibble is not. To the console
// this device is on a USB port, and a wired DualSense always reports charging;
// relaying the pad's own "discharging" would say once per report that the
// controller it is talking to is not the one plugged into it. The capacity is
// passed through, so a full pad reads FULL and any other level reads CHARGING.
inline std::array<uint8_t, dualsense::INPUT_USB_SIZE>
make_usb_report(std::span<const uint8_t> payload)
{
    std::array<uint8_t, dualsense::INPUT_USB_SIZE> usb{};
    usb[0] = dualsense::REPORT_INPUT_USB;
    memcpy(&usb[dualsense::INPUT_BODY_OFF_USB], &payload[BT_BODY_OFF],
           dualsense::INPUT_BODY_LEN);

    uint8_t &status = usb[dualsense::INPUT_BODY_OFF_USB + dualsense::STATUS_OFF];
    const uint8_t capacity = status & 0x0f;
    const uint8_t charging = capacity >= 10 ? dualsense::STATUS_CHARGING_FULL
                                            : dualsense::STATUS_CHARGING_CHARGING;
    status = static_cast<uint8_t>((charging << 4) | capacity);
    return usb;
}

// Press PS in a report already framed for USB. This transport carries no
// checksum of its own - the console sends short transfers and checks none - so
// unlike the Bluetooth path there is nothing to re-stamp afterwards.
inline void set_ps_on_usb_report(std::span<uint8_t> usb)
{
    if (usb.size() != dualsense::INPUT_USB_SIZE)
        return;
    usb[dualsense::INPUT_BODY_OFF_USB + BUTTON_PS_OFF] |= BUTTON_PS;
}

}  // namespace transforms

#endif  // PS5MITM_TRANSFORMS_H
