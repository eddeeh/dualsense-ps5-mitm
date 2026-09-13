#pragma once

// The HID interface of the gadget, as bytes: what FunctionFS is told to present
// and how a control request on ep0 is read. No I/O, so tests/test_hid_gadget.cpp
// can check it without a UDC.
//
// This used to be the kernel's job. A patched f_hid carried four ioctls that
// handed GET_REPORT and SET_REPORT to userspace, which meant a kernel built from
// source and a module that had to match it. FunctionFS hands userspace ep0 as it
// is, so the stock kernel does. The descriptors below are the ones that f_hid
// presented - same interface, same HID descriptor, same endpoints and polling
// interval - so the console sees exactly the device it registered before; only
// the side of the kernel boundary answering it has moved.

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

#include <linux/usb/ch9.h>
#include <linux/usb/functionfs.h>

namespace hidgadget {

// HID class requests and descriptor types (HID 1.11, 7.2 and 7.1). Spelled out
// rather than taken from <linux/hid.h>, whose names would otherwise reach every
// file that includes the gadget; tests/test_hid_gadget.cpp holds them against
// the kernel's.
inline constexpr uint8_t REQ_GET_REPORT   = 0x01;
inline constexpr uint8_t REQ_GET_IDLE     = 0x02;
inline constexpr uint8_t REQ_GET_PROTOCOL = 0x03;
inline constexpr uint8_t REQ_SET_REPORT   = 0x09;
inline constexpr uint8_t REQ_SET_IDLE     = 0x0a;
inline constexpr uint8_t REQ_SET_PROTOCOL = 0x0b;

inline constexpr uint8_t DT_HID    = 0x21;
inline constexpr uint8_t DT_REPORT = 0x22;

// The high byte of a SET_REPORT's wValue for an output report.
inline constexpr uint8_t REPORT_TYPE_OUTPUT = 2;

// DualSense USB HID Report Descriptor, 289 bytes, read back from real
// hardware via /sys/bus/hid/devices/0003:054C:0CE6.0001/report_descriptor.
// The 273-byte version this replaced was missing feature reports 0x0B and
// 0x0C (42 bytes each). A report the descriptor does not declare cannot be
// requested over USB at all, so the console had no way to ask for them.
inline constexpr std::array<uint8_t, 289> REPORT_DESCRIPTOR = {
    0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31,
    0x09, 0x32, 0x09, 0x35, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff,
    0x00, 0x75, 0x08, 0x95, 0x06, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20,
    0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07,
    0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81,
    0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0f, 0x15, 0x00, 0x25,
    0x01, 0x75, 0x01, 0x95, 0x0f, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21,
    0x95, 0x0d, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x22, 0x15, 0x00, 0x26,
    0xff, 0x00, 0x75, 0x08, 0x95, 0x34, 0x81, 0x02, 0x85, 0x02, 0x09, 0x23,
    0x95, 0x2f, 0x91, 0x02, 0x85, 0x05, 0x09, 0x33, 0x95, 0x28, 0xb1, 0x02,
    0x85, 0x08, 0x09, 0x34, 0x95, 0x2f, 0xb1, 0x02, 0x85, 0x09, 0x09, 0x24,
    0x95, 0x13, 0xb1, 0x02, 0x85, 0x0a, 0x09, 0x25, 0x95, 0x1a, 0xb1, 0x02,
    0x85, 0x0b, 0x09, 0x41, 0x95, 0x29, 0xb1, 0x02, 0x85, 0x0c, 0x09, 0x42,
    0x95, 0x29, 0xb1, 0x02, 0x85, 0x20, 0x09, 0x26, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0x21, 0x09, 0x27, 0x95, 0x04, 0xb1, 0x02, 0x85, 0x22, 0x09, 0x40,
    0x95, 0x3f, 0xb1, 0x02, 0x85, 0x80, 0x09, 0x28, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0x81, 0x09, 0x29, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0x82, 0x09, 0x2a,
    0x95, 0x09, 0xb1, 0x02, 0x85, 0x83, 0x09, 0x2b, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0x84, 0x09, 0x2c, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0x85, 0x09, 0x2d,
    0x95, 0x02, 0xb1, 0x02, 0x85, 0xa0, 0x09, 0x2e, 0x95, 0x01, 0xb1, 0x02,
    0x85, 0xe0, 0x09, 0x2f, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf0, 0x09, 0x30,
    0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x31, 0x95, 0x3f, 0xb1, 0x02,
    0x85, 0xf2, 0x09, 0x32, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf4, 0x09, 0x35,
    0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf5, 0x09, 0x36, 0x95, 0x03, 0xb1, 0x02,
    0xc0
};

// f_hid's report_length, which it used for the endpoints' wMaxPacketSize and as
// the cap on every report in either direction. Nothing the pad sends or the
// console writes is longer.
inline constexpr size_t REPORT_LENGTH = 64;

// f_hid's intervals, left alone rather than set to what a real pad declares:
// 4 at high speed is a 1 ms poll, and every "USB 5s: in ok=" rate in the run
// logs was measured against it.
inline constexpr uint8_t INTERVAL_FULL_SPEED = 10;
inline constexpr uint8_t INTERVAL_HIGH_SPEED = 4;

// f_hid names the interface; the console reads it like any other string.
inline constexpr std::string_view INTERFACE_NAME = "HID Interface";

// The HID class descriptor, as the configuration descriptor carries it and as
// GET_DESCRIPTOR(HID) returns it. bcdHID 1.01 is f_hid's.
inline constexpr std::array<uint8_t, 9> HID_DESCRIPTOR = {
    9, DT_HID,
    0x01, 0x01,                                   // bcdHID 1.01
    0x00,                                         // bCountryCode
    1,                                            // bNumDescriptors
    DT_REPORT,
    uint8_t(REPORT_DESCRIPTOR.size() & 0xff),     // wDescriptorLength
    uint8_t(REPORT_DESCRIPTOR.size() >> 8),
};

namespace detail {

inline void put_le32(std::vector<uint8_t> &v, uint32_t x)
{
    for (int i = 0; i < 4; i++)
        v.push_back(uint8_t(x >> (8 * i)));
}

inline void set_le32(std::vector<uint8_t> &v, size_t at, uint32_t x)
{
    for (int i = 0; i < 4; i++)
        v[at + i] = uint8_t(x >> (8 * i));
}

// One speed's worth: interface, HID, interrupt IN, interrupt OUT. IN first,
// because FunctionFS names the endpoint files in this order - ep1 is IN, ep2 is
// OUT - and because f_hid listed them this way. The numbers in the addresses
// only have to be distinct and the same across speeds; the UDC assigns the real
// ones when the function binds.
inline void put_speed(std::vector<uint8_t> &v, uint8_t interval)
{
    const uint8_t descs[] = {
        9, USB_DT_INTERFACE, 0, 0, 2, USB_CLASS_HID, 0, 0, 1,
        HID_DESCRIPTOR[0], HID_DESCRIPTOR[1], HID_DESCRIPTOR[2], HID_DESCRIPTOR[3],
        HID_DESCRIPTOR[4], HID_DESCRIPTOR[5], HID_DESCRIPTOR[6], HID_DESCRIPTOR[7],
        HID_DESCRIPTOR[8],
        7, USB_DT_ENDPOINT, USB_DIR_IN | 1, USB_ENDPOINT_XFER_INT,
        uint8_t(REPORT_LENGTH), uint8_t(REPORT_LENGTH >> 8), interval,
        7, USB_DT_ENDPOINT, USB_DIR_OUT | 2, USB_ENDPOINT_XFER_INT,
        uint8_t(REPORT_LENGTH), uint8_t(REPORT_LENGTH >> 8), interval,
    };
    v.insert(v.end(), std::begin(descs), std::end(descs));
}

}  // namespace detail

// Descriptors per speed; what ffs_descriptors() declares as fs_count/hs_count.
inline constexpr uint32_t DESCRIPTORS_PER_SPEED = 4;

// The first thing written to ep0: FUNCTIONFS_DESCRIPTORS_MAGIC_V2 framing
// around the interface for full and high speed. The DWC2 on a Pi 5 is a
// high-speed part, so there is no SuperSpeed set.
inline std::vector<uint8_t> ffs_descriptors()
{
    std::vector<uint8_t> v;
    detail::put_le32(v, FUNCTIONFS_DESCRIPTORS_MAGIC_V2);
    detail::put_le32(v, 0);                     // length, set below
    detail::put_le32(v, FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC);
    detail::put_le32(v, DESCRIPTORS_PER_SPEED);
    detail::put_le32(v, DESCRIPTORS_PER_SPEED);
    detail::put_speed(v, INTERVAL_FULL_SPEED);
    detail::put_speed(v, INTERVAL_HIGH_SPEED);
    detail::set_le32(v, 4, uint32_t(v.size()));
    return v;
}

// The second: the one string the descriptors refer to (iInterface = 1), in
// US English.
inline std::vector<uint8_t> ffs_strings()
{
    std::vector<uint8_t> v;
    detail::put_le32(v, FUNCTIONFS_STRINGS_MAGIC);
    detail::put_le32(v, 0);                     // length, set below
    detail::put_le32(v, 1);                     // str_count
    detail::put_le32(v, 1);                     // lang_count
    v.push_back(0x09);                          // 0x0409, little endian
    v.push_back(0x04);
    v.insert(v.end(), INTERFACE_NAME.begin(), INTERFACE_NAME.end());
    v.push_back(0);
    detail::set_le32(v, 4, uint32_t(v.size()));
    return v;
}

// What a SETUP that reached the HID interface asks for. These are the requests
// f_hid answered; anything else it stalled, and so does USBHandler.
enum class Request {
    get_report,
    set_report,
    get_idle,
    set_idle,
    get_protocol,
    set_protocol,
    get_hid_descriptor,
    get_report_descriptor,
    unsupported,
};

inline Request classify(uint8_t request_type, uint8_t request, uint16_t value)
{
    constexpr uint8_t class_in  = USB_DIR_IN  | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
    constexpr uint8_t class_out = USB_DIR_OUT | USB_TYPE_CLASS | USB_RECIP_INTERFACE;
    constexpr uint8_t std_in    = USB_DIR_IN  | USB_TYPE_STANDARD | USB_RECIP_INTERFACE;

    if (request_type == class_in) {
        switch (request) {
        case REQ_GET_REPORT:   return Request::get_report;
        case REQ_GET_IDLE:     return Request::get_idle;
        case REQ_GET_PROTOCOL: return Request::get_protocol;
        }
    } else if (request_type == class_out) {
        switch (request) {
        case REQ_SET_REPORT:   return Request::set_report;
        case REQ_SET_IDLE:     return Request::set_idle;
        case REQ_SET_PROTOCOL: return Request::set_protocol;
        }
    } else if (request_type == std_in && request == USB_REQ_GET_DESCRIPTOR) {
        switch (value >> 8) {
        case DT_HID:    return Request::get_hid_descriptor;
        case DT_REPORT: return Request::get_report_descriptor;
        }
    }
    return Request::unsupported;
}

// The data stage of a GET_REPORT, the way f_hid built it: the report copied
// into a zeroed REPORT_LENGTH buffer and sent for wLength bytes, capped at
// REPORT_LENGTH. So an answer the pad gave over Bluetooth, CRC and all, is cut
// at the size the console asked for, and a shorter one is padded out to it.
struct ReportAnswer {
    std::array<uint8_t, REPORT_LENGTH> data{};
    size_t length = 0;

    std::span<const uint8_t> bytes() const { return {data.data(), length}; }
};

inline ReportAnswer get_report_answer(std::span<const uint8_t> report, uint16_t w_length)
{
    ReportAnswer a;
    std::copy_n(report.begin(), std::min(report.size(), REPORT_LENGTH), a.data.begin());
    a.length = std::min<size_t>(w_length, REPORT_LENGTH);
    return a;
}

}  // namespace hidgadget
