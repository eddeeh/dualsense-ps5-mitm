// The HID interface as FunctionFS is given it, and how ep0 requests are read.
//
// FunctionFS answers a descriptor set it does not like with EINVAL and nothing
// else, at the moment the relay starts, on the Pi. These walk the same bytes
// the way the kernel does, so a mistake shows up here first. The report
// descriptor is checked against feature answers the pad sent in a capture,
// because a report it declares at the wrong size is one the console reads
// wrong without any error at all.
#include "check.h"

#include "usb/HidGadget.h"
#include "fixtures.h"

#include <cstring>
#include <map>

#include <linux/hid.h>

namespace {

uint32_t le32(const std::vector<uint8_t> &v, size_t at)
{
    return uint32_t(v[at]) | uint32_t(v[at + 1]) << 8 | uint32_t(v[at + 2]) << 16
         | uint32_t(v[at + 3]) << 24;
}

uint16_t le16(const uint8_t *p)
{
    return uint16_t(p[0] | p[1] << 8);
}

// One speed's descriptors, split at their bLength the way ffs_do_descs does.
struct Descriptor {
    const uint8_t *p;
    size_t len;
};

std::vector<Descriptor> split(const std::vector<uint8_t> &v, size_t &at, uint32_t count)
{
    std::vector<Descriptor> out;
    for (uint32_t i = 0; i < count && at < v.size(); i++) {
        const size_t len = v[at];
        if (len < 2 || at + len > v.size())
            break;
        out.push_back({&v[at], len});
        at += len;
    }
    return out;
}

// Bytes each feature report occupies on the wire, report ID included, from the
// report descriptor's own Report ID / Report Size / Report Count items.
std::map<uint8_t, size_t> declared_feature_sizes()
{
    const auto &d = hidgadget::REPORT_DESCRIPTOR;
    std::map<uint8_t, size_t> bits;
    uint32_t report_size = 0, report_count = 0;
    uint8_t report_id = 0;

    for (size_t i = 0; i < d.size();) {
        const uint8_t prefix = d[i];
        if (prefix == 0xfe) {                   // long item: size, tag, data
            i += 3 + (i + 1 < d.size() ? d[i + 1] : 0);
            continue;
        }
        size_t n = prefix & 0x03;
        if (n == 3)
            n = 4;
        uint32_t value = 0;
        for (size_t k = 0; k < n && i + 1 + k < d.size(); k++)
            value |= uint32_t(d[i + 1 + k]) << (8 * k);

        switch (prefix & 0xfc) {
        case 0x74: report_size = value; break;            // Report Size
        case 0x94: report_count = value; break;           // Report Count
        case 0x84: report_id = uint8_t(value); break;     // Report ID
        case 0xb0: bits[report_id] += report_size * report_count; break;  // Feature
        }
        i += 1 + n;
    }

    std::map<uint8_t, size_t> bytes;
    for (const auto &[id, b] : bits)
        bytes[id] = 1 + b / 8;
    return bytes;
}

TEST(the_hid_constants_are_the_kernels)
{
    CHECK_EQ(int(hidgadget::REQ_GET_REPORT), int(HID_REQ_GET_REPORT));
    CHECK_EQ(int(hidgadget::REQ_GET_IDLE), int(HID_REQ_GET_IDLE));
    CHECK_EQ(int(hidgadget::REQ_GET_PROTOCOL), int(HID_REQ_GET_PROTOCOL));
    CHECK_EQ(int(hidgadget::REQ_SET_REPORT), int(HID_REQ_SET_REPORT));
    CHECK_EQ(int(hidgadget::REQ_SET_IDLE), int(HID_REQ_SET_IDLE));
    CHECK_EQ(int(hidgadget::REQ_SET_PROTOCOL), int(HID_REQ_SET_PROTOCOL));
    CHECK_EQ(int(hidgadget::DT_HID), int(HID_DT_HID));
    CHECK_EQ(int(hidgadget::DT_REPORT), int(HID_DT_REPORT));
}

TEST(the_descriptor_blob_frames_itself)
{
    const auto v = hidgadget::ffs_descriptors();
    CHECK(v.size() >= 20);
    if (v.size() < 20)
        return;
    CHECK_EQ(le32(v, 0), uint32_t(FUNCTIONFS_DESCRIPTORS_MAGIC_V2));
    CHECK_EQ(le32(v, 4), uint32_t(v.size()));
    CHECK_EQ(le32(v, 8), uint32_t(FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC));
    CHECK_EQ(le32(v, 12), hidgadget::DESCRIPTORS_PER_SPEED);
    CHECK_EQ(le32(v, 16), hidgadget::DESCRIPTORS_PER_SPEED);
}

TEST(both_speed_sets_split_at_their_counts_and_use_every_byte)
{
    const auto v = hidgadget::ffs_descriptors();
    size_t at = 20;
    const auto fs = split(v, at, hidgadget::DESCRIPTORS_PER_SPEED);
    const auto hs = split(v, at, hidgadget::DESCRIPTORS_PER_SPEED);
    CHECK_EQ(fs.size(), size_t(hidgadget::DESCRIPTORS_PER_SPEED));
    CHECK_EQ(hs.size(), size_t(hidgadget::DESCRIPTORS_PER_SPEED));
    // Anything left over is "garbage" to ffs_do_descs, and EINVAL.
    CHECK_EQ(at, v.size());
}

TEST(each_speed_is_one_hid_interface_with_the_endpoints_f_hid_had)
{
    const auto v = hidgadget::ffs_descriptors();
    size_t at = 20;
    const std::pair<std::vector<Descriptor>, uint8_t> speeds[] = {
        {split(v, at, hidgadget::DESCRIPTORS_PER_SPEED), hidgadget::INTERVAL_FULL_SPEED},
        {split(v, at, hidgadget::DESCRIPTORS_PER_SPEED), hidgadget::INTERVAL_HIGH_SPEED},
    };

    for (const auto &[d, interval] : speeds) {
        if (d.size() != 4) {
            CHECK_EQ(d.size(), size_t(4));
            continue;
        }
        // Lengths FunctionFS insists on for each type.
        CHECK_EQ(d[0].len, size_t(USB_DT_INTERFACE_SIZE));
        CHECK_EQ(d[1].len, size_t(9));
        CHECK_EQ(d[2].len, size_t(USB_DT_ENDPOINT_SIZE));
        CHECK_EQ(d[3].len, size_t(USB_DT_ENDPOINT_SIZE));

        CHECK_EQ(int(d[0].p[1]), USB_DT_INTERFACE);
        CHECK_EQ(int(d[0].p[4]), 2);                      // bNumEndpoints
        CHECK_EQ(int(d[0].p[5]), USB_CLASS_HID);
        CHECK_EQ(int(d[0].p[8]), 1);                      // iInterface: the one string

        // The HID descriptor is only accepted inside a HID interface, right
        // after it, and it is the same bytes GET_DESCRIPTOR(HID) returns.
        CHECK(std::memcmp(d[1].p, hidgadget::HID_DESCRIPTOR.data(), 9) == 0);

        for (int e = 2; e <= 3; e++) {
            CHECK_EQ(int(d[e].p[1]), USB_DT_ENDPOINT);
            CHECK_EQ(int(d[e].p[3]), USB_ENDPOINT_XFER_INT);
            CHECK_EQ(le16(&d[e].p[4]), uint16_t(hidgadget::REPORT_LENGTH));
            CHECK_EQ(int(d[e].p[6]), int(interval));
            // A zero endpoint number is refused.
            CHECK((d[e].p[2] & USB_ENDPOINT_NUMBER_MASK) != 0);
        }
        // IN first: FunctionFS numbers the endpoint files in this order, and
        // USBHandler opens ep1 for input and ep2 for output.
        CHECK((d[2].p[2] & USB_DIR_IN) != 0);
        CHECK((d[3].p[2] & USB_DIR_IN) == 0);
    }

    // Addresses must match across speeds, or FunctionFS refuses the set.
    if (speeds[0].first.size() == 4 && speeds[1].first.size() == 4) {
        CHECK_EQ(int(speeds[0].first[2].p[2]), int(speeds[1].first[2].p[2]));
        CHECK_EQ(int(speeds[0].first[3].p[2]), int(speeds[1].first[3].p[2]));
    }
}

TEST(the_hid_descriptor_points_at_the_whole_report_descriptor)
{
    const auto &h = hidgadget::HID_DESCRIPTOR;
    CHECK_EQ(int(h[0]), 9);
    CHECK_EQ(int(h[1]), HID_DT_HID);
    CHECK_EQ(int(h[6]), HID_DT_REPORT);
    CHECK_EQ(size_t(le16(&h[7])), hidgadget::REPORT_DESCRIPTOR.size());
    CHECK_EQ(hidgadget::REPORT_DESCRIPTOR.size(), size_t(289));
}

TEST(the_strings_blob_carries_the_interface_name)
{
    const auto v = hidgadget::ffs_strings();
    const std::string_view name = hidgadget::INTERFACE_NAME;
    CHECK_EQ(v.size(), size_t(16 + 2 + name.size() + 1));
    if (v.size() != 16 + 2 + name.size() + 1)
        return;
    CHECK_EQ(le32(v, 0), uint32_t(FUNCTIONFS_STRINGS_MAGIC));
    CHECK_EQ(le32(v, 4), uint32_t(v.size()));
    CHECK_EQ(le32(v, 8), uint32_t(1));
    CHECK_EQ(le32(v, 12), uint32_t(1));
    CHECK_EQ(le16(&v[16]), uint16_t(0x0409));
    CHECK(std::memcmp(&v[18], name.data(), name.size()) == 0);
    CHECK_EQ(int(v.back()), 0);
}

TEST(the_report_descriptor_declares_each_captured_feature_at_the_size_the_pad_sent)
{
    // The pad's Bluetooth answer, without its a3 header, is as long as the USB
    // report the descriptor declares - the CRC takes the place of the tail.
    const auto sizes = declared_feature_sizes();
    for (size_t i = 0; i < fixtures::FEATURE_REPORTS; i++) {
        const auto it = sizes.find(fixtures::feature_ids[i]);
        CHECK(it != sizes.end());
        if (it != sizes.end())
            CHECK_EQ(it->second, fixtures::feature_body_len[i]);
    }
}

TEST(the_report_descriptor_declares_the_reports_the_registration_uses)
{
    // Absent from the descriptor means the console cannot ask at all; the
    // 273-byte one this replaced lacked 0x0b and 0x0c.
    const auto sizes = declared_feature_sizes();
    for (uint8_t id : {uint8_t(0x05), uint8_t(0x08), uint8_t(0x09), uint8_t(0x0a),
                       uint8_t(0x0b), uint8_t(0x0c), uint8_t(0x20), uint8_t(0x80),
                       uint8_t(0x81), uint8_t(0xf0), uint8_t(0xf1), uint8_t(0xf2)})
        CHECK(sizes.count(id) == 1);
}

TEST(requests_are_read_the_way_f_hid_read_them)
{
    using hidgadget::Request;
    using hidgadget::classify;
    const auto is = [](Request got, Request want) { return int(got) == int(want); };

    // bRequestType, bRequest, wValue as they arrive in a SETUP.
    CHECK(is(classify(0xa1, HID_REQ_GET_REPORT, 0x0305), Request::get_report));
    CHECK(is(classify(0x21, HID_REQ_SET_REPORT, 0x030a), Request::set_report));
    CHECK(is(classify(0x21, HID_REQ_SET_REPORT, 0x0202), Request::set_report));
    CHECK(is(classify(0xa1, HID_REQ_GET_IDLE, 0), Request::get_idle));
    CHECK(is(classify(0x21, HID_REQ_SET_IDLE, 0), Request::set_idle));
    CHECK(is(classify(0xa1, HID_REQ_GET_PROTOCOL, 0), Request::get_protocol));
    CHECK(is(classify(0x21, HID_REQ_SET_PROTOCOL, 1), Request::set_protocol));
    CHECK(is(classify(0x81, USB_REQ_GET_DESCRIPTOR, 0x2200), Request::get_report_descriptor));
    CHECK(is(classify(0x81, USB_REQ_GET_DESCRIPTOR, 0x2100), Request::get_hid_descriptor));

    // Everything else is stalled.
    CHECK(is(classify(0x81, USB_REQ_GET_DESCRIPTOR, 0x0200), Request::unsupported));
    CHECK(is(classify(0xa0, HID_REQ_GET_REPORT, 0x0305), Request::unsupported));  // to the device
    CHECK(is(classify(0x21, HID_REQ_GET_REPORT, 0x0305), Request::unsupported));  // wrong direction
    CHECK(is(classify(0xc1, 0x01, 0), Request::unsupported));                     // vendor
}

TEST(a_get_report_answer_is_what_the_pad_sent_cut_to_what_was_asked)
{
    // As on_feature_report gets it: the pad's answer after the a3 header.
    const std::span<const uint8_t> answer(&fixtures::feature_05[1],
                                          fixtures::feature_05.size() - 1);
    const auto a = hidgadget::get_report_answer(answer, 41);
    CHECK_EQ(a.length, size_t(41));
    CHECK(std::memcmp(a.bytes().data(), answer.data(), 41) == 0);

    const auto shorter = hidgadget::get_report_answer(answer, 16);
    CHECK_EQ(shorter.length, size_t(16));
    CHECK(std::memcmp(shorter.bytes().data(), answer.data(), 16) == 0);
}

TEST(a_short_answer_is_padded_with_zeros_to_the_requested_length)
{
    const uint8_t report[] = {0xf2, 0x00, 0x01, 0x12};
    const auto a = hidgadget::get_report_answer(report, 16);
    CHECK_EQ(a.length, size_t(16));
    CHECK(std::memcmp(a.bytes().data(), report, sizeof(report)) == 0);
    for (size_t i = sizeof(report); i < a.length; i++)
        CHECK_EQ(int(a.data[i]), 0);
}

TEST(an_answer_is_never_longer_than_the_report_length)
{
    const std::span<const uint8_t> answer(&fixtures::feature_20[1],
                                          fixtures::feature_20.size() - 1);
    CHECK_EQ(hidgadget::get_report_answer(answer, 0x0100).length,
             hidgadget::REPORT_LENGTH);
}

TEST(no_answer_at_all_is_zeros)
{
    // f_hid's fallback when userspace missed the window and nothing had ever
    // been written for that report.
    const auto a = hidgadget::get_report_answer({}, 16);
    CHECK_EQ(a.length, size_t(16));
    for (size_t i = 0; i < a.length; i++)
        CHECK_EQ(int(a.data[i]), 0);
}

}  // namespace
