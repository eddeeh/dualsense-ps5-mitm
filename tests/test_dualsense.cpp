// Report layout and checksums, against captured bytes where possible.
#include "check.h"

#include "dualsense/DualSense.h"
#include "fixtures.h"

#include <cstring>

namespace {

TEST(crc32_matches_what_the_pad_computed)
{
    // The strongest test in here: our arithmetic against the pad's, over bytes
    // the pad signed itself. crc32_seeded() is meant to match crc32_le() with
    // the PS_*_CRC32_SEED prefix from hid-playstation.c.
    for (size_t i = 0; i < fixtures::INPUT_REPORTS; i++) {
        const auto &r = fixtures::input_report[i];
        const uint32_t theirs = uint32_t(r[75]) | uint32_t(r[76]) << 8
                              | uint32_t(r[77]) << 16 | uint32_t(r[78]) << 24;
        const uint32_t ours = dualsense::crc32_seeded(
            dualsense::CRC_SEED_INPUT, &r[1], dualsense::INPUT_BT_SIZE - 4);
        CHECK_EQ(ours, theirs);
    }
}

TEST(append_crc32_writes_little_endian_in_place)
{
    auto r = fixtures::input_report[0];
    const auto original = r;

    // Blank the trailing checksum, then put it back.
    r[75] = r[76] = r[77] = r[78] = 0;
    dualsense::append_crc32(dualsense::CRC_SEED_INPUT, &r[1],
                            dualsense::INPUT_BT_SIZE);

    CHECK(std::memcmp(r.data(), original.data(), r.size()) == 0);
}

TEST(the_seed_is_part_of_the_checksum)
{
    // The HIDP header byte is the seed and is already payload[0], which is why
    // the CRC covers payload[1..] and not payload[0..]. Seeding it differently
    // has to produce a different answer, or that reading is untested luck.
    const auto &r = fixtures::input_report[0];
    const uint32_t a = dualsense::crc32_seeded(dualsense::CRC_SEED_INPUT,
                                               &r[1], 8);
    const uint32_t b = dualsense::crc32_seeded(dualsense::CRC_SEED_OUTPUT,
                                               &r[1], 8);
    CHECK(a != b);
}

TEST(feature_report_sizes_are_known_for_the_cached_reports)
{
    // Over Bluetooth a feature report is always its full declared size with a
    // CRC32 in the last four bytes; a size of zero means the relay has to guess.
    CHECK(dualsense::feature_report_size(dualsense::REPORT_CALIBRATION) > 4);
    CHECK(dualsense::feature_report_size(dualsense::REPORT_PAIRING_INFO) > 4);
    CHECK(dualsense::feature_report_size(dualsense::REPORT_FIRMWARE) > 4);
    CHECK_EQ(dualsense::feature_report_size(0x7e), size_t{0});   // unknown
}

TEST(cacheable_reports_are_the_fixed_properties_only)
{
    // Anything else - the authentication chain above all - has to reach the
    // real controller on every request.
    CHECK(dualsense::is_cacheable_feature(dualsense::REPORT_CALIBRATION));
    CHECK(dualsense::is_cacheable_feature(dualsense::REPORT_PAIRING_INFO));
    CHECK(dualsense::is_cacheable_feature(dualsense::REPORT_FIRMWARE));

    CHECK(!dualsense::is_cacheable_feature(dualsense::REPORT_AUTH_SET));
    CHECK(!dualsense::is_cacheable_feature(dualsense::REPORT_AUTH_GET));
    CHECK(!dualsense::is_cacheable_feature(dualsense::REPORT_AUTH_STATUS));
    CHECK(!dualsense::is_cacheable_feature(dualsense::REPORT_PAIRING_SET));
    CHECK(!dualsense::is_cacheable_feature(dualsense::REPORT_TRANSPORT));
}

TEST(handshake_codes_are_named)
{
    // These appear in the log when the pad refuses something, and the name is
    // what separates "wrong report for this transport" from "wrong contents".
    CHECK(std::strstr(dualsense::handshake_name(
              dualsense::HANDSHAKE_ERR_INVALID_REPORT_ID), "REPORT_ID") != nullptr);
    for (uint8_t code = 0; code <= dualsense::HANDSHAKE_ERR_FATAL; code++)
        CHECK(dualsense::handshake_name(code) != nullptr
              && dualsense::handshake_name(code)[0] != '\0');
    CHECK(dualsense::handshake_name(0x0f) != nullptr);   // unknown, still named
}

TEST(the_battery_byte_sits_where_the_kernel_says)
{
    // Counted off struct dualsense_input_report in hid-playstation.c. If this
    // drifts, the battery line prints a random byte - which it used to.
    CHECK_EQ(dualsense::STATUS_OFF, size_t{52});
    CHECK(dualsense::STATUS_OFF < dualsense::INPUT_BODY_LEN);

    for (size_t i = 0; i < fixtures::INPUT_REPORTS; i++) {
        if (!fixtures::carries_state[i])
            continue;
        const uint8_t status =
            fixtures::input_report[i][2 + dualsense::STATUS_OFF];
        // A real state report never shows an impossible capacity.
        CHECK((status & 0x0f) <= 10);
    }
}

TEST(feature_report_sizes_match_what_the_pad_actually_sent)
{
    // The table in DualSense.h is what the relay re-frames a console feature
    // report to before putting it on the pad's control channel; a wrong entry
    // sends the pad a malformed report. These are the sizes the pad itself
    // used, taken off the wire.
    for (size_t i = 0; i < fixtures::FEATURE_REPORTS; i++)
        CHECK_EQ(dualsense::feature_report_size(fixtures::feature_ids[i]),
                 fixtures::feature_body_len[i]);
}

TEST(feature_answers_carry_a_crc_seeded_with_their_hidp_header)
{
    // Seed 0xa3, the HIDP DATA/FEATURE header, exactly as for input reports
    // with 0xa1. Nothing tested this seed before.
    auto verify = [](const auto &blob) {
        const size_t n = blob.size();
        const uint32_t theirs = uint32_t(blob[n - 4]) | uint32_t(blob[n - 3]) << 8
                              | uint32_t(blob[n - 2]) << 16
                              | uint32_t(blob[n - 1]) << 24;
        CHECK_EQ(dualsense::crc32_seeded(dualsense::CRC_SEED_FEATURE,
                                         &blob[1], n - 5), theirs);
    };
    verify(fixtures::feature_05);
    verify(fixtures::feature_09);
    verify(fixtures::feature_0b);
    verify(fixtures::feature_20);
    verify(fixtures::feature_81);
    verify(fixtures::feature_f2);
}

TEST(output_reports_carry_a_crc_seeded_with_theirs)
{
    // Seed 0xa2. This is the checksum l2cap_send_output_report() has to get
    // right or the pad ignores everything the console sends it.
    const std::array<const uint8_t *, 3> blobs = {
        fixtures::output_report_0.data(), fixtures::output_report_1.data(),
        fixtures::output_report_2.data()};
    const size_t n = fixtures::output_report_0.size();
    for (const uint8_t *p : blobs) {
        const uint32_t theirs = uint32_t(p[n - 4]) | uint32_t(p[n - 3]) << 8
                              | uint32_t(p[n - 2]) << 16 | uint32_t(p[n - 1]) << 24;
        CHECK_EQ(dualsense::crc32_seeded(dualsense::CRC_SEED_OUTPUT, &p[1], n - 5),
                 theirs);
    }
}

TEST(the_consoles_output_variant_is_not_the_one_the_relay_builds)
{
    // Worth pinning, because the test that assumed otherwise failed here. The
    // header and the report id are shared, but byte 3 is not: the relay writes
    // OUTPUT_TAG (0x10) there, following hid-playstation.c, while the console
    // puts a subtype in it - these captures are all 0x97, the handover report
    // that names the radio it wants the controller to move to. DualSense.h says
    // as much; this makes it a check rather than a comment.
    for (const auto *p : {fixtures::output_report_0.data(),
                          fixtures::output_report_1.data(),
                          fixtures::output_report_2.data()}) {
        CHECK_EQ(int(p[0]), int(dualsense::HIDP_DATA_OUTPUT));
        CHECK_EQ(int(p[1]), int(dualsense::REPORT_BT));
        CHECK_EQ(int(p[3]), 0x97);                     // OUTPUT_SUBTYPE_HANDOVER
        CHECK(int(p[3]) != int(dualsense::OUTPUT_TAG));
        // The sequence lives in the high nibble of the seq_tag, low nibble zero,
        // which is what output_seq_ << 4 assumes when the relay builds one.
        CHECK_EQ(int(p[2] & 0x0f), 0);
    }
    CHECK_EQ(fixtures::output_report_0.size(),
             size_t{1 + dualsense::OUTPUT_BT_SIZE});

    // Consecutive reports, so the sequence really does step in that nibble.
    CHECK_EQ(int(fixtures::output_report_1[2] >> 4),
             int(fixtures::output_report_0[2] >> 4) + 1);
    CHECK_EQ(int(fixtures::output_report_2[2] >> 4),
             int(fixtures::output_report_1[2] >> 4) + 1);
}

TEST(the_handover_report_carries_a_bluetooth_address)
{
    // relay_console_output_to_pad() reads the console's other radio out of
    // bytes 10..15 of a 0x97 report. If that offset is wrong the handover is
    // missed, which is a whole run lost.
    const auto &p = fixtures::output_report_0;
    CHECK(p.size() >= 17);
    bool any = false;
    for (size_t i = 10; i < 16; i++)
        any = any || p[i] != 0;
    CHECK(any);
}

}  // namespace
