// Byte-order helpers: golden vectors from LIVE board payloads recorded
// 2026-11-15 (Sirius HEQ5 ttyUSB0 strace; HDX-110 INDI session).
#include "testkit.h"
#include "sateq/byteorder.h"
#include "sateq/protocol.h"

using namespace sateq;

int main()
{
    TEST("revu24 golden (live payloads)");
    // Sirius :a reply "00B289" = 9,024,000 microsteps/rev (NOT big-endian 45705)
    CHECK_EQ(revu24_parse("00B289"), 9024000u);
    // Sirius :b reply "A7FD00" = 64,935 steps/worm
    CHECK_EQ(revu24_parse("A7FD00"), 64935u);
    // command args use the same codec both directions (indi long2Revu24str)
    {
        char buf[7];
        revu24_format(0x7BAA, buf);
        CHECK(std::string(buf) == "AA7B00");
        CHECK_EQ(revu24_parse(buf), 0x7BAAu);
        // actual HDX-110 trace: driver wrote :E2007BAA → counter value =
        // revu24("007BAA"), i.e. nibble-pair swapped 0xAA7B00 (~half range)
        CHECK_EQ(revu24_parse("007BAA"), 0xAA7B00u);
    }

    TEST("revu24 round-trip");
    const uint32_t vals[] = {0, 1, 15, 16, 255, 64935, 598, 9024000, 16777215, 123456};
    for (uint32_t v : vals)
    {
        char buf[7];
        revu24_format(v, buf);
        CHECK_EQ(revu24_parse(buf), v & 0xFFFFFF);
    }

    TEST("mount code from :e payload");
    CHECK_EQ(mount_code_from_version_payload("020B04"), 0x04); // HDX-110 EQ8
    CHECK_EQ(mount_code_from_version_payload("020401"), 0x01); // Sirius HEQ5
    CHECK(is_eq8_class_features(0x04));
    CHECK(!is_eq8_class_features(0x01));

    TEST("highstr (:g)");
    CHECK_EQ(highstr_parse("10"), 16u); // both live boards

    TEST("hex_nibble");
    CHECK_EQ(hex_nibble('0'), 0);
    CHECK_EQ(hex_nibble('A'), 10);
    CHECK_EQ(hex_nibble('f'), 15);
    CHECK_THROWS(hex_nibble('Z'), std::invalid_argument);

    return testkit::summary("byteorder");
}
