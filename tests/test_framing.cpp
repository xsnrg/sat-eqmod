#include "testkit.h"
#include "sateq/framing.h"

using namespace sateq;

int main()
{
    TEST("build_command framing");
    CHECK_EQ(build_command('e', '1', ""), std::string(":e1\r"));
    CHECK_EQ(build_command('I', '1', "00B289"), std::string(":I100B289\r"));
    CHECK_EQ(build_command('G', '2', "10"), std::string(":G210\r"));
    CHECK_EQ(build_command('q', '1', "010000"), std::string(":q1010000\r"));

    TEST("accumulator: byte-at-a-time");
    ResponseAccumulator acc;
    for (const char* p = "=020401"; *p; ++p)
        CHECK(!acc.feed(*p).has_value());
    auto f = acc.feed('\r');
    CHECK(f.has_value());
    if (f) CHECK_EQ(*f, std::string("=020401"));

    TEST("accumulator: stray CR noise dropped");
    ResponseAccumulator a2;
    CHECK(!a2.feed('\r').has_value()); // noise CR with empty buffer
    a2.feed("=100");
    auto f2 = a2.feed('\r');
    CHECK(f2.has_value());
    if (f2) CHECK_EQ(*f2, std::string("=100"));

    TEST("accumulator: chunked");
    ResponseAccumulator a3;
    CHECK(!a3.feed("=00B2").has_value());
    auto f3 = a3.feed("89\r");
    CHECK(f3.has_value());
    if (f3) CHECK_EQ(*f3, std::string("=00B289"));

    TEST("parse_response");
    auto ok = parse_response("=020401");
    CHECK(ok.ok);
    CHECK(!ok.malformed);
    CHECK_EQ(ok.payload, std::string("020401"));

    auto ack_empty = parse_response("=");
    CHECK(ack_empty.ok);
    CHECK_EQ(ack_empty.payload, std::string(""));

    auto nak = parse_response("!0");
    CHECK(!nak.ok);
    CHECK(!nak.malformed);
    CHECK_EQ(nak.nak_code, 0);

    auto nak3 = parse_response("!3");
    CHECK_EQ(nak3.nak_code, 3);

    auto junk = parse_response("XYZ");
    CHECK(!junk.ok);
    CHECK(junk.malformed);

    auto empty = parse_response("");
    CHECK(empty.malformed);

    return testkit::summary("framing");
}
