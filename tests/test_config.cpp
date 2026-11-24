// Config file parser: sections, comments, merge precedence (first wins),
// typed accessors, unused-key detection.
#include "testkit.h"
#include "sateq/config.h"

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <string>

using namespace sateq;

static void write_file(const std::string& path, const std::string& body)
{
    std::ofstream f(path);
    f << body;
}

static std::string tmpname(const char* tag)
{
    return std::string("/tmp/sateq_test_config_") + tag + ".conf";
}

int main()
{
    TEST("parse key=value, comments, whitespace");
    {
        Config c;
        CHECK(c.parse_line("  lat = 44.95  ", "t"));
        CHECK(c.parse_line("# full-line comment", "t"));
        CHECK(c.parse_line("", "t"));
        CHECK(c.parse_line("port = /dev/ttyUSB0 # trailing comment", "t"));
        CHECK_NEAR(c.num("lat", 0), 44.95, 1e-12);
        CHECK_EQ(c.str("port"), std::string("/dev/ttyUSB0"));
        CHECK(c.warnings().empty());
    }

    TEST("hash inside value survives; section prefixing");
    {
        Config c;
        CHECK(c.parse_line("path = /tmp/a#1.conf", "t")); // no whitespace before #
        CHECK_EQ(c.str("path"), std::string("/tmp/a#1.conf"));
        CHECK(c.parse_line("[mount]", "t"));
        CHECK(c.parse_line("port = /dev/ttyUSB1", "t"));
        CHECK(c.parse_line("site.elev_m = 300", "t")); // dotted key: still sectioned
        CHECK_EQ(c.str("mount.port"), std::string("/dev/ttyUSB1"));
        CHECK_NEAR(c.num("mount.site.elev_m", 0), 300, 1e-12);
        CHECK(!c.has("port")); // sectioned keys are prefixed
        CHECK(c.parse_line("[other]", "t"));
        CHECK(c.parse_line("mount.port = x", "t")); // already-prefixed stays
        CHECK(c.parse_line("x = 1", "t"));
        CHECK_EQ(c.str("mount.port"), std::string("/dev/ttyUSB1")); // not mount.mount.port
        CHECK_NEAR(c.num("other.x", 0), 1, 1e-12);
    }

    TEST("malformed lines warn, do not throw");
    {
        Config c;
        CHECK(!c.parse_line("no equals here", "t"));
        CHECK(!c.parse_line("= novalue", "t"));
        CHECK(!c.parse_line("[unterminated", "t"));
        CHECK_EQ(c.warnings().size(), 3u);
    }

    TEST("typed accessors: defaults, bools, garbage throws");
    {
        Config c;
        CHECK_EQ(c.str("missing", "dflt"), std::string("dflt"));
        CHECK_NEAR(c.num("missing", 7.5), 7.5, 1e-12);
        CHECK_EQ(c.integer("missing", 3), 3);
        CHECK(c.flag("missing", true));
        c.parse_line("b1 = yes", "t"); c.parse_line("b2 = OFF", "t");
        CHECK(c.flag("b1", false));
        CHECK(!c.flag("b2", true));
        c.parse_line("bad = abc", "t");
        bool threw = false;
        try { c.num("bad", 0); } catch (const std::exception&) { threw = true; }
        CHECK(threw);
        threw = false;
        c.parse_line("bad2 = 12x", "t");
        try { c.integer("bad2", 0); } catch (const std::exception&) { threw = true; }
        CHECK(threw);
    }

    TEST("merge_defaults: first value wins");
    {
        Config hi, lo;
        hi.parse_line("port = /dev/gps", "hi");
        hi.parse_line("rotctld_port = 4534", "hi");
        lo.parse_line("port = /dev/mount", "lo");
        lo.parse_line("lat = 44.95", "lo");
        hi.merge_defaults(lo);
        CHECK_EQ(hi.str("port"), std::string("/dev/gps")); // hi wins
        CHECK_NEAR(hi.num("lat", 0), 44.95, 1e-12);        // lo fills gap
        CHECK_EQ(hi.integer("rotctld_port", 0), 4534);
    }

    TEST("load file + unused-key detection");
    {
        auto p = tmpname("load");
        write_file(p,
                   "# station\n"
                   "lat = 44.95\n"
                   "lon = -93.28\n"
                   "[mount]\n"
                   "port = /dev/serial/by-id/usb-FTDI\n"
                   "stale_key = 42\n");
        Config c;
        CHECK(c.load(p));
        CHECK_NEAR(c.num("lat", 0), 44.95, 1e-12);
        CHECK_EQ(c.str("mount.port"), std::string("/dev/serial/by-id/usb-FTDI"));
        c.num("lon", 0); // consumed
        // lat + mount.port consumed via num/str above; lon just now
        auto unused = c.unused_keys();
        CHECK(std::find(unused.begin(), unused.end(), "lat") == unused.end());
        CHECK(std::find(unused.begin(), unused.end(), "mount.port") == unused.end());
        CHECK(std::find(unused.begin(), unused.end(), "mount.stale_key") != unused.end());
        std::remove(p.c_str());
    }

    TEST("load missing file fails cleanly");
    {
        Config c;
        CHECK(!c.load("/tmp/sateq_definitely_missing_9f3a.conf"));
        CHECK_EQ(c.warnings().size(), 1u);
    }

    return testkit::summary("config");
}
