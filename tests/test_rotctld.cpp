// rotctld protocol handler. Wire behaviors VERIFIED empirically against
// hamlib 4.7.2 (dummy rotctld + fake-server capture) on 2026-11-15:
//  - "\\dump_state\\n" -> "1\\n1\\n...done\\n"
//  - "P az el\\n" -> "RPRT 0\\n"
//  - "p\\n" -> "az\\nel\\n" (no RPRT on success)
//  - bogus -> "RPRT -1\\n"
#include "testkit.h"
#include "sateq/rotctld.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <mutex>
#include <stdexcept>
#include <sstream>
#include <thread>
#include <vector>

using namespace sateq;

namespace {
struct FakeHandler : RotctldHandler {
    double set_az = -1, set_el = -1;
    double cur_az = 12.5, cur_el = 67.25;
    int stops = 0, parks = 0, resets = 0;
    void on_set_target(double az, double el) override { set_az = az; set_el = el; }
    void on_get_position(double& az, double& el) override { az = cur_az; el = cur_el; }
    void on_stop() override { ++stops; }
    void on_park() override { ++parks; }
    void on_reset() override { ++resets; }
};

struct ThrowingHandler : RotctldHandler {
    void on_set_target(double, double) override {}
    void on_get_position(double&, double&) override
    {
        throw std::runtime_error("simulated board fault");
    }
    void on_stop() override {}
    void on_park() override {}
    void on_reset() override {}
};
} // namespace

int main()
{
    FakeHandler h;

    TEST("status extension: panel body + RPRT 0; unsupported handler -> RPRT -1");
    {
        // FakeHandler does not override on_status -> empty -> RPRT -1,
        // exactly like an unknown command on another vendor's server.
        CHECK(rotctld_handle_line("status", h) == "RPRT -1\n");
        struct StatusHandler : FakeHandler
        {
            std::string on_status() override { return "state: TRACK\nclients:  1\n"; }
        } sh;
        const std::string r = rotctld_handle_line("status", sh);
        CHECK(r == "state: TRACK\nclients:  1\nRPRT 0\n");
    }

    TEST("dump_state handshake (hamlib 4.7.2 exact shape)");
    {
        std::string r = rotctld_handle_line("\\dump_state", h);
        CHECK(r.rfind("1\n1\n", 0) == 0);
        CHECK(r.find("rot_type=AzEl") != std::string::npos);
        CHECK(r.find("south_zero=0") != std::string::npos);
        CHECK(r.size() > 5 && r.compare(r.size() - 5, 5, "done\n") == 0);
    }

    TEST("P az el sets target");
    {
        auto r = rotctld_handle_line("P 123.5 40.25", h);
        CHECK_EQ(r, std::string("RPRT 0\n"));
        CHECK_NEAR(h.set_az, 123.5, 1e-9);
        CHECK_NEAR(h.set_el, 40.25, 1e-9);
        // hamlib sends full precision
        rotctld_handle_line("P 45.500000 30.250000", h);
        CHECK_NEAR(h.set_az, 45.5, 1e-9);
    }

    TEST("p returns az\\nel\\n only");
    {
        auto r = rotctld_handle_line("p", h);
        CHECK(r.find("RPRT") == std::string::npos);
        std::istringstream is(r);
        double az, el;
        is >> az >> el;
        CHECK_NEAR(az, 12.5, 1e-3);
        CHECK_NEAR(el, 67.25, 1e-3);
    }

    TEST("stop / park / reset");
    {
        rotctld_handle_line("S", h);
        CHECK_EQ(h.stops, 1);
        rotctld_handle_line("\\stop", h);
        CHECK_EQ(h.stops, 2);
        rotctld_handle_line("K", h);
        CHECK_EQ(h.parks, 1);
        rotctld_handle_line("R", h);
        CHECK_EQ(h.resets, 1);
    }

    TEST("legacy '+' prefix tolerated (GPredict old-style)");
    {
        auto r = rotctld_handle_line("+P 10 20", h);
        CHECK_EQ(r, std::string("RPRT 0\n"));
        CHECK_NEAR(h.set_az, 10.0, 1e-9);
    }

    TEST("bogus command and malformed args -> RPRT -1");
    {
        CHECK_EQ(rotctld_handle_line("boguscmd", h), std::string("RPRT -1\n"));
        CHECK_EQ(rotctld_handle_line("P 12", h), std::string("RPRT -1\n"));
        CHECK_EQ(rotctld_handle_line("P", h), std::string("RPRT -1\n"));
        CHECK_EQ(rotctld_handle_line("", h), std::string("RPRT -1\n"));
    }

    TEST("q closes (empty reply)");
    {
        CHECK_EQ(rotctld_handle_line("q", h), std::string(""));
        CHECK_EQ(rotctld_handle_line("Q", h), std::string(""));
    }

    TEST("long names accepted");
    {
        CHECK_EQ(rotctld_handle_line("set_pos 1 2", h), std::string("RPRT 0\n"));
        CHECK_EQ(rotctld_handle_line("\\get_pos", h).find("RPRT"), std::string::npos);
    }

    TEST("caps '1': GridTracker2 az+el contract (rotor.js rotctld.parse)");
    {
        std::string r = rotctld_handle_line("1", h);
        // 1) a line matches /^Max Elevation:\s*(-?\d+(?:\.\d+)?)/ with value > 0
        bool has_el = false;
        for (size_t pos = 0; pos < r.size();)
        {
            size_t nl = r.find('\n', pos);
            std::string line = r.substr(pos, nl == std::string::npos ? nl : nl - pos);
            if (line.rfind("Max Elevation:", 0) == 0 &&
                std::atof(line.c_str() + 14) > 0)
                has_el = true;
            // 2) no numeric-ONLY lines: GT2 pairs those as az/el positions
            std::string t = line;
            CHECK(!t.empty()); // no blank lines
            bool numeric = !t.empty();
            for (char c : t)
                if (!isdigit((unsigned char)c) && c != '.' && c != '-')
                    numeric = false;
            CHECK(!numeric);
            // 3) never "RPRT -" (GT2 latches rotor Error on it)
            CHECK(line.rfind("RPRT -", 0) != 0);
            if (nl == std::string::npos) break;
            pos = nl + 1;
        }
        CHECK(has_el);
        CHECK(r.rfind("sat-eqmod\n", 0) == 0);
        CHECK(r.compare(r.size() - 7, 7, "RPRT 0\n") == 0);
    }

    TEST("multi-client: two live connections interleaved; stop() wakes both");
    {
        FakeHandler h2;
        RotctldServer srv(0, h2);
        srv.start();
        int port = srv.bound_port();
        auto cli = [port]() {
            int s = ::socket(AF_INET, SOCK_STREAM, 0);
            sockaddr_in a{};
            a.sin_family = AF_INET;
            a.sin_port = htons((uint16_t)port);
            a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            if (::connect(s, (sockaddr*)&a, sizeof(a)) != 0) { ::close(s); return -1; }
            return s;
        };
        auto xfer = [](int s, const std::string& cmd) {
            if (::send(s, cmd.data(), cmd.size(), MSG_NOSIGNAL) <= 0) return std::string();
            char buf[256];
            ssize_t n = ::recv(s, buf, sizeof(buf), 0);
            return n <= 0 ? std::string() : std::string(buf, (size_t)n);
        };
        int a = cli(), b = cli();
        CHECK(a > 0 && b > 0);
        // both get valid position replies concurrently, GT2-poll style
        for (int i = 0; i < 3; ++i)
        {
            std::string ra = xfer(a, "p\n"), rb = xfer(b, "p\n");
            std::istringstream ia(ra), ib(rb);
            double az, el;
            ia >> az >> el;
            CHECK_NEAR(az, 12.5, 1e-3);
            ib >> az >> el;
            CHECK_NEAR(el, 67.25, 1e-3);
        }
        // a P from one client is served while the other stays connected
        CHECK_EQ(xfer(b, "P 45 30\n"), std::string("RPRT 0\n"));
        CHECK_NEAR(h2.set_az, 45.0, 1e-9);
        CHECK_EQ(xfer(a, "p\n").find("12.5000"), std::string::size_type(0));
        // stop() with BOTH clients still connected must join (no hang/crash)
        srv.stop();
        ::close(a);
        ::close(b);
    }

    TEST("handler exception -> RPRT -1, connection survives (no thread death)");
    {
        ThrowingHandler th;
        RotctldServer srv(0, th);
        srv.start();
        int port = srv.bound_port();
        int s = ::socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons((uint16_t)port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        CHECK(::connect(s, (sockaddr*)&a, sizeof(a)) == 0);
        for (int i = 0; i < 3; ++i) // faults repeat: must stay alive every time
        {
            const char* cmd = "p\n";
            ::send(s, cmd, 2, MSG_NOSIGNAL);
            char buf[32] = {};
            ssize_t n = ::recv(s, buf, sizeof(buf) - 1, 0);
            CHECK(n > 0);
            CHECK_EQ(std::string(buf, n > 0 ? (size_t)n : 0), std::string("RPRT -1\n"));
        }
        ::close(s);
        srv.stop();
    }

    TEST("client observer: live-count transitions, last client gone fires");
    {
        FakeHandler h3;
        std::mutex cm;
        std::vector<int> seen;
        RotctldServer srv(0, h3);
        srv.set_client_observer([&](int n) {
            std::lock_guard<std::mutex> lk(cm);
            seen.push_back(n);
        });
        srv.start();
        int port = srv.bound_port();
        auto cli = [port]() {
            int s = ::socket(AF_INET, SOCK_STREAM, 0);
            sockaddr_in a{};
            a.sin_family = AF_INET;
            a.sin_port = htons((uint16_t)port);
            a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
            if (::connect(s, (sockaddr*)&a, sizeof(a)) != 0) { ::close(s); return -1; }
            return s;
        };
        auto seen_last = [&]() {
            std::lock_guard<std::mutex> lk(cm);
            return seen.empty() ? -1 : seen.back();
        };
        auto wait_for = [&](int want) {
            for (int i = 0; i < 300 && seen_last() != want; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            CHECK_EQ(seen_last(), want);
        };
        int a = cli();
        wait_for(1);
        int b = cli();
        wait_for(2);
        // one client closing while the other lives must NOT read as "gone"
        ::close(a);
        wait_for(1);
        // stop() with the last client still connected must reap it (count 0)
        srv.stop();
        wait_for(0);
        ::close(b);
    }

    return testkit::summary("rotctld");
}
