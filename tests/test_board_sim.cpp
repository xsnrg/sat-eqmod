// Board protocol client against the in-process simulator. Every expectation
// here mirrors LIVE traffic recorded 2026-11-15 from both real boards.
#include "testkit.h"
#include "sateq/board.h"
#include "sateq/simulator.h"

#include <chrono>
#include <deque>
#include <memory>
#include <optional>
#include <thread>

using namespace sateq;

static std::shared_ptr<SimulatorTransport> g_sim;

static Board make_board(SimProfile prof, std::shared_ptr<SimulatorTransport>* keep = nullptr)
{
    auto t = std::make_shared<SimulatorTransport>(std::move(prof));
    if (keep) *keep = t;
    return Board(t);
}

// Scripted transport for --debug trace forensics: replays a canned list of
// read_frame outcomes (nullopt = nothing arrived / timeout), so the trace's
// retry, timeout and garbage-frame records can be pinned without a sim.
struct ScriptedTransport : Transport {
    std::deque<std::optional<std::string>> script;
    void open() override {}
    void close() override {}
    bool is_open() const override { return true; }
    void write(const std::string&) override {}
    std::optional<std::string> read_frame(int) override
    {
        if (script.empty()) return std::nullopt;
        auto r = script.front();
        script.pop_front();
        return r;
    }
    void flush_input() override {}
};

int main()
{
    TEST("HEQ5 handshake golden (Sirius live: =020401, :q NAK)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        CHECK_EQ(b.mount_code(), 0x01);
        CHECK(!b.probe_eq8_features()); // live Sirius answered "!0"
        auto log = g_sim->command_log();
        CHECK_EQ(log[0], std::string(":e1"));
        CHECK_EQ(log[1], std::string(":q1010000"));
    }

    TEST("EQ8 handshake golden (HDX-110 live: =020B04, :q ACK)");
    {
        auto b = make_board(sim_eq8());
        b.connect();
        CHECK_EQ(b.mount_code(), 0x04);
        CHECK(b.probe_eq8_features());
        CHECK_EQ(b.home_sensor(AxisRA), 0x7BAAu); // EQ8 :d accepted
    }

    TEST("inquire_axis live Sirius constants");
    {
        auto b = make_board(sim_heq5());
        b.connect();
        auto info = b.inquire_axis(AxisRA);
        CHECK_EQ(info.steps_360, 9024000u);
        CHECK_EQ(info.steps_worm, 64935u);
        CHECK_EQ(info.hs_ratio, 16u);
        // and as the wire said: :a payload "00B289", :b payload "A7FD00"
    }

    TEST("position get/set + :F arms without disturbing the counter (live 2026-11-16)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        CHECK_EQ(b.position(AxisRA), 0x80u); // sim default, live "=000080"
        b.set_position(AxisDE, 0x1234);
        CHECK_EQ(b.position(AxisDE), 0x1234u);
        CHECK(!b.status(AxisRA).initialized); // live: unarmed at power-on (":f" d3=0)
        b.initialize(AxisRA);                 // arms the axis (live: ACK)
        auto st = b.status(AxisRA);
        CHECK(st.initialized);                // live: ":f" digit3 flips 0->1
        CHECK_EQ(b.position(AxisRA), 0x80u);  // live: :F left the counter alone
    }

    TEST("status bits while slewing (wire-accurate, live goldens)");
    {
        // live ":f" digits: d1 bit0 slew, bit1 backward, bit2 high-speed;
        // d2 bit0 running; d3 bit0 initialized. Goldens: idle "100",
        // fwd "111", back "311", hi fwd "511", hi back "711".
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        auto idle = b.status(AxisRA);
        // golden "100": digit1=1 (idle carries the slew digit), not running,
        // digit3=0 (counters NOT yet initialized via :F)
        CHECK(!idle.initialized);
        CHECK(!idle.running);
        CHECK(!idle.backward);
        CHECK(!idle.high_speed);

        b.set_motion_mode(AxisRA, SLEW_LOW, BACKWARD);
        b.set_step_period(AxisRA, 1000);
        b.start_motion(AxisRA);
        auto run = b.status(AxisRA);
        CHECK(run.running);
        CHECK(run.backward);
        CHECK(run.slew_mode);
        CHECK(!run.high_speed);

        // live rule: :G is NAK'd while running, so stop before re-mode
        b.stop_axis(AxisRA, true);
        b.set_motion_mode(AxisRA, SLEW_HIGH, FORWARD);
        b.set_step_period(AxisRA, 1000);
        b.start_motion(AxisRA);
        auto hi = b.status(AxisRA);
        CHECK(hi.running);
        CHECK(hi.high_speed);
        CHECK(!hi.backward);

        b.stop_axis(AxisRA, true); // :L
        auto stopped = b.status(AxisRA);
        CHECK(!stopped.running);
    }

    TEST(":f raw wire goldens (live payloads; digit3 = :F-armed flag)");
    {
        SimulatorTransport sim(sim_heq5());
        sim.open();
        auto x = [&](const std::string& cmd) -> std::string {
            sim.write(cmd + "\r");
            auto f = sim.read_frame(100);
            return f ? *f : std::string("<none>");
        };
        CHECK_EQ(x(":f1"), std::string("=100")); // power-on idle, unarmed
        CHECK_EQ(x(":J1"), std::string("!4"));   // live NAK 4: axis not armed
        x(":F1");
        CHECK_EQ(x(":f1"), std::string("=101")); // armed, still idle
        x(":G111"); x(":I10003E8"); x(":J1");
        CHECK_EQ(x(":f1"), std::string("=311")); // low-speed backward
        x(":L1");
        CHECK_EQ(x(":f1"), std::string("=301")); // live: mode digit persists, only running clears
        x(":G110"); x(":J1");
        CHECK_EQ(x(":f1"), std::string("=111")); // low-speed forward
        x(":L1"); x(":G130"); x(":J1");
        CHECK_EQ(x(":f1"), std::string("=511")); // high-speed forward
        x(":L1"); x(":G131"); x(":J1");          // mode change needs a stop first
        CHECK_EQ(x(":f1"), std::string("=711")); // high-speed backward
        x(":L1");
        CHECK_EQ(x(":f1"), std::string("=701")); // idle, high+back mode bits retained
    }

    TEST("running-axis wire rules (live NAK-2 semantics, raw wire)");
    {
        SimulatorTransport sim(sim_heq5());
        sim.open();
        auto x = [&](const std::string& cmd) -> std::string {
            sim.write(cmd + "\r");
            auto f = sim.read_frame(100);
            return f ? *f : std::string("<none>");
        };
        x(":F1");
        x(":G110"); x(":I10003E8"); x(":J1");
        CHECK_EQ(x(":G110"), std::string("!2"));      // :G while running, even identical
        CHECK_EQ(x(":I10002D39"), std::string("="));  // :I while running LOW: legal
        x(":L1");
        x(":G130"); x(":I1000078"); x(":J1");         // running in high speed
        CHECK_EQ(x(":G130"), std::string("!2"));
        CHECK_EQ(x(":I1000079"), std::string("!2"));  // :I while running HIGH: NAK
        x(":K1");
        CHECK_EQ(x(":G130"), std::string("="));       // stopped again: legal
        x(":L1");
    }

    TEST("slew_rate obeys running rules on the wire (board level)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        auto writes_only = [&]() {
            std::vector<std::string> w;
            for (const auto& f : g_sim->command_log())
                if (is_query_command(f.size() > 1 ? f[1] : ' '))
                    continue;
                else
                    w.push_back(f);
            return w;
        };
        b.slew_rate(AxisRA, 50.0); // cold axis: full :G/:I/:J (+ lazy :F)
        CHECK(!b.take_stop_restart(AxisRA));

        g_sim->clear_log();
        b.slew_rate(AxisRA, 50.0); // identical program, axis running -> no writes
        CHECK(writes_only().empty());

        g_sim->clear_log();
        b.slew_rate(AxisRA, 60.0); // low-speed running, in-place :I only
        {
            auto w = writes_only();
            CHECK_EQ(w.size(), 1u);
            CHECK(!w.empty() && w[0].rfind(":I", 0) == 0);
        }
        CHECK(!b.take_stop_restart(AxisRA));

        g_sim->clear_log();
        b.slew_rate(AxisRA, -40.0); // direction flip while running: K..G..J
        {
            auto w = writes_only();
            int i_k = -1, i_g = -1, i_j = -1;
            for (size_t i = 0; i < w.size(); ++i)
            {
                if (w[i] == ":K1" && i_k < 0) i_k = static_cast<int>(i);
                if (w[i].rfind(":G", 0) == 0 && i_g < 0) i_g = static_cast<int>(i);
                if (w[i] == ":J1" && i_j < 0) i_j = static_cast<int>(i);
            }
            CHECK(i_k >= 0 && i_g > i_k && i_j > i_g);
        }
        CHECK(b.take_stop_restart(AxisRA));

        b.slew_rate(AxisRA, 300.0); // high-speed start (axis running back: K-stop-restart)
        (void)b.take_stop_restart(AxisRA);
        g_sim->clear_log();
        b.slew_rate(AxisRA, 380.0); // high-speed period change -> stop/restart, never :I
        {
            auto w = writes_only();
            bool has_k = false, has_j = false, has_i_after_k = false;
            bool saw_k = false;
            for (const auto& f : w)
            {
                if (f == ":K1") has_k = true;
                if (f == ":J1") has_j = true;
                if (f.rfind(":I", 0) == 0 && saw_k) has_i_after_k = true;
                if (f == ":K1") saw_k = true;
            }
            CHECK(has_k && has_j && has_i_after_k);
        }
        CHECK(b.take_stop_restart(AxisRA));
        b.stop_axis(AxisRA, true);
    }

    TEST("Board auto-arms at start_motion (:F before :J on the wire)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        b.slew_rate(AxisRA, 20.0); // first-ever motion on a fresh sim board
        auto log = g_sim->command_log();
        int i_f = -1, i_j = -1;
        for (size_t i = 0; i < log.size(); ++i)
        {
            if (log[i] == ":F1" && i_f < 0) i_f = static_cast<int>(i);
            if (log[i] == ":J1" && i_j < 0) i_j = static_cast<int>(i);
        }
        CHECK(i_f >= 0);                       // we sent :F
        CHECK(i_j >= 0 && i_f < i_j);          // armed before starting
        b.stop_axis(AxisRA, true);
    }

    TEST("board power-cycle mid-session: NAK-4 on :J re-arms once automatically");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        b.slew_rate(AxisRA, 20.0);
        b.stop_axis(AxisRA, true);
        g_sim->clear_armed();          // board loses power and comes back
        b.slew_rate(AxisRA, 20.0);     // stale arming cache -> NAK 4 -> retry
        CHECK(g_sim->sim_running(AxisRA));
        b.stop_axis(AxisRA, true);
        int n_f = 0;
        for (const auto& f : g_sim->command_log())
            if (f == ":F1")
                ++n_f;
        CHECK_EQ(n_f, 2);              // initial arm + re-arm after the NAK
    }

    TEST("high-speed motion mode bits (:G 3x)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        b.set_motion_mode(AxisRA, SLEW_HIGH, FORWARD);
        auto log = g_sim->command_log();
        bool saw_g = false;
        for (auto& c : log)
            if (c == ":G130") saw_g = true; // mode '3' dir '0'
        CHECK(saw_g);
    }

    TEST("slew_rate animates simulator position at physical rate");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        b.slew_rate(AxisRA, 10.0); // 10x: period 62 us, exactly representable
        uint32_t p0 = b.position(AxisRA);
        g_sim->advance(10.0);
        uint32_t p1 = b.position(AxisRA);
        // expected steps: 10x * (steps_360/STELLAR_DAY) steps/s * 10s
        double exp_steps = 10.0 * (9024000.0 / STELLAR_DAY_SEC) * 10.0; // ≈ 10,470
        double got = (int64_t)p1 - (int64_t)p0;
        CHECK_NEAR(got / exp_steps, 1.0, 0.02);
    }

    TEST("slew_rate backward decreases position");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        // start well above zero: 50x for 5 s ≈ 26,200 steps back, no wrap
        b.set_position(AxisDE, 100000);
        b.slew_rate(AxisDE, -50.0);
        uint32_t p0 = b.position(AxisDE);
        g_sim->advance(5.0);
        uint32_t p1 = b.position(AxisDE);
        CHECK((int64_t)p1 < (int64_t)p0);
    }

    TEST("rate limits enforced client-side (board max 800x)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        CHECK_THROWS(b.slew_rate(AxisRA, 1000.0), BoardError);
        CHECK_THROWS(b.slew_rate(AxisRA, 0.01), BoardError);
    }

    TEST("retry on silence then success (transport blip)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        g_sim->silence_next('j', 2); // board misses first two :j polls
        CHECK_EQ(b.position(AxisRA), 0x80u);
        auto log = g_sim->command_log();
        int js = 0;
        for (auto& c : log)
            if (c == ":j1") ++js;
        CHECK_EQ(js, 3); // 2 silent + 1 ok
    }

    TEST("persistent silence -> BoardError timeout");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        g_sim->silence_next('j', 10);
        CHECK_THROWS(b.position(AxisRA), BoardError);
    }

    TEST("truncated :j payload retried (live crash regression 2026-11-16)");
    {
        // Real wire byte-loss made "=1008\r": a valid ACK dispatch accepts
        // but too short for revu24. It escaped Board::position with no
        // retry and killed the daemon via an uncaught p-poll exception.
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        g_sim->truncate_next('j', 1); // one short reply, then clean
        CHECK_EQ(b.position(AxisRA), 0x80u);
        g_sim->truncate_next('j', 20); // never completes -> still fatal
        CHECK_THROWS(b.position(AxisRA), BoardError);
    }

    TEST("NAK surfaces as BoardError::Nak with code");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        g_sim->fail_next('a', 1);
        bool nak = false;
        try { b.inquire_axis(AxisRA); }
        catch (const BoardError& e)
        {
            nak = e.kind == BoardError::Nak && e.nak_code == 0;
        }
        CHECK(nak);
    }

    TEST("stop_all issues :K (or :L) on both axes");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        g_sim->clear_log();
        b.stop_all(true);
        auto log = g_sim->command_log();
        CHECK_EQ(log.size(), 2u);
        if (log.size() == 2)
        {
            CHECK_EQ(log[0], std::string(":L1"));
            CHECK_EQ(log[1], std::string(":L2"));
        }
    }

    TEST("write gate: queries flow, writes are queued not sent (bench consent)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        b.block_writes(true);
        CHECK(b.writes_blocked());

        // Queries still reach the board normally
        CHECK_EQ(b.mount_code(), 0x01);
        const uint32_t p0 = b.position(AxisRA);

        // A full "slew" sequence: every write ACKed, none delivered
        b.set_motion_mode(AxisRA, SLEW_LOW, FORWARD);
        b.set_step_period(AxisRA, 1000);
        b.start_motion(AxisRA);
        g_sim->advance(2.0);
        CHECK_EQ(b.position(AxisRA), p0);        // axis never moved
        CHECK(!g_sim->sim_running(AxisRA));

        // Wire saw queries only
        for (const auto& f : g_sim->command_log())
            CHECK(is_query_command(f.size() > 1 ? f[1] : ' '));

        // Held frames carry the exact bytes they WOULD have sent
        const auto held = b.held_frames();
        bool has_i = false, has_g = false, has_j = false;
        for (const auto& f : held)
        {
            if (f.rfind(":I", 0) == 0) has_i = true;
            if (f.rfind(":G", 0) == 0) has_g = true;
            if (f.rfind(":J", 0) == 0) has_j = true;
            CHECK(!f.empty() && f[0] == ':' && f.back() != '\r');
        }
        CHECK(has_i && has_g && has_j);
        CHECK(b.held_frames().empty());          // take-on-read semantics

        // Even STOP is a write: held, not sent (nothing was moving anyway)
        b.stop_axis(AxisRA, /*instant=*/true);
        CHECK_EQ(b.held_count(), 1u);
        CHECK(b.held_frames()[0] == std::string(":L1"));

        // Gate released: writes flow and the axis actually moves
        b.block_writes(false);
        b.set_motion_mode(AxisRA, SLEW_LOW, FORWARD);
        b.set_step_period(AxisRA, 1000);
        b.start_motion(AxisRA);
        g_sim->advance(2.0);
        CHECK(g_sim->sim_running(AxisRA));
        CHECK(b.position(AxisRA) != p0);
        b.stop_axis(AxisRA, true);
    }

    TEST("probe_mount_code fails cleanly on unusable paths (auto-discovery leg)");
    {
        // --mount auto probes every profile port; a probe that throws would
        // take the daemon down mid-discovery, so every failure must be -1.
        CHECK(probe_mount_code("/nonexistent/port", 100) == -1);
        CHECK(probe_mount_code("/dev/null", 100) == -1);            // not a tty
        CHECK(probe_mount_code("/tmp/sateq_test_board.dir/port", 100) == -1); // missing dir
    }

    TEST("set_wall_clock: real sleeps animate axes (eqtool --sim wizard mode)");
    {
        auto sim = std::make_shared<SimulatorTransport>(sim_heq5());
        Board b(sim);
        b.connect();
        // default (virtual clock): a real 150 ms sleep must NOT move the axes
        b.slew_rate('1', 10.0);
        const uint32_t v0 = b.position('1');
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        CHECK_EQ(b.position('1'), v0);  // frozen: virtual clock idle
        b.stop_axis('1', true);

        // wall mode: switching mid-session must not teleport; then a real
        // 200 ms drive at 10x sidereal delivers ~0.2 s of issuance.
        sim->set_wall_clock(true);
        const uint32_t w0 = b.position('1');
        CHECK_EQ(w0, v0);  // re-anchored clock: no step teleport on enable
        b.slew_rate('1', 10.0);
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        const uint32_t w1 = b.position('1');
        b.stop_axis('1', true);
        const int32_t d = static_cast<int32_t>(w1 - w0);
        // 10x sidereal ~ 150.4 "/s; HEQ5 0.1436 "/step -> ~1047 steps/s ->
        // 200 ms ~ 209 steps. Allow generous timing slop but require motion.
        CHECK(d > 100 && d < 400);
    }

    // ---- backlash play model ------------------------------------------------
    // set_backlash(axis, S) puts gear slop between the issued step counter
    // (:j-visible, what the wire reports) and sim_physical(axis), the hidden
    // delivered position. Model = symmetric play of half-width h = S/2:
    // sustained motion lags the counter by h; a reversal then holds physical
    // still for the full S of counter travel before tracking again.
    TEST("backlash play model: physical lags h sustained, holds S across reversal");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        // default (backlash 0): physical IS the counter — every existing
        // test's view of the sim must be untouched.
        b.slew_rate('1', 10.0);
        g_sim->advance(1.0); // ~1047 steps
        b.stop_axis('1');
        CHECK(g_sim->sim_physical('1') == static_cast<double>(b.position('1')));
    }
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        g_sim->set_backlash('1', 1000.0); // h = 500 steps
        const uint32_t c0 = b.position('1');
        b.slew_rate('1', 10.0);
        g_sim->advance(1.0); // ~1047 issued: first 500 take up slack,
        b.stop_axis('1');    // then physical tracks 1:1
        const uint32_t c1 = b.position('1');
        const double p1 = g_sim->sim_physical('1');
        CHECK(c1 - c0 > 900);
        CHECK_NEAR(static_cast<double>(c1) - p1, 500.0, 30.0); // sustained lag = h
        // reversal: physical must HOLD while the counter backs up through the
        // full slop window (2h from the lag state == S issued steps)
        b.slew_rate('1', -10.0);
        g_sim->advance(0.45); // ~470 issued back, < S: nothing delivered yet
        const uint32_t c2 = b.position('1');
        const double p2 = g_sim->sim_physical('1');
        CHECK(static_cast<double>(c1) - static_cast<double>(c2) > 400);
        CHECK_NEAR(p2, p1, 30.0); // physical held
        g_sim->advance(1.6);      // total back ~2100 > S: re-engaged, now on
        const uint32_t c3 = b.position('1');   // the other flank: lag = -h
        const double p3 = g_sim->sim_physical('1');
        CHECK_NEAR(p1 - p3, static_cast<double>(c1) - static_cast<double>(c3) - 1000.0, 40.0);
        CHECK_NEAR(static_cast<double>(c3) - p3, -500.0, 30.0);
        // other axis untouched by axis '1' backlash
        CHECK(g_sim->sim_physical('2') == static_cast<double>(b.position('2')));
    }

    // ---- --debug wire trace -------------------------------------------------
    TEST("wire trace OFF: zero records, zero cost shape (dispatch unchanged)");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        CHECK(!b.tracing());
        (void)b.position(AxisRA);
        (void)b.position(AxisDE);
        CHECK(b.take_trace().empty()); // nothing recorded while off
    }

    TEST("wire trace: clean exchanges recorded raw, drain-on-take");
    {
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        b.set_trace(true);
        CHECK(b.tracing());
        CHECK_EQ(b.mount_code(), 0x01);          // :e1 query
        (void)b.position(AxisRA);                // :j1 query
        auto tr = b.take_trace();
        CHECK_EQ(tr.size(), 2u);
        if (tr.size() == 2)
        {
            CHECK_EQ(tr[0].tx, std::string(":e1"));
            CHECK(tr[0].rx.size() > 1 && tr[0].rx[0] == '='); // raw frame, '=' kept
            CHECK(tr[0].err.empty() && !tr[0].held && tr[0].nak == -1);
            CHECK_EQ(tr[1].tx, std::string(":j1"));
            CHECK_EQ(tr[1].rx, std::string("=800000")); // sim datum payload, verbatim
        }
        CHECK(b.take_trace().empty()); // take drains
        b.set_trace(false);            // toggling off discards stale records
        (void)b.position(AxisRA);
        CHECK(b.take_trace().empty());
    }

    TEST("wire trace: timeout retries, NAK throw, garbage frame, held writes");
    {
        auto t = std::make_shared<ScriptedTransport>();
        Board b(t);
        // no connect(): the scripted script has no handshake replies, and
        // dispatch only needs is_open() (ScriptedTransport is always open)
        b.set_trace(true);

        // attempt 1: nothing arrived (timeout); attempt 2: NAK-2 -> throw.
        t->script.push_back(std::nullopt);
        t->script.push_back(std::string("!2"));
        bool nak = false;
        try { b.dispatch('J', '1', "03"); }
        catch (const BoardError& e) { nak = e.kind == BoardError::Nak && e.nak_code == 2; }
        CHECK(nak);
        auto tr = b.take_trace();
        CHECK_EQ(tr.size(), 2u);
        if (tr.size() == 2)
        {
            CHECK_EQ(tr[0].tx, std::string(":J103")); // full frame incl. arg
            CHECK_EQ(tr[0].err, std::string("timeout"));
            CHECK(tr[0].rx.empty());
            CHECK_EQ(tr[1].err, std::string("nak"));
            CHECK_EQ(tr[1].nak, 2);
            CHECK_EQ(tr[1].rx, std::string("!2"));
        }

        // garbage frame: retried as malformed, final throw is Timeout; every
        // attempt lands in the trace with the RAW bytes (that is the point:
        // unparseable traffic must survive outside the exception text).
        t->script.push_back(std::string("\x01\xff" "zz"));
        bool timeout = false;
        try { b.dispatch('G', '1', "0100"); }
        catch (const BoardError& e) { timeout = e.kind == BoardError::Timeout; }
        CHECK(timeout);
        tr = b.take_trace();
        CHECK_EQ(tr.size(), 3u); // 1 malformed + 2 timeout (empty script)
        if (!tr.empty())
        {
            CHECK_EQ(tr[0].err, std::string("malformed"));
            CHECK_EQ(tr[0].rx, std::string("\x01\xff" "zz")); // verbatim
        }

        // write gate: a held frame is recorded as HELD and never reaches
        // the wire (no script entry consumed; rx empty, latency 0).
        b.block_writes(true);
        b.stop_axis(AxisRA, true); // :L1 is a WRITE -> held
        tr = b.take_trace();
        CHECK_EQ(tr.size(), 1u);
        if (!tr.empty())
        {
            CHECK_EQ(tr[0].tx, std::string(":L1"));
            CHECK(tr[0].held);
            CHECK_EQ(tr[0].err, std::string("held"));
            CHECK(tr[0].rx.empty());
        }
        b.block_writes(false);
        b.take_trace();
    }

    TEST(":K coast fixture (set_coast): counter coasts |rate|*s; :L instant; default off");
    {
        // The physical :K deceleration tail (live Sirius 600x: ~1.0 s /
        // ~18-19k steps ~ 0.74 deg) — the replay blind spot that hid the
        // AO-73 coast-pumped ping-pong. Fixture displacement = |rate|·s
        // exactly (v(t) = v0·(1 − t/(2s)), closed form), which is the
        // contract the tracker's high_coast_s landing model assumes.
        auto b = make_board(sim_heq5(), &g_sim);
        b.connect();
        g_sim->set_coast('1', 0.3);
        b.slew_rate(AxisRA, 600.0); // 600x: promoted HIGH band
        g_sim->advance(1.0);        // full rate (no accel model unless set)
        b.stop_all(false);          // :K on a running axis -> coast armed
        uint32_t p0 = b.position(AxisRA);
        g_sim->advance(2.0);        // one huge tick spanning the whole tail
        uint32_t p1 = b.position(AxisRA);
        const double rate_steps_s = 600.0 * 9024000.0 / STELLAR_DAY_SEC;
        const double exp_steps = rate_steps_s * 0.3;
        const double got = (int64_t)p1 - (int64_t)p0;
        // +~3%: one sim issuance chunk (~10 ms) runs at full rate before the
        // :K command is processed — board-polled-stops-have-the-same-lag.
        CHECK_NEAR(got / exp_steps, 1.0, 0.05);
        CHECK(!g_sim->sim_running('1')); // running bit still clears at once
        g_sim->advance(5.0);             // coast ended: dead rest
        CHECK_EQ(b.position(AxisRA), p1);
        // :L is instant on metal: no coast displacement
        b.slew_rate(AxisRA, 600.0);
        g_sim->advance(1.0);
        b.stop_all(true); // :L
        uint32_t q0 = b.position(AxisRA);
        g_sim->advance(2.0);
        CHECK_EQ((int64_t)b.position(AxisRA), (int64_t)q0);
        // default (fixture off, fresh sim): legacy instant-stop semantics
        auto b2 = make_board(sim_heq5(), &g_sim);
        b2.connect();
        b2.slew_rate(AxisRA, 600.0);
        g_sim->advance(1.0);
        b2.stop_all(false);
        uint32_t r0 = b2.position(AxisRA);
        g_sim->advance(2.0);
        CHECK_EQ((int64_t)b2.position(AxisRA), (int64_t)r0);
    }

    return testkit::summary("board_sim");
}
