// drive_dryrun: rehearsal harness for consented-motion drive experiments on
// the production Tracker + simulator plant (metal-plant knobs as in the
// replay matrix). Push a target, settle, drop client, stale-halt, re-home
// self-push, and grade the AZ-FIRST cord-unwind law: leg 1 carries RA home
// with the ledger pinned, leg 2 unwinds DEC along the datum meridian;
// floor-safety, narratives, :K census, arrival line, no latches.
// ADOPTED WORKFLOW (2026-11-22): dry-run every planned live motion here
// BEFORE commanding it — the first use predicted a live dive angle to 1 deg
// and settle pose to 0.1 deg, and caught a handover-storm law defect
// (waypoint mode) that never reached metal.
//
//   g++ -O2 -I src tools/analysis/drive_dryrun.cpp build/libsateq.a -o /tmp/dd
//   /tmp/dd                       # default target az 210 el 45, direct
//   /tmp/dd 230 45                # drive that pose, then auto-home
//   /tmp/dd 230 45 270 70         # two-stage via waypoint (REHEARSED: from
//                                 # deep-canonical waypoints this STORMS —
//                                 # see AGENTS handover-storm backlog)
#include "sateq/simulator.h"
#include "sateq/tracker.h"
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <vector>
using namespace sateq;
using SC = std::chrono::steady_clock;
using UC = std::chrono::system_clock;

struct Fixture {
    std::shared_ptr<SimulatorTransport> sim;
    std::shared_ptr<Board> board;
    TrackerConfig cfg;
    Tracker tr;
    SC::time_point t0;
    UC::time_point u0;
    SC::duration vt{0};
    std::vector<std::string> log;
    explicit Fixture()
        : sim(std::make_shared<SimulatorTransport>(sim_heq5())),
          board(std::make_shared<Board>(sim)),
          tr(*board, bench_cfg()), t0(SC::now()), u0(UC::now())
    {
        tr.set_log_sink([this](const std::string& s) { log.push_back(s); });
        // metal-plant knobs (replay-matrix storm set)
        for (char ax : {'1', '2'})
        {
            sim->set_coast(ax, 0.3);
            sim->set_accel(ax, 1.0);
            sim->set_slow_ramp(ax, 25.0);
        }
        board->connect();
    }
    static TrackerConfig bench_cfg()
    {
        TrackerConfig c;
        c.orientation = Orientation::AltAzLevel;
        c.home_az_deg = 90.0;
        c.home_el_deg = 0.0;
        c.site = LatLon{44.95, -93.28};
        c.ra_sign = -1.0;                      // bench sirius profile
        c.safety.max_ra_travel_deg = 115.0;
        c.safety.max_dec_travel_deg = 180.0;
        c.rehome_idle_s = 60.0;
        c.high_coast_s = 0.3;
        c.high_gate_err_deg = 2.5;
        return c;
    }
    void cal() { tr.calibrate(u0, t0); }
    void advance(double s)
    {
        vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(s));
        sim->advance(s);
    }
    SC::time_point mono() { return t0 + vt; }
    UC::time_point utc() { return u0 + std::chrono::duration_cast<UC::duration>(vt); }
    void target(double az, double el) { tr.set_target_at(az, el, mono()); }
};

static const double SPS = 9024000.0 / 360.0;
static double ledger(Fixture& f, unsigned long datum_de)
{
    long d = (long)((f.sim->sim_position('2') & 0xFFFFFFu) - (datum_de & 0xFFFFFFu));
    if (d > 0x7FFFFF) d -= 0x1000000;
    if (d < -0x800000) d += 0x1000000;
    return (double)d / SPS;
}
static double ra_off(Fixture& f, unsigned long datum_ra)
{
    long d = (long)((f.sim->sim_position('1') & 0xFFFFFFu) - (datum_ra & 0xFFFFFFu));
    if (d > 0x7FFFFF) d -= 0x1000000;
    if (d < -0x800000) d += 0x1000000;
    return -f.cfg.ra_sign * (double)d / SPS;   // angle space (ra_sign-aware)
}

int main(int argc, char** argv)
{
    double tgt_az = argc >= 3 ? atof(argv[1]) : 210.0;
    double tgt_el = argc >= 3 ? atof(argv[2]) : 45.0;
    bool two_stage = argc >= 5;
    double wp_az = two_stage ? atof(argv[3]) : 0, wp_el = two_stage ? atof(argv[4]) : 0;
    std::printf("target %.1f %.1f  mode: %s\n", tgt_az, tgt_el,
                two_stage ? "two-stage" : "direct");
    Fixture f;
    f.cal();
    unsigned long d_ra = f.board->position('1'), d_de = f.board->position('2');
    std::printf("mode: %s  datum ledger %.1f\n", two_stage ? "two-stage" : "direct",
                ledger(f, d_de));

    double t = 0, min_el_A = 1e9, min_el_C = 1e9;
    int kc = 0, stable = 0;
    bool settled = false, halted = false, arrived = false;
    auto tick = [&](void (*)(Fixture&, double), double phase)
    {
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
    };
    (void)tick;

    // ---- phase A: drive to wound pose (2 s GT2-style re-push cadence) ----
    HsCoord p{};
    double last_push = -99;
    for (int i = 0; i < 9000 && !settled; ++i)
    {
        double az = tgt_az, el = tgt_el;
        if (two_stage)
        {
            p = f.tr.current_pointing(f.utc());
            double dist_wp = std::hypot(wp_az - p.az_deg, wp_el - p.el_deg);
            if (dist_wp > 1.5) { az = wp_az; el = wp_el; }
        }
        if (t - last_push > 2.0) { f.target(az, el); last_push = t; }
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        t += 0.1;
        p = f.tr.current_pointing(f.utc());
        if (p.el_deg < min_el_A) min_el_A = p.el_deg;
        auto s = f.tr.snapshot();
        if (std::hypot(p.az_deg - tgt_az, p.el_deg - tgt_el) < 0.3 && !s.halted)
        {
            if (++stable > 30) settled = true;
        }
        else stable = 0;
    }
    std::printf("A: settle t=%.0f az %.2f el %.2f min_el %.2f ledger %.1f ra_off %.1f halted=%d\n",
                t, p.az_deg, p.el_deg, min_el_A, ledger(f, d_de), ra_off(f, d_ra),
                (int)f.tr.snapshot().halted);

    // ---- phase B: drop client, run 200 s (stale-halt + rehome trigger) ----
    for (int i = 0; i < 2000; ++i)
    {
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        t += 0.1;
        if (f.tr.snapshot().halted && !halted)
        {
            halted = true;
            std::printf("B: HALTED at t=%.0f\n", t);
        }
        if (halted && i % 50 == 0)
            std::printf("B t=%4.0f ra_off %+7.1f ledger %+7.1f el %+6.2f\n",
                        t, ra_off(f, d_ra), ledger(f, d_de),
                        f.tr.current_pointing(f.utc()).el_deg);
    }
    // ---- phase C: home flight (already inside the 200 s above; continue) ----
    for (int i = 0; i < 9000; ++i)
    {
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        t += 0.1;
        p = f.tr.current_pointing(f.utc());
        if (p.el_deg < min_el_C) min_el_C = p.el_deg;
        if (i % 50 == 0)
            std::printf("C t=%4.0f ra_off %+7.1f ledger %+7.1f el %+6.2f\n",
                        t, ra_off(f, d_ra), ledger(f, d_de), p.el_deg);
        for (auto& l : f.log)
            if (l.find("arrived at datum") != std::string::npos) arrived = true;
        if (arrived) break;
    }
    std::printf("C: min_el %.2f  final az %.2f el %.2f  ledger %.1f ra_off %.1f arrived=%d\n",
                min_el_C, p.az_deg, p.el_deg, ledger(f, d_de), ra_off(f, d_ra), (int)arrived);
    auto s = f.tr.snapshot();
    std::printf("halted=%d latched=%d travel_returning=%d\n",
                (int)s.halted, 0, (int)s.travel_returning);
    for (auto& c : f.sim->command_log())
        if (c.size() >= 2 && c[0] == ':' && c[1] == 'K') ++kc;
    std::printf(":K count: %d\n", kc);
    std::printf("--- narratives ---\n");
    for (auto& l : f.log)
        std::printf("  %s\n", l.c_str());
    return 0;
}
