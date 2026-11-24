// Closed-loop tracker tests against the in-process board simulator with a
// virtual clock: calibration, static hold, satellite following, horizon
// floor, staleness stop, cord-wrap latch, motor-stall skip, e-stop, and
// comms-failure latch.
#include "testkit.h"
#include "sateq/simulator.h"
#include "sateq/tracker.h"

#include <algorithm>
#include <memory>

using namespace sateq;
using SC = std::chrono::steady_clock;
using UC = std::chrono::system_clock;

namespace {

// Fixture owns sim + board + tracker; vt is the virtual clock offset that
// BOTH the simulator (advance) and the tracker step() timestamps use.
struct Fixture {
    std::shared_ptr<SimulatorTransport> sim;
    std::shared_ptr<Board> board;
    TrackerConfig cfg;
    Tracker tr;
    SC::time_point t0;
    UC::time_point u0;
    SC::duration vt{0};

    explicit Fixture(TrackerConfig c = {})
        : sim(std::make_shared<SimulatorTransport>(sim_heq5())),
          board(std::make_shared<Board>(sim)),
          cfg(prep(c)),
          tr(*board, cfg),
          t0(SC::now()),
          u0(UC::now())
    {
        board->connect();
    }
    static TrackerConfig prep(TrackerConfig c)
    {
        c.site = LatLon{44.95, -93.28};
        // The mechanical travel envelope models the LEVEL-mount stand
        // (Sirius bench): a polar pier winds RA continuously and relies on
        // the cumulative cord-wrap guard instead, so polar profiles set
        // travel = 0 (disabled) — mirror that here for the polar-fixture
        // default of the suite. The envelope has dedicated tests below.
        if (c.orientation == Orientation::Polar)
        {
            c.safety.max_ra_travel_deg = 0.0;
            c.safety.max_dec_travel_deg = 0.0;
        }
        // default home pose for the suite; explicit caller choices are kept
        if (c.home_az_deg == 180.0 && c.home_el_deg == 45.0)
        {
            c.home_az_deg = 180.0;
            c.home_el_deg = 60.0;
        }
        return c;
    }
    static TrackerConfig altaz()
    {
        TrackerConfig c;
        c.orientation = Orientation::AltAzLevel;
        c.home_az_deg = 90.0; // session start: tube level EAST (level bubble)
        c.home_el_deg = 0.0;
        return c;
    }
    void cal() { tr.calibrate(u0, t0); }
    void advance(double s)
    {
        vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(s));
        sim->advance(s);
    }
    SC::time_point mono(double at = -1) { return t0 + vt + std::chrono::microseconds((long)(at < 0 ? 0 : at * 1e6)); }
    UC::time_point utc() { return u0 + std::chrono::duration_cast<UC::duration>(vt); }
    // set target timestamped exactly at the current virtual time
    void target(double az, double el) { tr.set_target_at(az, el, mono()); }
    // does the log contain a start-motion (:J..) command?
    bool any_motion_start()
    {
        for (auto& c : sim->command_log())
            if (c.size() >= 2 && c[0] == ':' && c[1] == 'J')
                return true;
        return false;
    }
};

} // namespace

int main()
{
    // ---- 1. calibration reports the configured home pointing --------------
    TEST("calibrate: reported pointing == home az/el");
    {
        Fixture f;
        f.cal();
        HsCoord p = f.tr.current_pointing(f.utc());
        CHECK_NEAR(p.az_deg, 180.0, 1e-6);
        CHECK_NEAR(p.el_deg, 60.0, 1e-6);
    }

    // ---- 2. static target at current pointing: no motion ever --------------
    TEST("static target: zero axis motion");
    {
        Fixture f;
        f.cal();
        for (int i = 0; i < 30; ++i)
        {
            f.target(180.0, 60.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.any_motion_start());
        auto s = f.tr.snapshot();
        CHECK(s.tracking);
        CHECK(!s.halted);
        CHECK_NEAR(f.sim->sim_position('1'), 0x80u, 2u);
        CHECK_NEAR(f.sim->sim_position('2'), 0x80u, 2u);
    }

    // ---- 3. acquisition + tracking a moving "satellite" ---------------------
    TEST("follows moving target within 0.3 deg (steady state)");
    {
        Fixture f;
        f.cal();
        // az sweeps 180 -> 220 at 1 deg/s, el 60 -> 64 (LEO-like rates)
        double t = 0, max_err = 0;
        for (int i = 0; i < 400; ++i)
        {
            double az = 180.0 + t;        // 1 deg/s
            double el = 60.0 + 0.1 * t;
            f.tr.set_target_at(az, el, f.t0 +
                std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(t)));
            f.advance(0.1);
            t += 0.1;
            f.tr.step(f.utc(), f.mono());
            HsCoord p = f.tr.current_pointing(f.utc());
            double err = std::hypot(p.az_deg - az, p.el_deg - el);
            if (t > 5.0)
                max_err = std::max(max_err, err);
        }
        CHECK(max_err < 0.3);
        auto s = f.tr.snapshot();
        CHECK(s.tracking);
        CHECK(!s.halted);
        CHECK(f.any_motion_start());
    }

    // ---- 3b. GPredict-like 1 Hz staircase updates ---------------------------
    // The controller must glide the setpoint between updates (velocity
    // extrapolation): with raw staircase tracking the 10 Hz loop would chase
    // and always sit ~0.5-1 deg behind a 1 deg/s target.
    TEST("1 Hz target updates: extrapolation keeps steady-state error small");
    {
        Fixture f;
        f.cal();
        double t = 0, max_err = 0, max_t = 0;
        for (int i = 0; i < 400; ++i)
        {
            if (i % 10 == 0) // new az/el only once per virtual second
                f.tr.set_target_at(180.0 + t, 60.0 + 0.1 * t, f.t0 +
                    std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(t)));
            f.advance(0.1);
            t += 0.1;
            f.tr.step(f.utc(), f.mono());
            if (t > 6.0) // allow estimator warm-up
            {
                HsCoord p = f.tr.current_pointing(f.utc());
                double az = 180.0 + t, el = 60.0 + 0.1 * t;
                double e = std::hypot(p.az_deg - az, p.el_deg - el);
                if (e > max_err) max_err = e, max_t = t;
            }
        }
        // Value print: this metric sat at ~0.125 against a 0.15 threshold
        // pre-iteration-2 (pre-tuning HEAD: 0.1416); printing it keeps
        // regressions legible and lets gain sweeps parse the number out of
        // the run log. Threshold raised 0.15 -> 0.22 (iteration-2
        // 2026-11-19): the scenario crosses a genuine axis reversal (az
        // 208/el 63 meridian region) whose confirmation window delays the
        // turnaround by ~window x pattern-rate = 0.19 deg PEAK transient at
        // one instant (steady-state segments still ~0.13). That lag is
        // inherent to ANY reversal confirmation — the deliberate trade
        // that took FO-29 dense replay 315 -> 99 :K; a far smaller window
        // re-buys the wire storm instead (see tracker.h reversal_confirm_s).
        fprintf(stderr, "[m-1hz] %.4f at t=%.1f\n", max_err, max_t);
        CHECK(max_err < 0.22);
    }

    // ---- 4. below-horizon target: never moves --------------------------------
    TEST("target below track stop elevation: motors stay stopped");
    {
        Fixture f;
        f.cal();
        f.sim->clear_log();
        for (int i = 0; i < 20; ++i)
        {
            f.target(120.0, -30.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.any_motion_start());
        auto s = f.tr.snapshot();
        CHECK(s.halted);
        CHECK(s.halt_reason.find("below") != std::string::npos);
    }

    // ---- 4b. pass elevation policy: arm at −5 rising, stop at LOS ------------
    TEST("track gate: tracks in −5..0 band, holds below −5, re-arms for next pass");
    {
        Fixture f;
        f.cal();
        // below-horizon pose just ABOVE the stop gate: must command motion
        // (this is the pre-AOS arm: motors get up to speed below the horizon)
        f.sim->clear_log();
        for (int i = 0; i < 10; ++i)
        {
            f.target(120.0, -3.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.any_motion_start());
        CHECK(f.tr.snapshot().tracking);
        // converge, then the pass sets below the gate: stop + hold
        for (int i = 0; i < 400; ++i)
        {
            f.target(120.0, -3.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking);
        f.sim->clear_log();
        for (int i = 0; i < 5; ++i)
        {
            f.target(120.0, -6.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        auto s = f.tr.snapshot();
        CHECK(!s.tracking);
        CHECK(s.halted);
        CHECK(s.halt_reason.find("stop elevation") != std::string::npos);
        CHECK(!f.any_motion_start()); // stopped, never restarted below the gate
        // next pass rising into the band: re-arms by itself (no operator action)
        for (int i = 0; i < 5; ++i)
        {
            f.target(120.0, -4.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking);
        CHECK(!f.tr.snapshot().halted);
        CHECK(f.any_motion_start());
    }

    // ---- 4c. the two floors are independent layers ----------------------------
    TEST("defense-in-depth: mechanical floor blocks even if gate is lowered past it");
    {
        TrackerConfig c;
        c.safety.track_stop_el = -16; // below the −15 mechanical floor: profile
                                      // validation rejects this, but the floor
                                      // must hold on its own merits too
        Fixture f{c};
        f.cal();
        f.sim->clear_log();
        for (int i = 0; i < 10; ++i)
        {
            f.target(120.0, -15.5);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.any_motion_start());
        auto s = f.tr.snapshot();
        CHECK(s.halted);
        CHECK(s.halt_reason.find("mechanical") != std::string::npos);
    }

    // ---- 5. stale target stops the motors ------------------------------------
    TEST("stale rotctld target: stop and hold");
    {
        TrackerConfig c;
        c.target_stale_s = 5.0; // pin the timeout: exercise the stale-stop
                                 // path specifically (the shipped default
                                 // 90 s deliberately exceeds GT2 throttle
                                 // gaps, which would make this test's
                                 // 6 s gap "normal service")
        Fixture f{c};
        f.cal();
        for (int i = 0; i < 10; ++i)
        {
            f.target(200.0, 55.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        auto s1 = f.tr.snapshot();
        CHECK(s1.rate_ra_x != 0.0 || s1.rate_dec_x != 0.0); // moving
        f.advance(6.0); // client died: no fresh target beyond stale timeout
        f.tr.step(f.utc(), f.mono());
        auto s2 = f.tr.snapshot();
        CHECK(!s2.tracking);
        CHECK(s2.halted);
        CHECK_EQ(s2.rate_ra_x, 0.0);
        CHECK_EQ(s2.rate_dec_x, 0.0);
        CHECK(s2.halt_reason.find("stale") != std::string::npos);
        CHECK(!f.sim->sim_running('1'));
        CHECK(!f.sim->sim_running('2'));
    }

    // ---- 5b. last rotctld client disconnecting = stop -----------------------
    TEST("last client disconnect drops the target and halts; reconnect resumes");
    {
        Fixture f;
        f.cal();
        for (int i = 0; i < 10; ++i)
        {
            f.target(200.0, 55.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().rate_ra_x != 0.0 || f.tr.snapshot().rate_dec_x != 0.0);
        f.tr.on_clients_changed(0); // operator closed the socket (GT2 stop)
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        auto s = f.tr.snapshot();
        CHECK(s.halted);
        CHECK_EQ(s.rate_ra_x, 0.0);
        CHECK_EQ(s.rate_dec_x, 0.0);
        CHECK(s.halt_reason.find("no target") != std::string::npos);
        CHECK(!f.sim->sim_running('1'));
        CHECK(!f.sim->sim_running('2'));
        // reconnect + a fresh P must resume without any latch/reset dance
        f.tr.on_clients_changed(1);
        f.target(200.0, 55.0);
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        CHECK(!f.tr.snapshot().halted);
        // a second client dropping while one remains must NOT stop anything
        f.tr.on_clients_changed(1); // still one live
        f.target(200.0, 55.0);
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        CHECK(!f.tr.snapshot().halted);
    }

    // ---- 5c. dive routing + floor guard (RS-44 pass regression) -------------
    TEST("far low target: routed over transit el, never dived below horizon");
    {
        // Exact live scenario, 2026-11-16 RS-44 pass: altaz park az 90/el 0,
        // GT2 pre-aim target az 232.2/el 0. Unguarded axis-space convergence
        // arced the mount down to -58 deg elevation. The dive router now
        // handles this PROACTIVELY (climb to transit el, sweep, descend at
        // the target az): the acquisition must stay essentially above the
        // horizon and arrive without the reactive floor guard ever binding.
        // (Envelope OFF: after the 2026-11-19 stand discovery this target
        // — 232.2/0, SW horizon — is physically UNREACHABLE (would be
        // refused outright); this scenario predates it and grades the dive
        // router, not the wall. Envelope has its own tests below.)
        TrackerConfig rc44 = Fixture::altaz();
        rc44.safety.max_ra_travel_deg = rc44.safety.max_dec_travel_deg = 0.0;
        Fixture f(rc44);
        f.cal();
        double min_el = 90.0, guard_ticks = 0;
        double last_az = 90.0;
        for (int i = 0; i < 1400; ++i) // 140 s
        {
            f.target(232.2, 0.0); // client keeps pushing (target stays fresh)
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            min_el = std::min(min_el, s.current.el_deg);
            if (s.floor_guard) ++guard_ticks;
            last_az = s.current.az_deg;
        }
        CHECK(min_el > -2.0);            // routed at transit el: coast + the
                                         // final descent may dip a little
        CHECK(guard_ticks == 0);         // router pre-empts the guard
        CHECK(last_az > 200.0);          // and it still reaches the azimuth
    }
    TEST("floor guard backstop still holds with the router disabled");
    {
        // Same RS-44 scenario with routing off (route_az_min_deg = 0):
        // the reactive velocity-projection floor guard must still clip the
        // dive arc, hold above the floor, and slide toward the target.
        TrackerConfig c = Fixture::altaz();
        c.route_az_min_deg = 0.0; // disable the router
        c.safety.max_ra_travel_deg = c.safety.max_dec_travel_deg = 0.0; // as
        Fixture f(c);           // in routed variant: 232.2/0 predates the wall
        f.cal();
        double min_el = 90.0;
        bool guard_engaged = false;
        double last_az = 90.0;
        for (int i = 0; i < 1400; ++i) // 140 s
        {
            f.target(232.2, 0.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            min_el = std::min(min_el, s.current.el_deg);
            guard_engaged = guard_engaged || s.floor_guard;
            last_az = s.current.az_deg;
        }
        CHECK(guard_engaged);            // the arc WAS clipped (scenario bites)
        CHECK(min_el > -14.6);           // true floor -15, guard binds at -14,
                                         // ~0.9 deg dynamic undershoot observed
        CHECK(last_az > 120.0);          // and it still slid toward the target
    }

    // ---- 6. cord-wrap: cumulative RA rotation latches -------------------------
    TEST("cord-wrap: RA cumulative limit stops and latches");
    {
        Fixture f;
        f.cal();
        // Circle the sky around the pole at dec=+80 (circumpolar at lat 44.95,
        // min elevation ~35 deg): HA — and with it the RA axis — sweeps
        // continuously until the 540 deg cumulative budget runs out.
        // (Azimuth circles at fixed elevation do NOT accumulate RA: near the
        // zenith the axes only rock back and forth.)
        double t = 0;
        bool latched = false;
        std::string reason;
        for (int i = 0; i < 20000 && !latched; ++i)
        {
            double ha = std::fmod(2.0 * t, 360.0); // HA sweep 2 deg/s (~480x)
            HsCoord hs = ha_dec_to_hs(ha, 80.0, 44.95);
            f.tr.set_target_at(hs.az_deg, hs.el_deg,
                               f.t0 + std::chrono::duration_cast<SC::duration>(
                                          std::chrono::duration<double>(t)));
            f.advance(0.1);
            t += 0.1;
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.halted && s.halt_reason.find("unwrap") != std::string::npos)
            {
                latched = true;
                reason = s.halt_reason;
            }
        }
        CHECK(latched);
        CHECK(reason.find("RA cumulative") != std::string::npos);
        CHECK(t < 600.0); // 540 deg at ~2 deg/s must latch in ~5-6 min virtual
        // Latched: fresh valid targets must not restart motion (needs reset)
        f.sim->clear_log();
        for (int i = 0; i < 50; ++i)
        {
            f.target(90.0, 45.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.any_motion_start());
    }

    // ---- 7. motor stall (skip) detection ---------------------------------------
    TEST("stalled axis: skip deficit latches within ~1 s");
    {
        Fixture f;
        f.cal();
        // jam BOTH axes from the start: powered, answering commands, but the
        // step counters never advance — the only failure the serial counters
        // can ever see (physical missed steps are invisible to :j).
        f.sim->set_stalled('1', true);
        f.sim->set_stalled('2', true);
        bool latched = false;
        std::string reason;
        for (int i = 0; i < 300 && !latched; ++i)
        {
            f.target(195.0, 60.0); // 2° away: mount should power to close in
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.halted && s.halt_reason.find("skip") != std::string::npos)
            {
                latched = true;
                reason = s.halt_reason;
            }
        }
        CHECK(latched);
        CHECK(reason.find("skip") != std::string::npos);
        CHECK(!f.sim->sim_running('1'));
        CHECK(!f.sim->sim_running('2'));
    }

    TEST("board accel ramp after :J must not false-latch skip (live Sirius 2026-11-16)");
    {
        // The live board ramps its step-issue rate after :J (the :j counters
        // follow issuance); a +600x restart mid-acquisition false-latched
        // skip before skip_ramp_grace_s existed. Model the ramp and drive
        // that scenario: far acquisition (hundreds-x rates) on ramping axes.
        Fixture f;
        f.cal();
        f.sim->set_accel('1', 2.0);
        f.sim->set_accel('2', 2.0);
        bool latched = false;
        for (int i = 0; i < 600 && !latched; ++i)
        {
            f.target(195.0, 60.0); // 2 deg away: demands more than the ramp delivers
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            latched = s.halted && s.halt_reason.find("skip") != std::string::npos;
        }
        CHECK(!latched);
        // and it still converges on a ramping board
        for (int i = 0; i < 600; ++i)
        {
            f.target(195.0, 60.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        auto s = f.tr.snapshot();
        CHECK(!s.halted);
        HsCoord p = f.tr.current_pointing(f.utc());
        CHECK_NEAR(p.az_deg, 195.0, 0.3);
        CHECK_NEAR(p.el_deg, 60.0, 0.3);
    }

    TEST("slow-ramp band (55-104x) arms the long grace on entry, not just cold :J");
    {
        // classification boundaries (Phase-1 + live sweep 2026-11-17):
        SafetyConfig sc;
        CHECK_NEAR(cold_start_grace_s(sc, 62.0), sc.skip_ramp_grace_low_s, 1e-9);
        CHECK_NEAR(cold_start_grace_s(sc, -90.0), sc.skip_ramp_grace_low_s, 1e-9);
        CHECK_NEAR(cold_start_grace_s(sc, 50.0), sc.skip_ramp_grace_s, 1e-9);
        // HIGH band takes the COLD profile (blind + short ramp, AO-123
        // 2026-11-20): full authority at skip_cold_grace_high_s, not the
        // generic linear skip_ramp_grace_s.
        CHECK_NEAR(cold_start_grace_s(sc, 104.0), sc.skip_cold_grace_high_s, 1e-9);
        CHECK_NEAR(cold_start_grace_s(sc, 600.0), sc.skip_cold_grace_high_s, 1e-9);
        CHECK_NEAR(cold_start_grace_s(sc, 0.0), sc.skip_ramp_grace_s, 1e-9);

        // behavior: reproduce the live sweep's DEC starvation — the axis
        // runs LOW in the slow band (55-104x) while the board's issuance
        // transient delivers ~0.45 at entry, converging over 25 s (the
        // real latch accumulated 0.54 of 1.02 deg = 53 % deficit; the
        // sim floor is injected below the steady Phase-1 ratios to model
        // that cold-start transient). The old flat-3 s grace latched on
        // real metal at t=24 s; the band-aware 25 s grace must not.
        auto sweep_latches = [](double low_grace) -> bool
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.skip_ramp_grace_low_s = low_grace;
            c.safety.skip_ramp_grace_low_min_x = 55.0;
            // isolate the GRACE mechanism: this test predates the sustained
            // supply floor (RS-44) — expect full issuance as it always did.
            c.safety.skip_supply_slow_band = 1.0;
            // Cap below the 104x high-speed promotion: live the glide's
            // PI could boost past the band and self-heal the starvation;
            // held in-band (the real 62x DEC) the deficit accumulates.
            c.safety.max_rate_x_sidereal = 100.0;
            Fixture f(c);
            f.cal();
            f.sim->set_slow_ramp('1', 25.0, 0.45, 55.0);
            f.sim->set_slow_ramp('2', 25.0, 0.45, 55.0);
            for (int i = 0; i < 600; ++i) // 60 s
            {
                const double t = i * 0.1;
                f.target(90.0 + 0.30 * t, 0.30 * t);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                auto s = f.tr.snapshot();
                if (s.halted && s.halt_reason.find("skip") != std::string::npos)
                    return true;
            }
            return false;
        };
        CHECK(sweep_latches(3.0));   // pre-fix behavior: latched in-band
        CHECK(!sweep_latches(25.0)); // fix: band entry arms the long grace
    }

    TEST("slow-ramp band SUSTAINED supply floor: 100x dwell must not latch (RS-44)");
    {
        // Phase-1 read the 55-104x deficit as a TRANSIENT (short runs,
        // 15-25 s) and the skip model converges expected->1.0 with the
        // grace. Three sustained live encounters since show it is a FLOOR:
        // the RS-44 apex dwell 2026-11-20 latched DEC at [sent +100x band 1
        // grace 25s ramp_age 48s] with the ~0.8 deficit still growing long
        // after grace expiry — the vintage board simply loses timer ticks
        // at those step periods (vendor PDF: HIGH >128x is a different
        // mechanism; this band is LOW one-step-per-interrupt). Model it as
        // a per-band supply multiplier on the ISSUANCE expectation, keyed
        // off the sent program: a genuine jam still misses ~everything
        // against the discounted baseline and trips at the usual 20 %.
        auto latches = [](double supply) -> bool
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.max_rate_x_sidereal = 100.0; // pin the axis at the LOW
                                                   // ceiling (RS-44 DEC sat
                                                   // at +100x by design —
                                                   // mgate 90 kept it LOW)
            c.safety.skip_supply_slow_band = supply;
            Fixture f(c);
            f.cal();
            // sustained (converge time 1e9 s) in-band issue at 0.72 of
            // commanded — brackets the measured 0.78-0.80 sustained floor
            // from below for a robust legacy latch (same bracketing style
            // as the 0.45 cold-entry transient above; the 20 % ratio guard
            // makes a razor-thin 0.78-vs-1.0 deficit the razor's edge the
            // live RS-44 latch only cleared after 48 s).
            f.sim->set_slow_ramp('1', 1e9, 0.72, 55.0);
            f.sim->set_slow_ramp('2', 1e9, 0.72, 55.0);
            for (int i = 0; i < 2000; ++i) // 200 s dwell, 8x the 25 s grace
            {
                const double t = i * 0.1;
                f.target(90.0 + 0.30 * t, 0.30 * t);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                auto s = f.tr.snapshot();
                if (s.halted && s.halt_reason.find("skip") != std::string::npos)
                    return true;
            }
            return false;
        };
        CHECK(latches(1.0));   // legacy (expect full issuance): reproduces
                               // the RS-44 false-latch — 22 % deficit vs a
                               // converging-to-1.0 expectation never stops
                               // growing, and the 20 % ratio guard cannot
                               // save it.
        CHECK(!latches(0.85)); // fix: floor covers the measured 0.78-0.80
                               // dwell; deficit 8 % stays under the ratio
                               // guard forever.
    }

    TEST("HIGH :K restart starves issuance: blind window prevents latch (sweep-6)");
    {
        // profile boundaries: HIGH programs (and the promotion edge) take
        // the HIGH blind; LOW programs take the short LOW blind.
        SafetyConfig sc;
        CHECK_NEAR(restart_blind_s(sc, 600.0), sc.skip_restart_blind_high_s, 1e-9);
        CHECK_NEAR(restart_blind_s(sc, -104.0), sc.skip_restart_blind_high_s, 1e-9);
        CHECK_NEAR(restart_blind_s(sc, 103.9), sc.skip_restart_blind_low_s, 1e-9);
        CHECK_NEAR(restart_blind_s(sc, 0.0), sc.skip_restart_blind_low_s, 1e-9);

        // behavior: reproduce live sweep-6 (2026-11-18). A rate-capped dive
        // reverses an axis mid-slew: the forced :K stop/restart leaves the
        // board coasting + re-accelerating (sim: set_restart_blind, 2 s of
        // zero issuance), and with the old linear 0->1 grace the skip
        // detector accumulated the entire starvation as deficit — DEC
        // latched at ramp_age 2.5 of 3.0 s, [sent -600x band 2]. The blind
        // window must cover coast+re-accel so only post-blind ticks count.
        auto latches = [](double blind_high) -> bool
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.max_rate_x_sidereal = 300.0;
            c.safety.skip_restart_blind_high_s = blind_high;
            c.safety.skip_restart_grace_high_s = 2.5; // the window THIS test
                // predates: cold-profile grace 2.5 paired with the sweep-6
                // blind (skip_restart_grace_high_s now owns the HIGH restart
                // window; default 4.0, regression-tested below)
            Fixture f(c);
            f.cal();
            f.sim->set_restart_blind('1', 2.0);
            f.sim->set_restart_blind('2', 2.0);
            // dive HIGH away, then reverse the demand hard: forced :K
            for (int i = 0; i < 40; ++i)
            {
                f.target(150.0, 30.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            for (int i = 0; i < 500; ++i)
            {
                f.target(40.0, 0.0); // reversal: :K stop/restart + starvation
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                auto s = f.tr.snapshot();
                if (s.halted && s.halt_reason.find("skip") != std::string::npos)
                    return true;
            }
            return false;
        };
        CHECK(latches(0.0));  // pre-fix: the 2 s coast blind latched (sweep-6)
        CHECK(!latches(2.0)); // fix: blind window blanks the starvation
    }

    TEST("cold HIGH dive starves issuance: cold blind prevents latch (AO-123)");
    {
        // Live AO-123 acquisition 2026-11-20 03:23: the daemon idled at the
        // datum, GT2 pushed a target ~80 deg away, and the dive law went to
        // +600x HIGH on BOTH axes from standstill (cold :J). The board's
        // cold step-issuance is DEAD early — measured delivery 0.36 of
        // commanded at t=2.0 s vs the linear 0->1/3 s grace model's 0.67 —
        // and the growing deficit skip-latched DEC at ramp_age 2.0 of 3.0
        // ("sent +600x band 2 grace 3s blind 0.0s"). GT2 never sends `R`,
        // so the latch would have stranded the pass but for a manual `R`.
        // The cold profile (blind ~2 s, full authority by 2.5 s) matches
        // the measured delivery curve; Phase-1's ~1 s HIGH accel still
        // bounds the ramp end.
        auto cold_dive = [](double blind, double grace_high) -> bool
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.skip_cold_blind_high_s = blind;
            c.safety.skip_cold_grace_high_s = grace_high;
            Fixture f(c);
            f.cal();
            // cold (from-stopped) :J: zero issuance for 2.0 s, then the
            // commanded rate — mirrors the measured 0.36-at-2 s delivery.
            f.sim->set_cold_blind('1', 2.0);
            f.sim->set_cold_blind('2', 2.0);
            for (int i = 0; i < 400; ++i) // 40 s
            {
                f.target(170.0, 30.0); // far: dive saturates both axes HIGH
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                auto s = f.tr.snapshot();
                if (s.halted && s.halt_reason.find("skip") != std::string::npos)
                    return true;
            }
            return false;
        };
        // legacy model: blind 0 + grace_high 0 disables the cold profile,
        // falling back to the linear skip_ramp_grace_s the live latch fired on:
        CHECK(cold_dive(0.0, 0.0));
        CHECK(!cold_dive(2.0, 2.5)); // shipped cold profile: no latch
    }

    TEST("restart_grace_s: HIGH window strictly exceeds the HIGH blind");
    {
        // The 2026-11-22 skip DOUBLE-latch (03:06 refusal-return + 06:40
        // re-home rewind, both "grace 2s blind 3.0 ramp_age 2.7"): the
        // restart stamp paired skip_restart_blind_high_s (3.0 s, measured)
        // with the COLD profile's grace (2.5 s) — expectation snapped back
        // to full at 2.5 s while issuance is starved to 3.0 s, a
        // deterministic ~1.2 deg false deficit per HIGH restart at 600x
        // (analytic repro: latch at ramp_age 2.65, live: 2.7 — both latches
        // matched; operator eyes at the pose: tube exactly at model
        // pointing, so NOT mechanical). Fix: HIGH restarts own their window
        // (skip_restart_grace_high_s) and update_skip clamps any
        // blind >= grace pairing up to blind + 0.5 s ramp.
        SafetyConfig sc;
        CHECK(restart_grace_s(sc, 600.0) > restart_blind_s(sc, 600.0));
        CHECK_NEAR(restart_grace_s(sc, 600.0), sc.skip_restart_grace_high_s, 1e-9);
        CHECK_NEAR(restart_grace_s(sc, -104.0), sc.skip_restart_grace_high_s, 1e-9);
        // LOW/slow bands keep the cold band logic (their restart blind is
        // far inside those windows, no inversion possible):
        CHECK_NEAR(restart_grace_s(sc, 62.0), sc.skip_ramp_grace_low_s, 1e-9);
        CHECK_NEAR(restart_grace_s(sc, 10.0), sc.skip_ramp_grace_s, 1e-9);
        // shipped defaults carry the invariant too (a config edit that
        // inverts the pair is the regression this pins):
        CHECK(sc.skip_restart_grace_high_s > sc.skip_restart_blind_high_s);
    }

    TEST("HIGH restart at shipped defaults: 3.0 s starvation must NOT latch "
         "(2026-11-22 double-latch regression)");
    {
        auto restart_run = [](double sim_blind) -> bool
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.max_rate_x_sidereal = 300.0; // like sweep-6's dive cap
            Fixture f(c);                          // shipped skip defaults
            f.cal();
            f.sim->set_restart_blind('1', sim_blind);
            f.sim->set_restart_blind('2', sim_blind);
            for (int i = 0; i < 40; ++i)
            {
                f.target(150.0, 30.0); // dive HIGH away
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            for (int i = 0; i < 600; ++i) // 60 s of reversals: forced :K
            {                          // restarts + board starvation
                f.target(40.0, 0.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                auto s = f.tr.snapshot();
                if (s.halted && s.halt_reason.find("skip") != std::string::npos)
                    return true;
            }
            return false;
        };
        // sim starvation EXACTLY matching the shipped restart blind (3.0 s,
        // the measured real-board figure): the shipped window (blind 3.0 +
        // ramp to full at 4.0) blanks it completely. On the pre-fix pairing
        // (grace = cold 2.5 < blind 3.0, no clamp) this scenario latched
        // analytically at ramp_age 2.65 — reproducing both live latches.
        CHECK(!restart_run(3.0));
    }

    TEST("HIGH restart starvation BEYOND the window still latches (sensitivity)");
    {
        // The fix must not become a blanket blind: a board that starves
        // issuance past the full restart window (5.5 s vs the 4.0 s window
        // end) is a genuine stall and still trips the skip detector.
        TrackerConfig c = Fixture::altaz();
        c.safety.max_rate_x_sidereal = 300.0;
        Fixture f(c);
        f.cal();
        f.sim->set_restart_blind('1', 5.5);
        f.sim->set_restart_blind('2', 5.5);
        bool latched = false;
        for (int i = 0; i < 40; ++i)
        {
            f.target(150.0, 30.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        for (int i = 0; i < 900; ++i)
        {
            f.target(40.0, 0.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.halted && s.halt_reason.find("skip") != std::string::npos)
            {
                latched = true;
                break;
            }
        }
        CHECK(latched);
    }

    TEST("HIGH deadband 25 % bounds :K reprograms under a rate-capped dive");
    {
        // Live sweep-6: a dive pinned at the rate cap wobbled DEC -507x ->
        // -600x (18 %), and the old 10 % HIGH deadband re-paid the full
        // :K coast+blind cost on every swing: 153 events in minutes, ending
        // in the skip latch above. Same-scenario wire counts: the 25 %
        // deadband must strictly reduce stop/restart events vs 10 %.
        auto storm = [](double db) -> int
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.max_rate_x_sidereal = 300.0;
            c.high_reprogram_db = db;
            Fixture f(c);
            f.cal();
            for (int i = 0; i < 800; ++i) // 80 s of oscillating far target
            {
                const double t = i * 0.1;
                f.target(120.0 + 30.0 * std::sin(t * 0.5), 5.0 + 12.0 * std::sin(t * 0.33));
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            auto s = f.tr.snapshot();
            return s.stop_restart_ra + s.stop_restart_de;
        };
        const int n25 = storm(0.25), n10 = storm(0.10);
        CHECK(n25 > 0);
        CHECK(n10 >= 4 * n25 / 3); // the wobble band WAS the storm source:
        CHECK(n25 <= n10 * 3 / 4); // 25 % must cut >=25 % of the events
                                   // (measured 207 -> 131; the rest are the
                                   // scenario's genuine direction reversals)
    }

    // ---- 8. rotctld stop drives instant stop on both axes -----------------------
    TEST("rotctld stop: :L on both axes, state halted");
    {
        Fixture f;
        f.cal();
        f.target(200.0, 50.0);
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        f.sim->clear_log();
        f.tr.on_stop();
        auto log = f.sim->command_log();
        bool l1 = false, l2 = false;
        for (auto& c : log)
        {
            if (c == ":L1") l1 = true;
            if (c == ":L2") l2 = true;
        }
        CHECK(l1);
        CHECK(l2);
        CHECK(f.tr.snapshot().halted);
    }

    // ---- 9. board I/O failure latches ----------------------------------------------
    TEST("serial failure mid-tracking: stop + latch");
    {
        Fixture f;
        f.cal();
        for (int i = 0; i < 5; ++i)
        {
            f.target(200.0, 55.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        f.sim->silence_next('j', 99); // cable pulled
        f.target(200.0, 55.0);
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        auto s = f.tr.snapshot();
        CHECK(s.halted);
        CHECK(s.halt_reason.find("board I/O failure") != std::string::npos);
    }

    // ---- 10. reset + recalibrate re-arms tracking ------------------------------------
    TEST("reset after wrap latch + recalibrate re-arms tracking");
    {
        Fixture f;
        f.cal();
        f.tr.safety().trip("RA cumulative rotation ... manual unwrap required");
        f.target(181.0, 60.0);
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        CHECK(f.tr.snapshot().halted);
        f.tr.on_reset();
        // operator re-states reference where the mount physically stands
        f.tr.calibrate(f.utc(), f.mono());
        for (int i = 0; i < 40; ++i)
        {
            f.target(181.0, 60.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        HsCoord p = f.tr.current_pointing(f.utc());
        CHECK_NEAR(p.az_deg, 181.0, 0.3);
        CHECK_NEAR(p.el_deg, 60.0, 0.3);
    }

    // ---- 12. AltAzLevel orientation: session start EAST, static + moving ----
    TEST("altaz: calibrate reports EAST/level home pose");
    {
        Fixture f(Fixture::altaz());
        f.cal();
        HsCoord p = f.tr.current_pointing(f.utc());
        CHECK_NEAR(p.az_deg, 90.0, 1e-6);
        CHECK_NEAR(p.el_deg, 0.0, 1e-6);
    }

    TEST("altaz: static target at home: zero axis motion");
    {
        Fixture f(Fixture::altaz());
        f.cal();
        for (int i = 0; i < 50; ++i)
        {
            f.target(90.0, 0.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.any_motion_start());
    }

    TEST("altaz: tracks a meridional satellite-like sweep through S sky");
    {
        TrackerConfig mc = Fixture::altaz();
        mc.safety.max_ra_travel_deg = mc.safety.max_dec_travel_deg = 0.0;
        Fixture f(mc);      // W-set tail of the sweep leaves the stand box
                            // (2026-11-19); this scenario grades great-
                            // circle control quality (envelope: own tests)
        f.cal();
        // Great-circle pass (satellite apparent paths ARE great circles, and
        // near-constant angular rate ~0.6 deg/s like a fast LEO): rise due E,
        // transit due S at el=65, set due W. Sampled el 9..65..9, ~267 s.
        // A naive az/el sin() bulge instead demands >1000x sidereal axis
        // rates near the horizon (altaz RA-axis map amplifies az rate by
        // ~1/sin(el) there — real sats move slowly in az/el near horizon).
        const double OM = 0.6; // deg/s along the great circle
        const double T = 150.0 / OM;
        double max_err = 0, max_t = 0;
        for (double t = 0; t <= T; t += 0.1)
        {
            double s = (75.0 - 150.0 * t / T) * DEG2RAD; // E -> S -> W
            double ex = std::sin(s), en = -std::cos(s) * std::sin(25.0 * DEG2RAD),
                   eu = std::cos(s) * std::cos(25.0 * DEG2RAD);
            double el = std::asin(eu) * RAD2DEG;
            double az = deg_norm360(std::atan2(ex, en) * RAD2DEG);
            f.target(az, el);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            if (t > 15.0)
            {
                HsCoord p = f.tr.current_pointing(f.utc());
                double err = std::hypot(p.az_deg - az, p.el_deg - el);
                if (err > max_err) { max_err = err; max_t = t; }
            }
            auto sn = f.tr.snapshot();
            CHECK(!sn.halted); // meridian crossing must not latch anything
        }
        fprintf(stderr, "[m-meridional] %.4f at t=%.1f (el %.1f)\n", max_err, max_t,
                std::asin(std::cos((75.0 - 150.0 * max_t / T) * M_PI / 180.0) *
                          std::cos(25.0 * M_PI / 180.0)) * 180.0 / M_PI);
        // Threshold history: 0.3 held through iteration-2 (value then
        // 0.2955 legacy, razor-thin). Raised 0.3 -> 0.35 for the
        // 2026-11-21 HIGH-rescale direction-persistence law (see
        // TrackerConfig::high_recommit_s): ONE genuine acceleration
        // flip in the early sweep (t~14.7, sent 124x, demand walking
        // up at 5x/tick) reads as a wobble candidate and pays a 0.5 s
        // budget-capped hold, moving this metric 0.2955 -> 0.3060
        // (+0.01 deg at the 1/sin(az) lever, well inside any Yagi).
        // The law's hold cost is bounded by the budget itself — same
        // threshold-trade precedent as the m-1hz reversal-window pin.
        CHECK(max_err < 0.35);
    }

    TEST("restore(): resumed session continues geometry and safety budget");
    {
        // Session A: calibrate, wind up a real rotation, "crash" the daemon
        Fixture f;
        f.cal();
        f.board->slew_rate(AxisRA, 100.0);
        f.advance(10.0);                                   // ~4.2 deg of RA
        f.board->stop_axis(AxisRA, /*instant=*/true);
        const AxisAngles axes1 = f.tr.current_axes();
        const uint32_t rr = f.tr.ref_count_ra();
        const uint32_t rd = f.tr.ref_count_de();
        const AxisAngles ref = f.tr.ref_axes();

        // Session B (fresh Tracker object, same powered board): counters
        // still match the persisted reference => restore, don't re-home.
        TrackerConfig c2;
        c2.site = LatLon{44.95, -93.28};
        c2.home_az_deg = 180.0;
        c2.home_el_deg = 60.0;
        Tracker tr2(*f.board, c2);
        tr2.restore(rr, rd, ref, f.utc(), f.mono());
        CHECK(tr2.calibrated());

        const AxisAngles axes2 = tr2.current_axes();
        CHECK_NEAR(axes2.ra_deg, axes1.ra_deg, 1e-9);       // same geometry
        CHECK_NEAR(axes2.dec_deg, axes1.dec_deg, 1e-9);
        // budget continuity: cumulative rotation vs the RESTORED reference is
        // the same (still ~4.2 deg, not reset to zero, not doubled)
        CHECK(tr2.safety().reference().ra_deg == ref.ra_deg);
        CHECK(tr2.safety().check_cumulative(axes2).result == GuardResult::Ok);

        // tracking resumes: hold current pointing, no halt, converges
        const HsCoord pt = tr2.current_pointing(f.utc());
        tr2.set_target_at(pt.az_deg, pt.el_deg, f.mono());
        for (int i = 0; i < 30; ++i)
        {
            f.advance(0.1);
            tr2.step(f.utc(), f.mono());
        }
        const auto s = tr2.snapshot();
        CHECK(!s.halted);
        CHECK_NEAR(s.current.az_deg, pt.az_deg, 0.05);
        CHECK_NEAR(s.current.el_deg, pt.el_deg, 0.05);
    }

    TEST("writes_gated: no false stall latch when board writes are blocked");
    {
        // Read-only bench session: the board gate holds every write, so the
        // counters can NEVER follow commanded motion. That must not read as
        // a stall; only the explicit tracker flag turns skip latching off.
        Fixture f;
        f.cal();
        f.board->block_writes(true);

        TrackerConfig gated = f.cfg;
        gated.writes_gated = true;
        Tracker tr_gated(*f.board, gated);
        tr_gated.calibrate(f.utc(), f.mono());
        for (int i = 0; i < 100; ++i)   // 10 s of commanded motion, zero delivered
        {
            f.advance(0.1);
            tr_gated.set_target_at(200.0, 45.0, f.mono());  // keep target fresh
            tr_gated.step(f.utc(), f.mono());
        }
        CHECK(!tr_gated.snapshot().halted);   // would have latched before the fix

        // Control: same gated board WITHOUT the config flag still latches —
        // the guard must be an explicit choice, not an accident.
        TrackerConfig strict = f.cfg;
        strict.writes_gated = false;
        Tracker tr_strict(*f.board, strict);
        tr_strict.calibrate(f.utc(), f.mono());
        bool latched = false;
        for (int i = 0; i < 100 && !latched; ++i)
        {
            f.advance(0.1);
            tr_strict.set_target_at(200.0, 45.0, f.mono());  // keep target fresh
            tr_strict.step(f.utc(), f.mono());
            auto s = tr_strict.snapshot();
            // latch specifically via the skip reason, not staleness
            latched = s.halted && s.halt_reason.find("skip") != std::string::npos;
        }
        CHECK(latched);
    }

    // ---- 9. transport watchdog: HOLD semantics (live GT2 2026-11-16) --------
    TEST("watchdog: client gap halts with HOLD (not operator latch) and a fresh push resumes");
    {
        // The 6 h deadlock (2026-11-16): one 6 s client gap tripped the
        // watchdog through safety_.trip() = operator latch, and nothing
        // short of a manual R could recover. Contract now: client-absent
        // watchdog stops and HOLDS; the next rotctld command (a push)
        // clears it by itself — GT2's reconnect-and-push must just work.
        Fixture f;
        f.cfg.target_stale_s = 3600.0; // isolate the watchdog from stale-target
        f.cal();
        f.target(200.0, 55.0);
        for (int i = 0; i < 30; ++i) // 3 s: up to speed
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking);
        f.sim->clear_log();
        // 30 s of total client silence: >> watchdog_timeout_s (5 s)
        for (int i = 0; i < 300; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        auto s = f.tr.snapshot();
        CHECK(s.halted);
        CHECK(s.halt_reason.find("watchdog") != std::string::npos);
        CHECK(!f.sim->sim_running('1'));
        CHECK(!f.sim->sim_running('2'));
        // HOLD means stop-and-wait, not e-stop: no :L frames after the trip
        bool estop = false;
        for (auto& c : f.sim->command_log())
            if (c.size() >= 2 && c[0] == ':' && c[1] == 'L') estop = true;
        CHECK(!estop);
        // a fresh push is the recovery signal: no on_reset(), no re-calibrate
        f.target(210.0, 55.0);
        for (int i = 0; i < 50; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.tr.snapshot().halted);
        CHECK(f.tr.snapshot().tracking);
    }

    TEST("watchdog: rotor polls alone keep the client alive (GT2 throttle contract)");
    {
        // GT2 iq-work holds the socket and polls `p` every 2 s while its
        // follow throttle legitimately spaces pushes 20-60 s. Push-only
        // watchdog feeding deadlocked real sessions on the first gap
        // (replay 2026-11-16); any rotctld command must re-arm it.
        Fixture f;
        f.cfg.target_stale_s = 3600.0; // only the watchdog may fire here
        f.cal();
        f.target(200.0, 55.0);
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        bool watchdog_halt = false;
        for (int i = 0; i < 600; ++i) // 60 s = 12x watchdog timeout
        {
            if (i % 20 == 0)
            {
                double a, e;
                f.tr.on_get_position(a, e); // GT2's 2 s rotor poll
            }
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.halted && s.halt_reason.find("watchdog") != std::string::npos)
                watchdog_halt = true;
        }
        CHECK(!watchdog_halt);
        // control: without the polls the very same scenario DOES halt —
        // the poll, not elapsed time, is what kept the session alive.
        Fixture g(f.cfg);
        g.cal();
        g.target(200.0, 55.0);
        bool did_halt = false;
        for (int i = 0; i < 300; ++i)
        {
            g.advance(0.1);
            g.tr.step(g.utc(), g.mono());
            auto s = g.tr.snapshot();
            if (s.halted && s.halt_reason.find("watchdog") != std::string::npos)
                did_halt = true;
        }
        CHECK(did_halt);
    }

    TEST("watchdog: command ARRIVAL alone keeps the client alive (stall fix)");
    {
        // Live sweeps 2-4 (2026-11-17/18): a board stop/restart waited for
        // the deceleration tail while holding the tracker lock, so rotctld
        // commands sat unread for 5-6 s — the handling-stamped watchdog
        // false-tripped at a client that was pushing at 1 Hz the whole
        // time, and the HALT/resume cascade then skip-latched DEC.
        // serve_client() now reports ARRIVAL (on_client_rx) before the
        // command reaches the lock; step() must consume it as liveness.
        Fixture f;
        f.cfg.target_stale_s = 3600.0; // only the watchdog may fire here
        f.cal();
        f.target(200.0, 55.0);
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        bool watchdog_halt = false;
        for (int i = 0; i < 600; ++i) // 60 s = 12x watchdog timeout
        {
            if (i % 10 == 0)
                f.tr.on_client_rx(); // bytes reached the socket, unhandled
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.halted && s.halt_reason.find("watchdog") != std::string::npos)
                watchdog_halt = true;
        }
        CHECK(!watchdog_halt);
    }

    // ---- 10. wire economy on a held pre-aim pose (idle :K storm) ------------
    TEST("duplicate stationary pushes: settled hold emits zero wire, zero drift");
    {
        // Live dense replay (2026-11-16): GT2 pre-aim re-pushed the same
        // 0.1-deg-quantized az/el every 2 s; estimator velocity noise x the
        // alive-extrapolation horizon made the mount creep, and the
        // HIGH-band corrections turned creep into a :K-per-tick storm
        // (7219 :K frames; guard rework brought it to ~4.5k with these
        // buckets at 8-24/30min). A settled stationary hold must be
        // SILENT on the wire: no :J restarts, no :K churn, no wander.
        Fixture f(Fixture::altaz());
        f.cfg.target_stale_s = 3600.0;
        f.cal();
        for (int i = 0; i < 1000; ++i) // 100 s: travel to az 212.1, settle
        {
            if (i % 20 == 0) f.target(212.1, 0.0); // GT2 pre-aim push
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        double az0 = f.tr.snapshot().current.az_deg;
        f.sim->clear_log();
        double max_rate = 0.0;
        for (int i = 0; i < 300; ++i) // held window: 30 s of duplicates
        {
            if (i % 20 == 0) f.target(212.1, 0.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            max_rate = std::max(max_rate, std::max(std::fabs(s.rate_ra_x),
                                                   std::fabs(s.rate_dec_x)));
        }
        long k = 0, j = 0;
        for (auto& c : f.sim->command_log())
        {
            if (c.rfind(":K", 0) == 0 || c.rfind(":L", 0) == 0) ++k;
            if (c.rfind(":J", 0) == 0) ++j;
        }
        CHECK(k == 0); // no stop churn while holding a settled pose
        CHECK(j == 0); // no restart churn either: the axis never needed to move
        CHECK(max_rate < 5.0); // est. noise must not press the throttle
        double az1 = f.tr.snapshot().current.az_deg;
        CHECK_NEAR(az1, az0, 0.05); // and no drift
    }

    TEST("GT2 micro-push storm: 0.15 s duplicate pushes must not storm the wire (live AO-73)");
    {
        // GT2 iq-work follow-gate double-fire (live AO-73 pass, 2026-11-18):
        // push PAIRS 0.1-0.4 s apart carrying a fraction-of-a-degree step of
        // newer info. Unfiltered, ScalarABFilter::v += beta*r/dt turns the
        // second sample into a deg/s velocity kick -> pattern sign flips ->
        // HIGH grants -> :K storm (live: 265 :K, max err 6.2 deg; dense
        // replay reproduces, dataset-filtered control fixes it to :K 99).
        // Scenario: mid-el rise (el 20->35, az -0.05 deg/s) fed GT2-cadence
        // 0.1-deg-quantized pushes every 2 s, every 6th push duplicated at
        // +0.15 s carrying the next step. Measured with the fix (minimum
        // estimator cadence): 28 :J/:K (duplicate-free baseline: 17);
        // with pattern_min_dt_s = 0 (legacy estimator): 102 -> the test
        // pins BOTH sides so the failure mode itself is regression-guarded.
        const double az0 = 130.0, azr = -0.05, el0 = 20.0, elr = 0.25;
        struct Wire { long j = 0, k = 0; };
        auto run = [&](bool legacy) -> Wire
        {
            TrackerConfig c = Fixture::altaz();
            if (legacy)
            {
                // Legacy = the FULL pre-iteration-2 law: un-floored
                // estimator AND no demand smoothing AND the short 0.5 s
                // reversal window (the AO-73 storm reproduced live and in
                // replay under exactly that configuration).
                c.pattern_min_dt_s = 0.0;
                c.demand_lpf_tau_s = 0.0;
                c.reversal_confirm_s = 0.5;
                c.reversal_hold_deg = 1e9; // un-capped (pre-budget semantics)
                // The pre-2026-11-20 HIGH gate exempted ANY moving pattern
                // (3x sidereal); without restoring it the tightened default
                // (90x: only patterns LOW cannot follow) suppresses most of
                // this very storm in the legacy twin and the pin collapses.
                // (That suppression is itself evidence for the mgate fix:
                // legacy j/k drops from ~102 to ~42 on the gate alone.)
                c.high_gate_moving_x = 3.0;
                c.glide_promote_holdoff_s = 0.0;
                c.high_recommit_s = 0.0; // pre-2026-11-21: no scale confirm
            }
            c.target_stale_s = 3600.0;
            Fixture f(c);
            f.cal();
            for (int i = 0; i < 800; ++i) // pre-aim + settle
            {
                if (i % 20 == 0) f.target(az0 + 1.0, el0 - 1.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            f.sim->clear_log();
            for (int i = 0; i < 600; ++i) // 60 s climb
            {
                const double t = i * 0.1;
                if (i % 20 == 0)
                {
                    const double az = az0 + azr * t, el = el0 + elr * t;
                    f.target(std::round(az * 10.0) / 10.0, std::round(el * 10.0) / 10.0);
                }
                if (i % 120 == 6) // duplicate 0.15 s later, +0.3 deg newer
                {
                    const double t2 = t - 0.15;
                    f.target(std::round((az0 + azr * t2 + 0.3) * 10.0) / 10.0,
                             std::round((el0 + elr * t2 + 0.3) * 10.0) / 10.0);
                }
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            Wire w;
            for (auto& c : f.sim->command_log())
            {
                if (c.rfind(":K", 0) == 0 || c.rfind(":L", 0) == 0) ++w.k;
                if (c.rfind(":J", 0) == 0) ++w.j;
            }
            auto s = f.tr.snapshot(); // pointing check per run
            CHECK(std::fabs(s.current.az_deg - (az0 + azr * 60.0)) < 1.5);
            CHECK(std::fabs(s.current.el_deg - (el0 + elr * 60.0)) < 1.5);
            return w;
        };
        const Wire fixed_ = run(false);
        const Wire legacy_ = run(true);
        // Value print: pins the iteration-2 storm suppression magnitude in
        // the regression log (LPF 0.3 + reversal budget; pre-fix baseline
        // ran ~90-100 :J/:K in this scenario).
        fprintf(stderr, "[m-micropush] fixed j=%ld k=%ld legacy j=%ld k=%ld\n",
                fixed_.j, fixed_.k, legacy_.j, legacy_.k);
        CHECK(fixed_.j <= 60 && fixed_.k <= 60);
        CHECK(legacy_.j >= 80 && legacy_.k >= 80);
    }

    TEST("HIGH rescale pump: meridian-corridor follow must not storm the wire (live SO-50)");
    {
        // SO-50 pass replay 2026-11-21 (data/gt2-iqwork-2026-11-21-so50):
        // near-horizon sweep through the az ~186 meridian corridor, where
        // the RA<->az Jacobian lever 1/|sin az| (~10-30x) turns a 0.15
        // deg/s azimuth motion into a genuinely HIGH 150-450x AXIS rate.
        // GT2 2 s / 0.1 deg quantized pushes then step the axis demand by
        // ~1-3 deg at a time: the glide P rescales HIGH->HIGH past the
        // 25 % deadband several times per minute and each commit is
        // self-perturbing (:K coast + issuance starvation), so the demand
        // walks back and forth across the db — a push-cadence limit cycle
        // (replay census: 149 :K/min at el > 10, alternating rescales
        // +402->225->163->204->265->168). Law under test: the HIGH
        // rescale direction-persistence confirm + wobble-regime
        // acceleration freeze (high_recommit_s family): a magnitude-
        // direction flip against the last committed rescale is held at
        // the sent program (wire-free) until it persists, and inside a
        // storm regime (leaky commit counter + throttled-client age)
        // acceleration rescales touching HIGH freeze entirely unless
        // the position loop (>= 2 deg axis error) or the floor guard
        // demands them. Legacy twin = high_recommit_s = 0 ONLY
        // (isolates this law; the envelope is disabled for both runs:
        // this corridor sits outside the level-mount stand box, same
        // precedent as the m-meridional scenario).
        struct Wire
        {
            long j = 0, k = 0;
            double err = 0.0;
        };
        const double az0 = 183.5, azr = 0.15, el0 = 12.0, elr = 0.04;
        const double win = 46.0; // az 183.5 -> 190.4, across the az-186 lever peak
        auto run = [&](bool legacy) -> Wire
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.max_ra_travel_deg = c.safety.max_dec_travel_deg = 0.0;
            c.target_stale_s = 3600.0;
            if (legacy) c.high_recommit_s = 0.0; // pre-law: immediate commit
            Fixture f(c);
            f.cal();
            for (int i = 0; i < 900; ++i) // pre-aim at the corridor entry + settle
            {
                if (i % 20 == 0)
                    f.target(std::round(az0 * 10.0) / 10.0, std::round(el0 * 10.0) / 10.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            f.sim->clear_log();
            Wire w;
            for (int i = 0; i < (int)(win * 10.0); ++i)
            {
                const double t = i * 0.1;
                if (i % 20 == 0) // GT2 follow cadence: 2 s, 0.1 deg grid
                {
                    const double az = az0 + azr * t, el = el0 + elr * t;
                    f.target(std::round(az * 10.0) / 10.0, std::round(el * 10.0) / 10.0);
                }
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                HsCoord p = f.tr.current_pointing(f.utc());
                const double e = std::hypot(p.az_deg - (az0 + azr * t),
                                            p.el_deg - (el0 + elr * t));
                if (e > w.err) w.err = e;
                auto sn = f.tr.snapshot();
                CHECK(!sn.halted); // the corridor must never latch anything
                long j = 0, k = 0;
                for (auto& s : f.sim->command_log())
                {
                    if (s.rfind(":J", 0) == 0) ++j;
                    if (s.rfind(":K", 0) == 0 || s.rfind(":L", 0) == 0) ++k;
                }
                w.j = j; w.k = k;
            }
            return w;
        };
        const Wire fixed_ = run(false);
        const Wire legacy_ = run(true);
        fprintf(stderr, "[m-rescalepump] fixed j=%ld k=%ld err %.2f | legacy j=%ld k=%ld err %.2f\n",
                fixed_.j, fixed_.k, fixed_.err, legacy_.j, legacy_.k, legacy_.err);
        // Pins (full-freeze defaults, 2026-11-21): the corridor's
        // push-cadence rescale churn collapses 70 -> ~5 :K while the
        // pointing is unchanged (both runs track the corridor to the
        // same 0.35 deg: the frozen rungs cost nothing the deceleration
        // commits do not already deliver at this lever).
        CHECK(fixed_.k <= 20 && fixed_.err < 0.5);
        CHECK(legacy_.k >= 50);
    }

    TEST("wobble-regime demand-LPF escalation: fires past storm bar, inert disabled/LOW-band");
    {
        // 0a4(h) law (live 2026-11-22 22:00-pass DEC churn, 60-107 :K/min
        // at follow err < 0.7 deg): while an axis's wobble regime runs at
        // STORM GRADE (leaky commit counter rc past demand_lpf_regime_
        // min_rc), the glide demand LPF temporarily escalates to
        // demand_lpf_regime_tau_s. The regime detector itself is the
        // high_recommit_* family's; THIS test pins the LPF escalation and
        // its storm-grade bar. The live pump sits at rc ~33 mean (100 % of
        // storm ticks past 15). NOTE the SO-50 corridor fixture below is
        // NOT excluded by the bar — its regimes clear 12 (escalation-
        // forced-off probe reaches raw rc 39.5); the small wire cost
        // at flat 0.35 pointing there is the documented edge-class trade
        // (see TrackerConfig::demand_lpf_regime_min_rc). Pins:
        //   disabled twin: corridor with a never-reached bar -> byte-
        //     identical to the pre-escalation law;
        //   fires pin: same corridor at the shipped bar 12 -> escalation
        //     engages and the wire changes (direction of the net effect is
        //     context-dependent; the slice REPLAY pass2200-rel.csv — storm
        //     DEC 187 -> 88 at flat err — is the shipped-value evidence,
        //     this pins the mechanism);
        //   LOW-band inert: no HIGH commits -> huge tau byte-inert.
        struct Wire
        {
            long j = 0, k = 0;
            double err = 0.0;
        };
        const double az0 = 183.5, azr = 0.15, el0 = 12.0, elr = 0.04;
        const double win = 46.0;
        auto corridor = [&](double lrpf, double min_rc) -> Wire
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.max_ra_travel_deg = c.safety.max_dec_travel_deg = 0.0;
            c.target_stale_s = 3600.0;
            c.demand_lpf_regime_tau_s = lrpf;
            c.demand_lpf_regime_min_rc = min_rc;
            Fixture f(c);
            f.cal();
            for (int i = 0; i < 900; ++i)
            {
                if (i % 20 == 0)
                    f.target(std::round(az0 * 10.0) / 10.0, std::round(el0 * 10.0) / 10.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            f.sim->clear_log();
            Wire w;
            for (int i = 0; i < (int)(win * 10.0); ++i)
            {
                const double t = i * 0.1;
                if (i % 20 == 0)
                {
                    const double az = az0 + azr * t, el = el0 + elr * t;
                    f.target(std::round(az * 10.0) / 10.0, std::round(el * 10.0) / 10.0);
                }
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                HsCoord p = f.tr.current_pointing(f.utc());
                const double e = std::hypot(p.az_deg - (az0 + azr * t),
                                            p.el_deg - (el0 + elr * t));
                if (e > w.err) w.err = e;
                CHECK(!f.tr.snapshot().halted);
                long j = 0, k = 0;
                for (auto& s : f.sim->command_log())
                {
                    if (s.rfind(":J", 0) == 0) ++j;
                    if (s.rfind(":K", 0) == 0 || s.rfind(":L", 0) == 0) ++k;
                }
                w.j = j; w.k = k;
            }
            return w;
        };
        const Wire off_ = corridor(0.0, 12.0);    // pre-escalation law
        const Wire dis_ = corridor(1.0, 1e12);    // escalation disabled by bar
        const Wire arms_ = corridor(1.0, 12.0);  // shipped bar/tau: fires
        fprintf(stderr, "[m-lpfregime] corridor off j=%ld k=%ld err %.2f | disabled j=%ld k=%ld err %.2f | armed j=%ld k=%ld err %.2f\n",
                off_.j, off_.k, off_.err, dis_.j, dis_.k, dis_.err, arms_.j, arms_.k, arms_.err);
        // Disabled twin: with the bar unreachable, the law must be byte-
        // identical to pre-escalation (rollback lever verified).
        CHECK(dis_.j == off_.j && dis_.k == off_.k && dis_.err == off_.err);
        CHECK(off_.k <= 20 && off_.err < 0.5); // unchanged corridor economics
        // Fires: the fixture's regimes clear the shipped bar, so the
        // escalated filter must actually change the closed-loop behavior
        // (5 -> 7 :K here, the documented edge-class cost; the slice
        // replay is the benefit side).
        CHECK(arms_.k != off_.k || arms_.j != off_.j);
        CHECK(arms_.err < 0.5); // demand smoothing costs nothing at fixture resolution

        // Inert side: LOW-band follow (az 93 el 5 from the 90/0 datum,
        // near-unity Jacobian -> demand stays far below the 104x HIGH
        // gate, zero HIGH commits -> regime ledger decays at 0): a huge
        // escalation tau must not change ONE wire byte.
        auto lowband = [&](double lrpf, Wire& out)
        {
            TrackerConfig c = Fixture::altaz();
            c.safety.max_ra_travel_deg = c.safety.max_dec_travel_deg = 0.0;
            c.demand_lpf_regime_tau_s = lrpf;
            Fixture f(c);
            f.cal();
            for (int i = 0; i < 600; ++i)
            {
                if (i % 10 == 0) f.target(93.0, 5.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            f.sim->clear_log();
            for (int i = 0; i < 600; ++i)
            {
                if (i % 10 == 0) f.target(93.0, 5.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            for (auto& s : f.sim->command_log())
            {
                if (s.rfind(":J", 0) == 0) ++out.j;
                if (s.rfind(":K", 0) == 0 || s.rfind(":L", 0) == 0) ++out.k;
            }
            HsCoord p = f.tr.current_pointing(f.utc());
            out.err = std::hypot(p.az_deg - 93.0, p.el_deg - 5.0);
        };
        Wire l_off, l_on;
        lowband(0.0, l_off);
        lowband(30.0, l_on); // huge tau: if it ever armed, tracking would choke
        fprintf(stderr, "[m-lpfregime] lowband off j=%ld k=%ld err %.3f | on(30s) j=%ld k=%ld err %.3f\n",
                l_off.j, l_off.k, l_off.err, l_on.j, l_on.k, l_on.err);
        CHECK(l_on.j == l_off.j && l_on.k == l_off.k); // regime unarmed => byte-identical
        CHECK(l_on.err < 0.5);
    }

    TEST("alive pattern horizon scales with pattern rate: stationary hold does not wander");
    {
        // pattern_alive_pred_max_s (25 s) extends pattern extrapolation
        // while the client polls — correct for a MOVING target between
        // throttled pushes, but a stationary pattern's alpha-beta velocity
        // is quantization noise: unscaled, 0.1 deg/2 s duplicate pushes
        // gave +-0.05 deg/s noise x 25 s = 1.25 deg of idle wander, which
        // in the full replay wound RA past the 540 deg wrap trip. The
        // extension is therefore scaled by |pattern velocity| (full at
        // pattern_alive_full_x_rate). Contract: after a stationary fit,
        // polls with NO pushes must hold, not creep.
        Fixture f(Fixture::altaz());
        f.cfg.target_stale_s = 3600.0;
        f.cal();
        for (int i = 0; i < 1000; ++i) // 100 s settled on duplicate pushes
        {
            if (i % 20 == 0) f.target(212.1, 0.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        double az0 = f.tr.snapshot().current.az_deg;
        // 20 s: polls keep the client alive, but NO pushes arrive
        for (int i = 0; i < 200; ++i)
        {
            if (i % 20 == 0)
            {
                double a, e;
                f.tr.on_get_position(a, e);
            }
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        double az1 = f.tr.snapshot().current.az_deg;
        CHECK_NEAR(az1, az0, 0.1); // the noise-scaled horizon holds position
    }

    // ---- 11. HIGH-band actuator-quantum gate --------------------------------
    TEST("HIGH gate: resume from stale coast clamps unjustified demand; authority returns when moving");
    {
        // The live replay trigger (recorded t=16224): pre-aim settled at a
        // far el-0 pose, GT2 throttled pushes off, target went stale (coast
        // stop ~0.05 deg off pose), GT2 re-pushed the SAME target. The
        // resume's tight-accel demand had nothing behind it — pattern at
        // dead stop, axis error tiny — yet the law pressed the throttle
        // (measured: HIGH-gate OFF max demand 100x vs 32.7x gated). The
        // gate clamps unjustified demand to the coast-free LOW band; the
        // moving case (below) proves a real pattern rate buys it back.
        auto run = [](const TrackerConfig& base) -> double {
            TrackerConfig c = base;
            c.site = LatLon{44.95, -93.28};
            c.orientation = Orientation::AltAzLevel;
            c.home_az_deg = 90.0;
            c.home_el_deg = 0.0;
            c.target_stale_s = 15.0; // stale-coast fast-forward for the test
            c.nudge_i_unwind_gain = 1.0;
                                     // this scenario GENERATES the unjustified
                                     // resume demand from dive-phase integral
                                     // windup; the unwind bleed (default 5)
                                     // drains exactly that, which collapses the
                                     // gated/open contrast without touching the
                                     // gate (measured: at gain 5 both sides
                                     // resume at ~3x — legitimately tiny). Pin
                                     // the legacy integration here to isolate
                                     // the gate mechanism; the bleed has its
                                     // own test below.
            c.safety.max_ra_travel_deg = c.safety.max_dec_travel_deg = 0.0;
                                     // coast pose az 212.1/el 0 predates the
                                     // stand discovery (unreachable now);
                                     // this scenario grades the HIGH gate
            auto sim = std::make_shared<SimulatorTransport>(sim_heq5());
            Board board(sim);
            board.connect();
            Tracker tr(board, c);
            auto t0 = SC::now();
            auto u0 = UC::now();
            tr.calibrate(u0, t0);
            SC::duration vt{0};
            auto mono = [&]() { return t0 + vt; };
            auto utc = [&]() { return u0 + std::chrono::duration_cast<UC::duration>(vt); };
            auto tick = [&](double s)
            {
                vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(s));
                sim->advance(s);
                tr.step(utc(), mono());
            };
            for (int i = 0; i < 1000; ++i) // 100 s: travel + settle at 212.1
            {
                if (i % 20 == 0) tr.set_target_at(212.1, 0.0, mono());
                tick(0.1);
            }
            // stale coast: polls feed the watchdog, no pushes => stale halt
            for (int i = 0; i < 400; ++i)
            {
                if (i % 20 == 0) { double a, e; tr.on_get_position(a, e); }
                tick(0.1);
            }
            // resume: GT2 re-pushes the same target (live 2 s cadence);
            // measure 30 s of demand
            tr.set_target_at(212.1, 0.0, mono());
            double max_rate = 0.0;
            for (int i = 0; i < 300; ++i)
            {
                if (i % 20 == 0)
                {
                    double a, e;
                    tr.on_get_position(a, e);
                    tr.set_target_at(212.1, 0.0, mono());
                }
                tick(0.1);
                auto s = tr.snapshot();
                max_rate = std::max(max_rate, std::max(std::fabs(s.rate_ra_x),
                                                       std::fabs(s.rate_dec_x)));
            }
            CHECK_NEAR(tr.snapshot().current.az_deg, 212.1, 0.3); // arrived
            return max_rate;
        };
        const double gated = run(TrackerConfig{});
        TrackerConfig off{};
        off.high_gate_err_deg = 1e9; // justified = never (pattern stopped) -> clamp off
        off.high_gate_moving_x = 0.0;
        const double open = run(off);
        // the gate is a >2x clamp on the unjustified resume (measured
        // gated=16.8x vs open=67.7x in this scenario; the live recording
        // showed the same signature at t=16224)
        CHECK(gated * 2.0 < open);
        CHECK(gated <= 30.0);
    }

    TEST("glide settle: unwind bleed drains dive-overshoot integral windup");
    {
        // Live 2026-11-19 (Sirius, P 90 10 probe): after the dive + one
        // reversal :K, the glide settle STRANDED at 0.27 deg elevation and
        // crawled ~0.0075 deg/s — 30+ s later still 0.27 off. Sim twin (this
        // exact scenario, 1 Hz pushes at a stationary target 10 deg away):
        // with nudge_i_unwind_gain 1 the tail is the loop's slow pole
        // Ki/P = tau ~ 50 s (el err 0.096 -> 0.082 -> 0.067 -> 0.037 over
        // t=10..60 s), because the overshoot winds nudge_err_ opposite the
        // present error and the plain err x dt integration must unwind the
        // whole stale area before the correction escapes. With the bleed
        // (gain 5) the same run: 0.082 -> 0.018 -> 0.006 -> 0.002, i.e.
        // tau ~ 7 s — dead-on within the 0.05 deg Yagi-relevant band.
        auto run = [](double gain, int at_s) -> double {
            TrackerConfig c;
            c.site = LatLon{44.95, -93.28};
            c.orientation = Orientation::AltAzLevel;
            c.home_az_deg = 90.0;
            c.home_el_deg = 0.0;
            c.nudge_i_unwind_gain = gain;
            auto sim = std::make_shared<SimulatorTransport>(sim_heq5());
            Board board(sim);
            board.connect();
            Tracker tr(board, c);
            auto t0 = SC::now();
            auto u0 = UC::now();
            tr.calibrate(u0, t0);
            SC::duration vt{0};
            auto mono = [&]() { return t0 + vt; };
            auto utc = [&]() { return u0 + std::chrono::duration_cast<UC::duration>(vt); };
            for (int tick = 0; tick < 10 * (at_s + 1); ++tick)
            {
                if (tick % 10 == 0) tr.set_target_at(90.0, 10.0, mono());
                tr.step(utc(), mono());
                vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(0.1));
                sim->advance(0.1);
            }
            return tr.current_pointing(utc()).el_deg - 10.0;
        };
        const double fixed30 = run(5.0, 30);
        const double fixed90 = run(5.0, 90);
        const double legacy30 = run(1.0, 30);
        const double legacy90 = run(1.0, 90);
        // fixed: inside 0.05 deg by t=30 (measured 0.006) and settled by
        // t=90 (measured 0.002); legacy: still outside at BOTH checks
        CHECK(std::fabs(fixed30) < 0.05);
        CHECK(std::fabs(fixed90) < 0.01);
        CHECK(std::fabs(legacy30) > 0.05);
        CHECK(std::fabs(legacy90) > 0.01);
    }

    TEST("HIGH gate releases while the pattern is moving");
    {
        // Moving-target authority: a 1 deg/s az pattern (~220x axis, HIGH
        // band) at tiny pushed-target lag must be fully trackable with the
        // gate ON — near-zenith LEO passes outrun a clamped law. Measured:
        // steady rt ~ -220x (HIGH), sawtooth error < 2.5 deg peak under
        // 2 s-quantized GT2-rate pushes.
        TrackerConfig c{};
        c.site = LatLon{44.95, -93.28};
        c.orientation = Orientation::AltAzLevel;
        c.home_az_deg = 90.0;
        c.home_el_deg = 0.0;
        auto sim = std::make_shared<SimulatorTransport>(sim_heq5());
        Board board(sim);
        board.connect();
        Tracker tr(board, c);
        auto t0 = SC::now();
        auto u0 = UC::now();
        tr.calibrate(u0, t0);
        SC::duration vt{0};
        auto mono = [&]() { return t0 + vt; };
        auto utc = [&]() { return u0 + std::chrono::duration_cast<UC::duration>(vt); };
        double max_rate = 0.0, max_err = 0.0, sum_err = 0.0;
        int n_err = 0;
        for (int i = 0; i < 600; ++i) // 60 s at 1 deg/s az
        {
            if (i % 20 == 0) tr.set_target_at(90.0 + 1.0 * (vt.count() / 1e9), 0.0, mono());
            vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(0.1));
            sim->advance(0.1);
            tr.step(utc(), mono());
            auto s = tr.snapshot();
            max_rate = std::max(max_rate, std::max(std::fabs(s.rate_ra_x),
                                                   std::fabs(s.rate_dec_x)));
            const double e = std::fabs(s.target.az_deg - s.current.az_deg);
            if (i > 200) { max_err = std::max(max_err, e); sum_err += e; ++n_err; }
        }
        CHECK(max_rate > 128.0); // pattern rate bought HIGH authority (measured ~600x transient)
        CHECK(max_err < 2.5); // sawtooth peak under 2 s-quantized pushes
        CHECK(sum_err / n_err < 1.5); // mean ~1.1 deg on a 1 deg/s ramp
    }

    // ---- 12. reversal persistence guard (:K churn) --------------------------
    TEST("reversal guard: a sign flip waits the confirm window before touching the wire");
    {
        // Saturated glide P limit-cycled near the horizon (live replay:
        // :K storms while the pattern hovered at zero crossing). Demand
        // flips must PERSIST reversal_confirm_s before the board is
        // reprogrammed (each HIGH flip = :K + ~1 s coast); while pending,
        // the demand is HELD AT THE SENT PROGRAM — zero wire traffic goes
        // out, and pointing keeps tracking the old direction.
        auto run = [](double confirm_s, double hold_deg = 1e9) -> int {
            TrackerConfig c{};
            c.reversal_confirm_s = confirm_s;
            c.reversal_hold_deg = hold_deg;
            c.site = LatLon{44.95, -93.28};
            c.orientation = Orientation::AltAzLevel;
            c.home_az_deg = 90.0;
            c.home_el_deg = 0.0;
            auto sim = std::make_shared<SimulatorTransport>(sim_heq5());
            Board board(sim);
            board.connect();
            Tracker tr(board, c);
            auto t0 = SC::now();
            auto u0 = UC::now();
            tr.calibrate(u0, t0);
            SC::duration vt{0};
            auto mono = [&]() { return t0 + vt; };
            auto utc = [&]() { return u0 + std::chrono::duration_cast<UC::duration>(vt); };
            // phase 1: steady +az pattern at 6 deg/s (HIGH band running)
            double az_end = 90.0;
            for (int i = 0; i < 400; ++i)
            {
                if (i % 20 == 0)
                {
                    az_end = 90.0 + 6.0 * (vt.count() / 1e9);
                    tr.set_target_at(az_end, 0.0, mono());
                }
                vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(0.1));
                sim->advance(0.1);
                tr.step(utc(), mono());
            }
            sim->clear_log();
            size_t flip_tick = 0;
            // phase 2: pattern reverses at full speed; find the first :K
            for (int i = 0; i < 200; ++i)
            {
                if (i % 20 == 0)
                    tr.set_target_at(az_end - 6.0 * (i * 0.1), 0.0, mono());
                vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(0.1));
                sim->advance(0.1);
                tr.step(utc(), mono());
                if (flip_tick == 0)
                {
                    auto log = sim->command_log_tail(0);
                    for (auto& fr : log)
                        if (fr.rfind(":K", 0) == 0) { flip_tick = i + 1; break; }
                }
            }
            return (int)flip_tick;
        };
        int guard_on = run(0.5);
        int guard_off = run(0.0);
        CHECK(guard_off >= 1);  // scenario bites: an ungated flip reprograms fast
        CHECK(guard_on >= 5);   // confirm window: first flip waits >= 0.45 s
        // and the guard is a DELAY, not a denial: motion resumes either way
    }

    // ---- travel envelope (stand clearance, live freeze 2026-11-19) --------
    TEST("envelope: unreachable target refused BEFORE motion, auto-return home, legal push resumes");
    {
        // GT2 pre-aim geometry that drove the tube into the stand: from the
        // datum (az 90 / el 0 level east) az 270 el 40 has NO axis solution
        // inside the +-105 box (canonical RA offset -140, flip dec +180).
        Fixture f(Fixture::altaz());   // envelope ON by default (105)
        f.cal();
        bool returning = false, ever_moved = false;
        std::string reason;
        for (int i = 0; i < 300; ++i) // 30 s of 1 Hz pushes at the illegal target
        {
            if (i % 10 == 0) f.target(270.0, 40.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            returning = returning || s.travel_returning;
            if (s.travel_returning && reason.empty()) reason = s.travel_reason;
            HsCoord p = f.tr.current_pointing(f.utc());
            if (std::hypot(p.az_deg - 90.0, p.el_deg) > 2.0) ever_moved = true;
            CHECK(!s.halted);          // refusal must NOT latch anything
        }
        CHECK(returning);
        CHECK(!ever_moved);            // predictive: zero motion toward it
        CHECK(reason.find("RA") != std::string::npos ||   // names the axis...
              reason.find("DEC") != std::string::npos);
        CHECK(reason.find("envelope") != std::string::npos);
        f.sim->clear_log();
        for (int i = 0; i < 100; ++i)  // hold: stays home, no motion cmd
        {
            f.target(270.0, 40.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.any_motion_start());
        // legal push (canonical solution well inside the box) resumes:
        double max_err = 0;
        for (int i = 0; i < 1500; ++i) // 150 s to dive ~45 deg + settle
        {
            if (i % 10 == 0) f.target(120.0, 30.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            CHECK(!s.halted);
            if (i > 1200)
            {
                double e = std::hypot(s.current.az_deg - 120.0,
                                      s.current.el_deg - 30.0);
                max_err = std::max(max_err, e);
            }
        }
        CHECK(!f.tr.snapshot().travel_returning);
        CHECK(max_err < 0.3);
    }

    TEST("envelope: near-north low-el strip converges via the over-the-top flip branch, never refused");
    {
        // az 350 el 0.5: canonical axis solution is RA offset -177 (outside),
        // the flip branch (offset +2.9, +100) sits INSIDE the box and points
        // identically. The tracker must chase the flip branch (RA barely
        // moves, DEC tilts back over the datum) — the exact "head must be
        // able to flip over the top" authority the user required.
        Fixture f(Fixture::altaz());
        f.cal();
        const uint32_t datum1 = f.sim->sim_position('1') & 0xFFFFFFu;
        double max_err = 0;
        for (int i = 0; i < 2500; ++i) // 250 s: ~100 deg DEC dive + settle
        {
            if (i % 10 == 0) f.target(350.0, 0.5);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            CHECK(!s.travel_returning);   // reachable: must NEVER refuse
            CHECK(!s.halted);
            if (i > 2200)
            {
                double e = std::hypot(s.current.az_deg - 350.0,
                                      s.current.el_deg - 0.5);
                max_err = std::max(max_err, e);
            }
        }
        CHECK(max_err < 0.3);
        // flip branch means the RA axis stayed near its datum (offset 2.9 deg
        // = 73 k steps), NOT wound the -177 deg canonical way (4.4 M steps).
        double ra_off_steps = (double)(int32_t)((f.sim->sim_position('1') & 0xFFFFFFu) - datum1);
        CHECK(std::fabs(ra_off_steps) < 150000.0);
    }

    TEST("envelope: resume hysteresis — a target dithering at the wall cannot flip-flop");
    {
        // Enter refusal at the unreachable pre-aim geometry (az 270 el 40:
        // canonical RA offset ~-140 outside, flip DEC +180 outside), then
        // dither targets that are STILL outside (az 268: RA offset ~-140):
        // must stay in the return-home state (the 5 deg resume margin is
        // irrelevant while Blocked, but neither may it resume). Finally a
        // target well inside (az 120 el 30) resumes.
        Fixture f(Fixture::altaz());
        f.cal();
        for (int i = 0; i < 100; ++i)   // enter refusal at the illegal pre-aim
        {
            if (i % 10 == 0) f.target(270.0, 40.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().travel_returning);
        // az 266 el 40.1: still outside (268/40 canonical offset -136)
        for (int i = 0; i < 30; ++i)
        {
            f.target(268.0, 40.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            CHECK(f.tr.snapshot().travel_returning);
        }
        // legal target 12 deg deep (offset well beyond the 5 deg margin):
        for (int i = 0; i < 30; ++i)
        {
            f.target(120.0, 30.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(!f.tr.snapshot().travel_returning);
    }

    TEST("branch handover: setting satellite at the RA wall DEFERS the ~170 deg flip");
    {
        // rs44 2026-11-20 05:53:00 (live LOS): riding the flip branch, the
        // descending target pushed its RA offset past the +105 wall
        // (probe lat 44.95: at az 179.6 el 1.0 canonical offset (-68,-89)
        // IN box, flip (+112,-91) OUT). The pre-fix law teleported the
        // setpoint ~170 deg to the canonical image — a full dive across
        // the sky to point at the SAME dying satellite. Shipped law: keep
        // riding the old branch with the setpoint clamped at the wall
        // (branch images point identically); fly the handover only when
        // the pattern climbs again (test below).
        TrackerConfig c = Fixture::altaz();
        c.safety.max_dec_travel_deg = 180.0;    // bench sirius.conf profile
        Fixture f(c);
        f.cal();
        const double SPS = 25066.7;             // axis deg -> board steps
        const uint32_t datum = f.sim->sim_position('1') & 0xFFFFFFu;
        const uint32_t datum_de = f.sim->sim_position('2') & 0xFFFFFFu;
        auto wrap24 = [](int32_t v) {         // counter deltas live in
            while (v > 0x7FFFFF) v -= 0x1000000;   // 24-bit modular space
            while (v < -0x800000) v += 0x1000000;  // (2^24 steps = 669 deg!)
            return v;
        };
        auto ra_off = [&](void) {              // signed RA offset in deg
            int32_t d = wrap24((int32_t)((f.sim->sim_position('1') & 0xFFFFFFu) - datum));
            return (double)d / SPS;
        };
        // Seed: mount riding the FLIP branch at the corridor top, tube at
        // the (179.6, el 4) flip solution axes (RA offset +103.4, DEC
        // -92.4) — the pose the live session reached by following the
        // pass from an el-0 pre-aim whose canonical solution was outside
        // the RA wall (counter-seeded instead of flown: the flown path is
        // not the mechanism under test). Ramped under the 30 deg/tick
        // counter-jump detector bound.
        const double ra0 = 90.0 + 103.4;
        const int seed_n = 10;
        for (int i = 1; i <= seed_n; ++i)
        {
            f.board->set_position(AxisRA, datum + (uint32_t)std::llround((ra0 - 90.0) * SPS * i / seed_n));
            f.board->set_position(AxisDE, datum_de - (uint32_t)std::llround(92.4 * SPS * i / seed_n));
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(std::fabs(ra_off() - 103.4) < 1.0);
        // First push adopts the ridden branch (distance ~0: no far-branch
        // event), then descend on a 2 s push cadence at a realistic LOS
        // rate; crossing el ~2.35 pushes the flip offset past 105 + grace
        // -> the forced handover fires WITH a setting pattern -> DEFER,
        // and the setpoint slides at the wall.
        double min_ra = 1e9;
        bool tripped_printed = false;
        for (double el = 4.0; el >= 1.0 - 1e-9; el -= 0.02)
        {
            f.target(179.6, el);
            for (int i = 0; i < 2; ++i)
            {
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
        }
        for (int i = 0; i < 400; ++i)          // hold at el 1.0 (settling)
        {
            if (i % 20 == 0) f.target(179.6, 1.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.travel_returning && !tripped_printed)
            {
                tripped_printed = true;
                (void)std::printf("  los-defer TRIP: RA %.1f reason=[%s]\n",
                                  ra_off(), s.travel_reason.c_str());
            }
            CHECK(!s.travel_returning);
            CHECK(!s.halted);
            min_ra = std::min(min_ra, ra_off());
        }
        // The teleport would have wound RA from +105 to -68 (a 173 deg
        // dive). Riding the wall keeps the offset pinned at the wall.
        CHECK(min_ra > 95.0);
        CHECK(ra_off() > 99.0);
        auto s = f.tr.snapshot();
        // pointing stays near the corridor: the wall costs a bounded
        // following error on a setting satellite, never a sky crossing.
        CHECK(std::fabs(s.current.az_deg - 179.6) < 3.0);
        CHECK(s.current.el_deg > -4.0);
        (void)std::printf("  los-defer: RA off %.1f, pointing (%.2f, %.2f)\n",
                          ra_off(), s.current.az_deg, s.current.el_deg);
    }

    TEST("branch handover: climbing pattern FLIES the handover as a fresh dive");
    {
        // Same seeded corridor scene; after the deferral armed at el 1.0,
        // push the target CLIMBING to el 2.2 on the 2 s cadence (flip
        // still beyond the wall: +112, pattern clearly up). The handover
        // must now execute: dive ~170 deg to the canonical image and
        // converge — proof the deferral is not "never flip".
        TrackerConfig c = Fixture::altaz();
        c.safety.max_dec_travel_deg = 180.0;
        Fixture f(c);
        std::vector<std::string> fly_log;
        f.tr.set_log_sink([&](const std::string& s) { fly_log.push_back(s); });
        f.cal();
        const double SPS = 25066.7;
        const uint32_t datum = f.sim->sim_position('1') & 0xFFFFFFu;
        const uint32_t datum_de = f.sim->sim_position('2') & 0xFFFFFFu;
        auto ra_off = [&](void) {
            int32_t d = (int32_t)((f.sim->sim_position('1') & 0xFFFFFFu) - datum);
            while (d > 0x7FFFFF) d -= 0x1000000;    // 24-bit modular (669 deg)
            while (d < -0x800000) d += 0x1000000;
            return (double)d / SPS;
        };
        const double ra0 = 90.0 + 103.4;
        for (int i = 1; i <= 10; ++i)
        {
            f.board->set_position(AxisRA, datum + (uint32_t)std::llround((ra0 - 90.0) * SPS * i / 10));
            f.board->set_position(AxisDE, datum_de - (uint32_t)std::llround(92.4 * SPS * i / 10));
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        for (double el = 4.0; el >= 1.0 - 1e-9; el -= 0.02)
        {
            f.target(179.6, el);
            for (int i = 0; i < 2; ++i)
            {
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
        }
        for (int i = 0; i < 100; ++i)           // hold el 1.0: deferral armed
        {
            if (i % 20 == 0) f.target(179.6, 1.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(ra_off() > 95.0);
        for (int i = 0; i < 20; ++i)            // climb pushes: el 2.2, 2 s
        {
            f.target(179.6, 2.2);
            f.advance(2.0);
            f.tr.step(f.utc(), f.mono());
        }
        for (int i = 0; i < 3000; ++i)          // settle the handover dive
        {
            if (i % 20 == 0) f.target(179.6, 2.2);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        auto s = f.tr.snapshot();
        CHECK(!s.travel_returning);
        CHECK(!s.halted);
        for (auto& l : fly_log)
            if (l.find("branch") != std::string::npos)
                (void)std::printf("  los-fly LOG: %s\n", l.c_str());
        double err = std::hypot(s.current.az_deg - 179.6, s.current.el_deg - 2.2);
        CHECK(err < 0.5);
        // Handover flew: the mount settled on the CANONICAL image — its
        // DEC sits near -88 (el 2.2), while the flip branch it deferred
        // on rides DEC ~-92. The long dive winds RA past the 24-bit
        // counter wrap, so the wrap-folded DEC offset and pointing are
        // the branch discriminators.
        int32_t de_raw = (int32_t)((f.sim->sim_position('2') & 0xFFFFFFu) - datum_de);
        while (de_raw > 0x7FFFFF) de_raw -= 0x1000000;   // 24-bit modular
        while (de_raw < -0x800000) de_raw += 0x1000000;
        double de_off = (double)de_raw / SPS;
        CHECK(std::fabs(de_off + 88.0) < 2.5);
        (void)std::printf("  los-fly: DEC off %.1f, err %.3f\n", de_off, err);
    }

    TEST("envelope: disabled limits (0) restore classic unlimited-wind behavior");
    {
        TrackerConfig c = Fixture::altaz();
        c.safety.max_ra_travel_deg = 0.0;
        c.safety.max_dec_travel_deg = 0.0;
        Fixture f(c);
        f.cal();
        bool refused = false;
        for (int i = 0; i < 300; ++i)
        {
            if (i % 10 == 0) f.target(270.0, 40.0);   // the refused target...
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            refused = refused || f.tr.snapshot().travel_returning;
        }
        CHECK(!refused);                              // ...now just a target
        CHECK(f.any_motion_start());
    }

    // ---- --debug tick tracing ------------------------------------------------
    TEST("--debug tick rows: exactly one per real tick, all exit paths, wire segment");
    {
        Fixture f;
        std::vector<std::string> rows;
        f.tr.set_debug_sink([&](const std::string& r) { rows.push_back(r); });
        f.tr.set_debug(true);

        // un-calibrated ticks produce NO row (no board I/O happened either)
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        CHECK(rows.empty());

        f.cal();
        // no target yet: step_impl takes the stale/none early return, yet
        // the autopsy row must STILL be emitted (a halt row is the point)
        for (int i = 0; i < 5; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK_EQ(rows.size(), 5u);
        if (rows.size() >= 5)
        {
            for (const auto& r : rows)
            {
                CHECK(r.rfind("[TICK] ", 0) == 0);
                CHECK(r.find(" seq=") != std::string::npos);
                CHECK(r.find(" tg=none") != std::string::npos);
                CHECK(r.find(" cur=") != std::string::npos);
                CHECK(r.find(" srk=") != std::string::npos);
                CHECK(r.find(" des=") != std::string::npos); // poisoned-NaN, not stale
                // observer effect: the :f poll per axis is in this tick's wire
                CHECK(r.find(":f1") != std::string::npos);
                CHECK(r.find(":f2") != std::string::npos);
                // wire segment non-empty: the :j reads happened before the return
                CHECK(r.find(" [:") != std::string::npos);
            }
        }

        // full-path ticks with a far target: dive produces motion frames and
        // finite setpoint/error fields
        rows.clear();
        f.target(200.0, 55.0);
        for (int i = 0; i < 30; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK_EQ(rows.size(), 30u);
        bool saw_j = false;
        for (const auto& r : rows)
            if (r.find(" [:J") != std::string::npos) saw_j = true;
        CHECK(saw_j);
        const std::string& last = rows.back();
        CHECK(last.find("des=nan") == std::string::npos &&
              last.find("des=-nan") == std::string::npos); // full path: finite
        CHECK(last.find(" tg=none") == std::string::npos);
        CHECK(last.find(" err=") != std::string::npos);
        CHECK(last.find(" halt=") == std::string::npos); // clean dive, no halt

        // toggle OFF: no more rows AND the board wire trace stops accumulating
        f.tr.set_debug(false);
        for (int i = 0; i < 3; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK_EQ(rows.size(), 30u);
    }

    // ---- 12. coast-aware wire commits (live AO-73 ping-pong) --------------
    // These two twins run the FULL physical :K coast fixture (sim set_coast,
    // pinned in board_sim) with the landing gate ON (high_coast_s 0.3) vs
    // OFF (0 = legacy). They pin NO-REGRESSION: a setpoint-tracking dive and
    // a setpoint-reversal glide must converge identically with real coast
    // physics (the gate only DENIES brake commits whose coast cannot reach
    // the setpoint — the AO-73 pattern-reversal ping-pong class is exercised
    // by the FO-29/AO-73 replay, the micropush scenario below, and the live
    // pass; a plain arrival never triggers the gate by design).
    TEST("coast gate: dive terminal with physical :K coast — fixed overshoot vs legacy");
    {
        // The sim :K coast fixture (board_sim test) makes HIGH commits cost
        // ~coast distance of real counter motion — the replay blind spot
        // that hid the AO-73 coast-pumped ping-pong. A dive must now flip
        // its braking commit no later than the coast can land on the
        // setpoint; the overshoot past a full-authority dive is bounded by
        // the landing model instead of riding the coast through.
        // overshoot/endpoint collected inside the twin loop below
        auto run_peak = [](double high_coast_s, double& peak, double& fin)
        {
            TrackerConfig c = Fixture::altaz();
            c.target_stale_s = 3600.0;
            c.high_coast_s = high_coast_s;
            Fixture f(c);
            f.cal();
            f.sim->set_coast('1', 0.3);
            f.sim->set_coast('2', 0.3);
            peak = 0.0;
            for (int i = 0; i < 1200; ++i)
            {
                if (i % 20 == 0) f.target(90.0, 6.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                peak = std::max(peak, f.tr.snapshot().current.el_deg);
            }
            fin = f.tr.snapshot().current.el_deg;
        };
        double peak_fixed = 0, fin_fixed = 0, peak_legacy = 0, fin_legacy = 0;
        run_peak(0.3, peak_fixed, fin_fixed);
        run_peak(0.0, peak_legacy, fin_legacy);
        fprintf(stderr, "[c-dive] fixed peak=%.2f final=%.3f legacy peak=%.2f final=%.3f\n",
                peak_fixed, fin_fixed, peak_legacy, fin_legacy);
        CHECK_NEAR(fin_fixed, 6.0, 0.4); // converges like before
        CHECK(peak_fixed < 7.5);         // bounded terminal
    }

    TEST("coast gate: HIGH flip denied while the coast would land short of the setpoint");
    {
        // Landing test (tracker.h:high_coast_s): a braking commit (flip or
        // stop) on a running HIGH axis is held when the signed setpoint
        // error is still farther ahead than coast+margin — a commit there
        // cannot reach its own goal, it is disturbance. Twin runs share
        // the physical coast fixture; legacy (gate 0) commits the early
        // flip and UNDERSHOOTS short of the setpoint by ~one coast, the
        // fixed law keeps running and lands. Discriminator: the legacy
        // trace reverses while still > coast ahead; the fixed trace never
        // gives that ground back.
        auto run = [](double high_coast_s, double& max_err, double& final_err)
        {
            TrackerConfig c = Fixture::altaz();
            c.target_stale_s = 3600.0;
            c.high_coast_s = high_coast_s;
            Fixture f(c);
            f.cal();
            f.sim->set_coast('1', 0.3);
            f.sim->set_coast('2', 0.3);
            for (int i = 0; i < 300; ++i) // pre-aim settle at el 8
            {
                if (i % 20 == 0) f.target(90.0, 8.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            // Reversal: target steps to el 5.5 — the axis must come back
            // DOWN. While running UP the landing test governs when the
            // flip may commit; the fixed law may only commit where the
            // coast lands (never stranding > coast + margin short). Track
            // the largest transient undershoot past the NEW setpoint on
            // the way down (flip-too-early strands ABOVE it on descent).
            max_err = 0.0;
            for (int i = 0; i < 800; ++i)
            {
                if (i % 20 == 0) f.target(90.0, 5.5);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                max_err = std::max(max_err, f.tr.snapshot().current.el_deg - 5.5);
            }
            final_err = std::fabs(f.tr.snapshot().current.el_deg - 5.5);
        };
        double mx_fixed = 0, fin_fixed = 0, mx_legacy = 0, fin_legacy = 0;
        run(0.3, mx_fixed, fin_fixed);
        run(0.0, mx_legacy, fin_legacy);
        fprintf(stderr, "[c-flip] fixed hold=%.2f final=%.3f legacy hold=%.2f final=%.3f\n",
                mx_fixed, fin_fixed, mx_legacy, fin_legacy);
        CHECK_NEAR(fin_fixed, 0.0, 0.35);
        CHECK_NEAR(fin_legacy, 0.0, 0.35);
    }

    // ---- 13. re-home after pass complete -----------------------------------
    TEST("rehome: target-gone idle self-pushes the datum, parks, and stays parked");
    {
        TrackerConfig c = Fixture::altaz();
        c.target_stale_s = 5.0;
        c.rehome_idle_s = 5.0;
        Fixture f(c);
        f.cal();
        for (int i = 0; i < 300; ++i) // pass pose az 120 el 20
        {
            if (i % 20 == 0) f.target(120.0, 20.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking);
        // client silence: stale-halt at t=5, rehome fires at t=10
        bool flew_home = false;
        int i = 0;
        for (; i < 1500 && !flew_home; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.tracking) flew_home = true; // daemon-owned motion
            CHECK(!s.halted || !s.tracking);  // never claim tracking while halted
        }
        CHECK(flew_home); // flew_home
        // arrive: parked at the datum and halted by the normal path
        bool parked = false;
        for (; i < 3000; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (!s.tracking && s.halted &&
                std::fabs(s.current.az_deg - 90.0) < 0.8 &&
                std::fabs(s.current.el_deg - 0.0) < 0.8)
            {
                parked = true;
                break;
            }
        }
        CHECK(parked);
        // done-flag: no second flight after another idle period
        const uint32_t park_ra = f.sim->sim_position('1');
        const uint32_t park_de = f.sim->sim_position('2');
        for (int k = 0; k < 1000; ++k)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK_EQ(f.sim->sim_position('1'), park_ra);
        CHECK_EQ(f.sim->sim_position('2'), park_de);
    }

    TEST("rehome: real client push mid-flight cancels and wins; stop suppresses; reset re-arms");
    {
        TrackerConfig c = Fixture::altaz();
        c.target_stale_s = 5.0;
        c.rehome_idle_s = 5.0;
        {
            // cancel mid-flight: a fresh push re-owns the target
            Fixture f(c);
            f.cal();
            for (int i = 0; i < 300; ++i)
            {
                if (i % 20 == 0) f.target(120.0, 20.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            bool flying = false; // fly until rehome takes over (arrival at
            for (int i = 0; i < 400 && !flying; ++i) // the datum would also
            {                                       // end the flight early
                f.advance(0.1);                      // — any owned tick works:
                f.tr.step(f.utc(), f.mono());        // the push below must win
                flying = f.tr.snapshot().tracking;
            }
            CHECK(flying); // rehome owns it
            for (int i = 0; i < 600; ++i)    // client returns: az 60 el 15
            {
                if (i % 20 == 0) f.target(60.0, 15.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            auto s = f.tr.snapshot();
            CHECK(s.tracking);
            CHECK(std::fabs(s.current.az_deg - 60.0) < 1.5); // chases the push
            CHECK(std::fabs(s.current.az_deg - 90.0) > 5.0); // NOT home
        }
        {
            // explicit stop suppresses; R re-arms
            Fixture f(c);
            f.cal();
            for (int i = 0; i < 300; ++i)
            {
                if (i % 20 == 0) f.target(120.0, 20.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            f.tr.on_stop(); // operator: stay put
            const uint32_t stop_ra = f.sim->sim_position('1');
            for (int i = 0; i < 1200; ++i) // 120 s: >> stale+idle, no flight
            {
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            CHECK(!f.tr.snapshot().tracking);
            CHECK_EQ(f.sim->sim_position('1'), stop_ra);
            f.tr.on_reset(); // operator back: re-armed
            bool flew = false;
            for (int i = 0; i < 2000 && !flew; ++i)
            {
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                if (f.tr.snapshot().tracking) flew = true;
            }
            CHECK(flew); // rehome re-armed and flying
        }
    }

    TEST("rehome: FLIP-pose homeflight rewinds direct (no routed detour)");
    {
        // live 2026-11-20 incident: re-home fired while the tube
        // rode the FLIP branch at the LOS pose (az 2.3 el 8.5). The old
        // code force-routed the datum through canonical hs waypoints
        // (climb to el 81, sweep): the waypoint's canonical axis solve
        // sits beyond the RA wall from the flip side, the belt clamp
        // pinned the setpoint at the wall, and RA wound the LONG way
        // (~265 deg at -600x, el sliding through the low north corridor)
        // where the correct move was a ~95 deg rewind. Shipped law: a
        // homing flight that starts flip-riding skips routing
        // (home_no_route_) and flies the wrap-aligned datum as a fresh
        // far-branch handover dive.
        TrackerConfig c = Fixture::altaz();
        c.target_stale_s = 2.0;
        c.rehome_idle_s = 2.0;
        Fixture f(c);
        std::vector<std::string> lg;
        f.tr.set_log_sink([&](const std::string& s) { lg.push_back(s); });
        f.cal();
        const double SPS = 25066.7;
        const uint32_t datum = f.sim->sim_position('1') & 0xFFFFFFu;
        const uint32_t datum_de = f.sim->sim_position('2') & 0xFFFFFFu;
        auto wrap24 = [](int32_t v) {
            while (v > 0x7FFFFF) v -= 0x1000000;
            while (v < -0x800000) v += 0x1000000;
            return v;
        };
        auto ra_off = [&](void) {
            return (double)wrap24((int32_t)((f.sim->sim_position('1') & 0xFFFFFFu) - datum)) / SPS;
        };
        // Ride the flip branch at the LOS-like corridor pose (same seed
        // as the los-defer test: RA offset +103.4, DEC -92.4; target
        // (179.6, 4.0) whose flip solution is inside the +-105 wall).
        for (int i = 1; i <= 10; ++i)
        {
            f.board->set_position(AxisRA, datum + (uint32_t)std::llround(103.4 * SPS * i / 10));
            f.board->set_position(AxisDE, datum_de - (uint32_t)std::llround(92.4 * SPS * i / 10));
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        for (int i = 0; i < 800; ++i)          // settle on the corridor target
        {
            if (i % 20 == 0) f.target(179.6, 4.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(ra_off() > 95.0);                // confirmed flip-riding
        // Client silence: stale-halt at t=2, re-home trigger at t=4.
        double max_off = -1e9, min_off = 1e9;  // envelope watch mid-flight
        bool flew = false;
        for (int i = 0; i < 400 && !flew; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            CHECK(!s.travel_returning);        // rewind stays in the box
            CHECK(!(s.halted && s.tracking));
            if (s.tracking) flew = true;
        }
        CHECK(flew);                           // daemon-owned homeflight
        // Arrive at the datum: the flight must stay inside the envelope
        // (the long way could only wind out through the wall) and land.
        bool parked = false;
        for (int i = 0; i < 6000 && !parked; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            CHECK(!s.travel_returning);
            CHECK(!s.halted || !s.tracking);
            max_off = std::max(max_off, ra_off());
            min_off = std::min(min_off, ra_off());
            if (!s.tracking && s.halted &&
                std::fabs(s.current.az_deg - 90.0) < 0.8 &&
                std::fabs(s.current.el_deg - 0.0) < 0.8)
                parked = true;
        }
        CHECK(parked);
        CHECK(max_off < 110.0);                // never wall-pinned the far way
        CHECK(min_off > -110.0);
        // Narrative: the flip rewind latched once.
        int rewind_lines = 0;
        for (auto& l : lg)
            if (l.find("rewinding to datum direct") != std::string::npos)
                ++rewind_lines;
        CHECK_EQ(rewind_lines, 1);
        // Operator rule (2026-11-20): after ARRIVAL, no new homeflight
        // until a real move command re-arms the idle cycle.
        const uint32_t home_ra = f.sim->sim_position('1');
        for (int i = 0; i < 3000; ++i)         // 300 s of silence: 150x idle
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            CHECK(!f.tr.snapshot().tracking);
        }
        CHECK_EQ(f.sim->sim_position('1'), home_ra);
        // A real pass far away, then gone: the idle cycle re-arms and the
        // (canonical-riding) homeflight routes normally again.
        for (int i = 0; i < 400; ++i)
        {
            if (i % 20 == 0) f.target(120.0, 20.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        flew = false;
        for (int i = 0; i < 2000 && !flew; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            if (f.tr.snapshot().tracking) flew = true;
        }
        CHECK(flew);
        (void)std::printf("  rehome-flip: off range [%.1f, %.1f]\n", min_off, max_off);
    }

    TEST("rehome el-gate: pre-aim-only never arms; a real pass arms once");
    {
        // live 2026-11-20: the first re-home enable fired in the
        // gap between a GT2 el-0 pre-aim and real AOS — 60 s of legit
        // pre-pass silence read as "pass over" and the mount unwound
        // 146 deg home moments before acquisition. Shipped law: only a
        // push with el > rehome_min_el_deg (default 2) arms the idle
        // clock; el-0 pushes (pre-aims, post-LOS ghosts) cancel but
        // neither arm NOR disarm.
        TrackerConfig c = Fixture::altaz();
        c.target_stale_s = 2.0;
        c.rehome_idle_s  = 2.0;   // rehome_min_el_deg stays default 2.0
        Fixture f(c);
        f.cal();
        // Phase 1: pre-aim ONLY (el 0, exactly GT2's rotorPoint(aosAz,0)),
        // then silence: stale-halt, and the idle clock must NEVER fire.
        for (int i = 0; i < 200; ++i)
        {
            if (i % 20 == 0) f.target(236.0, 0.0); // GT2 pre-aim: el 0
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking); // parked on the pre-aim pose
        const uint32_t pre_ra = f.sim->sim_position('1');
        const uint32_t pre_de = f.sim->sim_position('2');
        for (int i = 0; i < 3000; ++i) // 300 s: 150x idle past stale
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            CHECK(!f.tr.snapshot().tracking); // never armed -> never flies
        }
        CHECK_EQ(f.sim->sim_position('1'), pre_ra);
        CHECK_EQ(f.sim->sim_position('2'), pre_de);
        // Phase 2: a REAL pass arrives (el > gate): normal follow.
        for (int i = 0; i < 300; ++i)
        {
            if (i % 20 == 0) f.target(120.0, 25.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking);
        // Phase 3: post-LOS ghost tail (el-0 pushes keep coming after the
        // real LOS): they must NOT disarm the flight the real pass earned...
        for (int i = 0; i < 200; ++i)
        {
            if (i % 20 == 0) f.target(300.0, 0.0); // GT2 el-0 ghosts
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        // ...then ghosts stop (GT2 follow-gate closes), stale-halt, idle:
        // the homeflight flies.
        bool flew = false;
        for (int i = 0; i < 2000 && !flew; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            CHECK(!(s.halted && s.tracking));
            if (s.tracking) flew = true;
        }
        CHECK(flew); // ghost tail did not veto the unwind flight
        // Park at the datum, then GT2 pre-aims AGAIN for the next pass:
        // arrival disarmed, so the pre-aim quiet must stay quiet.
        bool parked = false;
        for (int i = 0; i < 6000 && !parked; ++i)
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (!s.tracking && s.halted &&
                std::fabs(s.current.az_deg - 90.0) < 0.8 &&
                std::fabs(s.current.el_deg - 0.0) < 0.8)
                parked = true;
        }
        CHECK(parked);
        for (int i = 0; i < 200; ++i)
        {
            if (i % 20 == 0) f.target(210.0, 0.0); // next pass pre-aim
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        const uint32_t home_ra = f.sim->sim_position('1');
        for (int i = 0; i < 3000; ++i) // must stay parked: pre-aim quiet
        {
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            CHECK(!f.tr.snapshot().tracking);
        }
        CHECK_EQ(f.sim->sim_position('1'), home_ra);
    }

    // ---- cord wrap law (operator spec 2026-11-21): DEC cord ledger ------
    TEST("cord window: el-0 ghost at the RA wall refuses and unwinds home");
    {
        // Live forensics (daemon-1120k TICK rows 21:29:58+): riding the flip
        // branch at the RA wall (+117 of +115) with DEC ledger -574.6, the
        // el-0 ghost's branch-selected solution (335.0, -574.6) sat ACROSS
        // the RA meridian: demand outward at the wall, axis err 127.98 deg,
        // x=0 flags=trk forever — the fold box called the target legal and
        // stale-halt never fires (GT2 keeps pushing). Law (wrap clauses
        // 2+3, operator spec 2026-11-21): the ghost's only fold-legal
        // branch lands at DEC -685 — PAST the +-270 cord window AND
        // DEEPER than the wounded -574.6 (a dig) — while the flip image
        // sits across the RA meridian, so no branch is legal and the
        // envelope refuses: non-latching return home on the CORD-FREE
        // route (clause 3; the -720 cur-aligned image that points
        // identically is what an unrouted flight landed on, leaving SO-50
        // wound), resuming on any cord-legal push — which here recaptures
        // the ghost itself at its legal landing once the ledger is back
        // inside the window (no freeze, no latch, no 11-min x=0 stall).
        // (The wall-strain watchdog in the handover deferral remains the
        // backstop for the ride-then-drift class; the along-wall test
        // below guards it against false trips.)
        TrackerConfig c = Fixture::altaz();
        c.safety.max_dec_travel_deg = 180.0;
        c.safety.max_ra_travel_deg  = 115.0;   // bench sirius.conf
        Fixture f(c);
        std::vector<std::string> log;
        f.tr.set_log_sink([&](const std::string& s) { log.push_back(s); });
        f.cal();
        const double SPS = 25066.7;
        const uint32_t datum = f.sim->sim_position('1') & 0xFFFFFFu;
        const uint32_t datum_de = f.sim->sim_position('2') & 0xFFFFFFu;
        auto cpos = [&](char a) {
            int32_t d = (int32_t)((f.sim->sim_position(a) & 0xFFFFFFu) -
                                  (a == '1' ? datum : datum_de));
            while (d > 0x7FFFFF) d -= 0x1000000;      // 24-bit modular space
            while (d < -0x800000) d += 0x1000000;
            return d;
        };
        // Seed (counter-ramped under the 30 deg/tick jump bound): the live
        // frozen pose — RA offset +117 (wall+grace), DEC ledger -574.6
        // (deep wrap; hs = az 33.4 el 47.2, the tube's real pointing).
        const int seed_n = 40;
        for (int i = 1; i <= seed_n; ++i)
        {
            f.board->set_position(AxisRA, datum + (uint32_t)std::llround(117.0 * SPS * i / seed_n));
            f.board->set_position(AxisDE, datum_de - (uint32_t)std::llround(574.6 * SPS * i / seed_n));
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(cpos('1') / SPS > 116.0);
        // The ghost keeps pushing at the 2 s client cadence (never stale).
        bool refused = false, returning = false;
        for (int i = 0; i < 600; ++i)         // 60 s of ghost pushes
        {
            if (i % 20 == 0) f.target(55.4, 0.0);   // the live el-0 ghost
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.travel_returning)
            {
                returning = true;
                if (s.travel_reason.find("envelope") != std::string::npos ||
                    s.travel_reason.find("cannot") != std::string::npos)
                    refused = true;
            }
            CHECK(!s.halted);
        }
        CHECK(returning);
        CHECK(refused);           // the wrap-window/branch-box refusal,
                                  // not a stale-halt or a dive
        // Return-home with the cord law: the datum override + routed-leg
        // DEC pin (clause 3) fly the CORD-FREE image — never the -720
        // cur-aligned image that points identically and stays wound.
        for (int i = 0; i < 60000; ++i)
        {
            if (i % 20 == 0) f.target(55.4, 0.0);   // locked ghost persists
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        const double ra_f = cpos('1') / SPS, de_f = cpos('2') / SPS;
        (void)std::printf("  ghost: final RA off %.1f DEC %.1f\n", ra_f, de_f);
        // The refusal flew the cord-free home route (clause 3): RA came
        // back to the datum meridian and the DEC unwound 574 -> the
        // FIRST cord-legal landing, at which point the ghost IS legal
        // (canonical image RA 0, DEC +34.6, landing inside the window)
        // and envelope semantics take over: resume normal tracking, no
        // latch. Ledger 0 itself is only promised when no legal push
        // exists short of it — the ghost's landing at wrap +34.6 is the
        // law's own endpoint here (a cord inside +-270 is the spec's
        // healthy state; keeping it parked against a now-legal target
        // would be the freeze in a cleaner suit).
        CHECK(std::fabs(ra_f) < 2.0);
        CHECK(std::fabs(de_f - 34.6) < 3.0);
        // A legal target resumes tracking (non-latching, envelope semantics).
        for (int i = 0; i < 400; ++i)
        {
            if (i % 20 == 0) f.target(120.0, 30.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        auto s = f.tr.snapshot();
        CHECK(!s.travel_returning);
        CHECK(std::fabs(s.current.az_deg - 120.0) < 2.0);
        CHECK(std::fabs(s.current.el_deg - 30.0) < 2.0);
    }

    TEST("wall strain: a legal along-wall slide never trips the watchdog");
    {
        // The los-defer corridor ride (above) IS the wall-slide pin; this
        // test adds the deep-wrap variant the freeze hinged on: same kind
        // of corridor slide but the ledger at -500 deg (a legacy wound
        // state from pre-wrap-law nights). Geometry (probe lat 44.95):
        // az 179.6 el 30->2 has canonical RA offset -89 -> -79 (in box)
        // and flip +91 -> +101 (in box) — the ride never crosses a wall;
        // BOTH branch landings sit past the +-270 window (-420/-480 down
        // to -448/-452) but every one of them is SHALLOWER than the
        // wounded cur, i.e. the slide UNWINDS. The wind-deeper exclusion
        // must never refuse this (the law keeps riding toward cord
        // health); a sibling-comparison exclusion was tried and rejected
        // — the two landings agree within ~4 deg here and the tie kicked
        // the RIDDEN branch out, force-fed a ~190 deg handover and then
        // the envelope refusal (the very rs44-teleport class). The
        // wall-strain watchdog (in the handover deferral) must stay
        // asleep: no handover, no pin, no strain.
        TrackerConfig c = Fixture::altaz();
        c.safety.max_dec_travel_deg = 180.0;
        c.safety.max_ra_travel_deg  = 115.0;
        Fixture f(c);
        f.cal();
        const double SPS = 25066.7;
        const uint32_t datum = f.sim->sim_position('1') & 0xFFFFFFu;
        const uint32_t datum_de = f.sim->sim_position('2') & 0xFFFFFFu;
        const int seed_n = 40;
        for (int i = 1; i <= seed_n; ++i)
        {
            f.board->set_position(AxisRA, datum + (uint32_t)std::llround(113.0 * SPS * i / seed_n));
            f.board->set_position(AxisDE, datum_de - (uint32_t)std::llround(500.0 * SPS * i / seed_n));
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        // Slow descent then hold — the along-wall corridor (az fixed, el
        // down) at deeper wrap, los-defer cadence.
        for (double el = 30.0; el >= 2.0 - 1e-9; el -= 0.05)
        {
            f.target(179.6, el);
            for (int i = 0; i < 2; ++i)
            {
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
        }
        for (int i = 0; i < 400; ++i)
        {
            if (i % 20 == 0) f.target(179.6, 2.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
            auto s = f.tr.snapshot();
            if (s.travel_returning)
                (void)std::printf("  along-wall TRIP reason=[%s]\n",
                                  s.travel_reason.c_str());
            CHECK(!s.travel_returning);   // along-wall slide: legal, stay on
            CHECK(!s.halted);
        }
        // Liveness (not vacuous): the ride must actually FOLLOW the corridor,
        // i.e. land on the target pointing. (Ledger depth is not observable
        // from 24-bit counters — the wind-deeper exclusion is pinned by the
        // ghost test above; here its ABSENCE is the assertion: no refusal,
        // no return-home on an unwinding slide.)
        auto s = f.tr.snapshot();
        CHECK(std::fabs(s.current.az_deg - 179.6) < 2.0);
        CHECK(std::fabs(s.current.el_deg - 2.0) < 2.0);
    }

    // ---- 14. cord-unwind datum flight: azimuth-first leg order ------------
    TEST("re-home from a cord-wound pose unwinds azimuth-first (never floor-dips)");
    {
        // Live 2026-11-22 (LATE-NIGHT SKIP DOUBLE-LATCH route forensics):
        // the cord-unwind datum flight flew the DIRECT un-routed axis line
        // (routing skipped whenever |wrap| > 180). From the live latch-pose
        // family (RA offset ~+96, DEC ledger 168.5..195, pointing high in
        // the W half-sky) that line passes 12..18 deg UNDER the elevation
        // floor (probe against production astro): the floor guard slid the
        // path at el ~-2 while RA kept its 600x authority through the
        // slide and OVERSHOT its datum share by ~50 deg az, then a
        // climb-leg re-route wound it back at elevation. Operator fix:
        // the unwind flies AZIMUTH-FIRST — RA goes home while DEC holds
        // the ledger (leg 1), then DEC unwinds at datum RA (leg 2, the
        // operator's free-spin regime). Geometry (probe): the datum RA
        // meridian IS the horizon circle (el ≡ 0 for every ledger) and
        // the RA-home leg at held ledger never dips across the winding
        // band, so the leg law's min el is ~0 — the residual graze is the
        // meridian descent's RA-error × depth coupling (el ≈ -(90-L)·err²/2
        // → -0.2 at settled RA). Seeds ledger +195 (NOT the +168.5 live
        // number): the LEGACY twin must sit past the old-law >180 routing
        // skip to reproduce the straight-line regime at all — at 168 the
        // legacy path routes (floor-safe) and the A/B grades nothing.
        // Also pins the envelope-resume gate: while the re-home self-push
        // owns the target, the datum self-solve is trivially branch-OK
        // and used to clear the route-skip latch one tick after trigger,
        // handing the leg flight to the canonical hs router (the
        // (90,12)-waypoint hijack).
        const double SPS = 25066.7;
        struct Run { bool latched, ra_first, arrived; double min_el, t; };
        auto cord_run = [&](bool azf) -> Run
        {
            TrackerConfig c = Fixture::altaz();   // datum az 90 el 0
            // Bench profile truth (sirius.conf): RA wall 115, DEC is the
            // free axis at 180. The TrackerConfig default (105 both) would
            // put the seeded pose (folded DEC 165) itself OUTSIDE the DEC
            // box — the envelope would treat the mount's own current pose
            // as a refusal case and this test would exercise the refusal
            // return, not the re-home unwind, from tick one.
            c.safety.max_ra_travel_deg = 115.0;
            c.safety.max_dec_travel_deg = 180.0;
            c.cord_unwind_route = azf;
            c.target_stale_s = 5.0;
            c.rehome_idle_s = 5.0;    // short clock: the law under test is
                                      // the flight plan, not the timeout
            Fixture f(c);
            f.cal();
            const uint32_t datum = f.board->position('1');
            const uint32_t datum_de = f.board->position('2');
            auto off = [](uint32_t now, uint32_t ref, double sps) {
                int32_t d = (int32_t)((now & 0xFFFFFFu) - (ref & 0xFFFFFFu));
                while (d > 0x7FFFFF) d -= 0x1000000;      // 24-bit modular
                while (d < -0x800000) d += 0x1000000;
                return (double)d / sps;
            };
            auto ra_off  = [&]() { return off(f.board->position('1'), datum, SPS); };
            auto de_off  = [&]() { return off(f.board->position('2'), datum_de, SPS); };
            // Seed the live latch pose (ramped under the 30 deg/tick
            // counter-jump bound, stepping between increments like the
            // handover tests).
            const int seed_n = 10;                    // <= 25 deg/tick
            for (int i = 1; i <= seed_n; ++i)
            {
                f.board->set_position(AxisRA, datum + (uint32_t)std::llround(96.2 * SPS * i / seed_n));
                f.board->set_position(AxisDE, datum_de + (uint32_t)std::llround(195.0 * SPS * i / seed_n));
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            CHECK(ra_off() > 95.0);
            CHECK(de_off() > 194.0);
            // Arm re-home with a real (el > rehome_min_el) pass push, then
            // go silent: stale-halt at ~t=10, re-home self-push ~5 s later.
            for (int i = 0; i < 40; ++i)
            {
                if (i % 10 == 0) f.target(140.0, 30.0);
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
            }
            Run r{false, false, false, 1e9, 1e9};
            double t_start = -1, de_hold = 1e9;   // ledger when RA first got home
            for (int i = 0; i < 6000; ++i)
            {
                f.advance(0.1);
                f.tr.step(f.utc(), f.mono());
                auto s = f.tr.snapshot();
                double now = f.mono().time_since_epoch().count() / 1e9;
                if (s.halted && s.halt_reason.find("skip") != std::string::npos)
                    { r.latched = true; return r; }
                if (!s.tracking) continue;        // flight not started yet
                if (t_start < 0) t_start = now;
                r.min_el = std::min(r.min_el, s.current.el_deg);
                // Leg 1 signature: the DEC ledger is PINNED (cord held)
                // until RA arrives — the ledger still sits ~seed-deep
                // when RA first gets home. The straight chord winds both
                // axes together: DEC has already unwound ~half its share
                // (the seed ratio) by then. de_hold < 150 proves the
                // chord; > 165 proves the leg.
                if (ra_off() < 8.0) de_hold = std::min(de_hold, de_off());
                if (de_hold > 165.0) r.ra_first = true;
                if (ra_off() < 0.5 && std::fabs(de_off()) < 0.5)
                {
                    r.arrived = true;
                    r.t = now - t_start;
                    return r;
                }
            }
            return r;
        };
        Run a = cord_run(true);     // shipped law: azimuth-first legs
        CHECK(!a.latched);
        CHECK(a.arrived);
        CHECK(a.ra_first);          // cord held until RA home, then unwind
        CHECK(a.min_el > -1.0);     // never meets the floor (analysis: 0.00)
        CHECK(a.t < 200.0);         // dive authority like the direct line
        Run d = cord_run(false);    // legacy direct line: the live dip
        CHECK(!d.latched);
        CHECK(d.arrived);
        // The reactive floor guard arrests the slide at ~el −1.3 (its
        // margin) — but the LIVE flight slid along the floor for minutes
        // with RA overshoot; any real deficit vs the legs twin (el ≥ 0
        // all the way, no slide, RA-first arrival) is the regression.
        CHECK(d.min_el < -1.0);
        (void)std::printf("  cord-return: az-first min_el %.2f t %.0f | direct min_el %.2f t %.0f\n",
                          a.min_el, a.t, d.min_el, d.t);
    }

    // ---- 15. board reset mid-session (counter-jump detector) --------------
    // live 2026-11-20: mount power-cycled while the daemon idled;
    // counters re-datum at 0x800000 unobserved, the exit-save re-anchored
    // them against the stale anchor, and the next daemon "resumed" 159 deg
    // away from the tube. The detector must halt+stick (anchor is a lie),
    // survive R, and only a fresh calibrate/restore may re-arm it.
    TEST("counter jump >30 deg: halt, sticky across R, recalibrate re-arms");
    {
        Fixture f(Fixture::altaz());
        f.cal();
        for (int i = 0; i < 100; ++i) // glide in on a real target…
        {
            if (i % 20 == 0) f.target(120.0, 20.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking);
        // …then teleport the raw counters 60 deg (a power cycle mid-slew).
        const uint32_t ra = f.tr.live_count_ra();
        f.board->set_position(AxisRA, ra + 1500000u); // ~59.8 deg
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        auto s = f.tr.snapshot();
        CHECK(s.halted);
        CHECK(s.halt_reason.find("counter jumped") != std::string::npos);
        CHECK(f.tr.board_reset_latched());
        // A fresh legal target must NOT restart motion (authority):
        for (int i = 0; i < 100; ++i)
        {
            if (i % 20 == 0) f.target(100.0, 30.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().halted);
        // R clears skip latches but NOT this one — no trusted pose remains:
        f.tr.on_reset();
        f.advance(0.1);
        f.tr.step(f.utc(), f.mono());
        CHECK(f.tr.snapshot().halted);
        // The ritual: re-home the mount, restart fresh => calibrate re-arms.
        f.tr.calibrate(f.utc(), f.mono());
        CHECK(!f.tr.board_reset_latched());
        for (int i = 0; i < 200; ++i)
        {
            if (i % 20 == 0) f.target(95.0, 10.0);
            f.advance(0.1);
            f.tr.step(f.utc(), f.mono());
        }
        CHECK(f.tr.snapshot().tracking); // live again
        CHECK(!f.tr.snapshot().halted);
    }

    return testkit::summary("tracker");
}
