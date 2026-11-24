// Replay a recorded GT2 push cadence (sateqd --record CSV) against the
// Tracker + board simulator on a virtual clock, and report:
//   * pointing error vs the latest pushed target (az/el)
//   * wire cost: :I in-place rate writes, :G reprograms, :K/:L stops
//   * rate smoothness: largest commanded-rate step per axis
//   * tracker events: floor guard / rate cap / latch / mode switches
// usage: replay_gt2 CSV [--sparse] [--trace]
//   --sparse: resample the pushes the way GT2 would with a rotor that is
//   actually following (new target only when it moved >= 1 deg AND >= 2 s
//   since last push); the raw file is the dense "rotator lagging > 1 deg"
//   variant that re-pushes every poll.
// Home pose + site match the live --mount sirius --orientation altaz
// session: tube level east (az 90 el 0), Minneapolis.
#include "sateq/simulator.h"
#include "sateq/tracker.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

using namespace sateq;
using SC = std::chrono::steady_clock;
using UC = std::chrono::system_clock;

namespace {

struct Push { double t, az, el; };

std::vector<Push> load(const std::string& path)
{
    std::ifstream f(path);
    if (!f) { std::fprintf(stderr, "cannot open %s\n", path.c_str()); std::exit(2); }
    std::string line;
    std::getline(f, line); // header
    std::vector<Push> out;
    while (std::getline(f, line))
    {
        std::istringstream ss(line);
        std::string a, b, c, d;
        if (!std::getline(ss, a, ',') || !std::getline(ss, b, ',') ||
            !std::getline(ss, c, ',') || !std::getline(ss, d, ','))
            continue;
        out.push_back({std::stod(b), std::stod(c), std::stod(d)});
    }
    return out;
}

// GT2 iq-work follow throttle (rotor.js, b79ff90), emulated inline in the
// replay loop: raw file pushes are the POLL samples; a push actually goes on
// the wire when >= 2 s since the last one AND either the target moved >= 1 deg
// since the last PUSH or the (simulated) rotor is >= 1 deg behind the target
// (the "a rotator that stops when it isn't told for a while gets there all
// the same" re-push rule). Offline pre-resampling cannot do this because the
// re-push condition depends on the rotor state the replay produces.
struct FollowGate {
    bool have = false;
    double last_t = 0, last_az = 0, last_el = 0;
    static double dist(double a1, double e1, double a2, double e2)
    {
        double daz = std::fabs(std::fmod(a1 - a2 + 540.0, 360.0) - 180.0);
        return std::hypot(daz * std::cos(e1 * 0.0174532938), e1 - e2);
    }
    bool pass(double t, double az, double el,
              double rotor_az, double rotor_el, bool rotor_valid)
    {
        if (!have) { have = true; last_t = t; last_az = az; last_el = el; return true; }
        if (t - last_t < 2.0) return false;
        // LOS/pre-aim marker (el 0) after real tracking: always through.
        if (el <= 0.05 && last_el > 0.05) { last_t = t; last_az = az; last_el = el; return true; }
        if (dist(az, el, last_az, last_el) >= 1.0 ||
            (rotor_valid && dist(az, el, rotor_az, rotor_el) >= 1.0))
        { last_t = t; last_az = az; last_el = el; return true; }
        return false;
    }
};

struct Counters {
    long i_frames = 0, g_frames = 0, k_frames = 0, l_frames = 0, j_starts = 0;
long k_ra = 0, k_de = 0; // per-axis :K attribution (log frame f[2] = axis char)
};

// Count wire frames in a log TAIL (simulator command_log_tail) without
// re-copying the whole log every poll: a 8-hour replay grows the log to
// 100k+ frames and per-poll full copies made the tool O(n^2).
Counters count_since(const std::vector<std::string>& tail, size_t& seen)
{
    Counters c;
    for (const auto& f : tail)
    {
        if (f.size() > 2)
        {
            switch (f[1])
            {
                case 'I': ++c.i_frames; break;
                case 'G': ++c.g_frames; break;
                case 'K':
                    ++c.k_frames;
                    if (f.size() > 2 && f[2] == '1') ++c.k_ra;
                    else if (f.size() > 2 && f[2] == '2') ++c.k_de;
                    break;
                case 'L': ++c.l_frames; break;
                case 'J': ++c.j_starts; break;
                default: break;
            }
        }
    }
    seen += tail.size();
    return c;
}

} // namespace

int main(int argc, char** argv)
{
    if (argc < 2) { std::fprintf(stderr, "usage: replay_gt2 CSV [--sparse] [--trace] [--debug] [--no-envelope] [--ra-limit D] [--dec-limit D] [--lpf S] [--lrpf S] [--lrcbar R] [--lrrel D] [--rcs S] [--hold D] [--unwind G] [--coast S] [--hcs S] [--rehome S] [--promote S] [--coldblind B] [--coldgrace G] [--cblind S] [--accel S] [--slowramp S]\n"); return 2; }
    bool trace = false, sp = false, no_env = false, dbg = false, nowrap = false;
    double lpf = -1.0;   // demand LPF tau sweep knob (s)
    double lrpf = -1.0;  // regime demand-LPF escalation tau sweep knob (s)
    double lrcbar = -1.0; // escalation rc bar sweep knob (0 = plain regime latch)
    double lrrel = -1.0;  // escalation error-release sweep knob (deg)
    double rcs = -1.0;   // reversal_confirm_s sweep knob (s)
    double hold = -1.0;  // reversal_hold_deg sweep knob (axis deg)
    double unwind = -1.0; // nudge_i_unwind_gain sweep knob (1.0 = legacy)
    double ra_lim = -1.0;  // envelope sweep knobs (production defaults
    double bmargin = -1.0; // branch handover margin (rs44 LOS A/B)
    double dec_lim = -1.0; // 105/105 unless overridden; 0 = axis disabled)
    double coast = -1.0;   // sim :K coast seconds (physical tail on metal)
    double hcs = -1.0;     // tracker high_coast_s landing-model knob
    double rehome = -1.0;  // rehome_idle_s knob (0/default = disabled)
    double promote = -1.0; // glide_promote_holdoff_s knob (0 = legacy instant)
    double cblind = -1.0;  // sim cold-:J issuance starvation seconds
    double accel = -1.0;   // sim :J issuance ramp seconds (board accel)
    double slowramp = -1.0;// sim LOW slow-ramp band seconds (starved LOW)
    double hrcreg = -1.0;  // high_recommit_regime_s knob
    double hrcage = -1.0;  // high_recommit_regime_age_s knob
    double gerr = -1.0;    // high_gate_err_deg knob (glide HIGH permission)
    double hrx = -1.0;     // high_recommit_escape_deg knob
    double hrcdb = -1.0;   // high_recommit_regime_db knob
    double hrcb = -1.0;  // high_recommit_hold_deg knob
    double hrc = -1.0;     // high_recommit_s knob (HIGH reprogram confirm)
    double hrdb = -1.0;    // high_reprogram_db knob (HIGH->HIGH reprogram deadband)
    double mgate = -1.0;   // high_gate_moving_x knob (pattern x justifying HIGH)
    double cb_tr = -1.0;   // skip_cold_blind_high_s knob (0 = legacy linear)
    double cg_tr = -1.0;   // skip_cold_grace_high_s knob (0 = legacy 3 s linear)
    for (int i = 2; i < argc; ++i)
    {
        if (!std::strcmp(argv[i], "--trace")) trace = true;
        else if (!std::strcmp(argv[i], "--sparse")) sp = true;
        else if (!std::strcmp(argv[i], "--debug")) dbg = true;
        else if (!std::strcmp(argv[i], "--lpf") && i + 1 < argc) lpf = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--lrpf") && i + 1 < argc) lrpf = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--lrcbar") && i + 1 < argc) lrcbar = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--lrrel") && i + 1 < argc) lrrel = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--rcs") && i + 1 < argc) rcs = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hold") && i + 1 < argc) hold = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--unwind") && i + 1 < argc) unwind = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--ra-limit") && i + 1 < argc) ra_lim = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--dec-limit") && i + 1 < argc) dec_lim = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--branchmargin") && i + 1 < argc) bmargin = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--coast") && i + 1 < argc) coast = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hcs") && i + 1 < argc) hcs = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--rehome") && i + 1 < argc) rehome = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--promote") && i + 1 < argc) promote = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--coldblind") && i + 1 < argc) cb_tr = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--coldgrace") && i + 1 < argc) cg_tr = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--cblind") && i + 1 < argc) cblind = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--accel") && i + 1 < argc) accel = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--slowramp") && i + 1 < argc) slowramp = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--mgate") && i + 1 < argc) mgate = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--no-envelope")) no_env = true;
        else if (!std::strcmp(argv[i], "--nowrap")) nowrap = true;
        else if (!std::strcmp(argv[i], "--hrdb") && i + 1 < argc) hrdb = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hrc") && i + 1 < argc) hrc = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hrch") && i + 1 < argc) hrcb = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hrcr") && i + 1 < argc) hrcreg = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hrca") && i + 1 < argc) hrcage = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--gerr") && i + 1 < argc) gerr = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hrx") && i + 1 < argc) hrx = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--hrcdb") && i + 1 < argc) hrcdb = std::atof(argv[++i]);
    }
    auto pushes = load(argv[1]);
    std::printf("replaying %zu raw pushes (%s) over %.0f s\n",
                pushes.size(), sp ? "GT2 follow-gate" : "dense", pushes.back().t);

    auto sim = std::make_shared<SimulatorTransport>(sim_heq5());
    auto board = std::make_shared<Board>(sim);
    board->connect();
    if (coast >= 0.0) // physical :K coast tail on metal (~0.3 s = 0.74 deg
    {                 // at 600x) — the replay blind spot that hid AO-73
        sim->set_coast('1', coast);
        sim->set_coast('2', coast);
    }
    if (cblind >= 0.0) // cold-:J dead issuance (AO-123 delivery profile)
    {
        sim->set_cold_blind('1', cblind);
        sim->set_cold_blind('2', cblind);
    }
    if (accel >= 0.0) // board :J issuance acceleration ramp (~1 s HIGH)
    {
        sim->set_accel('1', accel);
        sim->set_accel('2', accel);
    }
    if (slowramp >= 0.0) // LOW slow-ramp band starvation (Phase-1 ~0.85)
    {
        sim->set_slow_ramp('1', slowramp);
        sim->set_slow_ramp('2', slowramp);
    }
    TrackerConfig tc;
    tc.site = LatLon{44.95, -93.28};
    tc.orientation = Orientation::AltAzLevel;
    tc.home_az_deg = 90.0;
    tc.home_el_deg = 0.0;
    if (lpf >= 0.0) tc.demand_lpf_tau_s = lpf;     // sweep knob (0 = legacy)
    if (lrpf >= 0.0) tc.demand_lpf_regime_tau_s = lrpf; // sweep knob (0 = off)
    if (lrcbar >= 0.0) tc.demand_lpf_regime_min_rc = lrcbar; // escalation rc bar sweep knob
    if (lrrel >= 0.0) tc.demand_lpf_regime_release_err_deg = lrrel; // sweep knob (0 = off)
    if (rcs >= 0.0) tc.reversal_confirm_s = rcs;   // sweep knob
    if (hold >= 0.0) tc.reversal_hold_deg = hold;  // sweep knob
    if (unwind >= 0.0) tc.nudge_i_unwind_gain = unwind; // sweep knob
    if (hcs >= 0.0) tc.high_coast_s = hcs;         // landing-model knob
    if (rehome >= 0.0) tc.rehome_idle_s = rehome;  // re-home-after-idle knob
    if (promote >= 0.0) tc.glide_promote_holdoff_s = promote; // band-edge knob
    if (mgate >= 0.0) tc.high_gate_moving_x = mgate;          // HIGH-gate knob
    if (hrdb >= 0.0) tc.high_reprogram_db = hrdb;             // HIGH-reprogram db
    if (hrc >= 0.0) tc.high_recommit_s = hrc;                 // HIGH recommit confirm
    if (hrcb >= 0.0) tc.high_recommit_hold_deg = hrcb;        // hold budget
    if (hrcreg >= 0.0) tc.high_recommit_regime_s = hrcreg;    // regime window
    if (hrcage >= 0.0) tc.high_recommit_regime_age_s = hrcage;
    if (gerr >= 0.0) tc.high_gate_err_deg = gerr;
    if (hrx >= 0.0) tc.high_recommit_escape_deg = hrx;
    if (hrcdb >= 0.0) tc.high_recommit_regime_db = hrcdb;
    if (hrc == 0.0) tc.high_recommit_hold_deg = 0.0;          // disabled = off
    if (cb_tr >= 0.0) tc.safety.skip_cold_blind_high_s = cb_tr; // skip-grace
    if (cg_tr >= 0.0) tc.safety.skip_cold_grace_high_s = cg_tr; // cold-profile
    if (no_env)   // A/B: restore the pre-2026-11-19 unlimited-wind law
    {
        tc.safety.max_ra_travel_deg = 0.0;
        tc.safety.max_dec_travel_deg = 0.0;
    }
    if (ra_lim >= 0.0)  tc.safety.max_ra_travel_deg = ra_lim;
    if (dec_lim >= 0.0) tc.safety.max_dec_travel_deg = dec_lim;
    if (bmargin >= 0.0) tc.branch_handover_margin_deg = bmargin;
    if (nowrap)   // A/B: pre-2026-11-21 law (no cord ledger laws at all)
    {
        tc.wrap_home_unwind = false;
        tc.wrap_window_deg = 0.0;
        tc.wrap_tiebreak_margin_deg = 0.0;
    }
    Tracker tr(*board, tc);
    if (dbg)
    {
        // the SAME autopsy rows sateqd --debug writes on live metal (the
        // simulator transport makes them comparable); stdout redirection is
        // the file mechanism. The census below stays unaffected: :f polls
        // are queries.
        tr.set_debug_sink([](const std::string& row) { std::printf("%s\n", row.c_str()); });
        tr.set_debug(true);
    }
    // Narrative lines (branch handover fly/defer etc.): surface them with
    // the virtual replay clock so offline runs show the same story the
    // live daemon log tells.
    tr.set_log_sink([](const std::string& s)
    { (void)std::printf("[narr] %s\n", s.c_str()); });
    tr.calibrate(UC::now(), SC::now());

    auto t0 = SC::now();
    auto u0 = UC::now();
    SC::duration vt{0};
    auto mono = [&]() { return t0 + vt; };
    auto utc = [&]() { return u0 + std::chrono::duration_cast<UC::duration>(vt); };
    auto advance = [&](double s) {
        vt += std::chrono::duration_cast<SC::duration>(std::chrono::duration<double>(s));
        sim->advance(s);
    };

    size_t seen = 0;
    size_t push_i = 0;
    double push_az = 90, push_el = 0;        // last pushed target
    bool have_push = false;
    FollowGate gate;
    long n_dropped = 0, n_wire = 0;
    double rot_az = 90, rot_el = 0; bool rot_valid = false;
    double next_poll_t = 0.0;               // GT2 2 s rotor-poll grid
    double max_err = 0, sum_sq = 0; long n_err = 0, n_big = 0;
    double max_rate_step_ra = 0, max_rate_step_de = 0;
    double prev_ra_x = 0, prev_de_x = 0; bool had_rate = false;
    int prev_glide = -1, mode_sw = 0;
    long ev_floor = 0, ev_rl = 0, ev_halting = 0;
    long ev_return = 0; double t_return = 0;   // travel-envelope refusals
    int prev_floor = 0, prev_rl = 0, prev_halted = 0, prev_return = 0;
    Counters tot;

    for (double t = 0.1; t <= pushes.back().t + 1.0; t += 0.1)
    {
        // GT2 polls rotor position every 2 s while connected (rotor.js
        // ROTOR_POLL_MS). On the live link any rotctld command feeds the
        // client watchdog (Tracker::on_get_position); a replay without the
        // poll would artifact watchdog holds through every LOS gap and
        // pre-aim idle (2026-11-17 session: LOS gaps 200-2600 s).
        while (t + 1e-9 >= next_poll_t)
        {
            tr.safety().note_client_command(mono());
            next_poll_t += 2.0;
        }
        while (push_i < pushes.size() && pushes[push_i].t <= t)
        {
            const auto& p = pushes[push_i++];
            // In --sparse mode, emulate what GT2 sends when the rotor is
            // actually following (the live recording has the rotor frozen:
            // read-only mount-off session -> the >1° lag rule re-pushed
            // every 2 s). With a follower, only the target-motion rule and
            // occasional lag re-pushes survive.
            if (!sp || gate.pass(p.t, p.az, p.el, rot_az, rot_el, rot_valid))
            {
                push_az = p.az; push_el = p.el;
                tr.set_target_at(push_az, push_el, mono());
                have_push = true;
                ++n_wire;
            }
            else ++n_dropped;
        }
        advance(0.1);
        tr.step(utc(), mono());
        auto st = tr.snapshot();
        auto c = count_since(sim->command_log_tail(seen), seen);
        tot.i_frames += c.i_frames; tot.g_frames += c.g_frames;
        tot.k_frames += c.k_frames; tot.l_frames += c.l_frames;
        tot.j_starts += c.j_starts;
        if (had_rate && st.tracking)
        {
            max_rate_step_ra = std::max(max_rate_step_ra, std::fabs(st.rate_ra_x - prev_ra_x));
            max_rate_step_de = std::max(max_rate_step_de, std::fabs(st.rate_dec_x - prev_de_x));
        }
        if (st.tracking) { prev_ra_x = st.rate_ra_x; prev_de_x = st.rate_dec_x; had_rate = true; }
        else had_rate = false;
        if (prev_glide >= 0 && (int)st.glide != prev_glide) ++mode_sw;
        prev_glide = (int)st.glide;
        ev_floor += (int)st.floor_guard > prev_floor; prev_floor = (int)st.floor_guard;
        ev_rl += (int)st.rate_limited > prev_rl; prev_rl = (int)st.rate_limited;
        ev_halting += (int)st.halted > prev_halted; prev_halted = (int)st.halted;
        ev_return += (int)st.travel_returning > prev_return; prev_return = (int)st.travel_returning;
        t_return += 0.1 * (int)st.travel_returning;
        if (trace && (c.k_frames || c.j_starts))
            std::printf("  WIRE t=%7.1f :K=+%ld(K1=%ld K2=%ld) :J=+%ld :I=+%ld :G=+%ld tgt=(%7.2f,%6.2f) cur=(%7.2f,%6.2f) rt=(%8.1f,%8.1f) glide=%d\n",
                        t, c.k_frames, c.k_ra, c.k_de, c.j_starts, c.i_frames, c.g_frames,
                        push_az, push_el, st.current.az_deg, st.current.el_deg,
                        st.rate_ra_x, st.rate_dec_x, (int)st.glide);
        if (trace && static_cast<int>(t * 10) % 10 == 0)
            std::printf("t=%6.0f tgt=(%7.2f,%6.2f) cur=(%7.2f,%6.2f) rt=(%8.1f,%8.1f) "
                        "trk=%d glide=%d floor=%d rl=%d halted=%d %s\n",
                        t, push_az, push_el, st.current.az_deg, st.current.el_deg,
                        st.rate_ra_x, st.rate_dec_x, (int)st.tracking, (int)st.glide,
                        (int)st.floor_guard, (int)st.rate_limited, (int)st.halted,
                        st.halt_reason.c_str());
        // error vs latest push, only when that target is a real pointing
        // (el > 0: el==0 lines are GT2 pre-aim/LOS markers)
        if (have_push && push_el > 0.05)
        {
            HsCoord p = tr.current_pointing(utc());
            double daz = std::fabs(std::fmod(p.az_deg - push_az + 540.0, 360.0) - 180.0);
            double de = p.el_deg - push_el;
            double e = std::hypot(daz * std::cos(push_el * 0.0174532938), de);
            max_err = std::max(max_err, e);
            sum_sq += e * e; ++n_err;
            if (e > 1.0) ++n_big;
        }
        (void)ev_halting;
        rot_az = st.current.az_deg; rot_el = st.current.el_deg; rot_valid = true;
    }
    std::printf("pushes: %ld on wire, %ld dropped by follow-gate (%zu raw)\n",
                n_wire, n_dropped, push_i);
    std::printf("err vs pushed target: max %.3f deg, rms %.3f deg (n=%ld, >1deg: %ld)\n",
                max_err, std::sqrt(sum_sq / std::max(1L, n_err)), n_err, n_big);
    std::printf("wire: :I=%ld :G=%ld :K=%ld :L=%ld :J=%ld, glide<->dive switches: %d\n",
                tot.i_frames, tot.g_frames, tot.k_frames, tot.l_frames, tot.j_starts, mode_sw);
    std::printf("max commanded-rate step: RA %.0fx  DE %.0fx; floor-guard edges %ld, rate-cap edges %ld, halt edges %ld\n",
                max_rate_step_ra, max_rate_step_de, ev_floor, ev_rl, ev_halting);
    std::printf("envelope: %ld refusals (return-home, %.0f s total); %s\n",
                ev_return, t_return,
                no_env ? "limits DISABLED (--no-envelope)"
                       : (ra_lim >= 0.0 || dec_lim >= 0.0)
                             ? (std::string("limits RA +") +
                                (tc.safety.max_ra_travel_deg > 0.0
                                     ? std::to_string((int)tc.safety.max_ra_travel_deg) : "off") +
                                "/DEC +" +
                                (tc.safety.max_dec_travel_deg > 0.0
                                     ? std::to_string((int)tc.safety.max_dec_travel_deg) : "off"))
                               .c_str()
                             : "limits ON (+-105/+-105 default box)");
    return 0;
}
