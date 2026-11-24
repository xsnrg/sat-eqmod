// Tracker: the control core. rotctld delivers satellite pointing as
// (az, el) updates; the tracker converts sky pointing into physical axis
// angles, reads live axis angles from the step counters, gates through the
// SafetyMonitor, and commands signed axis rates (x sidereal) to the board.
//
// Time model: step(now) is a pure function of external time — unit tests
// drive it with a fake clock; the production Runner thread calls it at
// ~10 Hz. No sleeping inside the logic.
//
// Calibration model (no absolute encoders on these mounts):
//   - At startup the operator states where the scope ACTUALLY points
//     (home_az/home_el, e.g. parked due-south/equatorial). Counters read
//     then become the reference. Cumulative rotation is exact from then on
//     (microstep counters + wrap accounting), which is also the cord-wrap
//     budget the SafetyMonitor enforces.
#pragma once
#include "sateq/astro.h"
#include "sateq/board.h"
#include "sateq/rotctld.h"
#include "sateq/safety.h"

#include <atomic>

#include <chrono>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace sateq {

// Internal helper for Tracker's pattern estimator (TrackerConfig knobs
// below tune it; full rationale at the estimator call site).
// Alpha-beta filter (steady-state critically-damped Kalman for a constant
// -velocity target), one scalar channel: time-update p += v*dt, then
// correct with innovation r = z - p: p += alpha*r, v += beta*r/dt.
// Ramp-unbiased (converged v is EXACT on constant velocity, unlike a plain
// EMA) and its velocity estimate carries no endpoint noise amplification
// (a sliding-window LSQ — quadratic especially — rings at its endpoint on
// quantized input: a +-0.05 deg/s velocity wiggle per push went into the
// glide reference, sign-flipping the commanded rate at LOW speed and
// paying :K+:J every push; live replay 2026-11-16, t=1002-1080).
// Duplicate pushes (GT2 re-sends the SAME rounded az/el while the rotor
// lags > 1 deg — 17% of the live recording) are harmless: the innovation
// simply says "held", which is true at sub-quantization rates.
struct ScalarABFilter
{
    double p = 0.0, v = 0.0;
    bool   ready = false;

    void reset(double z) { p = z; v = 0.0; ready = true; }

    void predict_update(double dt, double z, double alpha, double beta)
    {
        if (dt < 1e-3) dt = 1e-3;
        p += v * dt;
        const double r = z - p;
        p += alpha * r;
        v += beta * r / dt;
    }

    double pred_pos(double dt) const { return p + v * dt; }
};

struct TrackerConfig {
    LatLon site{};                       // station location
    Orientation orientation = Orientation::Polar; // head configuration (astro.h)
    double home_az_deg = 180.0;          // true sky pointing at startup ref pose
    double home_el_deg = 45.0;
    double ra_sign  = 1.0;               // motor-forward vs angle-increase (Phase-1 verified)
    double dec_sign = 1.0;
    // control law — dive mode (far from target: point-to-point, floor-guarded)
    double control_kp_per_s  = 0.8;      // extra proportional gain on axis angle error
    double control_kd_per_s  = 1.0;      // feedforward gain on target velocity
    // glide mode (following a moving pattern: constant rate + bounded
    // convergence — "run it like a telescope mount", 2026-11-16). The
    // reference integrates at pattern velocity plus a bounded convergence
    // term that walks it onto the predicted sample position; at steady
    // state that leaves a NEAR-CONSTANT rate equal to the pattern rate.
    double glide_engage_deg  = 5.0;      // engage glide within this axis-space error
    // --- branch handover margin (rs44 2026-11-20 LOS "teleport" fix) -------
    // A sky pointing has TWO axis solutions, (a,d) and its flip (a+180,180-d),
    // and the travel-box test is NOT invariant under that map (dec folds to
    // 180-d), so near the RA wall the two branches' memberships scrape every
    // tick at low elevation. The live rs44 LOS event at el 1.3 deg (05:53:00)
    // lost the RIDED flip branch's box test by 0.3 STEPS: the setpoint
    // teleported ~170 deg to the canonical solution and back the next tick
    // (branch oscillation). This margin is the hysteresis band: the branch
    // the mount rides keeps its box test with it added to the travel limits
    // (select_branch), and a near-equal route swap must beat the ridden
    // route by more than it. Forced handover (ridden branch beyond the band)
    // is a wall-outranks-continuity decision, never wobble, and the far-
    // branch rule in step_impl flies any ~180 deg flight as a fresh dive
    // instead of a setpoint jump. 5 deg matches the resume-margin scale and
    // is ~100x the rs44 scrape; the envelope clamp keeps the mount inside
    // the STRICT box while riding in the band. 0 = legacy nearest-tie-break.
    double branch_handover_margin_deg = 5.0;
    // Wall-strain watchdog (ISS LOS freeze 2026-11-20 evening: a latched
    // deferral rode the RA wall at el 47 for 11 min, flags=trk, axis err
    // 128 deg, demand permanently vetoed to 0 by the travel-wall rate
    // guard: the el-0 ghost's flip solution had walked ACROSS the RA
    // meridian, the fold-box envelope kept calling the target legal, and
    // nothing asked whether a LEGAL PATH to the setpoint exists). Lock:
    // the target's branch solution demands straight OUTWARD motion at a
    // travel wall with that axis component beyond this threshold;
    // sustained wall_strain_s -> non-latching return-home (same semantics
    // as the envelope refusal; a locked target also GATES the resume so
    // the ghost cannot pull the mount back into the freeze, while any
    // other legal push resumes). Wall-slides are UNAFFECTED: their
    // along-wall setpoint rides the wall beside the mount (los-defer
    // pins err inside the grace band), so they never meet the threshold.
    // 0 disables.
    double wall_strain_deg = 10.0;
    double wall_strain_s   = 5.0;   // sustained duration before tripping
    // --- dive waypoint routing (see tracker.cpp dive-routing block) ------
    // Far, low targets are reached via a climb / transit-elevation sweep /
    // descend profile instead of a straight axis-space dive: straight low
    // dives sink below the horizon (RS-44 -58 deg event) and wind the RA
    // shaft in the low-el due-north corridor (recorded-session replay:
    // 540 deg RA wrap trip from el-0 pre-aim slides alone).
    double route_transit_el  = 12.0;     // sweep elevation of the transit leg
    double route_az_min_deg  = 60.0;     // az sweep below which routing is off
                                         // (0 disables the router entirely)
    double route_reach_el_deg = 4.0;     // climb leg complete within this el
    double route_reach_az_deg = 8.0;     // sweep leg complete within this az
    double glide_disengage_deg = 8.0;    // leave glide beyond this (hysteresis)
    double pattern_pred_max_s = 2.5;     // cap on pattern-velocity extrapolation of a sample
    // Client-ALIVE extrapolation horizon. pattern_pred_max_s was sized for
    // a client that goes SILENT (glide to rest in ~5 s). But GT2 iq-work
    // keeps its socket AND its 2 s rotor poll while its follow throttle
    // legitimately spaces pushes 20-60 s (near-horizon, 1 deg/2 s rule):
    // the pattern is known-good, only the SAMPLES are sparse. With a fresh
    // poll the reference therefore extrapolates along the pattern for
    // pattern_alive_pred_max_s (LEO az/el motion is smooth over minutes;
    // the slow accel channel carries curvature) instead of parking the
    // antenna between pushes. A dead client falls back to the old
    // pred_max_s/hard-silence behavior after pattern_alive_grace_s.
    // The extension is SCALED by |pattern velocity|: the extrapolation
    // error budget is velocity-trust times horizon, and a "stationary"
    // pattern's α-β velocity is quantization noise (live dense replay
    // 2026-11-16: 0.1 deg/2 s pushes on a frozen pre-aim target gave
    // +-0.05 deg/s noise; x 25 s horizon = 1.25 deg idle wander that
    // wound the RA axis into the wrap trip). At zero pattern rate the
    // cap is the original pred_max_s; full extension at
    // pattern_alive_full_x_rate.
    double pattern_alive_pred_max_s = 25.0;
    double pattern_alive_grace_s    = 6.0;   // 3x GT2's 2 s rotor poll
    double pattern_alive_full_x_rate = 0.2;  // deg/s for full extension
    double pattern_conv_k_per_s = 0.75;  // convergence rate, deg/s, per deg of gap
                                         // (x engage threshold ≈ dive rate at
                                         // handoff: continuous command across
                                         // the dive->glide boundary)
    double pattern_conv_max_deg_s = 2.5; // convergence-rate bound (= dive rate
                                         // cap 600x: never faster than diving)
    double nudge_p_per_s    = 2.0;    // rate nudge from ref-cur error, deg/s per deg
    double nudge_des_p_per_s = 3.0;   // extra P on the TRUE setpoint error
                                      // (des-cur). With an FF'd reference the
                                      // ref converges to the pattern, but a
                                      // deadband-pinned HIGH axis cannot
                                      // follow ref's small corrections: P on
                                      // ref alone then holds the position
                                      // bias at the deadband edge. This term
                                      // gives the bias an escape ramp out of
                                      // the deadband (0 = Phase-4 behavior).
    double nudge_des_p_deadband = 0.0; // deg of (des-cur) charged free before
                                      // nudge_des_p_per_s bites
                                         // (gain must out-authorize the
                                         // high-band 10% chatter deadband:
                                         // 10% of a 0.5 deg/s pattern is
                                         // 0.05 deg/s, so at 0.1 deg of
                                         // slip the loop must already speak)
    double nudge_p_limit     = 1.5;      // deg/s
    double nudge_i_per_s2    = 0.04;     // rate nudge from sustained error, deg/s per deg*s
    double nudge_i_unwind_gain = 5.0;    // extra glide-I integration speed while the
                                         // integral OPPOSES (des - cur): after a dive
                                         // overshoot the integral must first unwind the
                                         // stale opposite-sign area at err x dt, and with
                                         // Ki/P = 0.04/2.0 the loop's slow pole is
                                         // tau ~ 50 s (live 2026-11-19: glide settle
                                         // stranded at 0.27 deg for 30+ s). Draining
                                         // windup n x faster cuts that tail; while the
                                         // integral AGREES with the error (its moving-
                                         // pattern job) the gain is untouched. The
                                         // pattern gate matters: on a MOVING pattern the
                                         // integral is live tracking authority and the
                                         // frozen-between-push target sawtooth crosses
                                         // zero every cycle — ungated drain cost FO-29
                                         // sparse rms 0.626 -> 0.706 at the fast-el
                                         // apex tail. Ramp: full gain at/below gate/5,
                                         // 1.0 at/above gate. 0 gate or 1.0 gain
                                         // disables.
    double nudge_i_unwind_gate_deg_s = 0.1;
    double nudge_i_limit     = 1.0;      // deg/s
    double nudge_i_errclamp  = 25.0;     // deg*s per axis: integral clamp
    // First-order low-pass on the GLIDE demand (iteration-2 :K-storm fix
    // 2026-11-19, seconds; 0 = off). The storm (FO-29 live 603 :K, dense
    // replay 315 post-fix-1) is NOT plant ringing: it is the glide law
    // faithfully executing its own setpoint sawtooth. GT2 pushes arrive on
    // a 2 s / 0.1 deg quantised cadence; the Jacobian lever near the
    // horizon (~1/sin az, ~2.5x at az 25) maps one 0.1 deg sky step to a
    // ~0.25 deg AXIS step, and the ref-convergence term (conv x step ~=
    // 0.2 deg/s) plus P(ref-cur) turn that step into a demand JUMP and sign
    // flip that executes (LOW sign flips pay the reversal-confirmation
    // path, HIGH ones pay :K coast) before the step decays (ref conv tau
    // ~1.3 s). Velocity-FEEDBACK damping was tried first and is NEGATIVE on
    // replay (ideal plant, nothing to ring; 315 -> 467 :K as the gain rose)
    // and a slew-RATE ratchet mode-couples with glide engagement (146+ dive
    // <-> glide thrash) — the fix is plain command smoothing. A first-order
    // LPF (NOT a magnitude deadband, which Phase-4 tuning proved freezes
    // sustained glide corrections into permanent offsets: this only delays
    // by tau, steady-state transparent, the I-term eats the residual) with
    // tau=0.3 s sits between the 10 Hz control bandwidth and the 2 s push
    // cadence: control-loop transients keep authority, per-push staircase
    // steps are low-passed and die inside the ref's conv decay instead of
    // reaching the wire. DIVE bypasses (catch-up authority; state tracks
    // dive demand so glide engagement starts continuous); floor-guard and
    // stop_quiet reseed. Swept {0.15..0.8} x 4 datasets: 0.3 is the knee
    // (FO-29 dense :K 315 -> 241, rms 0.393 -> 0.394; >=0.5 starts paying
    // the phase lag). Complements reversal_confirm_s, which absorbs the
    // flips that survive smoothing.
    double demand_lpf_tau_s    = 0.3;
    // Time constant of the measured-axis-rate diagnostic (EMA of the :j
    // counter derivative) surfaced as vm= in the --debug tick trace;
    // 0 disables the diagnostic. See Tracker::vel_est_.
    double vel_meas_tau_s    = 0.3;
    double update_hz         = 10.0;
    double target_stale_s    = 90.0;     // rotctld target older than this => hold.
                                         // NOT the primary client-liveness
                                         // safety (that is disconnect-means-
                                         // stop): GridTracker2 iq-work's
                                         // follow throttle can legitimately
                                         // gap 20-60 s between pushes on
                                         // near-horizon targets (1 deg/2 s
                                         // push rule; RS-44-class passes at
                                         // 0.02-0.05 deg/s apparent rate).
    double rate_epsilon_x    = 0.05;     // below this x-sid, command stop instead
    // --- wire-economy rules, from live Sirius board constraints (in-place
    // :I free in LOW band; :I NAKs while running HIGH, every HIGH change
    // costs :K + ~1 s coast) surfaced by the GT2 iq-work replay 2026-11-16:
    // a monotonically decelerating glide crossed the 10 % deadband every
    // tick and fired 11 :K/:J restarts in 2 s; idle PI dither of +-0.1x
    // crossing zero stop/started the axis every few seconds while holding.
    // HIGH-band economy design (GT2 iq-work replay + live-board rules,
    // 2026-11-16): demand is sent RAW (the board's HIGH grid is fine:
    // integer-us periods on a ~96 us carrier, ~1.2 % apart). A coarse
    // rate LADDER was tried and rejected: rung alternation pays the :K
    // penalty on every switch (937 :K in replay). A time-based HIGH
    // reprogram COOLDOWN was tried and rejected too: holding a stale
    // HIGH rate in a closed position loop freezes transients into
    // pointing error (test_tracker regressions at every value tried,
    // incl. speed-scaled). What works: the HIGH deadband in apply_axis plus
    // +-6 % hysteresis on the LOW/HIGH promotion boundary.
    // Sweep-6 (live Sirius 2026-11-18): under a RATE-CAPPED dive the deadband
    // was the only economy left and 10 % did not bound the wire — DEC swung
    // -507x -> -600x (18 %) ~3x/s, every swing a full :K coast+restart
    // (RA 26 / DEC 28 events in <10 s), and the final restart's re-accel
    // starvation skip-latched DEC at ramp_age 2.5/3.0 s. A HIGH rate change
    // changes position by (delta_x x sidereal) deg/s; at 600x the 10 % band
    // is 60x = 0.25 deg/s, while the :K it pays moves the antenna ~0.7 deg
    // on the coast ALONE. Re-program only beyond a delta that can plausibly
    // outrun its own coast cost: 25 % (sweeps 5-6; glide/dive P accumulates
    // that much lag in ~2 s of wrong rate, and dives close errors >> 0.7 deg
    // anyway. LOW band unchanged: in-place :I is free and stays at 10 %/6x
    // economics via rate_epsilon + sub-quantization floor.
    double high_reprogram_db   = 0.25;   // HIGH->HIGH same-dir swing needed
                                         // to pay a :K stop/restart
    // HIGH rescale DIRECTION-PERSISTENCE law (SO-50 demand pump
    // 2026-11-21: 149 :K/min at el>10 riding a genuinely HIGH-justified
    // axis pattern of 150-400x — the Jacobian-acceleration corridor.
    // Replay census: 225 HIGH->HIGH rescales at 20-48 % swings whose
    // demand-minus-sent direction ALTERNATES tick groups (+402->225->163
    // ->204->265->168->...), each commit self-perturbing (~0.7 deg coast
    // + ~2.5 s issuance starvation -> error moves -> demand re-crosses
    // the db): a limit cycle whose period scales WITH the deadband —
    // widening high_reprogram_db to 0.5/0.6 cut :K only 12/17 % at flat
    // error, proving the swings are push-cadence disturbance, not work.)
    // Law (full detail in tracker.cpp apply_axis): a HIGH-touching,
    // same-sign-swing GLIDE rescale whose magnitude direction FLIPS
    // against the last committed rescale is held at the sent program
    // (wire-free) for min(high_recommit_s, high_recommit_hold_deg /
    // mismatch_rate); re-entering the economy zones cancels it.
    // MONOTONIC ladders (decel before meridian reversals — the failure
    // mode of the first plain-persistence variant: 2-7 deg sky at the
    // 1/sin(az) lever in m-1hz/m-meridional) commit immediately. The
    // hold budget (same shape as reversal_hold_deg) bounds a hold that
    // turns out genuine at high_recommit_hold_deg of AXIS error.
    // high_recommit_s = 0 disables (legacy immediate commit) = instant
    // rollback lever; high_recommit_hold_deg = 0 removes the budget cap.
    double high_recommit_s = 1.0;
    double high_recommit_hold_deg = 0.1;
    // Position-loop ESCAPE of the HIGH rescale persistence law (confirm
    // window + regime acceleration freeze): once |axis error| reaches this
    // the law steps aside and the demand commits immediately — the freeze
    // may only pin pointing error up to this bound. Split off from
    // high_gate_err_deg (2026-11-21) because the two roles pull opposite
    // ways: raising the GLIDE HIGH *permission* gate makes quiet phases
    // wire-quieter, but the law's *escape* must stay tight or a storm
    // could hold the antenna 3 axis deg (x the 1/sin(az) sky lever) off
    // target before the escape fires. Default keeps the historical shared
    // value; escape = 0 makes the law inescapable by position error
    // (floor guard + DIVE escapes remain).
    double high_recommit_escape_deg = 2.0;
    // WOBBLE REGIME (full detail in tracker.cpp apply_axis): a leaky
    // rescale-COMMIT counter (time constant = this window) identifies a
    // HIGH rescale STORM on the axis (>= high_recommit_regime_n recent
    // commits) while the client cadence is throttled (age past
    // high_recommit_regime_age_s). Inside the regime the silent economy
    // deadband for acceleration rescales touching HIGH widens to
    // high_recommit_regime_db — time confirmation alone cannot absorb a
    // demand that persists out-of-band between 2-4 s pushes. Clean
    // passes (fast cadence, < 1 HIGH rescale commit / 4 s per axis)
    // never arm it. high_recommit_regime_s = 0 disables the escalation.
    double high_recommit_regime_s = 4.0;
    // WOBBLE-REGIME ACCELERATION FREEZE (see tracker.cpp apply_axis):
    // inside a regime, a same-direction rescale TOUCHING HIGH whose
    // magnitude DEMANDS an increase (acceleration) and swings less than
    // this fraction is silently held at the running program (economy-
    // zone contract: zero wire). Default 1e9 = freeze ALL regime
    // acceleration rescales — the operator law spec "never command HIGH
    // unless really out" under storm conditions: the position-loop
    // escape (|axis error| >= high_recommit_escape_deg) and the floor guard
    // keep authority, deceleration/demotion/reversals always commit
    // (they are the real work of meridian Jacobian ramps; holding those
    // cost 2-7 deg sky in m-meridional/m-1hz). Measured on SO-50: the
    // demand persists out-of-band at high_reprogram_db > 1 s between
    // 2-3.5 s pushes, so the time window alone cannot absorb it (115
    // :K/min survived the window law); the freeze cuts the replay storm
    // 361 -> 191 :K with in-pass storm-minute error rms 0.55 -> 0.50
    // deg BETTER (each frozen commit would have paid coast + issuance
    // starvation). A smaller value (e.g. 0.7) lets genuinely huge
    // acceleration swings commit through; 0 disables the freeze.
    double high_recommit_regime_db = 1e9;
    // WOBBLE REGIME detector threshold: leaky rescale-commit counter
    // (time constant high_recommit_regime_s) trips at this many recent
    // commits on the axis. Clean passes run < 1 HIGH rescale commit /
    // 4 s per axis; the SO-50 pump ran 4-10 (per-axis rate 60-150/min).
    double high_recommit_regime_n = 3.0;
    // WOBBLE REGIME client-staleness requirement: the pump is a
    // THROTTLED-CLIENT phenomenon (2-4 s quantized GT2 pushes against a
    // frozen pattern); fast-cadence ladders are work, not disturbance.
    // Latched with the regime window so pushes re-zeroing the age do
    // not open a commit window every cadence tick.
    // high_recommit_regime_age_s = 0 disables the requirement.
    double high_recommit_regime_age_s = 1.5;
    // WOBBLE-REGIME DEMAND LPF ESCALATION (live 2026-11-22 22:00-pass DEC
    // churn class, 0a4(h): 60-107 :K/min at follow err < 0.7 deg): while
    // an axis's wobble regime runs at STORM GRADE, the glide demand LPF
    // temporarily runs at THIS tau (deg-rate filter; the per-axis arm
    // flag is latched with a one-tick lag — the regime ledger is computed
    // later in the same tick's economy block). Rationale: the pump is a
    // push-cadence DISTURBANCE on the glide demand (0.1 deg quantized 2 s
    // pushes levered 4-6x by the flip-branch Jacobian into +-150x AXIS
    // dither); the confirm/freeze laws police individual commits but the
    // demand keeps walking away from sent at ~1 commit/s once the
    // oscillation defeats the error-growth ledger (commits land at err
    // troughs by construction). Smoothing the demand between pushes
    // attacks the source. RAISING THE GLOBAL TAU INSTEAD was measured and
    // REJECTED (tau 1.0 globally cuts the slice storm but costs the quiet
    // sets FO-29-1119 51 -> 66, AO-73 18 -> 27, 11-16 dense 1370 -> 1812
    // :K — LPF phase lag at the dive/reversal transients; the same lag
    // only exists during a regime if it is GATED). demand_lpf_regime_tau_s
    // = 0 disables the escalation; it is also inert unless
    // demand_lpf_tau_s > 0. Tuned on the 22:00-pass slice
    // (data/gt2-iqwork-2026-11-22-eve window, storm DEC :K 187 -> 102 at
    // flat err rms 0.66) with the guard-matrix replays as the no-
    // regression gate (final shipped triple, dense :K vs legacy: 11-16
    // 1370 -> 1109, LUCA 419 -> 348, night 599 -> 528, SO50 190 -> 172,
    // FO-29 both years + AO-73 + rs44 flat; sparse mode +10/+11 noise on
    // two sets). Intermediate tau/bar combinations were swept and are
    // CHAOTIC (the rc ledger self-feeds in regime, so the arm threshold
    // flips mid-burst): do not re-tune per-fixture, only re-validate the
    // triple on the slice + guard matrix as one law.
    //
    // STORM-GRADE BAR: escalation arms only while the leaky commit ledger
    // rc >= demand_lpf_regime_min_rc, NOT at the freeze law's plain latch
    // level (high_recommit_regime_n = 3). Evidence: night-1119 dense with
    // the plain latch (bar 0) costs +92 :K — its regimes sit in the
    // rc 3..12 mid-grade band where the dither is not yet a
    // self-sustaining pump and extra lag buys nothing; at bar 12 (with
    // the error release below) night lands BETTER than legacy. Corridor-
    // class regimes DO clear 12 (escalation-forced-off probe reaches raw
    // rc 39.5), so SO-50-style legitimate HIGH work pays the documented
    // few-:K edge cost at flat-or-better error. min_rc = 0 escalates on
    // the plain latch; a huge value disables like tau = 0.
    double demand_lpf_regime_min_rc = 12.0;
    // ERROR RELEASE: escalation also refuses to arm (and, via the
    // per-tick latch, releases) while THIS AXIS's |error| exceeds this.
    // The pump class lives at small bounded error (live 22:00 window:
    // mean 0.8 axis deg) while a genuinely outrun pattern walks its error
    // up and must keep FULL unsmoothed authority: without the release,
    // tau 1.0 on the m-GATE ramp fixture (1 deg/s az at el 0) took max
    // follow err 2.24 -> 3.65; with release at 1.0 deg the fixture lands
    // at 2.15 max / 1.13 mean (legacy 2.24/1.21 — the smoothing even
    // helps once the transient is excluded). 0 = disabled (pure rc bar).
    double demand_lpf_regime_release_err_deg = 1.0;
    double demand_lpf_regime_tau_s = 1.0;
    double high_band_demote_db  = 0.06;  // hold a running HIGH rate while a
                                         // LOW demand is within 6 % of the
                                         // promotion boundary (flip guard)
    double high_band_promote_db = 0.06;  // running LOW: demand must cross
                                         // 104x by more than 6 % before we
                                         // pay the promote (stop+restart)
    double reversal_min_x      = 3.0;    // a running axis ignores sign flips
                                         // demanding less than this (idle
                                         // dither -> pending stop, not a
                                         // coast-reverse); and a stopped
                                         // axis will not :J below it
    // Sign flips must PERSIST this long before they execute (0 disables).
    // While confirming, the demand is HELD AT THE SENT PROGRAM (wire-free:
    // apply_axis's identical-program path emits nothing) — the axis simply
    // keeps running while the flip proves itself. Rationale (sim replay
    // 2026-11-17/19 of the live GT2 sessions): near-horizon pointing
    // levered 0.25 deg-quantized GT2 pushes into ~1 deg/s pattern-velocity
    // flips; every flip escaped the same-direction deadband and paid a
    // full :K/:J reprogram — FO-29 dense replay limit-cycled 315-603 :K.
    // Oscillating flips re-stamp and never execute; a genuine reversal
    // costs confirm-time plus one coast. HOLD BUDGET caps the effective
    // window at reversal_hold_deg / |sent_rate| because holding runs the
    // axis on the old program: near-zero dithers (the oscillating
    // quantization class) are cheap to hold and get the full window,
    // HIGH-rate flips shorten automatically so a real reversal is never
    // ridden past (a flat 6 s window cost the 1 Hz suite test 0.125 ->
    // 0.19 deg and would ride a mid-pass HIGH reversal ~2 deg). Window
    // swept {0.5..10} x budget {0.3..1.0} on FO-29/AO-73/11-16 replays,
    // 1 Hz + meridional suite scenarios. Iteration-2 2026-11-19.
    // Floor-guard demands bypass (immediate safety authority).
    double reversal_confirm_s  = 6.0;
    double reversal_hold_deg   = 0.5;   // axis-deg overshoot a hold may cost
    // HIGH-band actuator-quantum gate (glide law). Every HIGH speed rate
    // change on the board costs a :K plus a SPEED-SCALED coast (live
    // Sirius: ~1.0 s / ~19k steps at 600x, ~0.03 s LOW) that also blocks
    // the tracker thread. While the pattern itself demands HIGH, that is
    // the cost of tracking. But chasing a small axis error at HIGH makes
    // the coast quantum (~1 deg axis) LARGER than the error being closed:
    // sim replay 2026-11-17 (live GT2 session) shows near-horizon fine
    // pointing (0.25 deg az = ~0.9 deg axis through the levered Jacobian)
    // limit-cycling its saturated P demand between ~45x and ~570x, and
    // every 104x boundary crossing or >10 % HIGH swing paid a :K — 10 Hz
    // storms with the antenna dithering +-0.15 deg az. So in GLIDE, demand
    // beyond the LOW ceiling is authorized only while the pattern velocity
    // itself is moving (high_gate_moving_x: the idle/pre-aim signature is
    // a pattern at dead stop), or the axis error exceeds high_gate_err_deg;
    // otherwise the demand is clamped to high_gate_max_x and acquisition
    // finishes in the coast-free LOW band (in-place :I is free). Dive
    // mode (err >= engage threshold, floor slides) keeps full authority.
    double high_gate_err_deg = 2.0;      // axis-space error authorizing HIGH
                                         // in the GLIDE band gate (permission
                                         // only; the recommit-law escape is
                                         // the separate high_recommit_escape_deg).
                                         // 2026-11-21 replay A/B (bench profile
                                         // runs 2.5): 2.5 cuts throttled-client
                                         // dither :K 3-6 % at flat error, but the
                                         // 1 Hz m-meridional CLIMB pays it (pattern
                                         // under mgate-90 there, so the error gate
                                         // must stay 2.0: 0.31 -> 0.40 deg); 3.0
                                         // costs follow lag for no wire gain.
    double high_gate_moving_x = 90.0;  // pattern rate (x sidereal) in glide
                                       // that alone authorizes HIGH beyond
                                       // the LOW ceiling: 90 = only patterns
                                       // LOW cannot follow (band-edge relay
                                       // fix, live LUCA 2026-11-20; legacy
                                       // 3.0 authorized EVERY moving pattern
                                       // and rode the relay ~20 :K/min)
    double high_gate_max_x   = 100.0;    // LOW ceiling the clamp holds
    // LOW->HIGH promotion hold-off (GLIDE; live LUCA pass 2026-11-20: the
    // pass churned ~20 :K/min riding the LOW +-104x ceiling — each transient
    // P/pattern spike past 104x promoted, paying a :K from the RUNNING HIGH
    // program (full speed-scaled coast + 2 s restart starvation), the demote
    // back being coast-cheap: the expensive direction is promotion OUT of
    // HIGH, and every spike paid one). While GLIDE demands beyond LOW, keep
    // chasing at the LOW ceiling (free in-place :I, and the demand LPF/conv
    // transients decay under the ceiling within this window); promote only
    // once the beyond-LOW demand PERSISTS: a genuine ISS-class pattern rate
    // is re-checked every tick and promotes hold-off s later. Suppressed
    // entirely while a skip ramp-grace is armed (the detector is blanked in
    // that window: running blind into HIGH is how AO-123 false-latched).
    // 0 disables (legacy: promote immediately). Mechanism replay-validated
    // (--promote: LUCA −25 % :K alone, +8 % stacked on the tightened
    // high_gate_moving_x) but default OFF: the tight synthetic-accuracy cost
    // ([m-1hz]/[m-meridional] each +0.012 deg) is not paid for an 8 % wire
    // win; the mgate tightening is the band-edge fix of record. Enable per
    // profile if a live pass still shows promotion churn.
    double glide_promote_holdoff_s = 0.0;
    double idle_stop_dwell_s   = 1.5;    // LOW-band zero-demand hold-off:
                                         // keep the running rate this long
                                         // before issuing :K (cancel if the
                                         // demand returns). HIGH band never
                                         // dwells: a zero demand there is a
                                         // real stop.
    double floor_coast_s     = 1.2;      // board :K deceleration-coast model
    // --- coast-aware wire commits (operator rule, live AO-73 2026-11-19) --
    // A commit that puts the brakes on a running HIGH axis (sign flip or
    // stop) is not free: the board coasts ~1 s / ~1 deg at 600x no matter
    // how the demand is shaped, landing the tube at cur + sign(sent)*
    // |sent|*high_coast_s. When the flip was noise (GT2 2 s push steps
    // levers into axis-space pattern-velocity sign flips near the horizon),
    // each full flip cycle pays TWO coasts and lands the tube ~1 deg past
    // the setpoint on the far side — the ±3 deg self-sustaining ping-pong
    // of the live AO-73 pass (530 :K in one pass). LANDING TEST: commit a
    // brake only when the landing point is already at/past the setpoint —
    // err*sign(sent) <= coast_dist + coast_margin_deg. While the setpoint
    // is farther than coast+margin ahead of the mount IN THE RUNNING
    // DIRECTION, a brake mathematically lands short of its own goal: it
    // can only be noise, so keep running (the demand that WOULD be right
    // for the landing point re-crosses within one glide tick once the
    // pattern settles). Genuine apex reversals cross the test in well
    // under a second (the closing rate is |sent|), and dive flips now
    // trigger at coast distance, shrinking the live ~1.5 deg dive
    // overshoot to ~margin. 0 disables (pre-FO-29 behavior, sim default).
    double high_coast_s     = 0.0;
    double coast_margin_deg = 0.3;       // landing tolerance around the
                                         // setpoint the test commits at
    // --- re-home after pass complete --------------------------------------
    // The mount's DEC axis has no internal cord wrap (operator, 2026-11-19),
    // but the antenna feedline does: end every pass parked at the datum so
    // the cord unwinds at home, not mid-sky, and the cumulative rotation
    // budget self-centers (the AO-73→FO-29 incident: a pass ending 271° of
    // DEC from home left the next pre-aim nowhere legal to go). After the
    // client goes silent for rehome_idle_s beyond target staleness, the
    // daemon synthesizes the datum as its own target and flies it home;
    // ANY client push cancels instantly; on arrival (err < rehome_arrive_deg)
    // it stops, forgets the target and marks itself done until the next
    // real push. The self-push also keeps the transport watchdog fed: this
    // is deliberate daemon-owned motion (a client-less bench is exactly
    // when re-home must still run); every safety net (floor guard, travel
    // envelope, cumulative budget, skip detector, serial watchdog) stays
    // armed. 0 disables.
    double rehome_idle_s    = 0.0;
    double rehome_arrive_deg = 0.5;
    // Elevation ARM gate (live 2026-11-20: re-home fired in the
    // gap BETWEEN a GT2 pre-aim (el 0) and real AOS — 60 s of legitimate
    // pre-pass silence read as "pass over", and the mount wound 146 deg
    // back home moments before acquisition; GT2's AOS push cancelled it
    // but that round trip wastes the settle we want at AOS). The idle
    // homeflight only arms once the session has TRACKED a real pass: an
    // accepted push with el > this gate. GT2 pre-aims and post-LOS ghost
    // pushes are el 0 (its display clamps el >= 0), so they never arm
    // it; operators who want the old always-on behavior set 0.
    double rehome_min_el_deg = 2.0;
    // --- DEC cord-wrap policy (antenna cable health; OPERATOR SPEC
    // 2026-11-21 night, after the SO-50/AO-73 demos) ------------------
    // The UNFOLDED DEC offset from the datum IS the cord-wrap ledger:
    // the cord winds exactly when the motor turns DEC, and the :j step
    // counters integrate commanded DEC motion (a hand-rotate moves the
    // cord invisibly, which is why every post-manual-shaft-work session
    // re-datums at home). Guards fold DEC (the flip fusion makes |d|>180
    // pointing-identical), so a wound cord is INVISIBLE to them: AO-73
    // from a wrap-0 datum ended a clean-looking pass at exactly ONE
    // revolution (-359.6 deg), and the re-home first flight then dug
    // 111 deg DEEPER on the way home (nearest-image landing from -249).
    // wrap_home_unwind: homeward datum flights (re-home self-push AND
    // the envelope-refusal return) land on the CORD-FREE datum image:
    // DEC offset EXACTLY 0, never a +-360k image. The datum's DEC is
    // canonical, so this also fixes the branch (canonical). The return
    // is therefore ALWAYS in the unwinding direction — "return should
    // always know cord wrap, and unwrap no matter which way it has to
    // go" (cost bounded by one revolution of extra DEC travel; RA keeps
    // its wrap-aligned short path). false = legacy nearest-image landing.
    bool wrap_home_unwind = true;
    // Azimuth-first cord-unwind datum flights (2026-11-22 law): a rewind
    // with the DEC ledger in the upper half-space (|ledger| mod 360 in
    // [90, 270] — where a legal el >= 0 pose can sit cord-wound) orders
    // the trip RA-first: leg 1 carries RA home while DEC holds the cord,
    // leg 2 unwinds DEC at datum RA (the operator's free-spin regime;
    // that meridian IS the horizon circle, el never below 0). The
    // straight axis line floor-dives (analysis: min el -17.9 for the
    // live 2026-11-22 latch-pose family, ledger 168.5) and its floor
    // slide overshot RA ~50 deg az live. False = legacy straight line.
    bool cord_unwind_route = true;
    // wrap_window_deg: branch SELECTION (select_branch) never CHOOSES a
    // branch image whose landing DEC offset exceeds this magnitude — a
    // pass may drift the wrap up to this far either way (operator: 270;
    // 180 would force handovers far too often) but cannot be routed
    // past it while the other branch is legal (the "pre-position the
    // other way" rule). The handover grace band rides along for the
    // ridden branch (no scrape teleports). The envelope refusal gate
    // applies the same window to BOTH branch images: both beyond =
    // dig-stop → return home unwinds. 0 disables the window.
    double wrap_window_deg = 270.0;
    // Wrap-preference tiebreak at branch selection: prefer the branch
    // that saves more than wrap_tiebreak_margin_deg of cord when its
    // travel is at most wrap_tiebreak_extra_deg longer than the pick
    // (both in box/window). Both of tonight's windings STARTED at the
    // pre-aim branch pick choosing a slightly-nearer but wrap-heavy
    // side (AO-73: flip 104 deg with |DEC| ~150 over canonical with
    // ~30); this turns that pick around when the distance gap is small.
    // 0 disables the tiebreak.
    double wrap_tiebreak_margin_deg = 90.0;
    double wrap_tiebreak_extra_deg  = 30.0;
    // --- pattern estimator: alpha-beta filter per az/el channel -------
    // Filters each client push (ScalarABFilter): ramp-unbiased, optimal
    // for an irregular quantized cadence, and NO endpoint-noise
    // amplification (see struct docs; sliding-window LSQ put a
    // sign-flipping velocity wiggle into the glide reference every push,
    // live replay 2026-11-16). Gains follow the classic critically-damped
    // recommendation (beta ~ alpha^2/(2-alpha)): alpha = 1 tracks the
    // newest sample exactly, beta = 0.5 converges velocity in ~3 pushes
    // with +-0.009 deg/s residual noise on GT2's 0.1 deg/2 s cadence —
    // 10x below what flips demands across zero, and below reversal_min_x.
    double pattern_ab_alpha = 1.0;     // position correction fraction
    double pattern_ab_beta  = 0.5;     // velocity correction fraction
    // Estimator minimum update cadence (s). Pushes arriving sooner than
    // this since the last ESTIMATOR update keep target freshness (watchdog,
    // dive target, timestamps all advance) but the alpha-beta filter holds;
    // the next real update uses the LATEST sample at full gain, so a fast
    // client costs bandwidth, never bias. Rationale (live AO-73 2026-11-18):
    // GT2's iq-work follow gate double-fires, pushing PAIRS 0.1-0.4 s apart
    // carrying <=0.4 deg of newer info; beta*r/dt with dt = 0.15 turns the
    // correlated second sample into a ~1 deg/s velocity kick (the filter
    // assumes independent samples), inverting the axis pattern near the
    // horizon and storming :K restarts (265 in one pass). One update per
    // pair, referenced to the previous real update, measures the same
    // trajectory derivative without the correlated-sample amplification.
    // 1 s: covers the observed duplicate spacing with margin and is below
    // any GT2-class client's tracking cadence (GT2: 2 s). 0 disables.
    double pattern_min_dt_s = 1.0;
    double pattern_gate_deg    = 0.5;  // sample vs filter prediction above
                                       // this => client discontinuity
                                       // (satellite switch-over), reset
    double pattern_gate_rate_deg_s = 6.0; // dt-scaled gate allowance rate:
                                       // keeps a moving target (or any
                                       // post-reset filter, v = 0) from
                                       // deadlocking the gate. Compared in
                                       // AZ/EL push space; the az/el -> axis
                                       // Jacobian can amplify ~4x near the
                                       // horizon/pole, so this must clear
                                       // 4x the max apparent LEO rate
                                       // (~1.2 deg/s), while genuine
                                       // switch-overs land degrees away.
    double pattern_gap_reset_s = 120.0;  // client gap => reset the filter.
                                         // MUST exceed the real GT2 iq-work
                                         // throttle gap: with the b79ff90
                                         // follow throttle (push when target
                                         // moved >=1 deg, rotor within 1 deg)
                                         // near-horizon pushes come every
                                         // 20-60 s at ~0.02-0.05 deg/s — a
                                         // 4 s rule restarted the estimator
                                         // (velocity -> 0) on EVERY push and
                                         // the reference stair-stepped
                                         // (sparse replay 2026-11-17). LOS
                                         // gaps (new satellite anyway) are
                                         // minutes and still reset.
    bool writes_gated = false;           // board write gate ON (read-only
                                         // bench mode): skip detection is
                                         // meaningless — motion never reaches
                                         // the board, non-moving counters are
                                         // EXPECTED, never latch on them
    SafetyConfig safety{};
};

struct TrackerState {
    HsCoord target{};          // last commanded az/el
    HsCoord current{};         // derived from counters
    bool tracking = false;
    bool halted = false;       // rates forced to zero (blocked/latched/idle)
    bool rate_limited = false; // last tick demanded > safety max (clamped)
    bool floor_guard = false;  // trajectory floor guard shortened the setpoint
    bool glide = false;        // control law in glide mode (vs diving to a far target)
                               // this tick (path to target pierced the el floor)
    bool travel_returning = false; // mechanical travel envelope violated: the
                                   // tracker is driving back to the datum and
                                   // ignoring the client target (see safety.h
                                   // max_{ra,dec}_travel_deg); resumes on any
                                   // legal push (non-latching by design)
    std::string travel_reason;     // refusal evidence, e.g. "RA travel envelope: ..."
    // count of board :K-stop/restart reprograms (slew_rate band/dir changes;
    // each costs the deceleration tail and blanks the skip detector briefly)
    int stop_restart_ra = 0, stop_restart_de = 0;
    std::string last_stop_restart; // e.g. "DEC +412x -> -96x"
    std::string halt_reason;
    double rate_ra_x = 0, rate_dec_x = 0;   // last commanded rates
};

class Tracker : public RotctldHandler {
public:
    Tracker(Board& board, TrackerConfig cfg);

    // Establish reference: reads counters, marks current pose as home az/el.
    void calibrate(std::chrono::system_clock::time_point utc_now,
                   std::chrono::steady_clock::time_point mono_now);

    // Resume a persisted session (daemon restart, mount never moved): the
    // reference counters and axes are restored from state so the rotation
    // budget continues where it left off — cumulative rotation is measured
    // from the restored reference, exactly as before the restart. Caller
    // must have verified the live counters still match the persisted
    // reference (SessionState::resumable) before calling.
    void restore(uint32_t ref_count_ra, uint32_t ref_count_de,
                 AxisAngles restored_ref,
                 std::chrono::system_clock::time_point utc_now,
                 std::chrono::steady_clock::time_point mono_now);

    // Persistence accessors: what a session state file must record to allow
    // resume (reference counters + the axis angles assigned at calibration).
    bool calibrated() const { return calibrated_; }
    // True once a raw-counter jump beyond kCounterJumpDeg (see tracker.cpp)
    // proved the board was power-cycled mid-session: the anchor is garbage,
    // step() halts+latches, and main.cpp must NOT persist (and delete any)
    // session state — the operator re-homes and restarts --no-resume.
    bool board_reset_latched();
    uint32_t ref_count_ra() const;
    uint32_t ref_count_de() const;
    uint32_t live_count_ra();  // most recently observed raw counters
    uint32_t live_count_de();  // (pair them with current_axes())
    AxisAngles ref_axes() const { return ref_axes_; }

    // One control tick (thread-free core; call at cfg.update_hz).
    // board_now/utc_now may be supplied by tests (virtual time).
    void step(std::chrono::system_clock::time_point utc_now,
              std::chrono::steady_clock::time_point mono_now);

    // Diagnostics / state snapshot
    TrackerState snapshot();
    SafetyMonitor& safety() { return safety_; }
    AxisAngles current_axes();          // continuous (unwrapped) physical angles
    HsCoord current_pointing(std::chrono::system_clock::time_point utc_now);

    // RotctldHandler -----------------------------------------------------
    void on_set_target(double az_deg, double el_deg) override;
    void on_get_position(double& az_deg, double& el_deg) override;
    void on_client_rx() override { client_rx_.store(true, std::memory_order_relaxed); }
    void on_stop() override;
    void on_park() override;
    void on_reset() override;
    std::string on_status() override;

    // rotctld client count changed. The LAST client disconnecting is the
    // primary "operator gone" signal (GT2's stop = close the connection) and
    // drops the target: the next tick halts. Staleness alone is only a
    // transport-hiccup backstop (see target_stale_s).
    void on_clients_changed(int live_clients);

    // Test hook: target update with explicit (virtual) timestamp.
    void set_target_at(double az_deg, double el_deg,
                       std::chrono::steady_clock::time_point mono_now);

    // Optional telemetry tap on EVERY target push (rotctld `P` from any
    // client, and set_target_at callers): receives the push timestamp and
    // the RAW az/el as the client sent them. Used to record real client
    // push cadence (e.g. GridTracker2 iq-work) to a CSV for offline replay
    // against the simulator. Called under the tracker lock: must be cheap
    // and must not call back into the Tracker.
    void set_target_tap(
        std::function<void(std::chrono::steady_clock::time_point, double, double)> tap)
    {
        target_tap_ = std::move(tap);
    }

    // --debug tick tracing ("one-line autopsy"). Every step() emits one
    // composed key=value row covering: the client push (az/el/age), pattern
    // velocity, branch-space setpoint + error, demanded/sent axis rates with
    // board bands, raw :j counters, the :f status words, safety/mode flags,
    // and EVERY wire exchange the tick produced (Board::take_trace: raw tx
    // and rx frames, timeouts, NAKs, garbage, write-gate holds, latency).
    // set_debug(true) also switches on the Board wire trace and adds one
    // :f poll per axis per tick — an OBSERVER EFFECT by design (it makes
    // the :f history visible on a wire forensics hunt; it does add serial
    // traffic). The sink receives one line (no timestamp) per tick, called
    // under the tracker lock: must be cheap and must not re-enter Tracker.
    void set_debug(bool on);
    void set_debug_sink(std::function<void(const std::string&)> sink)
    {
        dbg_sink_ = std::move(sink);
    }

    // Operator-log seam: one-shot narrative lines from the control law
    // (rs44 branch-handover census: each routed handover maneuver gets one
    // line in the MAIN daemon log, greppable post-pass). Like the debug
    // sink, called under the tracker lock: must be cheap and must not
    // re-enter the Tracker.
    void set_log_sink(std::function<void(const std::string&)> fn)
    {
        logfn_ = std::move(fn);
    }

private:
    // step() body (lock held by the wrapper, which appends the debug row).
    void step_impl(std::chrono::system_clock::time_point utc_now,
                   std::chrono::steady_clock::time_point mono_now);
    void emit_debug_tick(std::chrono::steady_clock::time_point mono_now,
                         double dt);
    struct Target {
        double az = 0, el = 0;
        std::chrono::steady_clock::time_point t{};
        bool valid = false;
    };

    Board& board_;
    TrackerConfig cfg_;
    SafetyMonitor safety_;

    std::mutex mu_;
    // reference pose
    uint32_t ref_count_ra_ = 0, ref_count_de_ = 0;
    AxisAngles ref_axes_{};
    std::chrono::system_clock::time_point ref_utc_{};
    bool calibrated_ = false;

    Target target_;
    TrackerState state_;
    bool any_axis_running_ = false;
    std::chrono::steady_clock::time_point last_step_{};

    // Target velocity estimation + linear extrapolation between rotctld
    // updates: GPredict refreshes az/el only ~1 Hz while LEO pointing moves
    // continuously, so the control setpoint is target + vel_est*age.
    AxisAngles last_upd_des_{};   // desired axis angles at last update
    bool have_upd_des_ = false;
    // Pattern estimator: alpha-beta filter per az/el channel (see
    // set_target_at); az is unwrapped against the previous push.
    // have_vel_ = the filter has produced at least one update.
    bool have_vel_ = false;
    ScalarABFilter pat_ab_[2];          // [0]=az (unwrapped), [1]=el
    double   pat_a_ema_[2] = {0.0, 0.0}; // slow acceleration channel (deg/s^2)
    bool   pat_ab_ready_ = false;       // false => reset on next push
    std::chrono::steady_clock::time_point last_push_t_{};
    std::chrono::steady_clock::time_point last_ab_t_{};  // last ESTIMATOR
                                        // update (>= pattern_min_dt_s apart)
    HsCoord last_upd_hs_{};             // previous push, az unwrapped vs it
    bool have_upd_hs_ = false;
    HsCoord pattern_hs_vel_{};          // filter velocity in az/el (deg/s)
    HsCoord pattern_hs_acc2_{};         // slow accel in az/el (deg/s^2)
    AxisAngles pattern_vel_{};          // Jacobian-mapped to axis angles (deg/s)
    AxisAngles pattern_acc2_{};         //   "            "     (deg/s^2)
    AxisAngles filt_pos_{};             // filtered az/el mapped to axis angles
                                        // (filtered setpoint base, valid iff
                                        // have_vel_)

    // Glide reference (rate-follower): a continuous integrated position
    // that runs at the PATTERN velocity (EMA of sample derivatives) plus a
    // bounded convergence term walking it onto the predicted newest sample.
    // Steady state => reference rate == pattern rate: telescope-like
    // constant-rate running between pushes, no gap/window tuning, identical
    // behaviour for 10 Hz and 1°/2 s clients. Pattern prediction tapers to
    // zero when samples go silent (>pred_max..pred_hard s), so the reference
    // settles instead of running away on a stale velocity. The mount command
    // is the reference rate plus bounded nudges keeping the mount ON the
    // reference; sustained error closes by running slightly faster (the
    // integral term), never by jumping or stop-start cycling (control-law
    // directive 2026-11-16, "run it like a telescope mount").
    AxisAngles ref_pos_{};        // integrated reference position (ref space)
    double ref_pos_time_ = 0;     // last integration time, seconds
    AxisAngles ref_target_{};     // newest sample, ref space
    double ref_target_time_ = 0;  // its stamp, seconds
    bool have_ref_target_ = false; // a sample has been stored (any mode)
    bool have_ref_ = false;        // ref_pos_ initialised (glide engaged)
    bool glide_active_ = false;   // mode latch (engage/disengage hysteresis)
    bool slide_latch_ = false;    // floor guard binding: force dive authority
    bool board_reset_ = false;    // counter-jump latch (see kCounterJumpDeg):
                                  // sticky until calibrate()/restore() re-anchor
    double slide_clear_ = 0;      // last tick the guard bound (for release)
    AxisAngles nudge_err_{};      // integral of (reference - actual), deg*s
    double nudge_prev_time_ = 0;  // for integral dt bookkeeping
    // Measured axis-rate estimate (EMA of the :j counter derivative), deg/s
    // in AXIS space, sign-applied like `cur`. Diagnostic only: surfaced as
    // vm= in the --debug tick trace (commanded-vs-measured gap = :K coast /
    // restart starvation evidence on live metal). A velocity-feedback
    // DAMPING term consuming this was tried and rejected 2026-11-19 (see
    // TrackerConfig::demand_lpf_tau_s). Seeded fresh after any re-anchor
    // (calibrate/restore), like nudge_err_.
    AxisAngles vel_est_{};
    AxisAngles vel_prev_{};
    double vel_prev_time_ = 0;
    bool vel_seeded_ = false;

    // Glide demand LPF state: last filtered demand (deg/s axis space) per
    // axis. See TrackerConfig::demand_lpf_tau_s.
    double dem_prev_ra_ = 0.0, dem_prev_de_ = 0.0;
    bool dem_seeded_ = false;
    bool dem_was_glide_ = false; // mode-transition reseed (see above)

    // Glide LOW->HIGH promotion hold-off (TrackerConfig::
    // glide_promote_holdoff_s): stamp of the current unbroken
    // demand-beyond-LOW-while-running-LOW episode (0 = none).
    double promote_stamp_ = 0.0;

    // --- mechanical travel envelope (see safety.h max_{ra,dec}_travel_deg) --
    // Latch (NOT a safety latch: clears on the first legal target) entered
    // when the current pose or the target's best axis solution leaves the
    // datum-relative box: while set, the tracker drives to the datum and
    // ignores the client target. Resume requires check_travel Ok with
    // travel_resume_margin_deg (a client dithering at the wall must not
    // alternate home/target).
    bool returning_home_ = false;
    // True when the current tick's setpoint came from the over-the-top flip
    // branch (a+180, 180-d points IDENTICALLY on the level geometry —
    // verified numerically 2026-11-19): dive waypoint routing (canonical
    // leg projection, hs-space) is skipped for flip setpoints; the direct
    // axis line is in-box by construction and the velocity-projection floor
    // guard (branch-agnostic) still bounds the path.
    bool des_on_flip_ = false;
    // True while a far-branch handover is being DEFERRED because the
    // pattern is setting (rs44 LOS fix, see step_impl far-branch rule):
    // the mount keeps riding the old branch's wall-clamped solution.
    // Cleared when the branch flag stabilizes or the handover flies.
    bool handover_defer_ = false;
    // Wall-strain watchdog timeline (TrackerConfig::wall_strain_*): mono
    // time the current sustained strain episode began (0 = none running).
    double wall_strain_mono_ = 0.0;
    // Latched at the START of a homeward flight (envelope refusal or the
    // re-home self-push) while the mount RIDES the flip branch (live
    // 2026-11-20: homing from the flip LOS pose force-routed the datum
    // through CANONICAL hs waypoints — climb to el 81/sweep — whose axis
    // solve sits beyond the RA wall from the flip side; the belt clamp
    // pinned the setpoint at the wall and RA wound the LONG way around,
    // ~265 deg where the correct move was a ~95 deg rewind). Routing is
    // canonical-geometry-only (its comment above); from a flip pose the
    // datum is flown as the direct wrap-aligned axis line (short rewind;
    // the far-branch handover event makes it a fresh dive, and the floor
    // guard + wall guards police the path). Cleared when the flight ends.
    bool home_no_route_ = false;
    // Glide-reference twin of des_on_flip_: latched at glide ENGAGE, the
    // reference space (ref_pos_, ref_target_) is the flip image of the
    // canonical pattern; ref_advance maps the pattern velocities to match
    // (DEC sign), and set_target_at stores push samples already in ref
    // space. Cleared on disengage (sample restored to canonical).
    bool ref_flip_ = false;
    // True once the envelope refusal has been reported (edge the log).
    bool travel_reported_ = false;
    // Canonical axis solution + over-the-top flip, box-filtered (the wall
    // outranks continuity — except that leaving is only forced once the
    // other branch beats the ridden one by more than the handover margin,
    // see TrackerConfig::branch_handover_margin_deg), else nearest-cur.
    // cur_on_flip = the branch the mount RIDES (previous tick's choice):
    // hysteresis and forcing are evaluated against it, not against an
    // in-box-canonical preference (rs44 LOS fix — the box test is NOT
    // invariant under the flip because dec maps 180-d). second = flip.
    // Both-outside returns canonical unchanged (the envelope gate below
    // refuses from it) and box-disabled degenerates to nearest-cur.
    std::pair<AxisAngles, bool> select_branch(const AxisAngles& can,
                                              const AxisAngles& cur,
                                              bool cur_on_flip) const;
    AxisAngles home_cordfree_des(const AxisAngles& cur) const;
    bool travel_in_box(const AxisAngles& ax) const;
    AxisAngles clamp_travel(const AxisAngles& des, const AxisAngles& cur) const;

    // integrate the reference to now_d and return its commanded rate
    AxisAngles ref_advance(double now_d, double client_age_s);
    // Transport-watchdog feed for rotctld callbacks (p poll, S, R) arriving
    // between loop ticks: stamp with the loop's own mono timeline (last_step_,
    // same clock basis as check_watchdog's input), real time only before the
    // first calibrate/step. Clock consistency matters: a real-time stamp fed
    // into a virtual-time harness (tests, replay) makes the watchdog fire
    // immediately on the next tick instead of clearing its hold.
    std::chrono::steady_clock::time_point client_stamp() const
    {
        return last_step_ == std::chrono::steady_clock::time_point{}
            ? std::chrono::steady_clock::now() : last_step_;
    }

    AxisAngles counters_to_axes(uint32_t c_ra, uint32_t c_de); // unwrapped

    // 24-bit counter wrap accounting: signed accumulated steps since reference
    int64_t accum_ra_ = 0, accum_de_ = 0;
    uint32_t last_raw_ra_ = 0, last_raw_de_ = 0;

    // feedforward: previous tick's desired continuous axis angles
    AxisAngles prev_desired_{};
    bool have_prev_desired_ = false;

    // last rates actually sent to the board (chatter suppression)
    double sent_ra_x_ = 0, sent_de_x_ = 0;
    double idle_pend_ra_ = 0, idle_pend_de_ = 0; // mono-secs stamp of a
    // LOW-band pending stop (0 = none); see TrackerConfig::idle_stop_dwell_s
    double flip_pend_ra_ = 0, flip_pend_de_ = 0; // mono-secs stamp of a
    // pending (confirming) sign flip; see TrackerConfig::reversal_confirm_s
    // HIGH reprogram persistence confirm (TrackerConfig::high_recommit_s):
    // mono-secs the current out-of-deadbands HIGH-touching demand episode
    // began (0 = none pending). Cleared on commit, on any tick the demand
    // falls back inside the band/economy zones, and on stops/reversals.
    double scale_pend_ra_ = 0.0, scale_pend_de_ = 0.0;
    // Last committed HIGH rescale MAGNITUDE direction per axis (+1 demand
    // above sent, -1 below, 0 none yet): the alternation ledger the
    // direction-persistence law reads (a rescale opposite to this is the
    // wobble candidate held for confirmation; same-direction ladders are
    // genuine work and commit immediately). Updated only on commits.
    int scale_dir_ra_ = 0, scale_dir_de_ = 0;
    // Wobble-regime leaky COMMIT counters per axis (see TrackerConfig::
    // high_recommit_regime_*): rescale commits + regime silent-holds add,
    // exponential decay with time constant high_recommit_regime_s; the
    // axis is in regime while the counter >= high_recommit_regime_n.
    // rs_* stamps the last decay update (mono-secs).
    double scale_reg_ra_ = 0.0, scale_reg_de_ = 0.0;
    double scale_reg_stamp_ra_ = 0.0, scale_reg_stamp_de_ = 0.0;
    // Last stamp where the client-push age passed the regime staleness
    // gate (mono-secs; see TrackerConfig::high_recommit_regime_age_s).
    double scale_reg_oldage_ra_ = 0.0, scale_reg_oldage_de_ = 0.0;
    // Per-axis wobble-regime flag latched at the end of each apply_axis
    // economy block; the demand-LPF escalation reads it with one-tick lag
    // (see TrackerConfig::demand_lpf_regime_tau_s).
    bool regime_ra_ = false, regime_de_ = false;
    // Was the escalated tau actually used on this tick's glide demand
    // per axis? Surfaced as the lrpf flag in the --debug tick row.
    bool dem_esc_ra_ = false, dem_esc_de_ = false;
    // Tick-row mirror of the leaky wobble-regime counters (reg= field;
    // last known value — the ledger itself decays continuously anyway).
    double dbg_reg_ra_ = 0.0, dbg_reg_de_ = 0.0;
    // Error ledger for the regime magnitude gate (see apply_axis):
    // |axis error| at the last rescale commit per axis; silent holds
    // require the error to have stopped improving against it.
    double rescale_err_ra_ = 0.0, rescale_err_de_ = 0.0;

    // re-home after pass complete; see TrackerConfig::rehome_idle_s
    double rehome_idle_mono_ = 0;   // mono-secs stamp of target-gone start
    bool rehome_active_ = false;    // daemon owns the target, flying home
    bool rehome_done_ = false;      // parked at datum; cleared by a push
    // el-gate arming (TrackerConfig::rehome_min_el_deg): a push with
    // el above the gate (a REAL pass, not a GT2 el-0 pre-aim or post-LOS
    // ghost) sets this; the idle clock only runs while it is set; it is
    // cleared at arrival (one homeflight per real pass) and by
    // calibrate/restore (a fresh session at the datum has nothing to
    // unwind).
    bool rehome_armed_ = false;

    // skip/stall detection: previous tick's measured angles and commanded rates
    AxisAngles prev_cur_{};
    bool have_prev_cur_ = false;
    double prev_ra_x_ = 0, prev_de_x_ = 0;
    // accumulated (commanded − measured) axis motion while powered; a real
    // stall drifts this past tracker_skip_deg within ~1 s of motion.
    double skip_ra_ = 0, skip_de_ = 0;
    double skip_exp_ra_ = 0, skip_exp_de_ = 0;
    // Board-acceleration grace: mono time of the last band event (cold :J
    // from stopped, band entry via in-place :I, or forced stop/restart) per
    // axis. The :j counters count
    // ISSUED steps, and the board ramps its issue rate up after :J, so the
    // counters legitimately lag the commanded rate during that ramp (live
    // Sirius 2026-11-16: a +600x restart false-latched skip mid-acquisition).
    // Skip deficit expectation scales 0->1 over the per-band grace duration
    // chosen at the stamp event (below); in-place low-speed :I changes that
    // stay in one band do NOT restamp (live-verified immediately effective).
    std::chrono::steady_clock::time_point ramp_ra_mono_{}, ramp_de_mono_{};
    // :K-coast starvation window INSIDE a grace period (stop/restart only):
    // issuance is zero from the restart until coast+re-accel elapse (sweep-6
    // forensics: HIGH restarts starve ~2 s; the old linear 0->1 ramp assumed
    // immediate delivery and skip-latched at ramp_age 2.5/3.0 s).
    double ramp_ra_blind_s_ = 0.0, ramp_de_blind_s_ = 0.0;
    // Grace window length paired with the stamp: chosen by board program
    // band at the stamp event (cold :J or band entry — see tracker.cpp
    // band classification in apply_axis): slow-ramp band [low_min_x, 104x)
    // gets skip_ramp_grace_low_s, everything else skip_ramp_grace_s.
    double ramp_ra_grace_s_ = 0.0, ramp_de_grace_s_ = 0.0;
    // Last band classification per axis: 0 LOW-slow, 1 slow-ramp band,
    // 2 HIGH (104x+). Band ENTRY (with hysteresis) restamps the grace.
    int ramp_ra_band_ = 0, ramp_de_band_ = 0;

    // rotctld command-arrival flag (server thread -> step()); see the
    // comment on RotctldHandler::on_client_rx.
    std::atomic<bool> client_rx_{false};
    // cumulative :K-stop/restart reprograms per axis (surfaced in state)
    int stop_restarts_ra_ = 0, stop_restarts_de_ = 0;
    // last rotctld client count (status panel; set in on_clients_changed)
    int live_clients_ = 0;
    // per-axis SENT rate at entry to apply_axis this tick (the "from" of a
    // stop/restart event, for last_stop_restart forensics)
    double pre_sent_ra_ = 0, pre_sent_de_ = 0;

    std::function<void(std::chrono::steady_clock::time_point, double, double)> target_tap_;

    // --- --debug tick tracing (emit_debug_tick): per-tick capture ----------
    // Plain stores in step_impl (a handful of doubles), so an off trace has
    // no branches in the hot path; only the COMPOSITION (formatting, Board
    // drain, :f poll) is gated behind debug_ + dbg_sink_.
    bool debug_ = false;
    uint64_t dbg_seq_ = 0;
    double dbg_target_age_ = 0;      // s since last client push
    AxisAngles dbg_cur_{}, dbg_des_{}; // measured / final setpoint (axis space)
    double dbg_err_ = 0;             // axis-space setpoint error (hypot)
    double dbg_x_ra_ = 0, dbg_x_de_ = 0; // demanded rates (post floor-guard +
                                         // joint cap, pre apply_axis economy)
    // event-delta bookkeeping between emitted rows:
    int dbg_sr_ra_ = 0, dbg_sr_de_ = 0;  // stop/restart counters at last row
    std::function<void(const std::string&)> dbg_sink_;
    std::function<void(const std::string&)> logfn_;  // narrative operator log
};

// Skip-detector grace window for a board-program event (cold :J or band
// entry) at a given commanded rate:
// slow-ramp LOW band [skip_ramp_grace_low_min_x, 104x) gets the long
// measured board-accel window; everything else the short default.
// (Definition + evidence in tracker.cpp.)
double cold_start_grace_s(const SafetyConfig& sc, double x_cmd);

// Zero-issuance window at a :K stop/restart (blind) for the band of the
// NEW program; 0 at cold :J / band entry. See SafetyConfig docs.
double restart_blind_s(const SafetyConfig& sc, double x_cmd);

// Full stop/restart grace window (blind + ramp-to-full END) at the restart
// stamp. HIGH takes skip_restart_grace_high_s — deliberately NOT the cold
// profile's grace, which is shorter than the restart blind and produced the
// 2026-11-22 double-latch crack (see SafetyConfig docs). LOW/slow-ramp bands
// reuse cold_start_grace_s (their restart blind is far inside it).
double restart_grace_s(const SafetyConfig& sc, double x_cmd);

// Production wrapper: calls step() at 10 Hz with real clocks.
class Runner {
public:
    explicit Runner(Tracker& tracker);
    void start();
    void stop();
private:
    void loop();
    Tracker& tr_;
    std::atomic<bool> running_{false};
    std::thread th_;
};

} // namespace sateq
