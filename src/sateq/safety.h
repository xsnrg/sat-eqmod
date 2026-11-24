// Safety gate for satellite tracking. The mount has NO absolute endstops —
// the real hazards are (a) pointing into the ground, (b) winding the ribbon
// cable around the RA axis, (c) commanding rates the boards can't follow,
// (d) losing the controller/serial link mid-slew. Each has a dedicated net.
//
// The tracker asks the guard before EVERY rate update; the guard sees each
// proposed sky pointing plus the live axis angles. Anything rejected returns
// a reason enum; anything requiring action beyond zeroing rates (cable limit)
// latches until cleared.
#pragma once
#include "sateq/astro.h"

#include <chrono>
#include <string>

namespace sateq {

struct SafetyConfig {
    double min_elevation_deg = -15.0;     // MECHANICAL floor: tube/pier/ground
                                          // contact. Pure backstop, checked in
                                          // SafetyMonitor against any pointing.
    double track_stop_el = -5.0;          // operational track gate (tracker):
                                          // command motion only at/above this.
                                          // Arm at -5 rising (motors up to
                                          // speed by AOS), stop at radio LOS.
    double max_ra_cumulative_deg  = 540;  // ±1.5 turns before unwrap required
    double max_dec_cumulative_deg = 270;  // bounds the FOLDED offset (flip
                                          // branch folds at +-180, so with
                                          // the default this never fires:
                                          // DEC is the free axis, RA guard
                                          // is the cable backstop)
    // MECHANICAL travel envelope, datum-relative (stand clearance): motion
    // is refused when the CURRENT pose or the commanded TARGET's axis
    // solution exceeds this offset from the calibration datum, folded into
    // (-180,180]. Unlike the cumulative guards above (a cabling budget
    // that latches for a human), this one is a wall the tube physically
    // hits: live 2026-11-19 a GT2 pre-aim wound RA ~160 deg from datum,
    // the tube stopped at the stand, and the stalled 600x slew dragged the
    // 12 V rail and killed the serial link. The tracker's response is
    // automatic return-to-datum + hold, NOT a latch (a client re-push
    // within limits resumes). 105 = 180 - 15: leaves 15 deg of horizon
    // authority on the last degrees of either flank (user decision).
    // 0 disables that axis (e.g. a pier with wrap rings).
    double max_ra_travel_deg  = 105;
    double max_dec_travel_deg = 105;
    double travel_resume_margin_deg = 5.0;  // hysteresis to LEAVE the
                                          // envelope refusal once entered:
                                          // a target must be this far back
                                          // inside the box to resume, so a
                                          // client dithering at the wall
                                          // cannot alternate home/target
    double max_rate_x_sidereal = 600.0;   // below board max (800) by margin
    double max_rate_step_x_per_s = 0;     // 0 = unlimited rate-slew limit
    double tracker_skip_deg = 0.5;        // |target−actual| beyond this = lost lock
    double skip_ramp_grace_s = 3.0;       // after a cold :J start or forced
                                          // stop/restart the board physically
                                          // ramps its step-issue rate (counter
                                          // follows issuance): expected motion
                                          // scales 0→1 over this window before
                                          // skip deficit accumulates (live
                                          // Sirius 2026-11-16: a +600x restart
                                          // false-latched skip with grace 0)
    double skip_ramp_grace_low_s = 25.0;  // slow-ramp LOW band: an axis whose
                                          // LOW program commands |rate| in
                                          // [low_min_x, 104x] starves/phase-
                                          // loses 10-25 % converging over
                                          // 15-25 s (Phase-1 Sirius: delivered
                                          // 0.78-0.90 at 80-100x; <55x fine
                                          // immediately). First live
                                          // Phase-4 sweep 2026-11-17: a ~60x
                                          // axis false-latched skip at t=24 s
                                          // on the 3 s (high-band) grace.
    double skip_ramp_grace_low_min_x = 55.0; // slow-ramp threshold (Phase-1:
                                          // 50x delivered 0.999, 60x 1.018;
                                          // deficit evidence starts 80x — 55
                                          // covers the sweep 62x latch)
    // :K-coast starvation INSIDE a skip grace, applied at a stop/restart
    // (never cold :J / band entry): the axis stops, free-wheels its decel
    // tail, then :J re-accelerates — issuance is ZERO for most of that
    // window. Linear 0->1 ramps from t=0 assumed immediate coasting and
    // still latched (live sweep-6 2026-11-18: DEC skip at ramp_age 2.5 of a
    // 3.0 s grace after a -600x :K restart; coast alone is ~1 s and ~0.7 deg
    // at HIGH). expected scales 0 until blind_s, then linearly to 1 at
    // grace_s. HIGH measured coast+re-accel ~2 s; LOW coast ~0.03-0.2 s.
    double skip_restart_blind_high_s = 3.0;  // |rate| >= 104x (HIGH program).
                                          // Live dive-roundtrip 2026-11-18:
                                          // real-board restart starved > 2.8 s
                                          // (skip fired at ramp_age 2.8 with
                                          // blind 2.0); sweep-6 evidence ~2 s
    double skip_restart_grace_high_s = 4.0;  // ...FULL expectation by this
                                          // age at a HIGH stop/restart. MUST
                                          // exceed skip_restart_blind_high_s:
                                          // the live skip DOUBLE-latch of
                                          // 2026-11-22 (03:06 refusal-return
                                          // + 06:40 re-home rewind, both
                                          // "grace 2s blind 3.0 ramp_age
                                          // 2.7") fired because the restart
                                          // stamp reused the COLD profile's
                                          // 2.5 s grace with the 3.0 s
                                          // restart blind — expectation
                                          // snapped back to full at 2.5 s
                                          // while the board is still starved
                                          // to 3.0 s, and that 0.5 s crack
                                          // accumulates ~1.2 deg of false
                                          // deficit at 600x. blind (3.0,
                                          // measured) + ~1 s re-accel ramp
                                          // (Phase-1 HIGH accel ~1 s).
    double skip_restart_blind_low_s  = 0.25; // LOW program stop/restart
    // Cold-:J zero-issuance window (board was NOT running): live AO-123
    // acquisition 2026-11-20 — cold dive to +600x HIGH delivered only
    // 0.31/0.86 deg at ramp_age 2.0 s against the linear grace model;
    // the board's cold acceleration is DEAD early then ramps (mirrors the
    // restart blind, which covers the :K-armed path). The 25 s LOW
    // slow-ramp grace starts from ~0.85 delivery (Phase-1) — no blind.
    double skip_cold_blind_high_s = 2.0;  // cold :J / band entry >=104x
    double skip_cold_grace_high_s  = 2.5; // ...FULL by this age: the AO-123
                                          // dive profile (dead ~2.0 s, then
                                          // the board snaps to commanded —
                                          // Phase-1: HIGH accelerates in
                                          // ~1 s). The old linear 0->1 over
                                          // 3 s profile expected 0.67 at
                                          // t=2 s vs actual 0.36 and the
                                          // deficit kept growing: same dive
                                          // skip-latched at ramp_age 2.0.
                                          // blind+short ramp reproduces the
                                          // evidence: expected integral
                                          // 0.24-of-full vs actual ~0.2.
    // SUSTAINED issue-rate floor of the slow-ramp LOW band [low_min_x,104x):
    // the Phase-1 "delivered converges to 1.0 over 15-25 s" reading came
    // from SHORT runs; three sustained live encounters since show the
    // vintage board's :j counters plateau at ~0.78-0.80 of commanded
    // INDEFINITELY there (sweep-2 2026-11-17 DEC latch, LUCA dwell
    // 2026-11-20, RS-44 apex +100x latch at ramp_age 48 s with deficit
    // still growing). Mechanism (vendor PDF + community): at these step
    // periods (6 µs class at 100x) the 2.xx step timer loses ticks to
    // other ISRs; integer tick quantization offers no headroom. The skip
    // expectation in this band therefore multiplies commanded issuance by
    // this factor AFTER the grace ramp; 1.0 disables (legacy: expect full
    // convergence). Chosen below the measured 0.78 floor for headroom —
    // a genuine jam/stop still shows ~full deficit against the discounted
    // baseline and trips at the usual 20 % threshold.
    double skip_supply_slow_band = 0.75;
    double skip_cold_blind_low_s  = 0.0;  // LOW programs (unused by
                                          // default: slow-ramp model has
                                          // its own start ratio)
    double watchdog_timeout_s = 5.0;      // stale rotctld link => e-stop
    double serial_stale_s = 2.0;          // no successful board I/O => e-stop
};

enum class GuardResult {
    Ok,               // safe to command
    Blocked,          // do not command this update (below horizon, rate limit)
    Latched,          // cable/unwrap or skip latch: stay stopped until clear()
};

struct GuardDecision {
    GuardResult result = GuardResult::Ok;
    std::string reason;
};

class SafetyMonitor {
public:
    explicit SafetyMonitor(SafetyConfig cfg = {});

    const SafetyConfig& config() const { return cfg_; }

    // Sky-level check for a proposed tracking target.
    GuardDecision check_pointing(double el_deg) const;
    // Rate sanity for a proposed axis rate.
    GuardDecision check_rate(double rate_x_sidereal) const;
    // Cumulative rotation from the reference (wrap) axes angles since set_reference().
    void set_reference(const AxisAngles& axes_now);
    AxisAngles reference() const { return ref_; }
    GuardDecision check_cumulative(const AxisAngles& axes_now) const;
    // Mechanical travel envelope (see SafetyConfig::max_{ra,dec}_travel_deg).
    // NON-latching refusal: returns Blocked when the current pose or the
    // proposed target's axis solution leaves the datum-relative box.
    // margin > 0 shrinks the box (hysteresis for the resume check).
    GuardDecision check_travel(const AxisAngles& current,
                               const AxisAngles& target,
                               double margin = 0.0) const;

    // Latches ------------------------------------------------------------
    void trip(const std::string& reason);     // e.g. skip, serial fail, e-stop
    void clear();
    bool latched() const { return latched_; }
    const std::string& latch_reason() const { return latch_reason_; }

    // Watchdog ------------------------------------------------------------
    void note_client_command(std::chrono::steady_clock::time_point t);
    void note_board_ok(std::chrono::steady_clock::time_point t);
    // seconds since the last rotctld command (huge if none seen): the
    // glide law's client-presence signal (tracker.h:pattern_alive_pred_max_s)
    double client_age_s(std::chrono::steady_clock::time_point t) const
    {
        if (!client_seen_) return 1e18;
        return std::chrono::duration<double>(t - last_client_).count();
    }
    // Returns a trip action when a watchdog expired (and trips+stops repeat).
    GuardDecision check_watchdog(std::chrono::steady_clock::time_point t,
                                 bool motors_may_be_running) const;
    // True between a client-absent watchdog trip and the next client
    // command: the tracker must keep holding (stopped) even though the
    // one-shot check_watchdog() already returned Ok, because the target
    // may still be "valid" while the operator is gone. Unlike trip(),
    // this clears automatically on note_client_command() — the returning
    // client IS the recovery signal.
    bool client_watchdog_active() const { return client_tripped_; }
    const std::string& watchdog_reason() const { return watchdog_reason_; }

private:
    SafetyConfig cfg_;
    AxisAngles ref_{};
    bool has_ref_ = false;
    bool latched_ = false;
    std::string latch_reason_;
    mutable std::chrono::steady_clock::time_point last_client_{};
    mutable std::chrono::steady_clock::time_point last_board_{};
    mutable bool client_seen_ = false;
    mutable bool client_tripped_ = false;  // cleared by note_client_command
    mutable bool board_tripped_ = false;   // board I/O watchdog (fault path)
    mutable std::string watchdog_reason_;
};

} // namespace sateq
