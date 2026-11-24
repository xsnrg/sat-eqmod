#include "sateq/tracker.h"

#include "sateq/version.h"

#include <algorithm>
#include <cstdarg>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <string>

namespace sateq {

namespace {
constexpr double SIDEREAL_DEG_PER_S = STELLAR_SPEED_ARCSEC_S / 3600.0; // ~0.004178
constexpr int64_t COUNTER_MOD       = 16777216;                        // 2^24

// --debug row composition helpers ------------------------------------------
void app(std::string& s, const char* fmt, ...)
{
    char tmp[512];
    va_list ap;
    va_start(ap, fmt);
    int n = std::vsnprintf(tmp, sizeof(tmp), fmt, ap);
    va_end(ap);
    if (n > 0) s.append(tmp, std::min<size_t>(static_cast<size_t>(n), sizeof(tmp) - 1));
}

// A raw wire frame embedded in the key=value row: bytes outside printable
// ASCII and the characters this format uses as delimiters (';' '"' ' ') are
// escaped as <XX> hex, so a garbage frame survives the row byte-exactly
// (that is the whole point of the trace: swallowing unprintables in a
// string field once hid a link failure inside an exception-only path).
// keep_space=true is for QUOTED human-readable reasons (halt, travel): those
// are delimited by the quotes, so only '"' and control bytes need escaping.
void append_wire_token(std::string& s, const std::string& raw, bool keep_space = false)
{
    for (unsigned char c : raw)
    {
        if (c >= 0x21 && c <= 0x7e && c != ';' && c != '"' && (keep_space || c != ' '))
            s += static_cast<char>(c);
        else if (keep_space && c == ' ')
            s += ' ';
        else
            app(s, "<%02X>", c);
    }
}

// signed 24-bit wrap resolution of a raw counter delta (poll deltas are
// always far below half a counter period at 10 Hz, so this is exact)
int64_t wrap24(uint32_t raw_u)
{
    int32_t d = static_cast<int32_t>(raw_u);
    if (d > COUNTER_MOD / 2) d -= static_cast<int32_t>(COUNTER_MOD);
    if (d < -COUNTER_MOD / 2) d += static_cast<int32_t>(COUNTER_MOD);
    return d;
}

double clampd(double v, double a, double b) { return std::min(b, std::max(a, v)); }

// Fold an axis offset into (-180, 180]: the mechanical travel envelope is
// a distance-from-datum check, and a pose cannot leave the box without
// crossing a wall, so taking the SHORT way round can never mask a
// violation (RA is physically 360° periodic; |offset| > 180 for DEC is
// unreachable by definition, folding is a no-op there).
double fold180(double v)
{
    while (v > 180.0) v -= 360.0;
    while (v < -180.0) v += 360.0;
    return v;
}

// Upper-half-space ledger (azimuth-first cord-unwind law, 2026-11-22):
// the datum's DEC offset (cord ledger) folded to [0, 360). TRUE for
// ledgers 90..270 — the winding depths a legal (el >= 0) pose can hold
// (below 90 the cord is nearly free and the short rewind is the
// direct-line regime live-validated 2026-11-21 AO-73). This band is the
// ENTRY gate for the cord-unwind datum flights (it latches the
// routing-skip home_no_route_); the leg order inside such a flight is
// gated by RA arrival, not by the ledger (see the leg block in
// step_impl: holding the cord while RA sweeps is floor-safe across the
// band; the probe shows the datum RA meridian is the horizon circle,
// el ≡ 0, and the descent dip is the RA-error × depth coupling).
bool ledger_upper_half(double ledger_deg)
{
    const double l = std::fmod(std::fabs(ledger_deg), 360.0);
    return l >= 90.0 && l <= 270.0;
}

// Map a canonical-branch axis quantity into the FLIP reference space:
// (a, d) and (a+180, 180-d) are the SAME tube pointing (level geometry,
// verified 2026-11-19), so a glide reference chasing the flip branch sees
// positions through this image and velocities through its derivative —
// RA shifts by a constant (velocity unchanged), DEC reflects (velocity
// sign flips). is_pos = false gives the velocity/acceleration map.
AxisAngles flip_map(const AxisAngles& x, bool flip, bool is_pos)
{
    if (!flip) return x;
    return {x.ra_deg + (is_pos ? 180.0 : 0.0),
            (is_pos ? 180.0 : 0.0) - x.dec_deg};
}
}

// Skip-detector grace length for a board-program event at this commanded
// rate (cold :J or band entry). The
// board's issuance ramp is band-dependent: Phase-1 + first live Phase-4
// sweep (2026-11-17): 60-90x LOW cold starts accelerate over 15-25 s
// (a 62x axis false-latched skip at t=24 s on the 3 s grace), while
// HIGH-band and low slow-ramp rates (<55x) reach commanded ratio in ~1 s.
// |rate| >= 104x programs HIGH (board promotes the 104..LOWSPEED_RATE band
// for the 6 us floor), so the slow-ramp band is [low_min_x, 104).
double cold_start_grace_s(const SafetyConfig& sc, double x_cmd)
{
    const double ax = std::fabs(x_cmd);
    if (ax >= sc.skip_ramp_grace_low_min_x && ax < 104.0)
        return sc.skip_ramp_grace_low_s;
    // HIGH band: cold profile = blind (skip_cold_blind_high_s, stamped at
    // the cold :J) then ramp to full at skip_cold_grace_high_s. AO-123
    // 2026-11-20: cold +600x dive delivered 0.36 at t=2 s vs the linear
    // profile's 0.67 and skip-latched — the board's cold issuance is DEAD
    // early, not linear.
    if (ax >= 104.0 && sc.skip_cold_grace_high_s > 0.0)
        return sc.skip_cold_grace_high_s;
    return sc.skip_ramp_grace_s;
}

// Zero-issuance window at a stop/restart (see SafetyConfig::skip_restart_-
// blind_*): :K coast + :J re-acceleration, chosen from the rate being
// REprogrammed (the board's band for the new program).
double restart_blind_s(const SafetyConfig& sc, double x_cmd)
{
    return std::fabs(x_cmd) >= 104.0 ? sc.skip_restart_blind_high_s
                                     : sc.skip_restart_blind_low_s;
}

double restart_grace_s(const SafetyConfig& sc, double x_cmd)
{
    // HIGH restart: dedicated window (blind + re-accel ramp). Pairing the
    // restart blind (3.0 s) with the COLD profile's grace (2.5 s) was the
    // 2026-11-22 double-latch: expectation resumed FULL at 2.5 s while the
    // board stays starved to 3.0 s — a deterministic ~1.2 deg false deficit
    // per HIGH restart at 600x. Both morning latches read "grace 2s blind
    // 3.0 ramp_age 2.7" — the crack between the two numbers.
    if (std::fabs(x_cmd) >= 104.0) return sc.skip_restart_grace_high_s;
    return cold_start_grace_s(sc, x_cmd);
}

Tracker::Tracker(Board& board, TrackerConfig cfg)
    : board_(board), cfg_(cfg), safety_(cfg.safety) {}

// ---------------- reference / geometry ----------------

void Tracker::calibrate(std::chrono::system_clock::time_point utc_now,
                        std::chrono::steady_clock::time_point mono_now)
{
    std::lock_guard<std::mutex> lk(mu_);
    ref_count_ra_ = board_.position(AxisRA);
    ref_count_de_ = board_.position(AxisDE);
    accum_ra_ = accum_de_ = 0;
    last_raw_ra_ = ref_count_ra_;
    last_raw_de_ = ref_count_de_;

    ref_axes_ = hs_to_axes(cfg_.home_az_deg, cfg_.home_el_deg, cfg_.site.lat_deg,
                           cfg_.orientation);
    ref_utc_ = utc_now;
    calibrated_ = true;
    have_prev_desired_ = false;
    have_prev_cur_ = false;
    have_upd_des_ = have_vel_ = false;
    pat_ab_ready_ = false;   // pattern estimator restarts on next push
    have_upd_hs_ = false;
    have_ref_ = have_ref_target_ = glide_active_ = slide_latch_ = false;
    ref_flip_ = false;
    nudge_err_ = {};
    vel_seeded_ = false; // re-anchored mapping: rate history invalid
    dem_seeded_ = false; //  "   "  demand-filter history invalid
    skip_ra_ = skip_de_ = skip_exp_ra_ = skip_exp_de_ = 0;
    prev_ra_x_ = prev_de_x_ = 0;
    safety_.set_reference(ref_axes_);
    safety_.note_board_ok(mono_now);
    board_reset_ = false; // new anchor: counter-integrity detection re-arms
    last_step_ = mono_now;
    // new datum (calibrate) or continued session (restore): the envelope
    // box is measured from ref_axes_, so its refusal latch resets here.
    returning_home_ = false;
    des_on_flip_ = false;
    handover_defer_ = false;
    wall_strain_mono_ = 0.0;
    home_no_route_ = false;
    state_.travel_returning = false;
    state_.travel_reason.clear();
}

void Tracker::restore(uint32_t ref_count_ra, uint32_t ref_count_de,
                      AxisAngles restored_ref,
                      std::chrono::system_clock::time_point utc_now,
                      std::chrono::steady_clock::time_point mono_now)
{
    // Caller verified SessionState::resumable(live counters) before calling:
    // the mount never moved while we were down, so the persisted reference
    // axes are still valid and the safety budget continues seamlessly —
    // cumulative rotation is derived from counters minus reference, no
    // snapshot arithmetic needed.
    std::lock_guard<std::mutex> lk(mu_);
    const uint32_t live_ra = board_.position(AxisRA);
    const uint32_t live_de = board_.position(AxisDE);
    ref_count_ra_ = ref_count_ra;
    ref_count_de_ = ref_count_de;
    accum_ra_ = wrap24(live_ra - ref_count_ra);   // normally 0 (unmoved)
    accum_de_ = wrap24(live_de - ref_count_de);
    last_raw_ra_ = live_ra;
    last_raw_de_ = live_de;
    ref_axes_ = restored_ref;
    ref_utc_ = utc_now;
    calibrated_ = true;
    have_prev_desired_ = false;
    have_prev_cur_ = false;
    have_upd_des_ = have_vel_ = false;
    pat_ab_ready_ = false;   // pattern estimator restarts on next push
    have_upd_hs_ = false;
    have_ref_ = have_ref_target_ = glide_active_ = slide_latch_ = false;
    ref_flip_ = false;
    nudge_err_ = {};
    vel_seeded_ = false; // re-anchored mapping: rate history invalid
    dem_seeded_ = false; //  "   "  demand-filter history invalid
    skip_ra_ = skip_de_ = skip_exp_ra_ = skip_exp_de_ = 0;
    prev_ra_x_ = prev_de_x_ = 0;
    safety_.set_reference(ref_axes_);
    safety_.note_board_ok(mono_now);
    board_reset_ = false; // new anchor: counter-integrity detection re-arms
    last_step_ = mono_now;
    // new datum (calibrate) or continued session (restore): the envelope
    // box is measured from ref_axes_, so its refusal latch resets here.
    returning_home_ = false;
    des_on_flip_ = false;
    handover_defer_ = false;
    wall_strain_mono_ = 0.0;
    home_no_route_ = false;
    state_.travel_returning = false;
    state_.travel_reason.clear();
}

uint32_t Tracker::ref_count_ra() const { return ref_count_ra_; }
uint32_t Tracker::ref_count_de() const { return ref_count_de_; }

uint32_t Tracker::live_count_ra()
{
    std::lock_guard<std::mutex> lk(mu_);
    return last_raw_ra_;
}
uint32_t Tracker::live_count_de()
{
    std::lock_guard<std::mutex> lk(mu_);
    return last_raw_de_;
}

bool Tracker::board_reset_latched()
{
    std::lock_guard<std::mutex> lk(mu_);
    return board_reset_;
}

// Physical ceiling for one raw-counter jump between consecutive :j reads.
// Counters follow step ISSUANCE, which is bounded by the axis rate cap
// (600x sidereal = 2.5 deg/s) and reads run at >= 10 Hz; even a multi-second
// serial stall at full speed moves a few degrees. Anything past 30 deg in a
// single read is mechanically impossible: it is a counter reset (the board's
// power datum 0x800000 after a power cycle, or corruption). The bench
// 2026-11-20: a mount power cycle during an idle session slipped through
// everything (halt ticks never run the skip detector), the exit-save then
// re-anchored the fresh counters against the stale anchor, and the next
// daemon "resumed" az 39.3 while the tube sat at home 90 — 159 deg of lie.
constexpr double kCounterJumpDeg = 30.0;

AxisAngles Tracker::counters_to_axes(uint32_t c_ra, uint32_t c_de)
{
    if (!board_reset_)
    {
        // The one raw-counter funnel: every consumer reads axes through here,
        // so the integrity check lives here too. Detection only (flag); the
        // stop+latch runs in step_impl where board I/O is single-owner —
        // counters_to_axes also serves the rotctld `p` thread, which must
        // never issue wire traffic from a position read.
        const double jpr = 360.0 / board_.axis_info(AxisRA).steps_360;
        const double jpd = 360.0 / board_.axis_info(AxisDE).steps_360;
        const double jump = std::max(std::fabs(wrap24(c_ra - last_raw_ra_) * jpr),
                                     std::fabs(wrap24(c_de - last_raw_de_) * jpd));
        if (jump > kCounterJumpDeg)
            board_reset_ = true; // step_impl halts on this or the next tick
    }
    accum_ra_ += wrap24(c_ra - last_raw_ra_);
    accum_de_ += wrap24(c_de - last_raw_de_);
    last_raw_ra_ = c_ra;
    last_raw_de_ = c_de;

    double dpr = 360.0 / board_.axis_info(AxisRA).steps_360;
    double dpd = 360.0 / board_.axis_info(AxisDE).steps_360;
    AxisAngles a;
    a.ra_deg  = ref_axes_.ra_deg + static_cast<double>(accum_ra_) * dpr * cfg_.ra_sign;
    a.dec_deg = ref_axes_.dec_deg + static_cast<double>(accum_de_) * dpd * cfg_.dec_sign;
    return a;
}

AxisAngles Tracker::current_axes()
{
    std::lock_guard<std::mutex> lk(mu_);
    return counters_to_axes(board_.position(AxisRA), board_.position(AxisDE));
}

HsCoord Tracker::current_pointing(std::chrono::system_clock::time_point utc_now)
{
    std::lock_guard<std::mutex> lk(mu_);
    AxisAngles ax = counters_to_axes(board_.position(AxisRA), board_.position(AxisDE));
    (void)utc_now;
    return axes_to_hs(ax, cfg_.site.lat_deg, cfg_.orientation);
}

// glide reference integration (rate-follower, tracker.h): run at the
// pattern velocity plus a bounded convergence term toward the newest sample
// extrapolated along that velocity. Pattern prediction tapers linearly to
// zero between pattern_pred_max_s and pattern_hard_s of sample age, so a
// client that goes silent leaves the reference GLIDING TO REST rather than
// running away on a stale velocity. Must be called once per tick while
// glide is engaged (advances ref_pos_ by wall time).
AxisAngles Tracker::ref_advance(double now_d, double client_age_s)
{
    constexpr double pred_hard_floor_s = 5.0; // pred_max -> hard: full -> zero gain
    // Client alive (fresh rotctld command — GT2's 2 s rotor poll keeps this
    // true between throttled pushes): extend the pattern extrapolation
    // horizon; see tracker.h:pattern_alive_pred_max_s. Silent client: the
    // original glide-to-rest envelope. The extension is scaled by the
    // pattern rate: with a stationary pattern the filter's velocity is
    // quantization noise and a long horizon amplifies it into wander.
    double pred_cap = cfg_.pattern_pred_max_s;
    if (client_age_s < cfg_.pattern_alive_grace_s)
    {
        const double scale = clampd(
            std::hypot(pattern_vel_.ra_deg, pattern_vel_.dec_deg) /
                cfg_.pattern_alive_full_x_rate,
            0.0, 1.0);
        pred_cap = cfg_.pattern_pred_max_s +
                   scale * (cfg_.pattern_alive_pred_max_s - cfg_.pattern_pred_max_s);
    }
    const double pred_hard_s = std::max(pred_hard_floor_s, pred_cap + 2.5);
    const double age = std::max(0.0, now_d - ref_target_time_);
    const double pred = std::min(age, pred_cap);
    const double taper = clampd((pred_hard_s - age) / (pred_hard_s - pred_cap),
                                0.0, 1.0);
    // Pattern extrapolation with the accel channel (same age^2-coefficient
    // convention as the dive setpoint, see set_target_at): without the 2nd
    // term the reference rate LAGS a ramping axis pattern — the true axis
    // rate ramps along a path even at constant az/el rates (Jacobian
    // varies), and glide then rides on P/I nudges that periodically punch
    // out of the HIGH-band deadband, paying a :K coast every ~2 s
    // (1 Hz test trace 2026-11-17). vel + 2*acc*pred is the derivative of
    // the position law below, so ref_vel feeds the nudge the true pattern
    // rate. Bounded: pred saturates at pattern_pred_max_s.
    // Pattern quantities arrive from the canonical-branch estimator; the
    // reference lives in the branch space it chases (ref_flip_, set at
    // glide engage) — positions were already mapped on entry (ref_target_
    // is stored in ref space); velocities lose the DEC sign under the
    // flip image, RA is shift-invariant.
    const double sgn = ref_flip_ ? -1.0 : 1.0;
    const double ar = pattern_acc2_.ra_deg * taper,
                 ad = sgn * pattern_acc2_.dec_deg * taper;
    const double vr = (pattern_vel_.ra_deg + 2.0 * ar * pred) * taper,
                 vd = sgn * (pattern_vel_.dec_deg + 2.0 * pattern_acc2_.dec_deg * taper
                             * pred) * taper;
    AxisAngles pat{ref_target_.ra_deg + vr * pred + ar * pred * pred,
                   ref_target_.dec_deg + vd * pred + ad * pred * pred};

    const double dt_r = clampd(now_d - ref_pos_time_, 0.0, 1.0);
    ref_pos_time_ = now_d;
    const double conv = cfg_.pattern_conv_k_per_s;
    const double cvmax = cfg_.pattern_conv_max_deg_s;
    const double cr = clampd(conv * (pat.ra_deg - ref_pos_.ra_deg), -cvmax, cvmax);
    const double cd = clampd(conv * (pat.dec_deg - ref_pos_.dec_deg), -cvmax, cvmax);
    const double ra_v = vr + cr, de_v = vd + cd;
    ref_pos_.ra_deg  += ra_v * dt_r;
    ref_pos_.dec_deg += de_v * dt_r;
    // Wall authority (rs44 LOS regression): the glide P term steers the
    // mount toward ref_pos_, so the reference itself must never live
    // outside the mechanical travel box — a wall-riding/deferred-handover
    // setpoint pinned at the wall otherwise drags the mount over the wall
    // through P (cur chased +105 -> +110 with the setpoint clamped, tripping
    // the reactive envelope refusal). clamp_travel is identity when the
    // envelope is disabled and when ref_pos_ is already in box; passing
    // ref_pos_ as the alignment anchor preserves its wrap count.
    ref_pos_ = clamp_travel(ref_pos_, ref_pos_);
#ifdef GLIDE_DEBUG
    printf("      refadv pat-ref=(%.4f,%.4f) ref_t=(%.3f,%.3f) pred=%.2f taper=%.2f\n",
           pat.ra_deg - ref_pos_.ra_deg, pat.dec_deg - ref_pos_.dec_deg,
           ref_target_.ra_deg, ref_target_.dec_deg, pred, taper);
#endif
    return AxisAngles{ra_v, de_v};
}

// ---------------- mechanical travel envelope ----------------

bool Tracker::travel_in_box(const AxisAngles& ax) const
{
    auto inside = [&](double v, double ref, double lim) {
        if (lim <= 0.0) return true;                 // axis unlimited
        return std::fabs(fold180(v - ref)) <= std::min(lim, 180.0);
    };
    return inside(ax.ra_deg,  ref_axes_.ra_deg,  cfg_.safety.max_ra_travel_deg)
        && inside(ax.dec_deg, ref_axes_.dec_deg, cfg_.safety.max_dec_travel_deg);
}

std::pair<AxisAngles, bool> Tracker::select_branch(const AxisAngles& can,
                                                   const AxisAngles& cur,
                                                   bool cur_on_flip) const
{
    // (a+180, 180-d) points the tube identically on the level geometry
    // (axes_to_hs round-trip, verified 2026-11-19): the flip branch is a
    // second legal route to the same sky pointing, mechanically ~180 deg
    // away in both axes. Pattern velocities are canonical-space
    // derivatives, so the caller extrapolates BEFORE the flip: dec flips
    // its rate sign automatically here.
    //
    // cur_on_flip is the branch the mount RIDES (setpoint branch of the
    // previous tick). The box test is NOT invariant under the flip (dec
    // maps 180-d), so near the RA wall the two branches' box memberships
    // scrape every tick at low el — hence the handover margin below.
    AxisAngles c = can;
    AxisAngles f{can.ra_deg + 180.0, 180.0 - can.dec_deg};
    const auto align = [&](AxisAngles& a) {
        a.ra_deg  += 360.0 * std::round((cur.ra_deg  - a.ra_deg)  / 360.0);
        a.dec_deg += 360.0 * std::round((cur.dec_deg - a.dec_deg) / 360.0);
    };
    align(c);
    align(f);
    if (cfg_.safety.max_ra_travel_deg <= 0.0 &&
        cfg_.safety.max_dec_travel_deg <= 0.0)
        return {c, false};               // envelope disabled: classic canonical

    const auto branch_box = [&](bool flip, double grace) {
        const AxisAngles& a = flip ? f : c;
        auto inside = [&](double v, double ref, double lim) {
            if (lim <= 0.0) return true;                    // axis unlimited
            return std::fabs(fold180(v - ref)) <= std::min(lim + grace, 180.0);
        };
        if (!inside(a.ra_deg,  ref_axes_.ra_deg,  cfg_.safety.max_ra_travel_deg) ||
            !inside(a.dec_deg, ref_axes_.dec_deg, cfg_.safety.max_dec_travel_deg))
            return false;
        // Cord window (wrap law clause 2, operator spec 2026-11-21): a
        // branch image whose LANDING DEC offset exceeds the window cannot
        // be chosen — the landing offset is the cur-aligned magnitude the
        // cord would carry there ("refuse or route short past +-270").
        // Exclusion test = beyond window AND DEEPER THAN WHERE THE CORD
        // SITS NOW (cur ledger). That ordering is load-bearing: the branch
        // a tracking mount RIDES lands at the current ledger by
        // construction, and any target whose route UNWINDS (landing
        // shallower than cur — the along-wall slide at −500, every route
        // out of a legacy deep state) is legal no matter how wound the
        // cord is now (a sibling comparison was tried and rejected: land
        // magnitudes of the two branches agree within ~4 deg near the
        // el-0 corridor, so the pair-tie kicked the RIDDEN branch out and
        // force-fed the ~190 deg handover this law exists to refuse).
        // What stays excluded is the dig: the ISS el-0 ghost at −574.6
        // whose only fold-legal landing was −685 (deeper AND past the
        // window) while the flip image sat across the RA meridian — both
        // branches fail, the envelope refuses, and clause 3 lands home
        // cord-FREE: refusal self-heals, it never strands.
        // Grace rides on the exclusion side (no scrape dither at the edge).
        if (false && cfg_.wrap_window_deg > 0.0)
        {
            const double land    = std::fabs(a.dec_deg - ref_axes_.dec_deg);
            const double cur_led = std::fabs(cur.dec_deg - ref_axes_.dec_deg);
            if (land > cfg_.wrap_window_deg + grace &&
                land > cur_led + grace)
                return false;
        }
        return true;
    };
    const auto branch_res = [&](bool flip) {
        return flip ? std::pair{f, true} : std::pair{c, false};
    };
    const auto branch_dist = [&](bool flip) {
        const AxisAngles& a = flip ? f : c;
        return std::hypot(a.ra_deg - cur.ra_deg, a.dec_deg - cur.dec_deg);
    };
    const auto branch_wrap = [&](bool flip) {
        const AxisAngles& a = flip ? f : c;
        return std::fabs(a.dec_deg - ref_axes_.dec_deg);
    };

    // Membership hysteresis (THE rs44 LOS fix): the branch the mount RIDES
    // keeps its box test with the handover grace added to the limits. The
    // live failure at el 1.3 deg lost the flip branch's box by 0.3 STEPS,
    // which teleported the setpoint ~170 deg to the canonical solution —
    // and back the next tick. A scrape must never flip the world over.
    const double grace = cfg_.branch_handover_margin_deg;
    const bool cur_grace = branch_box(cur_on_flip, grace);
    const bool oth_in    = branch_box(!cur_on_flip, 0.0);
    if (!cur_grace && !oth_in && !branch_box(cur_on_flip, 0.0))
    {
        // Neither branch can legally point here: mechanically unreachable.
        // Return the CANONICAL solution — the envelope gate refuses from
        // it and NEVER chases this result into motion.
        return {c, false};
    }
    if (cur_grace)
    {
        // The ridden branch is legal (within grace of the strict box):
        // ride it unless the other branch is clearly nearer — a genuine
        // route re-evaluation where the mount is much closer to the other
        // solution (the ride there is a normal dive, hand_dist small).
        // The margin kills wobble handovers between near-equal routes.
        //
        // Wrap-preference tiebreak FIRST (pre-aim "go the other way" rule,
        // operator spec 2026-11-21): both of tonight's cord windings began
        // at the pre-aim branch pick choosing a slightly nearer but
        // wrap-heavy side (AO-73: flip 104 deg with |DEC| ~150 chosen over
        // ~30). When the other branch saves a MAJOR share of cord and its
        // travel is only slightly longer, take it — a one-time handover
        // event buys a whole pass of cord health.
        const double d_cur = branch_dist(cur_on_flip);
        const double d_oth = branch_dist(!cur_on_flip);
        if (cfg_.wrap_tiebreak_margin_deg > 0.0 && oth_in &&
            branch_wrap(!cur_on_flip) + cfg_.wrap_tiebreak_margin_deg <
                branch_wrap(cur_on_flip) &&
            d_oth < d_cur + cfg_.wrap_tiebreak_extra_deg)
            return branch_res(!cur_on_flip);
        if (oth_in && d_oth + grace < d_cur)
            return branch_res(!cur_on_flip);
        return branch_res(cur_on_flip);
    }
    // Ridden branch illegal beyond the grace band and the other is legal:
    // FORCED handover (the wall outranks continuity, unchanged policy).
    // The far-branch rule at the call site turns the ~180 deg flight into
    // a deliberate dive with fresh estimator/glide state, never a jump.
    return branch_res(!cur_on_flip);
}

AxisAngles Tracker::clamp_travel(const AxisAngles& des, const AxisAngles& cur) const
{
    // Belt & braces: project the setpoint into the box (folded-offset
    // space about the datum) so the control law can never be pointed at
    // an illegal pose — the envelope gate above already refuses those,
    // this bounds any residual (routing waypoint near a wall, resume
    // tick with the 5 deg margin still holding a slightly-out des).
    auto cl = [&](double v, double ref, double lim) {
        if (lim <= 0.0) return v;
        return ref + clampd(fold180(v - ref), -std::min(lim, 180.0),
                            std::min(lim, 180.0));
    };
    AxisAngles r{cl(des.ra_deg,  ref_axes_.ra_deg,  cfg_.safety.max_ra_travel_deg),
                 cl(des.dec_deg, ref_axes_.dec_deg, cfg_.safety.max_dec_travel_deg)};
    r.ra_deg  += 360.0 * std::round((cur.ra_deg  - r.ra_deg)  / 360.0);
    r.dec_deg += 360.0 * std::round((cur.dec_deg - r.dec_deg) / 360.0);
    return r;
}

AxisAngles Tracker::home_cordfree_des(const AxisAngles& cur) const
{
    // Cord-free datum setpoint (operator spec 2026-11-21, SO-50 demo).
    // Every ±360k DEC image of the datum POINTS the same, but only the
    // offset-EXACTLY-0 image leaves the cord FREE: |unfolded DEC| is the
    // cord ledger, and the re-home first flight's nearest-image landing
    // dug 111 deg DEEPER (parked at wrap -360) before this law existed.
    // DEC is therefore never re-aligned here; RA keeps the wrap-aligned
    // short path (its cord question is the cumulative budget's job, and
    // RA images differing by a turn would fight the datum's az).
    AxisAngles d = ref_axes_;
    d.ra_deg += 360.0 * std::round((cur.ra_deg - d.ra_deg) / 360.0);
    return d;
}

// ---------------- control tick ----------------

// Wrapper: takes the lock once for the tick and appends the --debug autopsy
// row AFTER the control body, so EVERY exit path of step_impl (early halts,
// safety returns, the BoardError catch) still produces exactly one row.
void Tracker::step(std::chrono::system_clock::time_point utc_now,
                   std::chrono::steady_clock::time_point mono_now)
{
    std::lock_guard<std::mutex> lk(mu_);
    const double dt =
        std::chrono::duration<double>(mono_now - last_step_).count();
    step_impl(utc_now, mono_now);
    if (dbg_sink_ && debug_ && calibrated_ && dt > 0.0)
        emit_debug_tick(mono_now, dt);
}

void Tracker::set_debug(bool on)
{
    debug_ = on;
    board_.set_trace(on); // wire trace follows the tick trace (and is
    if (!on) board_.take_trace(); // never left to accumulate across a toggle)
}

void Tracker::step_impl(std::chrono::system_clock::time_point utc_now,
                        std::chrono::steady_clock::time_point mono_now)
{
    if (!calibrated_)
        return;

    double dt = std::chrono::duration<double>(mono_now - last_step_).count();
    last_step_ = mono_now;
    (void)utc_now; // pointing input is az/el; UTC enters when TLE propagation is added
    // --debug capture: poison the tick-derived fields so a row emitted from
    // an early return can never show the PREVIOUS tick's setpoint as if it
    // were this tick's (NaN survives to the printed row).
    dbg_des_ = AxisAngles{NAN, NAN};
    dbg_err_ = NAN;
    dbg_x_ra_ = dbg_x_de_ = NAN;
    if (dt <= 0)
        return;
    // Transport liveness by ARRIVAL (see RotctldHandler::on_client_rx):
    // a board stop/restart waiting for the deceleration tail holds the
    // tracker lock for seconds; commands that arrived meanwhile kept the
    // client alive on the wire but were unhandled, and the handling-stamp
    // watchdog false-tripped at a pushing client (live sweeps 2-4).
    if (client_rx_.exchange(false, std::memory_order_relaxed))
        safety_.note_client_command(client_stamp());
    const double now_d =
        std::chrono::duration<double>(mono_now.time_since_epoch()).count();

    auto stop_quiet = [&]() {
        try { board_.stop_all(false); } catch (...) {}
        any_axis_running_ = false;
        sent_ra_x_ = sent_de_x_ = 0;
        have_prev_cur_ = false; // fresh motion-start basis after any stop
        dem_seeded_ = false;    // demand ramp reseeds fresh after any stop
        skip_ra_ = skip_de_ = skip_exp_ra_ = skip_exp_de_ = 0;
        state_.rate_ra_x = state_.rate_dec_x = 0;
    };

    try
    {
        AxisAngles cur = counters_to_axes(board_.position(AxisRA), board_.position(AxisDE));
        safety_.note_board_ok(mono_now);
        state_.current = axes_to_hs(cur, cfg_.site.lat_deg, cfg_.orientation);
        state_.floor_guard = false; // recomputed below; early returns stay false
        dbg_cur_ = cur;

        // Counter jump (board power-cycled mid-session): the pointing anchor
        // is now a lie, so every setpoint this session would compute is
        // garbage — kill motion and latch until the operator re-homes and
        // restarts fresh. STICKY across R on purpose: clearing a skip latch
        // means "continue from the trusted pose", and there is no trusted
        // pose any more (calibrate()/restore() are the only re-arms).
        if (board_reset_)
        {
            stop_quiet();
            safety_.trip("board counter jumped >30 deg in one read: board was "
                         "reset mid-session; re-home the mount and restart "
                         "with --no-resume");
            state_.halted = true;
            state_.halt_reason = safety_.latch_reason();
            return;
        }

        // Measured axis-rate estimate: EMA of the :j counter derivative
        // (deg/s, axis space). Raw 10 Hz deltas carry a ~3x-sidereal rate
        // quantum; the EMA (tau ~0.3 s, ~3+ samples of averaging) reduces it
        // to ~1x. Consumed by the --debug tick trace (`vm=`): the commanded-
        // vs-measured rate gap is the direct evidence of :K coasts, restart
        // starvation and skip-relevant deficits on live metal. (A velocity-
        // feedback DAMPING term using this estimate was tried and rejected
        // 2026-11-19: negative on replay; the shipped fix is demand_lpf_tau_s.)
        // Updated on EVERY tick (even non-glide) so the estimate is warm at
        // glide engagement and after any stop.
        if (cfg_.vel_meas_tau_s > 0.0)
        {
            if (!vel_seeded_)
            {
                vel_prev_ = cur;
                vel_prev_time_ = now_d;
                vel_est_ = {};
                vel_seeded_ = true;
            }
            else if (double vdt = now_d - vel_prev_time_; vdt > 0.0)
            {
                const double a = std::min(1.0, vdt / cfg_.vel_meas_tau_s);
                vel_est_.ra_deg  += ((cur.ra_deg  - vel_prev_.ra_deg)  / vdt - vel_est_.ra_deg)  * a;
                vel_est_.dec_deg += ((cur.dec_deg - vel_prev_.dec_deg) / vdt - vel_est_.dec_deg) * a;
                vel_prev_ = cur;
                vel_prev_time_ = now_d;
            }
        }
        else
            vel_seeded_ = false;

        // --- rotctld target staleness: hold (stop), never latch -----------
        double target_age = std::chrono::duration<double>(mono_now - target_.t).count();
        dbg_target_age_ = target_age;
        if (!target_.valid || target_age > cfg_.target_stale_s)
        {
            // Re-home after pass complete (tracker.h:rehome_idle_s): the
            // pass ended (last-client-drop, or a transport gap past stale)
            // and no new push arrived within rehome_idle_s. Synthesize the
            // datum as a SELF-PUSHED target and fall through to the normal
            // tracking path: dive routing, the travel envelope, the floor
            // guard and the skip detector all still apply to the homeward
            // flight, and the cumulative rotation budget self-centers
            // (accum is net rotation FROM the datum). Daemon-owned motion:
            // the transport watchdog (a client-liveness proxy) is fed by
            // the per-tick self-push below; a client-less bench is exactly
            // when the cord must still be unwound.
            bool rehome_go = false;
            // rehome_armed_: el-gate (TrackerConfig::rehome_min_el_deg).
            // Live 2026-11-20: GT2 pre-aim (el 0) stale-halted 60 s before
            // real AOS and the idle clock fired — the daemon unwound a
            // 146 deg flight home in the pre-pass quiet, exactly when the
            // mount should sit ready at the pre-aim. Only a REAL pass
            // (accepted push with el > the gate) arms the homeflight.
            if (cfg_.rehome_idle_s > 0.0 && rehome_armed_ && !rehome_done_ &&
                !safety_.latched())
            {
                if (rehome_idle_mono_ == 0.0)
                    rehome_idle_mono_ = now_d; // start the idle clock
                if (now_d - rehome_idle_mono_ >= cfg_.rehome_idle_s)
                    rehome_go = true;
            }
            if (!rehome_go)
            {
                rehome_active_ = false; // client gone mid-flight-home:
                if (any_axis_running_)  //   cancel our own maneuver
                    stop_quiet();
                state_.tracking = false;
                state_.halted = true;
                state_.halt_reason = "no target / stale";
                have_prev_desired_ = false;
                return;
            }
            const HsCoord home =
                axes_to_hs(ref_axes_, cfg_.site.lat_deg, cfg_.orientation);
            target_.az   = home.az_deg;
            target_.el   = home.el_deg;
            target_.t    = mono_now;
            target_.valid = true;
            rehome_active_ = true;
            // Flip-pose homing (live 2026-11-20 rewind incident): if the
            // mount RIDES the flip branch, the datum must be flown as the
            // direct wrap-aligned axis line, NOT the canonical hs route
            // (see Tracker::home_no_route_). Sampled BEFORE this tick's
            // select_branch overwrites the ridden-branch flag. Also direct
            // when the CORD needs unwinding (|wrap| > 180): the unwind
            // line lives on the high-DEC cone, never in the low-el corridor
            // routing exists to dodge, and routing legs would realign DEC
            // to a wound datum image and re-wrap the cord on approach.
            home_no_route_ = des_on_flip_ ||
                             (cfg_.wrap_home_unwind &&
                              (std::fabs(cur.dec_deg - ref_axes_.dec_deg) > 180.0 ||
                               (cfg_.cord_unwind_route &&
                                ledger_upper_half(cur.dec_deg - ref_axes_.dec_deg))));
            if (home_no_route_ && logfn_)
                logfn_(des_on_flip_
                           ? std::string("re-home: riding flip pose — rewinding to datum "
                                         "direct (routing skipped)")
                           : std::string("re-home: cord unwind — direct rewind to cord-free "
                                         "datum (routing skipped)"));
            target_age  = 0.0;
            dbg_target_age_ = 0.0;
            // The previous pass's pattern velocity must NOT steer the
            // homeward glide (its extrapolation rides at sample age, but
            // its reference-advance term does not): zero the estimator,
            // restart it on the next real push. Datum is the canonical
            // pose; drop any flip-branch reference space, glide anchors
            // fresh at engagement, and the dive/glide demand filter and
            // the integral (wound with pass velocity) reset to datum-
            // settling state (the I-unwind bleed would otherwise have to
            // drain the whole pass's worth before authority arrives).
            have_vel_       = false;
            pat_ab_ready_   = false;
            pattern_vel_    = {};
            pattern_acc2_   = {};
            have_prev_desired_ = false; // no vff spike on the first home tick
            glide_active_   = false;
            state_.glide    = false;
            ref_flip_       = false;
            nudge_err_      = {};
            dem_seeded_     = false;
            have_ref_ = have_ref_target_ = false;
            safety_.note_client_command(mono_now);
        }
        else if (rehome_active_)
        {
            // datum self-push keeps itself fresh (and feeds the transport
            // watchdog: this is deliberate daemon-owned motion, see above)
            target_.t    = mono_now;
            target_age   = 0.0;
            dbg_target_age_ = 0.0;
            safety_.note_client_command(mono_now);
        }
        // --- operational track gate (pass elevation policy) -----------------
        // Command motion only at/above track_stop_el: arming at −5° rising
        // lets the board finish its 15–25 s acceleration ramp before AOS; a
        // setting pass stops at the same level (radio LOS). The target stays
        // valid — the next pass re-arms automatically. The mechanical floor
        // is checked independently in SafetyMonitor, below.
        if (target_.el < cfg_.safety.track_stop_el)
        {
            if (any_axis_running_)
                stop_quiet();
            state_.tracking = false;
            state_.halted = true;
            state_.halt_reason = "target below track stop elevation " +
                                 std::to_string((int)cfg_.safety.track_stop_el) + " deg";
            have_prev_desired_ = false;
            return;
        }
        state_.tracking = true;
        state_.halted = false;
        state_.halt_reason.clear();

        // --- desired continuous axis angles for the current sky target ----
        // Base = the LSQ-filtered push position (axis space, quantization
        // noise removed; see set_target_at) when the estimator has a fit,
        // else the raw pushed az/el. Either way it sits at the push instant,
        // so the same velocity glide applies to the setpoint between pushes.
        AxisAngles can = hs_to_axes(target_.az, target_.el,
                                    cfg_.site.lat_deg, cfg_.orientation);
        // Between rotctld updates (~1 Hz from GPredict) the setpoint glides
        // on the estimated target velocity, so the 10 Hz loop tracks a
        // smooth line instead of chasing a staircase.
        if (have_vel_)
        {
            double age = std::min(target_age, 2.5);
            can.ra_deg  += pattern_vel_.ra_deg  * age + pattern_acc2_.ra_deg  * age * age;
            can.dec_deg += pattern_vel_.dec_deg * age + pattern_acc2_.dec_deg * age * age;
        }
        // Kinematic branch: (a,d) and the over-the-top flip (a+180, 180-d)
        // point the tube IDENTICALLY on this level geometry (verified
        // numerically against axes_to_hs 2026-11-19), so a target whose
        // canonical solution leaves the mechanical travel envelope may
        // still be reachable via the flip branch inside it (near-north
        // low-el strip; see the envelope section below). Handover requires
        // the new branch to be BOX-FORCED or nearer by more than
        // branch_handover_margin_deg — evaluated against the branch the
        // mount RIDES (des_on_flip_ in, out again below). Envelope
        // disabled = classic canonical behavior, unchanged.
        auto br = select_branch(can, cur, des_on_flip_);
        AxisAngles des = br.first;
        bool flip = br.second;
        // the branch-selected TARGET solution (pre-routing, pre homing
        // override): what the envelope gate below grades for reachability.
        // CRITICAL (envelope test caught it): homing ticks must keep
        // grading the REAL target here — overwriting des first would make
        // the resume hysteresis see the datum and resume tracking the
        // unreachable pose the refusal exists to block.
        const AxisAngles des_target = des;
        // Cord health on homeward flights (wrap law 2026-11-21): the datum
        // must be reached on the CORD-FREE image (DEC offset exactly 0),
        // so a return from |wrap| > 180 flies the unwinding direction even
        // though a wound image sits nearer (the SO-50 re-home landed at
        // wrap -360 and the cord left wound one full revolution). Landing
        // on the canonical image makes the flip flag canonical too; the
        // far-handover block below turns the arrival into the fresh-dive
        // narrative it physically is (a wrap-length rewind).
        if ((rehome_active_ || returning_home_) && cfg_.wrap_home_unwind)
        {
            des  = home_cordfree_des(cur);
            flip = false;
            // Azimuth-first cord-unwind (2026-11-22, operator law spec):
            // a datum flight carries RA home FIRST while DEC holds the
            // cord, then unwinds DEC along the datum RA meridian — the
            // operator's free-spin regime, which is also the horizon
            // circle (el ≡ 0, probe against production astro). The old
            // straight axis chord pierces the floor (analysis: min el
            // -17.9 for the live 2026-11-22 latch-pose family) and its
            // floor slide kept RA authority through the slide,
            // overshooting datum by ~50 deg az before re-climbing (live
            // forensics). Leg gate = RA not yet home: holding the cord
            // while RA sweeps is floor-safe for any ledger a legal pose
            // can hold (probe: min el 0.00 across the winding band), and
            // once RA is settled (|err| ≤ 1 deg — inside the HIGH coast
            // so it cannot ping-pong) the meridian descent sinks el by
            // only ~(90-ledger)·err²/2 — negligible. An RA error beyond
            // 1 deg at any point re-pins the set to holding (the descent
            // dip is exactly the RA-error × depth coupling: live twin
            // graze at err 2.1, ledger 51.7 → el −1.33). Ledger band
            // [90, 270] mod 360 stays the routing-skip ENTRY (below it
            // the flight is a trivial rewind; the direct line there was
            // live-validated 2026-11-21 AO-73).
            const double ra_home = des.ra_deg;   // datum RA, wrap-aligned
            if (cfg_.cord_unwind_route &&
                std::fabs(cur.ra_deg - ra_home) > 1.0)
            {
                des.dec_deg = cur.dec_deg;       // leg 1: RA home, cord held
            }
        }

        // Far-branch setpoint (rs44 2026-11-20 LOS "teleport" fix): a
        // branch handover moves the setpoint ~180 axis deg while pointing
        // the SAME sky target — legal (the branch images point
        // identically), but it must never execute as a setpoint JUMP from
        // a warm glide reference. When the new branch is far from where
        // the mount rides, treat it like the re-home self-push: reset the
        // pattern estimator + glide + integral so no stale reference
        // steers the flight, and let the DIVE law close it (P + vff +
        // floor guard + routing as for any far target; a flip setpoint
        // keeps routing skipped — the straight axis line between two
        // in-box flip poses was live-validated 2026-11-20). Near-zero
        // distance: the handover is a routine route re-evaluation, keep
        // the current authority (glide nudges continue).
        if (flip != des_on_flip_)
        {
            des_on_flip_ = flip;
            const double hand_dist =
                std::hypot(des.ra_deg - cur.ra_deg, des.dec_deg - cur.dec_deg);
            if (hand_dist > std::max(3.0 * cfg_.glide_engage_deg, 15.0))
            {
                // Setting/descending satellite (rs44 05:53:00: el 1.0 LOS):
                // do NOT launch a ~170 deg dive to the other branch of a
                // dying pass. KEEP riding the old branch: des becomes its
                // (pointing-identical) solution, and the belt clamp + wall
                // rate guard below pin the setpoint at the travel wall, so
                // the mount slides along the wall until the pass ends. The
                // deferral LATCHES: a handover armed while setting flies
                // only on a clearly CLIMBING pattern (the GT2 ghost at el
                // 0 after LOS zeroes the rate — launching then would dive
                // for nothing); a fresh handover with an unknown or level
                // pattern (AOS engagement, mid-pass fusion crossing on a
                // rising pattern) flies as a fresh dive, which is the
                // normal far-target behavior.
                const bool up =
                    have_ref_target_ && pattern_hs_vel_.el_deg > 1e-3;
                const bool down =
                    have_ref_target_ && pattern_hs_vel_.el_deg < -1e-3;
                // The old branch's image: computed once, used both for the
                // deferred ride below AND the wall-strain authority check
                // (ISS LOS freeze 2026-11-20 — the 11 min x=0 err 128
                // stuck-at-wall autopsy, TrackerConfig::wall_strain_deg).
                AxisAngles old_br{flip ? des.ra_deg - 180.0
                                       : des.ra_deg + 180.0,
                                  180.0 - des.dec_deg};
                old_br.ra_deg  += 360.0 * std::round((cur.ra_deg  - old_br.ra_deg)  / 360.0);
                old_br.dec_deg += 360.0 * std::round((cur.dec_deg - old_br.dec_deg) / 360.0);
                // A deferred ride pins des to the OLD branch's image while
                // the target keeps setting; near el 0 its RA walks ACROSS
                // the +180 fold boundary: clamp_travel maps the beyond-wall
                // RA offset (fold-boundary) to the OTHER side (-115), the
                // cur-align then reads it as +128 of OUTWARD demand the
                // wall rate guard vetoes forever, and DEC — the only free
                // axis on that image — reads a zero-error mirror. The
                // mount "tracks" x=0, err 128 for 11 min: nothing owns
                // the contradiction unless someone grades it. Strain =
                // the clamped deferred setpoint sits wall_strain_deg+
                // from the mount for wall_strain_s. A genuine along-wall
                // slide never trips: its clamped setpoint rides the wall
                // BESIDE the mount (live RS-44 LOS + los-defer pins err
                // ~2 through 4 min of corridor descent).
                bool strain_trip = false;
                if (cfg_.wall_strain_deg > 0.0 && handover_defer_)
                {
                    const AxisAngles sp = clamp_travel(old_br, cur);
                    const double strain_err = std::hypot(
                        sp.ra_deg - cur.ra_deg, sp.dec_deg - cur.dec_deg);
                    if (strain_err > cfg_.wall_strain_deg)
                    {
                        if (wall_strain_mono_ == 0.0)
                            wall_strain_mono_ = now_d;
                        else if (now_d - wall_strain_mono_ > cfg_.wall_strain_s)
                            strain_trip = true;
                    }
                    else
                        wall_strain_mono_ = 0.0;
                }
                else
                    wall_strain_mono_ = 0.0;
                bool defer_now = (handover_defer_ ? !up : down) &&
                                 !strain_trip;
                if (defer_now)
                {
                    des = old_br;
                    des_on_flip_ = !flip;
                    if (!handover_defer_)
                    {
                        handover_defer_ = true;
                        if (logfn_)
                            logfn_("branch handover DEFERRED (pattern setting): "
                                   "riding wall on flip=" +
                                   std::to_string((int)!flip));
                    }
                }
                else if (strain_trip)
                {
                    // The wall cannot hold this target any more. The
                    // forced branch (canonical — what select_branch
                    // already chose) owns the setpoint with fresh dive
                    // authority, EXCEPT when the pattern is still clearly
                    // SETTING: a dive ~120 deg for an el-0 ghost is
                    // exactly what the deferral exists to refuse, so hand
                    // the mount to the datum instead (envelope-refusal
                    // semantics: non-latching, wrap-unwind home law,
                    // resume on any legal push).
                    wall_strain_mono_ = 0.0;
                    const bool was_deferred = handover_defer_;
                    handover_defer_ = false;
                    have_ref_ = have_ref_target_ = glide_active_ =
                        slide_latch_ = false;
                    ref_flip_ = false;
                    nudge_err_ = {};
                    pat_ab_ready_ = false;
                    pattern_acc2_ = {};
                    dem_seeded_ = false;
                    vel_seeded_ = false;
                    (void)was_deferred;
                    if (down)
                    {
                        returning_home_ = true;
                        state_.travel_returning = true;
                        state_.travel_reason =
                            "wall cannot hold target (deferred handover "
                            "setpoint stuck at travel wall)";
                        have_prev_desired_ = false;
                        home_no_route_ =
                            des_on_flip_ ||
                            (cfg_.wrap_home_unwind &&
                             (std::fabs(cur.dec_deg - ref_axes_.dec_deg) > 180.0 ||
                              (cfg_.cord_unwind_route &&
                               ledger_upper_half(cur.dec_deg - ref_axes_.dec_deg))));
                        glide_active_ = false;
                        state_.glide = false;
                        if (logfn_)
                            logfn_("wall strain: deferred setpoint stuck, "
                                   "pattern still setting — returning home");
                    }
                    else if (logfn_)
                        logfn_("wall strain: deferred setpoint stuck — "
                               "releasing handover to the forced branch");
                }
                else
                {
                    if (handover_defer_ && logfn_)
                        logfn_("branch handover: pattern climbing, flying it");
                    handover_defer_ = false;
                    have_ref_ = have_ref_target_ = glide_active_ = slide_latch_ = false;
                    ref_flip_ = false;
                    nudge_err_ = {};
                    pat_ab_ready_ = false;
                    pattern_acc2_ = {};
                    dem_seeded_ = false;
                    vel_seeded_ = false;
                    if (logfn_)
                        logfn_("branch handover: flip=" + std::to_string((int)flip) +
                               ", dive " + std::to_string((int)hand_dist) +
                               " deg to the other branch");
                }
            }
        }
        else
            handover_defer_ = false;

        // --- dive waypoint routing (Phase-4 big-bore catch-up) ------------
        // A straight axis-space line to a far target PROJECTS to a low /
        // below-horizon az/el arc (RS-44 post-mortem: dived to -58 deg),
        // and for this AltAzLevel geometry the RA shaft winds up fast in
        // the low-elevation due-north corridor: the recorded GT2 session
        // accumulated 540 deg of RA rotation on el~0 pre-aim slides alone
        // (live-replay wrap trip 2026-11-17). So when a far (az sweep >=
        // route_az_min_deg) target would pull the straight path below
        // route_transit_el, fly a three-leg profile instead — climb at
        // the current azimuth, sweep at transit elevation, descend at the
        // target azimuth — the way an operator walks a mount across the
        // sky. Stateless: legs are derived from the CURRENT pose every
        // tick, so the profile re-plans smoothly as the mount moves and
        // a mid-route target push just re-routes. The floor guard stays
        // as the reactive backstop for glide and leg transitions.
        // SKIPPED for flip setpoints: the legs project through CANONICAL
        // axis poses (hs_to_axes), which would walk the wrong branch;
        // the direct line to a flip solution runs inside the box by
        // construction and the velocity-projection floor guard —
        // branch-agnostic — still steers the path off the ground.
        // While a CLIMB/SWEEP waypoint steers the setpoint, glide must NOT
        // engage: FO-29 replay trace (2026-11-19) — a stationary pre-aim
        // (az 19.7/el 0) kept the routed waypoint inside the 5 deg engage
        // radius for ~50 s, so the glide reference (pulling toward the
        // zero pattern velocity) and the route (pushing along the legs)
        // fought each other: DEC pinned at the HIGH-gate LOW ceiling, RA
        // demand swinging +122 -> -515x per PI windup flips, 394 stop/-
        // restart edges (mostly 104x band ping-pong). The moment err
        // exceeded the disengage threshold and dive took over, the SAME
        // route converged in ~30 s at a near-constant capped rate with a
        // handful of :K. A waypoint is a dive-mode authority, same
        // contract as the floor-guard slide latch below.
        bool route_active = false;
        // Homeward flights (envelope refusal, or the re-home-after-idle
        // self-push) aim at the DATUM, which sits at the home elevation
        // (0 on the bench) — i.e. IN the in-band low-el Jacobian corridor.
        // A direct dive there limit-cycles: rehome test trace 2026-11-19,
        // ±600x bang-bang between az 93–96 / el 3–6, never landing (the
        // az error rides the DEC lever at ~1/sin el, exactly the FO-29
        // storm family). So homing always routes through the transit
        // altitude: close the az error at el 12, descend only at the
        // datum azimuth. (Also why datum arrival demands LOW-band rates.)
        const bool homing = rehome_active_ || returning_home_;
        HsCoord route_target{target_.az, target_.el};
        if (homing)
            route_target = axes_to_hs(ref_axes_, cfg_.site.lat_deg, cfg_.orientation);
        // NOTE: we used to force des_on_flip_ = false here ("datum is
        // canonical") — that LIE about the ridden branch is what let the
        // canonical hs route plan hijack a flip-pose homing flight (the
        // 2026-11-20 long-way-around incident). The datum IS canonical;
        // arriving there from the flip side is exactly the far-branch
        // HANDOVER event above (fly = fresh dive), and home_no_route_
        // keeps the route planner out of the way meanwhile.
        if (!des_on_flip_ && !home_no_route_ &&
            cfg_.route_az_min_deg > 0.0 &&
            route_target.el_deg < cfg_.route_transit_el)
        {
            HsCoord cur_hs = axes_to_hs(cur, cfg_.site.lat_deg, cfg_.orientation);
            double az_dist = std::fabs(std::fmod(cur_hs.az_deg - route_target.az_deg + 540.0, 360.0) - 180.0);
            // Homeward flights route by default (datum el sits in the
            // low-el Jacobian corridor; see the forced-routing comment
            // above) UNLESS the flight started flip-riding (latch: the
            // legs solve canonical axis poses and would walk the wall).
            bool dirty = homing && !home_no_route_;
            if (!dirty && az_dist >= cfg_.route_az_min_deg)
            {
                // sample the straight axis-space path's az/el profile
                AxisAngles tgt_ax = hs_to_axes(route_target.az_deg, route_target.el_deg,
                                               cfg_.site.lat_deg, cfg_.orientation);
                tgt_ax.ra_deg  += 360.0 * std::round((cur.ra_deg  - tgt_ax.ra_deg) / 360.0);
                tgt_ax.dec_deg += 360.0 * std::round((cur.dec_deg - tgt_ax.dec_deg) / 360.0);
                double min_el = 90.0;
                for (int k = 1; k <= 10; ++k)
                {
                    const double f = k / 10.0;
                    AxisAngles p{cur.ra_deg + (tgt_ax.ra_deg - cur.ra_deg) * f,
                                 cur.dec_deg + (tgt_ax.dec_deg - cur.dec_deg) * f};
                    min_el = std::min(min_el, axes_to_hs(p, cfg_.site.lat_deg,
                                                         cfg_.orientation).el_deg);
                }
                dirty = min_el < cfg_.route_transit_el;
            }
            if (dirty)
            {
                // Leg order: azimuth first, elevation inside it. The climb
                // leg may ONLY fire while the sweep is still outstanding
                // (az_dist > reach_az): a descent through el 8 must never
                // re-arm the climb leg — with homing's forced routing there
                // is no az_dist >= az_min escape hatch, and the old check
                // order ping-ponged el 7.6<->8.1 at the climb-complete
                // threshold (rehome trace 2026-11-19: des alternated
                // datum/(cur_az, 12) every ~10 s, ±600x RA).
                HsCoord sp = cur_hs;
                if (az_dist > cfg_.route_reach_az_deg)
                {
                    if (cur_hs.el_deg < cfg_.route_transit_el - cfg_.route_reach_el_deg)
                    { sp.el_deg = cfg_.route_transit_el; route_active = true; } // leg 1: climb here
                    else
                    { sp.az_deg = route_target.az_deg; sp.el_deg = cfg_.route_transit_el; route_active = true; } // leg 2: sweep
                }
                else
                    { sp.az_deg = route_target.az_deg; sp.el_deg = route_target.el_deg; } // leg 3: descend
                des = hs_to_axes(sp.az_deg, sp.el_deg, cfg_.site.lat_deg, cfg_.orientation);
                des.ra_deg  += 360.0 * std::round((cur.ra_deg - des.ra_deg) / 360.0);
                des.dec_deg += 360.0 * std::round((cur.dec_deg - des.dec_deg) / 360.0);
                // Cord-free home flight (wrap law clause 3): the waypoint's
                // DEC image is aligned to CUR, which re-wraps the ledger to
                // the wound image nearest the mount (trace 2026-11-21: the
                // (90,0) datum waypoint re-solved to DEC −720 on a −574
                // pose and the flight landed cord-wound two turns — the
                // e9dfc49 hijack class, wrap-image variant). Overriding the
                // routed DEC to the ledger-0 image is pointing-identical
                // (360-periodic) at every waypoint along the legs, keeps
                // the anti-bang-bang RA leg plan, and drives the DEC
                // monotonically to unwound. Gated at |wrap| > 180: that is
                // exactly when the cur-aligned image differs from the
                // cord-free one; below it the pin is a no-op in effect but
                // NOT harmless — DEC=0 pins the plan in the el-0 corridor
                // the climb leg exists to leave (the reversal-guard
                // fixture's datum flight limit-cycled when pinned at wrap
                // ~0, :K at phase-2 tick 1).
                if (homing && cfg_.wrap_home_unwind &&
                    std::fabs(cur.dec_deg - ref_axes_.dec_deg) > 180.0)
                    des.dec_deg = ref_axes_.dec_deg;
            }
        }

        // --- safety gates --------------------------------------------------
        if (auto d = safety_.check_pointing(target_.el); d.result != GuardResult::Ok)
        {
            if (any_axis_running_) stop_quiet();
            state_.halted = true;
            state_.halt_reason = d.reason;
            if (d.result == GuardResult::Latched)
                safety_.trip(d.reason);
            have_prev_desired_ = false;
            return;
        }
        if (auto d = safety_.check_cumulative(cur); d.result != GuardResult::Ok)
        {
            stop_quiet();
            state_.halted = true;
            state_.halt_reason = d.reason;
            safety_.trip(d.reason);
            return;
        }
        if (auto d = safety_.check_watchdog(mono_now, any_axis_running_);
            d.result == GuardResult::Latched)
        {
            // Client-absent watchdog: stop and HOLD — deliberately NOT
            // safety_.trip() (operator latch). Unlike real faults (skip,
            // serial failure), the recovery signal here is exactly what a
            // returning client produces: a fresh command re-arms the
            // monitor (note_client_command), so GT2's disconnect-then-
            // reconnect self-heals. Tripping it deadlocked every later
            // pass of the recorded 2026-11-16 evening session after one
            // 6 s client gap (replay: 6 h stuck at "watchdog: no rotctld
            // command for 5s" while fresh pushes flowed). The serial-
            // stale branch IS a fault (a wedged board thread needs a
            // human) and does latch.
            stop_quiet();
            if (!safety_.client_watchdog_active())
                safety_.trip(d.reason);
            state_.halted = true;
            state_.halt_reason = d.reason;
            have_prev_desired_ = false;
            return;
        }
        if (safety_.client_watchdog_active())
        {
            // Post-trip hold: check_watchdog fires once and the target can
            // stay "valid" for 30 s, so without this branch the normal path
            // would resume motion at the old target and the watchdog would
            // never re-arm (its flag is one-shot until a client command).
            if (any_axis_running_)
                stop_quiet();
            state_.tracking = false;
            state_.halted = true;
            state_.halt_reason = "watchdog hold: awaiting rotctld command";
            have_prev_desired_ = false;
            return;
        }

        // axis skip: we commanded motion but the board is not delivering it
        // (axis jammed so hard the board stopped issuing, motor-driver
        // failure, board confusion). The :j counters count ISSUED steps, so
        // outside the slow-ramp band they track the commanded rate within
        // ~1% quantization; a real jam shows up as counter deficit (inside
        // the band the board loses timer ticks — see skip_supply_slow_band
        // below). Physical
        // missed steps — phase loss, jumps a gear — are invisible to the
        // counters on real hardware and cannot be guarded here. Ramp caveat
        // (live Sirius 2026-11-16): right after :J the board ramps its issue
        // rate, so counters lag at first; skip_ramp_grace_s scales the
        // expectation through that window (a +600x mid-acquisition restart
        // false-tripped with grace 0). A pure target error is NOT a skip
        // (acquisition legitimately starts far).
        if (have_prev_cur_)
        {
            auto update_skip = [&](double x_sent, double sign, double actual_deg,
                                   double& accum, double& accum_exp,
                                   std::chrono::steady_clock::time_point ramp_mono,
                                   double grace_s, double blind_s, int band,
                                   const char* name) -> std::string {
                // Blind can never reach the grace end: with blind >= grace
                // the profile below degenerates to expected=0 for age<grace
                // then FULL — while issuance stays starved until blind_s, a
                // guaranteed false deficit (live double-latch 2026-11-22:
                // restart stamp paired blind 3.0 with the cold grace 2.5;
                // both latches read ramp_age 2.7 inside that crack). The
                // HIGH restart stamp now supplies a proper window
                // (restart_grace_s); this clamp is the config-inversion
                // backstop for ANY pairing, extending the ramp past blind.
                if (blind_s >= grace_s) grace_s = blind_s + 0.5;
                double expected = x_sent * SIDEREAL_DEG_PER_S * sign * dt;
                // Board-accel grace: scale the expected issue window toward
                // 1. Cold :J / band entry accelerate from rest -> linear
                // from t=0 (Phase-1). A stop/restart instead starves
                // issuance entirely for blind_s (:K coast + :J re-accel;
                // ~2 s at HIGH live sweep-6 2026-11-18), then ramps linearly
                // the rest of the stamped per-band window (ramp_*_mono_).
                if (grace_s > 0.0)
                {
                    double age = std::chrono::duration<double>(mono_now - ramp_mono).count();
                    if (age < grace_s)
                    {
                        const double ramp = std::max(0.5, grace_s - blind_s);
                        expected *=
                            age <= blind_s
                                ? 0.0
                                : std::min(1.0, (age - blind_s) / ramp);
                    }
                }
                // Sustained in-band supply floor (keyed off the SENT program,
                // never the stamped band, so a stale stamp cannot leak the
                // discount to other bands): a LOW program in [low_min_x,104x)
                // plateaus at ~0.8 of commanded issuance INDEFINITELY on this
                // vintage board (RS-44 apex latch 2026-11-20: +100x, deficit
                // still growing at ramp_age 48 s). Grace models the entry
                // transient; this models the floor. A genuine jam still
                // under-delivers ~fully against the discounted expectation.
                double skip_sup = 1.0;
                if (cfg_.safety.skip_supply_slow_band > 0.0 &&
                    cfg_.safety.skip_supply_slow_band < 1.0 &&
                    std::fabs(x_sent) >= cfg_.safety.skip_ramp_grace_low_min_x &&
                    std::fabs(x_sent) < 104.0)
                {
                    skip_sup = cfg_.safety.skip_supply_slow_band;
                    expected *= skip_sup;
                }
                if (cfg_.writes_gated || std::fabs(expected) < 1e-4)
                {
                    accum = accum_exp = 0; // no commanded motion reaches the
                                           // board (gated): no evidence, and
                                           // nothing stale to trip on later
                    return std::string();
                }
                if (expected * actual_deg < 0.0)
                {
                    // measured motion opposite the commanded direction (decel
                    // tail, direction flip): skip judgement is meaningless in
                    // a mixed frame — restart it.
                    accum = accum_exp = 0;
                    return std::string();
                }
                accum += expected - actual_deg;
                // Over-delivery (measured beyond commanded) is benign: clamp
                // only when accum crosses to the side OPPOSITE the commanded
                // direction — note the clamp is direction-relative, since for
                // negative rates the whole accumulation is signed negative.
                if (accum * expected < 0.0) accum = 0;
                accum_exp += std::fabs(expected);
                if (std::fabs(accum) > cfg_.safety.tracker_skip_deg &&
                    std::fabs(accum) > 0.2 * accum_exp)
                {
                    // full evidence for post-mortem: the sent rate driving
                    // the expectation, the ramp band/grace state, and how
                    // long this deficit has been accumulating (sweep-4's
                    // DEC latch at +14 s after a resume was unexplainable
                    // from the old terse message alone)
                    double ramp_age = ramp_mono == std::chrono::steady_clock::time_point{}
                                          ? -1.0
                                          : std::chrono::duration<double>(mono_now - ramp_mono).count();
                    char buf[192];
                    snprintf(buf, sizeof(buf),
                             "%s skip: %.2f of %.2f deg of commanded motion "
                             "not delivered (stall or missed steps) [sent %+.0fx band %d "
                             "grace %.0fs blind %.1fs sup %.2f ramp_age %.1fs]",
                             name, accum, accum_exp, x_sent, band, grace_s,
                             blind_s, skip_sup, ramp_age);
                    return std::string(buf);
                }
                return std::string();
            };
            std::string s_ra = update_skip(prev_ra_x_, cfg_.ra_sign,
                                           cur.ra_deg - prev_cur_.ra_deg,
                                           skip_ra_, skip_exp_ra_, ramp_ra_mono_,
                                           ramp_ra_grace_s_, ramp_ra_blind_s_,
                                           ramp_ra_band_, "RA");
            std::string s_de = update_skip(prev_de_x_, cfg_.dec_sign,
                                           cur.dec_deg - prev_cur_.dec_deg,
                                           skip_de_, skip_exp_de_, ramp_de_mono_,
                                           ramp_de_grace_s_, ramp_de_blind_s_,
                                           ramp_de_band_, "DEC");
            if (!s_ra.empty() || !s_de.empty())
            {
                // In a read-only/gated session the mount is commanded nothing,
                // so "commanded motion not delivered" is the expected condition
                // — never treat it as a stall.
                if (!cfg_.writes_gated)
                {
                    stop_quiet();
                    safety_.trip("tracker skip: " + (s_ra.empty() ? s_de : s_ra));
                    state_.halted = true;
                    state_.halt_reason = safety_.latch_reason();
                    return;
                }
            }
        }
        prev_cur_      = cur;
        have_prev_cur_ = true;
        prev_ra_x_     = sent_ra_x_;
        prev_de_x_     = sent_de_x_;

        // --- mechanical travel envelope (stand clearance) -------------------
        // Datum-relative box in folded-offset axis space
        // (SafetyConfig::max_{ra,dec}_travel_deg, default +-105 = 180-15:
        // keep 15 deg of horizon authority on the last degrees of either
        // flank; 0 disables an axis). Live freeze 2026-11-19: a GT2
        // pre-aim (306.8/0) whose axis solution sat ~160-180 deg of RA
        // from the datum drove the tube into the stand; the stalled RA at
        // 600x dragged the 12 V rail and killed the serial link mid-dive.
        // The 540/270 cumulative guard above is a CABLING budget and never
        // sees a local stall like this, and the elevation floor never
        // sees a horizontal-shaft wind-up at el 0.
        // Graded on des_target — the branch-selected TARGET solution,
        // BEFORE dive routing (waypoints are in-box en route to a legal
        // target; grading them would orbit the mount at a transit leg
        // forever instead of refusing). Refusal is predictive (before
        // motion) AND reactive (current pose outside: leave first, no
        // matter what the target says). Response (user decision):
        // "just return home; best to abort" — drive to the datum and
        // hold, ignoring the client target; do NOT depend on the client
        // re-pushing within any timeout. NON-latching by design (unlike
        // the wrap guard): the datum is always inside the box, and any
        // legal push (check_travel Ok with travel_resume_margin_deg
        // hysteresis so a client dithering at the wall cannot flip-flop)
        // resumes normal tracking. A board failure during the return
        // still latches hard via the serial watchdog path.
        const bool envelope_on = cfg_.safety.max_ra_travel_deg > 0.0 ||
                                 cfg_.safety.max_dec_travel_deg > 0.0;
        // Branch images of the target solution, graded as PAIRS (rs44
        // LOS consistency): select_branch may keep riding a setpoint that
        // sits within the handover grace OUTSIDE the strict box (that is
        // what prevents the scrape teleport); the gate must then NOT
        // refuse while the other branch is legal. Refusal means NO branch
        // can point here. check_travel also grades `cur` (reactive leave-
        // first): passing the grace as a NEGATIVE margin widens the limits
        // by the band — wall-riding (setpoint clamped at the wall plus
        // coast overshoot) lives in the band by definition and must not
        // trigger the reactive refusal either.
        const double grace = cfg_.branch_handover_margin_deg;
        const auto flip_image = [](const AxisAngles& a) {
            return AxisAngles{a.ra_deg + 180.0, 180.0 - a.dec_deg};
        };
        // Cur-aligned DEC landing of a branch image = the cord magnitude
        // the mount would carry pointing there (wrap law's ledger for the
        // branch-box tests; the ±360k image nearest the present ledger is
        // the one a continuous route reaches).
        const auto landing_wrap = [&](const AxisAngles& img) {
            const double d =
                img.dec_deg + 360.0 * std::round((cur.dec_deg - img.dec_deg) / 360.0);
            return std::fabs(d - ref_axes_.dec_deg);
        };
        // Wrap exclusion (same WIND-DEEPER rule as branch_box — the two
        // tests must agree or one of them freezes/routes against the
        // other): a branch is cord-excluded only if its cur-aligned
        // landing passes the window AND digs DEEPER than the cord sits
        // now. Unwinding routes are never excluded, so a legacy deep
        // state rides its along-wall slides normally (the wall-strain
        // fixture guards this at ledger -500), while the ISS ghost's dig
        // to -685 at -574.6 excludes its only fold-legal branch and the
        // refusal flies the cord-free home route — never the 11-min
        // silence of the live freeze.
        const double W = cfg_.wrap_window_deg;
        const double cur_led = std::fabs(cur.dec_deg - ref_axes_.dec_deg);
        const double l_can = landing_wrap(des_target);
        const double l_fli = landing_wrap(flip_image(des_target));
        const bool can_wrap_out =
            W > 0.0 && l_can > W + grace && l_can > cur_led + grace;
        const bool flip_wrap_out =
            W > 0.0 && l_fli > W + grace && l_fli > cur_led + grace;
        if (envelope_on)
        {
            if (!returning_home_)
            {
                auto d = safety_.check_travel(cur, des_target, -grace);
                const auto df =
                    safety_.check_travel(cur, flip_image(des_target), -grace);
                const bool can_ok = d.result == GuardResult::Ok && !can_wrap_out;
                const bool flip_ok =
                    df.result == GuardResult::Ok && !flip_wrap_out;
                if (!can_ok && !flip_ok)
                {
                    returning_home_ = true;
                    state_.travel_returning = true;
                    // If wrap excluded a branch whose fold box was legal,
                    // the cord window — not the stand — is what refused.
                    state_.travel_reason =
                        ((can_wrap_out && d.result == GuardResult::Ok) ||
                         (flip_wrap_out && df.result == GuardResult::Ok))
                            ? "Travel envelope (DEC cord window): no "
                              "cord-legal branch here (window +" +
                                  std::to_string((int)cfg_.wrap_window_deg) +
                                  " deg from datum)"
                            : d.reason;
                    have_prev_desired_ = false;  // no vff spike for the
                }                                // homeward first tick
                if (returning_home_)
                {
                    // Re-route event (same contract as the re-home
                    // self-push and the far-branch handover): the old
                    // target's pattern must not steer the homeward dive,
                    // and if we are FLIP-riding, the datum is flown as
                    // the direct rewind, never the canonical hs route
                    // (Tracker::home_no_route_; 2026-11-20 incident).
                    home_no_route_ = des_on_flip_ ||
                                     (cfg_.wrap_home_unwind &&
                                      (std::fabs(cur.dec_deg - ref_axes_.dec_deg) > 180.0 ||
                                       (cfg_.cord_unwind_route &&
                                        ledger_upper_half(cur.dec_deg - ref_axes_.dec_deg))));
                    if (home_no_route_ && logfn_)
                        logfn_("travel refusal: riding flip pose — "
                               "rewinding to datum direct (no routing)");
                    have_vel_ = false;
                    pat_ab_ready_ = false;
                    pattern_vel_ = {};
                    pattern_acc2_ = {};
                    glide_active_ = false;
                    state_.glide = false;
                    ref_flip_ = false;
                    nudge_err_ = {};
                    dem_seeded_ = false;
                    have_ref_ = have_ref_target_ = false;
                }
            }
            else
            {
                // Resume normal tracking once a push is legal AGAIN — per
                // BRANCH: fold box AND cord landing back INSIDE the window
                // (strict, no wind-deeper exemption: resuming is a NEW
                // engagement, and the operator spec's "beyond the window →
                // refuse or route short" means a fresh target must land
                // cord-legal; the unwinding exemption governs RIDING, not
                // re-chasing). Grading the tests on different branches is
                // the trap that made the ISS LOS ghost re-arm the freeze:
                // the ghost's flip landing sits at the ledger (fold-legal,
                // in window) while its RA image is across the meridian,
                // and the ghost's canonical RA is in the box while its
                // landing digs deeper past the window — OR-ed halves
                // resume into a route neither branch can actually fly.
                // (Reaching this else branch already implies
                // returning_home_.)
                const auto branch_ok = [&](const AxisAngles& img) {
                    return safety_.check_travel(cur, img,
                                                cfg_.safety.travel_resume_margin_deg)
                                   .result == GuardResult::Ok &&
                           (W <= 0.0 ||
                            landing_wrap(img) <=
                                W + cfg_.safety.travel_resume_margin_deg);
                };
                if (!rehome_active_ &&
                    (branch_ok(des_target) || branch_ok(flip_image(des_target))))
                {
                    // NOT a resume while the re-home self-push owns the
                    // target (2026-11-22): the datum self-solve is always
                    // branch-OK, so ungated this cleared home_no_route_
                    // one tick after the trigger and handed the leg
                    // flight to the canonical hs router — the (90,12)-
                    // waypoint hijack of the leg law (reproduced live as
                    // the "routing leg re-wound RA" phase of the 2026-11-22
                    // 06:40 return). Arrival is the rehome done-path's job.
                    returning_home_ = false;
                    state_.travel_returning = false;
                    home_no_route_ = false;
                }
            }
            if (returning_home_)
            {
                // des_on_flip_ is NOT forced here anymore: arriving at
                // the canonical datum from a flip pose IS the handover
                // event (see the refusal block above + far-branch rule).
                glide_active_ = false;      // glide engage is gated on
                state_.glide = false;       // !returning_home_ below
                // With wrap_home_unwind (default) the datum override above
                // select_branch ALREADY pinned des to the cord-free image on
                // every homing tick, and must not be re-wrapped here.
                if (!route_active && !cfg_.wrap_home_unwind)
                {                           // homeward setpoint this tick
                    des = ref_axes_;        // datum wins: no routing, no
                    des.ra_deg  += 360.0 * std::round((cur.ra_deg  - des.ra_deg)  / 360.0);
                    des.dec_deg += 360.0 * std::round((cur.dec_deg - des.dec_deg) / 360.0);
                }
            }
            else
                des = clamp_travel(des, cur);  // belt & braces: never
        }                                      // command outside the box

        // --- control law: dive (far) / glide (pattern following) ----------
        // Incoming rotctld updates are point-in-time samples of where the
        // antenna should be. DIVE mode closes a large error point-to-point
        // (P + velocity feedforward; the floor guard + joint cap below bound
        // the path). GLIDE mode follows the pattern like a telescope track:
        // a continuous constant-rate reference (tracker.h, ref_advance) plus
        // bounded nudges keeping the mount on it — sustained error is closed
        // by running slightly FASTER, never by jumping or stop-start cycles
        // (control-law directive 2026-11-16, "run it like a telescope
        // mount"; RS-44 post-mortem data).
        const double err_ra = des.ra_deg - cur.ra_deg;
        const double err_de = des.dec_deg - cur.dec_deg;
        const double err = std::hypot(err_ra, err_de);
        dbg_des_ = des;
        dbg_err_ = err;

        // Re-home arrival: parked at the datum. Stop self-pushing and let
        // the normal stale/halt path stop_quiet and settle; the done-flag
        // keeps the mount parked until the next REAL client push. The LOW-
        // band rate guard pins "settling", not "diving through": the 5°
        // glide radius normally guarantees this (arrival << engage), but a
        // :K coast at HIGH would blow through the whole arrival window.
        if (rehome_active_ &&
            std::fabs(err_ra) < cfg_.rehome_arrive_deg &&
            std::fabs(err_de) < cfg_.rehome_arrive_deg &&
            std::fabs(sent_ra_x_) <= 104.0 && std::fabs(sent_de_x_) <= 104.0)
        {
            rehome_active_ = false;
            rehome_done_   = true;
            rehome_armed_  = false; // one homeflight per real pass
            home_no_route_ = false;
            target_        = Target{};
            rehome_idle_mono_ = 0;
            if (logfn_)
            {
                const double wrap = cur.dec_deg - ref_axes_.dec_deg;
                logfn_("re-home: arrived at datum (DEC cord wrap now " +
                       std::to_string(std::lround(wrap)) + " deg)");
            }
        }

        bool glide_now = glide_active_;
        // While the envelope guard is driving home, the pattern glide must
        // NOT engage: its reference chases the (illegal) client target, not
        // the homeward setpoint. Same reasoning for route_active: while a
        // climb/sweep waypoint is the setpoint, the "error" is the distance
        // to an INTERMEDIATE point, and engaging glide there strands the
        // pattern reference and the route in competing attractors
        // (FO-29 :K storm; see routing block comment above).
        if (!glide_now && err <= cfg_.glide_engage_deg && !slide_latch_ &&
            !returning_home_ && !route_active)
            glide_now = true;
        else if (glide_now && (err > cfg_.glide_disengage_deg || slide_latch_ ||
                               route_active))
            glide_now = false;
        if (glide_now && !glide_active_) {
            // Entering glide: start the reference AT THE MOUNT, aligned into
            // the actual axis space (ref space is redefined here; the stored
            // sample may sit 360°k away in its own unwrapped space, and RA
            // is physically 360°-periodic so the nearest turn is the only
            // sane control target — a stale turn representation made the
            // reference chase a phantom full turn at ±600x, sim trace
            // 2026-11-16). Catch-up to the ≤ engage-threshold error then
            // proceeds at the bounded convergence rate, no dive spikes.
            // The reference lives in the space of the branch being chased:
            // on a flip-branch engagement, bring the stored (canonical)
            // pattern sample into the flip image BEFORE the turn
            // alignment — aligning a canonical sample against a flip-pose
            // cur is a ~180 deg coin flip in both axes and dragged the
            // reference back over the envelope wall (sim trace 2026-11-19:
            // tgt=(-87,80) vs des=(93,100) stall at err~5, mode chatter).
            ref_flip_ = des_on_flip_;
            if (!have_ref_target_)
            {
                // No pattern samples exist (re-home self-push deliberately
                // bypasses the estimator, whose state was zeroed at the
                // trigger): anchor the reference on the setpoint actually
                // being chased. The stored member is STALE from the last
                // client pass in that case, and turn-aligning it pulled the
                // glide ~0.9 deg/s toward the old pass pose while homing
                // (rehome trace 2026-11-19: el/divergence limit cycle).
                ref_target_ = des;
            }
            else if (ref_flip_)
                ref_target_ = flip_map(ref_target_, true, true);
            const double k_ra = std::round((cur.ra_deg - ref_target_.ra_deg) / 360.0);
            const double k_de = std::round((cur.dec_deg - ref_target_.dec_deg) / 360.0);
            ref_target_.ra_deg  += 360.0 * k_ra;
            ref_target_.dec_deg += 360.0 * k_de;
            ref_pos_ = cur;
            ref_pos_time_ = now_d;
            have_ref_ = true;
            nudge_err_ = {};
            nudge_prev_time_ = now_d;
        }
        glide_active_ = glide_now;
        if (!glide_now && ref_flip_)
        {   // disengaged while chasing the flip branch: return the pattern
            // sample to canonical space (flip_map is an involution) so a
            // later flip re-engage maps exactly once.
            ref_target_ = flip_map(ref_target_, true, true);
            ref_flip_ = false;
        }
        state_.glide = glide_now;

        double rate_ra = 0, rate_de = 0;
        AxisAngles vff{};
        if (glide_now) {
            const AxisAngles ref_vel = ref_advance(now_d, safety_.client_age_s(mono_now));
            const AxisAngles& ref_pos = ref_pos_;
            const double dt_n = std::max(1e-3, now_d - nudge_prev_time_);
            nudge_prev_time_ = now_d;
            // P acts on (ref - cur) — keeps the mount ON the reference (the
            // stable inner loop). I acts on (des - cur), the TRUE setpoint
            // error: the reference can sit in a self-consistent lag (conv
            // term exactly balancing its own deficit) that P can never see,
            // and at pattern rates the period quantization (7 us = 88.6x vs
            // 8 us = 77.5x at RA) only carries fine corrections as dither
            // DUTY — which a bounded I on the real error supplies. I on
            // (ref - cur) freezes transient offsets permanently (sim trace
            // 2026-11-16: 0.06 deg axis / 0.23 deg az stuck forever).
            // Anti-windup unwind bleed (live 2026-11-19 + sim repro): after a
            // dive overshoot, nudge_err_ sits wound with the OPPOSITE sign of
            // (des - cur), and its correction must first unwind that stale
            // opposite-sign area at the plain err x dt rate — with
            // nudge_i_per_s2 = 0.04 and nudge_p_per_s = 2.0 the loop's slow
            // pole is Ki/P = tau ~ 50 s, which stranded the live glide settle
            // at 0.27 deg for 30+ s (and the 1 Hz sim twin at 0.02 deg for a
            // minute). When the integral opposes the PRESENT error it is pure
            // windup legacy — it pulls AWAY from the target — so integrate
            // its unwind nudge_i_unwind_gain times faster. While the integral
            // AGREES with the error (its moving-pattern job, supplying the
            // duty the HIGH deadband and period quantization cannot) the
            // gain is untouched, so moving-target behavior is unchanged.
            // Pattern-speed gate: the stranding pathology needs a NEAR-
            // STATIONARY pattern (the live case: settled at el 10.27 with
            // |pattern_vel| ~ 1e-3 deg/s). On a moving pattern the I term is
            // live tracking authority and the frozen-between-push target
            // sawtooth crosses (des - cur) zero every cycle, so draining
            // there eats tracking margin (FO-29 sparse apex tail: rms
            // 0.626 -> 0.706, err>1deg samples 505 -> 830 ungated). Ramp the
            // excess gain down across [gate/5, gate] axis deg/s.
            const double pat_dps =
                std::hypot(pattern_vel_.ra_deg, pattern_vel_.dec_deg);
            const double unwind_scale = cfg_.nudge_i_unwind_gate_deg_s > 0.0
                ? clampd(1.0 - pat_dps / cfg_.nudge_i_unwind_gate_deg_s, 0.0, 1.0)
                : 0.0;
            const double bleed_gain = 1.0 + (cfg_.nudge_i_unwind_gain - 1.0) * unwind_scale;
            const double bleed_ra =
                (nudge_err_.ra_deg * (des.ra_deg - cur.ra_deg) < 0.0)
                    ? bleed_gain : 1.0;
            const double bleed_de =
                (nudge_err_.dec_deg * (des.dec_deg - cur.dec_deg) < 0.0)
                    ? bleed_gain : 1.0;
            nudge_err_.ra_deg  = clampd(nudge_err_.ra_deg  + (des.ra_deg  - cur.ra_deg)  * dt_n * bleed_ra,
                                        -cfg_.nudge_i_errclamp, cfg_.nudge_i_errclamp);
            nudge_err_.dec_deg = clampd(nudge_err_.dec_deg + (des.dec_deg - cur.dec_deg) * dt_n * bleed_de,
                                        -cfg_.nudge_i_errclamp, cfg_.nudge_i_errclamp);
            // Escape-ramp P, HIGH band only (|sent| > 104x): LOW axes
            // dither period quantization for free via in-place :I, so an
            // extra P there buys nothing but wire. Deadbanded HIGH axes
            // can hold a stuck bias that P-on-ref alone never escapes
            // (ref self-consistently follows the pattern), so charge the
            // excess of (des-cur) above a small deadband to give the bias
            // an escape ramp out of the deadband.
            auto desp = [&](double e, bool high) {
                if (!high)
                    return 0.0;
                const double d = cfg_.nudge_des_p_deadband;
                return cfg_.nudge_des_p_per_s *
                       (e > d ? e - d : (e < -d ? e + d : 0.0));
            };
            const bool ra_high = std::fabs(sent_ra_x_) > 104.0;
            const bool de_high = std::fabs(sent_de_x_) > 104.0;
            const double pr = clampd(cfg_.nudge_p_per_s * (ref_pos.ra_deg - cur.ra_deg) +
                                         desp(des.ra_deg - cur.ra_deg, ra_high),
                                     -cfg_.nudge_p_limit, cfg_.nudge_p_limit);
            const double pd = clampd(cfg_.nudge_p_per_s * (ref_pos.dec_deg - cur.dec_deg) +
                                         desp(des.dec_deg - cur.dec_deg, de_high),
                                     -cfg_.nudge_p_limit, cfg_.nudge_p_limit);
            const double ir = clampd(cfg_.nudge_i_per_s2 * nudge_err_.ra_deg,
                                     -cfg_.nudge_i_limit, cfg_.nudge_i_limit);
            const double id = clampd(cfg_.nudge_i_per_s2 * nudge_err_.dec_deg,
                                     -cfg_.nudge_i_limit, cfg_.nudge_i_limit);
            // (Velocity-feedforward damping — subtracting a measured-rate
            // feedback term here — was the first iteration-2 candidate and
            // is REJECTED: replay :K rose monotonically with the gain,
            // 315 -> 467 at kd 0 -> 1.5, error unchanged. The storm is not
            // plant ringing (the ideal sim plant has no delay to ring on;
            // on metal the delay is the board ramp, which the term would
            // actively amplify: it reads the ramp's rate deficit as an
            // error and boosts demand into it). The fix actually shipped is
            // demand-side: TrackerConfig::demand_lpf_tau_s below the
            // dive/glide merge.)
            rate_ra = ref_vel.ra_deg  + pr + ir;
            rate_de = ref_vel.dec_deg + pd + id;
            // HIGH-band actuator-quantum gate (tracker.h): while the
            // FILTERED PATTERN velocity justifies HIGH, or the axis error
            // is larger than the HIGH coast quantum, full authority.
            // Otherwise clamp to the LOW ceiling: near-horizon fine
            // pointing must close its (small) axis error with coast-free
            // LOW speeds, not with :K-per-tightening-loop.
            {
                const double ceil_dps =
                    cfg_.high_gate_max_x * SIDEREAL_DEG_PER_S;
                const double moving_dps =
                    cfg_.high_gate_moving_x * SIDEREAL_DEG_PER_S;
                auto gate = [&](double& r, double pat_v, double e_axis) {
                    const bool justified =
                        std::fabs(pat_v) > moving_dps ||
                        std::fabs(e_axis) > cfg_.high_gate_err_deg;
                    if (!justified && std::fabs(r) > ceil_dps)
                        r = (r > 0 ? 1.0 : -1.0) * ceil_dps;
                };
                gate(rate_ra, pattern_vel_.ra_deg, err_ra);
                gate(rate_de, pattern_vel_.dec_deg, err_de);
            }
#ifdef GLIDE_DEBUG
            printf("[%4.1f] mode=G cur=(%.2f,%.2f) ref=(%.2f,%.2f) rv=(%.3f,%.3f) "
                   "des=(%.2f,%.2f) err=%.2f | vel=(%.4f,%.4f) rt=(%.3f,%.3f) "
                   "nud=(%.3f,%.3f)\n",
                   now_d, cur.ra_deg, cur.dec_deg, ref_pos.ra_deg, ref_pos.dec_deg,
                   ref_vel.ra_deg, ref_vel.dec_deg, des.ra_deg, des.dec_deg, err,
                   pattern_vel_.ra_deg, pattern_vel_.dec_deg, rate_ra, rate_de, pr + ir, pd + id);
            printf("      tgt=(%.3f,%.3f) age=%.2f\n",
                   ref_target_.ra_deg, ref_target_.dec_deg, now_d - ref_target_time_);
#endif
        } else {
            // dive: P + velocity feedforward on the extrapolated setpoint
            if (have_prev_desired_)
            {
                vff.ra_deg  = (des.ra_deg - prev_desired_.ra_deg) / dt;
                vff.dec_deg = (des.dec_deg - prev_desired_.dec_deg) / dt;
            }
            rate_ra = cfg_.control_kp_per_s * err_ra + cfg_.control_kd_per_s * vff.ra_deg;
            rate_de = cfg_.control_kp_per_s * err_de + cfg_.control_kd_per_s * vff.dec_deg;
#ifdef GLIDE_DEBUG
            printf("[%4.1f] mode=D cur=(%.2f,%.2f) des=(%.2f,%.2f) err=%.2f "
                   "rt=(%.2f,%.2f)\n", now_d, cur.ra_deg, cur.dec_deg,
                   des.ra_deg, des.dec_deg, err, rate_ra, rate_de);
#endif
        }
        have_prev_desired_ = true;
        prev_desired_ = des;

        // --- glide demand low-pass (iteration-2 :K-storm fix 2026-11-19) -----
        // GT2 pushes arrive 2 s apart and 0.1 deg quantised; the Jacobian
        // lever near the horizon maps each step to a much larger AXIS step,
        // and glide's ff+conv+P convert it into a demand JUMP (up to ~0.4
        // deg/s, sign-inverting: trace seq 25985-92 FO-29 replay) that the
        // economy layer faithfully executes: :K per push per axis. The
        // pattern itself changes at most pattern_accel-clamped (~0.02
        // deg/s^2), so a demand step far above that is either quantisation
        // noise (reject) or the ref-internal conv transient (decays in
        // ~1 conv-k s anyway — letting it reach the wire gains nothing the
        // reference will not do smoothly a second later). A first-order
        // LPF at tau between the 10 Hz loop and the 2 s push cadence sits
        // in that gap: control transients keep authority, push staircase
        // steps are delayed past the ref's conv decay and die there. NOT a
        // magnitude deadband (Phase-4 proved those freeze sustained glide
        // corrections into permanent offsets — an LPF only delays, it is
        // transparent at constant demand; I-term eats the residual).
        // DIVE bypasses (catch-up authority) while the filter state tracks
        // the dive demand, so glide engagement starts continuous; a mode
        // transition also reseeds (engagement itself is a legitimate demand
        // step: anchor AT THE MOUNT). Floor-guard ticks RESEED: guard
        // projection is safety authority and a 1.2 s coast budget cannot
        // pay filter phase. 0 = off (legacy). Rejected alternatives on the
        // same four datasets: velocity-feedback damping (315 -> 467 :K, the
        // ideal sim plant has no delay to ring) and a slew-rate ratchet
        // (mode-coupled: 146+ dive<->glide thrash at tau-relevant limits).
        if (glide_active_ != dem_was_glide_)
            dem_seeded_ = false; // mode transition: reseed (engage = legit step)
        dem_was_glide_ = glide_active_;
        if (cfg_.demand_lpf_tau_s > 0.0)
        {
            // WOBBLE-REGIME ESCALATION (0a4(h), see TrackerConfig::
            // demand_lpf_regime_tau_s): while the axis's regime flag from
            // the previous tick is latched, run THIS axis's filter at the
            // escalated tau. State continuity makes the tau switch jump-
            // free (only the response speed changes).
            auto tau_for = [&](bool regim) {
                return (cfg_.demand_lpf_regime_tau_s > 0.0 && regim)
                           ? std::max(cfg_.demand_lpf_tau_s,
                                       cfg_.demand_lpf_regime_tau_s)
                           : cfg_.demand_lpf_tau_s;
            };
            auto filt = [&](double& r, double& prev, double tau) {
                if (!dem_seeded_) { prev = r; return; }
                const double a = std::min(1.0, dt / tau);
                prev = r = prev + (r - prev) * a;
            };
            if (glide_active_)
            {
                const bool esc_ra = cfg_.demand_lpf_regime_tau_s > 0.0 && regime_ra_;
                const bool esc_de = cfg_.demand_lpf_regime_tau_s > 0.0 && regime_de_;
                filt(rate_ra, dem_prev_ra_, tau_for(esc_ra));
                filt(rate_de, dem_prev_de_, tau_for(esc_de));
                dem_esc_ra_ = esc_ra;
                dem_esc_de_ = esc_de;
            }
            else
            {
                dem_prev_ra_ = rate_ra; // dive: full authority, state tracks
                dem_prev_de_ = rate_de;
                dem_esc_ra_ = dem_esc_de_ = false;
            }
            dem_seeded_ = true;
        }
        else
            dem_seeded_ = false;

        // --- glide LOW->HIGH promotion hold-off (live LUCA 2026-11-20) -----
        // Chasing the LOW +-104x ceiling cost ~20 :K/min on a NORMAL 30°
        // pass: every transient spike past the ceiling promoted, and a
        // promotion OUT of a running HIGH program is the EXPENSIVE wire
        // direction (:K at full speed: ~0.3 s / coast-speed-scaled block +
        // ~2 s issuance starvation) while the demote back is nearly free
        // (LOW coast ~0.03 s). The demand beyond LOW that survives one
        // hold-off window re-escalates normally (ISS-class patterns sustain
        // in seconds and promote once); spikes (P transients, GT2 push
        // levers, ref-conv) decay under the ceiling wire-free: LOW chases
        // them with free in-place :I. Floor-guard slides keep full
        // authority (safety). NOTE: the HIGH-gate above runs FIRST and its
        // moving-pattern exemption (high_gate_moving_x) already caps the
        // demand at high_gate_max_x before this sees it — tuning the two
        // together is what kills the band-edge relay (replay --mgate
        // A/B, live LUCA/FO-29-1120 datasets).
        if (cfg_.glide_promote_holdoff_s > 0.0 && glide_active_ &&
            !state_.floor_guard)
        {
            const double low_x = cfg_.high_gate_max_x + 4.0; // 104x ceiling
            const double low_dps = low_x * SIDEREAL_DEG_PER_S;
            // No grace-armed exemption here: the LUCA relay lives INSIDE
            // the slow-ramp LOW band's constantly re-armed 25 s grace, so a
            // grace check would disable the hold-off exactly where it is
            // needed. Promotion skip-starvation is already blanked by the
            // stop/restart event's blind stamp (the :K→:J reprogram fires
            // take_stop_restart); AO-123-style cold dives are sent==0
            // first-:J, which this hold-off never touches. Floor-guard
            // slides keep full authority (safety).
            const bool hold_ra =
                std::fabs(rate_ra) > low_dps &&
                std::fabs(sent_ra_x_) <= low_x &&
                sent_ra_x_ != 0.0; // promotions only, never the first :J
            const bool hold_de =
                std::fabs(rate_de) > low_dps &&
                std::fabs(sent_de_x_) <= low_x &&
                sent_de_x_ != 0.0;
            if (!hold_ra && !hold_de)
                promote_stamp_ = 0.0;
            else
            {
                if (promote_stamp_ == 0.0) promote_stamp_ = now_d;
                if (now_d - promote_stamp_ < cfg_.glide_promote_holdoff_s)
                {
                    if (hold_ra && rate_ra != 0.0)
                        rate_ra = std::copysign(low_dps, rate_ra);
                    if (hold_de && rate_de != 0.0)
                        rate_de = std::copysign(low_dps, rate_de);
                }
            }
        }
        else
            promote_stamp_ = 0.0;

        // --- travel-wall rate guard (rs44 LOS wall-slide) -------------------
        // clamp_travel pins the SETPOINT inside the box, but the velocity-
        // space feeds are not automatically zeroed when the demand runs INTO
        // a wall: the deferred-handover wall slide kept a +1.2 deg/s RA
        // pattern feed and walked the mount from the pinned setpoint over
        // the wall (regression 2026-11-20: cur 105 -> 111 deg, tripping the
        // reactive envelope refusal). Enforce the box in RATE space: at a
        // limit, the outward component of the demand is removed and the
        // tangential survives — the same velocity-projection law as the
        // elevation floor guard below, applied in axis space. Riding the
        // handover grace band (membership hysteresis) stays legal: this
        // guard only removes further-outward motion AT the strict wall,
        // and the envelope refusal's widened cur check gives the coast
        // room inside the band.
        {
            auto wall = [&](double& r, double v, double ref, double lim) {
                if (lim <= 0.0) return;               // axis unlimited
                const double off = fold180(v - ref);
                if (std::fabs(off) >= std::min(lim, 180.0) - 0.05 &&
                    r * off > 0.0)
                    r = 0.0;
            };
            wall(rate_ra, cur.ra_deg, ref_axes_.ra_deg,
                 cfg_.safety.max_ra_travel_deg);
            wall(rate_de, cur.dec_deg, ref_axes_.dec_deg,
                 cfg_.safety.max_dec_travel_deg);
        }

        // --- trajectory floor guard (final authority) --------------------------
        // The floor binds the commanded VELOCITY, not the target: incoming
        // az/el telemetry are point-in-time samples (GT2 pushes as rarely as
        // once per 1°/2 s), and far-target convergence is straight in axis
        // space but arcs through the ground in az/el (live RS-44 pass
        // 2026-11-16: an el-0 target was chased down to −58°). Instead of
        // stopping or scaling (which triggers the high-speed :K coast and
        // sinks the tube further), project the demanded axis-velocity onto
        // the legal elevation-rate half-plane: the sinking component is
        // removed, the TANGENTIAL component survives, so the mount keeps
        // moving and SLIDES ALONG the floor toward the target. The allowed
        // descent rate tapers with distance to the floor so that the board's
        // :K deceleration coast (live Sirius: ~1.0 s at high speed,
        // speed-scaled) cannot carry the tube below it:
        //     el + el_dot * floor_coast_s >= floor_el.
        // Glide's bounded nudge cannot make progress along a long floor
        // slide, so while the guard binds we latch into dive mode (full
        // authority) and hand back to glide once the path is clear again.
        {
        // 1.0° guard margin above the mechanical floor: live sim shows
        // the measured pose undershoots the guarded setpoint by ~0.4°
        // when a 600x floor-slide arrests; the margin must absorb the
        // worst dynamic error so the TRUE floor (-15) is never touched.
        const double floor_el = cfg_.safety.min_elevation_deg + 1.0;
            auto el_at = [&](const AxisAngles& a) {
                return axes_to_hs(a, cfg_.site.lat_deg, cfg_.orientation).el_deg;
            };
            const double cur_el = state_.current.el_deg;
            const double r_allowed = (floor_el - cur_el) / cfg_.floor_coast_s;
            // ∇el by central difference in axis-angle space (deg el / deg axis)
            const double h = 0.05;
            const double g_ra =
                (el_at(AxisAngles{cur.ra_deg + h, cur.dec_deg}) -
                 el_at(AxisAngles{cur.ra_deg - h, cur.dec_deg})) / (2 * h);
            const double g_de =
                (el_at(AxisAngles{cur.ra_deg, cur.dec_deg + h}) -
                 el_at(AxisAngles{cur.ra_deg, cur.dec_deg - h})) / (2 * h);
            const double g2 = g_ra * g_ra + g_de * g_de;
            const double e_dot = g_ra * rate_ra + g_de * rate_de; // predicted el rate
            if (g2 > 1e-9 && e_dot < r_allowed)
            {
                const double lam = (r_allowed - e_dot) / g2;
                rate_ra += lam * g_ra;
                rate_de += lam * g_de;
                state_.floor_guard = true;
                slide_latch_ = true;
                slide_clear_ = now_d;
                dem_seeded_ = false; // guard projection bypasses the ramp
            }
            else if (slide_latch_ && now_d - slide_clear_ > 3.0)
                slide_latch_ = false;
        }
        double x_ra = rate_ra * cfg_.ra_sign / SIDEREAL_DEG_PER_S;
        double x_de = rate_de * cfg_.dec_sign / SIDEREAL_DEG_PER_S;
        state_.rate_limited = false;

        // Rate cap JOINT across both axes, direction-preserving. Per-axis
        // clipping distorts the velocity direction (sim trace 2026-11-16: a
        // floor-slide velocity projected onto the elevation isoline came off
        // the cap tilted DOWN, and the mount sank to −32° despite a correct
        // guard). Scaling both axes together keeps the commanded heading —
        // which the floor guard already made legal — and only slows it.
        {
            const double cap = cfg_.safety.max_rate_x_sidereal;
            const double m = std::max(std::fabs(x_ra), std::fabs(x_de));
            if (m > cap)
            {
                const double s = cap / m;
                x_ra *= s;
                x_de *= s;
                state_.rate_limited = true;
            }
        }
        dbg_x_ra_ = x_ra;
        dbg_x_de_ = x_de;

        bool want_run = false;
        auto apply_axis = [&](char axis, double x, double& sent_cache,
                              double e_ax) {
            double& pend = (axis == AxisRA) ? idle_pend_ra_ : idle_pend_de_;
            if (axis == AxisRA)
                pre_sent_ra_ = sent_cache;
            else
                pre_sent_de_ = sent_cache;
            if (std::fabs(x) < cfg_.rate_epsilon_x)
                x = 0;
            if (x == 0 && sent_cache == 0)
            {
                pend = 0.0;
                return; // already stopped: don't spam :K
            }
            // Landing test (TrackerConfig::high_coast_s): a commit that
            // brakes a running HIGH axis (flip or stop) always coasts the
            // tube to cur + sign(sent)*coast_dist first. If the setpoint
            // is still farther than that landing point + margin ahead in
            // the running direction, the commit lands short of its own
            // goal and can only be disturbance, not maneuver: hold.
            // Floor-guard ticks bypass (guard velocity projection is
            // immediate safety authority, same as the reversal gate).
            auto landing_blocks = [&](double x_new) {
                if (cfg_.high_coast_s <= 0.0 || state_.floor_guard)
                    return false;
                if (std::fabs(sent_cache) <= 104.0)
                    return false; // LOW coast is ~0.03 s: free anyway
                const bool brake =
                    x_new == 0.0 ? sent_cache != 0.0
                                 : (x_new > 0.0) != (sent_cache > 0.0);
                if (!brake)
                    return false;
                const double s0 = sent_cache > 0.0 ? 1.0 : -1.0;
                const double coast = std::fabs(sent_cache) *
                                     SIDEREAL_DEG_PER_S * cfg_.high_coast_s;
                return e_ax * s0 > coast + cfg_.coast_margin_deg;
            };
            // Sign-flip at near-zero demand while running: physically this
            // is "crossing zero", not a reversal maneuver — treat it as a
            // (possibly pending) stop. Idle PI dither of +-0.1x otherwise
            // stop/started the axis every few seconds (replay trace 525 s).
            if (x != 0 && sent_cache != 0 && (x > 0) != (sent_cache > 0) &&
                std::fabs(x) < cfg_.reversal_min_x)
                x = 0;
            // Reversal persistence (tracker.h:reversal_confirm_s): while an
            // axis runs, a sign-flip demand must survive the confirm window
            // before it executes. During confirmation the demand is HELD AT
            // THE SENT PROGRAM (apply_axis's identical-program path emits no
            // wire traffic): the axis simply keeps running while the flip
            // proves itself. (Earlier variant decelerated toward zero during
            // the window; over a multi-second window that sagged every
            // transient zero-crossing off-pattern — hold-at-sent is both
            // wire-free and pointing-neutral, its cost being the hold budget
            // below.) A persistent flip executes normally once its
            // (budget-capped) window elapses; oscillating flips re-stamp and
            // never execute. Floor-guard ticks bypass: the guard's velocity
            // projection is immediate safety authority and must not be
            // delayed.
            {
                double& fp = (axis == AxisRA) ? flip_pend_ra_ : flip_pend_de_;
                const bool flip_now =
                    x != 0 && sent_cache != 0 && (x > 0) != (sent_cache > 0);
                if (flip_now && !state_.floor_guard && cfg_.reversal_confirm_s > 0.0)
                {
                    // HOLD BUDGET: holding runs the axis on the OLD program,
                    // so a genuine flip's overshoot accumulates at |sent|
                    // deg/s — cap the window by the angular budget a hold may
                    // cost: win = min(confirm, hold_deg / |sent_rate|).
                    // Near-zero dithers (the oscillating quantization class
                    // this guard exists for) are cheap to hold and get the
                    // full noise window; HIGH flips shorten automatically
                    // instead of riding past the reversal — the failure that
                    // made the flat legacy-era window dangerous on real
                    // passes (FO-29/1Hz suite overshoot at 6 s).
                    const double sent_dps =
                        std::fabs(sent_cache) * SIDEREAL_DEG_PER_S;
                    double win = cfg_.reversal_confirm_s;
                    if (cfg_.reversal_hold_deg > 0.0 && sent_dps > 1e-9)
                        win = std::min(win, cfg_.reversal_hold_deg / sent_dps);
                    if (fp == 0.0 || now_d - fp > 2.0 * win)
                        fp = now_d; // fresh request (stale stamp: re-arm)
                    if (now_d - fp < win)
                        x = sent_cache; // unconfirmed: keep running as-is
                    else if (landing_blocks(x))
                        x = sent_cache; // window done but the coast would
                                        // land short of the setpoint: the
                                        // flip is disturbance; keep running
                                        // (stamp re-arms per its own rule,
                                        // HIGH windows are short so a flip
                                        // that becomes justified executes
                                        // within ~one re-arm cycle)
                    else
                        fp = 0.0; // confirmed: execute the reversal
                }
                else
                    fp = 0.0;
            }
            if (x == 0 && landing_blocks(0.0))
                return; // the :K coast would still stop short of the
                        // setpoint: this zero demand is a transient riding
                        // past, not arrival. Keep the running HIGH program
                        // (wire-free; the glide PI re-issues the stop every
                        // tick, so genuine arrival stops within one tick).
            // Land HIGH flips in the LOW band (user law 2026-11-20: "never
            // command HIGH where we overshoot to the other side by much").
            // A flip commit out of HIGH lands the axis THROUGH its :K coast
            // (~0.74 deg at 600x live Sirius): the AO-73 ping-pong was the
            // law immediately re-committing the NEXT HIGH program on the far
            // side and coasting again, ~45 :K/min. The flip absorbs the
            // coast once; the program that lands must be LOW band (coast
            // ~0.03 s, in-place :I rate changes, no stop/restart wire cost).
            // A pattern that genuinely needs >104x re-escalates HIGH in the
            // SAME direction from LOW (LOW :K coast negligible, not a pump)
            // or re-opens the error past the HIGH-gate radius — every cycle
            // is coast-braked toward the setpoint, bounded, alternating
            // coasts no longer carry energy across it.
            if (x != 0 && cfg_.high_coast_s > 0.0 && !state_.floor_guard &&
                sent_cache != 0 && std::fabs(sent_cache) > 104.0 &&
                (x > 0) != (sent_cache > 0))
                x = std::copysign(std::min(std::fabs(x), 104.0), x);
            if (x == 0)
            {
                // LOW-band dwell: keep the running rate briefly; a demand
                // returning above eps cancels the pending stop without ever
                // touching the wire. HIGH band stops immediately (its coast
                // is long and speed-scaled; holding it would overshoot).
                const bool low = std::fabs(sent_cache) <= 104.0;
                if (low && cfg_.idle_stop_dwell_s > 0.0)
                {
                    if (pend == 0.0) { pend = now_d; return; }
                    if (now_d - pend < cfg_.idle_stop_dwell_s) return;
                }
                pend = 0.0;
            }
            else
                pend = 0.0; // demand alive: cancel any pending stop
            // A stopped axis does not :J for sub-threshold demands (idle
            // dither). Genuine pattern rates exceed reversal_min_x; glide's
            // P-term re-crosses it within milliseconds of real error.
            if (x != 0 && sent_cache == 0 && std::fabs(x) < cfg_.reversal_min_x)
                return;
            // HIGH speed: :I NAKs while running, so every rate change costs
            // :K + a speed-scaled coast (~1.0 s / ~1 deg at 600x live Sirius)
            // that also BLOCKS the tick in wait_running_clear and starves
            // issuance for ~2 s after the :J. LOW speed takes in-place :I
            // free at ~0.14x granularity; any percentage deadband there
            // SWALLOWS the glide loop's small sustained corrections and
            // freezes a transient into a permanent pointing offset (sim
            // trace 2026-11-16: 0.004-0.018 deg/s corrections under a 2 %
            // band left 0.23 deg az error forever). Keep only a sub-
            // quantization floor.
            // HIGH->HIGH economy (live sweep-6 2026-11-18): the 10 % dead-
            // band did NOT bound the wire under a rate-capped dive — DEC
            // swung -507x -> -600x (18 %) ~3x/s for 153 events, each paying
            // coast+blind starvation, ending in a skip latch. At 600x the
            // 10 % band is 60x = 0.25 deg/s of "free" rate error, while the
            // :K it pays moves the antenna ~1 deg on the coast alone: any
            // demand wobble inside one coast-cost of rate is strictly NEGATIVE
            // economy. Re-program only beyond high_reprogram_db (default 25 %:
            // at 600x that is 150x = 0.63 deg/s, which the closed position
            // loop accumulates in ~2-3 s, so a genuinely wrong rate cannot
            // freeze — the error term walks the demand out of the band).
            const bool sent_high = std::fabs(sent_cache) > 104.0;
            const bool want_high = std::fabs(x) > 104.0;
            // HIGH rescale DIRECTION-PERSISTENCE law (SO-50 demand pump
            // 2026-11-21, see TrackerConfig::high_recommit_s): a
            // same-direction commit that TOUCHES HIGH and flips the
            // demand-vs-sent MAGNITUDE direction against the last
            // committed rescale is a wobble candidate: hold at the sent
            // program (wire-free) for min(high_recommit_s,
            // high_recommit_hold_deg / mismatch); a demand that re-enters
            // the economy zones cancels the episode without ever touching
            // the wire. MONOTONIC rescales (same direction as the last
            // commit) are real work — decel ladders before meridian
            // reversals were the proof: holding those (plain persistence
            // window) cost 2-7 deg sky at the 1/sin(az) lever in
            // m-1hz/m-meridional — and commit after at most one tick.
            // Mirror image of the pattern-side reversal-persistence guard.
            // Scope GLIDE-only (the pump census rode push-cadence wobble
            // in follow; DIVE holds far-target authority) and escaped by
            // |axis error| >= high_recommit_escape_deg (position loop) and
            // the floor guard (safety steering is never delayed). Stops
            // and reversals are NOT in scope: they have their own laws.
            {
                double& sp = (axis == AxisRA) ? scale_pend_ra_ : scale_pend_de_;
                int& sd = (axis == AxisRA) ? scale_dir_ra_ : scale_dir_de_;
                // WOBBLE REGIME detector: leaky rescale-COMMIT counter
                // (rate-based, NOT flip/cancel-based: in the SO-50 storm
                // the demand PERSISTS out-of-band between GT2 pushes
                // (age 2-3.5 s), so confirm episodes COMMIT rather than
                // cancel and a cancel-triggered regime never armed — the
                // measured proof was regime knob values moving nothing).
                // Each rescale commit (or regime silent-hold) adds 1;
                // the counter decays with time constant
                // high_recommit_regime_s; at >= high_recommit_regime_n
                // recent commits the axis is storming (clean passes run
                // < 1 HIGH rescale commit / 4 s per axis; the SO-50 pump
                // ran 4-10). Inside the regime: the silent economy
                // deadband for HIGH-touching rescales widens to
                // high_recommit_regime_db and every remaining out-of-band
                // rescale goes through the confirm window with the looser
                // regime hold budget.
                double& rc = (axis == AxisRA) ? scale_reg_ra_ : scale_reg_de_;
                double& rs =
                    (axis == AxisRA) ? scale_reg_stamp_ra_ : scale_reg_stamp_de_;
                if (cfg_.high_recommit_s > 0.0 &&
                    cfg_.high_recommit_regime_s > 0.0)
                {
                    if (rs > 0.0 && now_d > rs)
                        rc *= std::exp(-(now_d - rs) /
                                       cfg_.high_recommit_regime_s);
                    rs = now_d;
                }
                // Client-STALENESS requirement: the pump is a THROTTLED-
                // CLIENT phenomenon — 0.1-deg-quantized pushes 2-4 s
                // apart (GT2 follow gate near the horizon) stepping the
                // axis demand ±1-2 deg against a frozen pattern. A
                // fast-cadence client's rescale ladders (dive approach,
                // meridian ramps) are disturbance-free WORK (a 1 Hz
                // twin of m-1hz showed exactly that false-positive).
                // LATCHED, not per-tick: pushes re-zero the age every
                // 2 s; the latch rides those fresh ticks (the pump
                // commits between pushes too).
                double& st =
                    (axis == AxisRA) ? scale_reg_oldage_ra_ : scale_reg_oldage_de_;
                if (cfg_.high_recommit_regime_age_s <= 0.0 ||
                    dbg_target_age_ >= cfg_.high_recommit_regime_age_s)
                    st = now_d; // young gate disabled or old: stamp
                const bool old_age =
                    cfg_.high_recommit_regime_age_s <= 0.0 ||
                    (st > 0.0 && now_d - st < cfg_.high_recommit_regime_s);
                const bool regime = cfg_.high_recommit_s > 0.0 &&
                                    cfg_.high_recommit_regime_s > 0.0 &&
                                    old_age &&
                                    rc >= cfg_.high_recommit_regime_n;
                // Latch for the demand-LPF escalation next tick (the LPF
                // sits upstream of this block in the tick). STRICTER BAR:
                // only storm-GRADE regimes (rc past demand_lpf_regime_
                // min_rc) escalate; a plain regime latch on legitimate
                // HIGH pattern work would add pure phase lag (see the
                // TrackerConfig comment).
                {
                    const bool rel = cfg_.demand_lpf_regime_release_err_deg > 0.0 &&
                                     std::fabs(e_ax) > cfg_.demand_lpf_regime_release_err_deg;
                    bool& rf = (axis == AxisRA) ? regime_ra_ : regime_de_;
                    rf = regime && rc >= cfg_.demand_lpf_regime_min_rc && !rel;
                    double& dr = (axis == AxisRA) ? dbg_reg_ra_ : dbg_reg_de_;
                    dr = rc;
                }
                const bool same_dir =
                    x != 0 && sent_cache != 0 && (x > 0) == (sent_cache > 0);
                bool out_of_band = false;
                double swing = 0.0; // |ax/as - 1| for the touched pair
                if (same_dir && (sent_high || want_high))
                {
                    const double ax = std::fabs(x), as = std::fabs(sent_cache);
                    swing = as > 0.0 ? std::fabs(ax / as - 1.0) : 1.0;
                    const auto near104 =
                        [&](double v) {
                            return v >= 104.0 *
                                       (1.0 - cfg_.high_band_demote_db) &&
                                   v <= 104.0 *
                                       (1.0 + cfg_.high_band_promote_db);
                                };
                    out_of_band =
                        (sent_high && want_high)
                            ? !(std::fabs(ax - as) < 6.0 ||
                                std::fabs(ax / as - 1.0) <
                                    cfg_.high_reprogram_db)
                              // Band crossings: only the ones that ACTUALLY
                              // commit (a near104 endpoint rides the
                              // silent-economy contract instead — the
                              // confirm must mirror the economy zones or
                              // it arms on demands that never hit wire).
                              // Small-swing crossings inside a regime are
                              // caught by the magnitude gate below (which
                              // is acceleration-only, see there).
                            : (!near104(as) && !near104(ax));
                    // Regime widens the SILENT zone for acceleration
                    // rescales only (deceleration ladders are the real
                    // work of meridian Jacobian ramps — the m-meridional
                    // 7-deg overshoot of the direction-blind gate): the
                    // episode logic is skipped, the magnitude gate owns
                    // them.
                    if (regime && swing < cfg_.high_recommit_regime_db &&
                        ax > as)
                        out_of_band = false;
                }
                const bool escape =
                    state_.floor_guard ||
                    std::fabs(e_ax) >= cfg_.high_recommit_escape_deg ||
                    cfg_.high_recommit_s <= 0.0 ||
                    !state_.glide; // DIVE = far-target position authority;
                                   // the pump is a GLIDE push-cadence
                                   // phenomenon (SO-50 census)
                // Error-GROWTH gate: the pump's commits are self-
                // perturbing (HIGH coast + issuance starvation move the
                // error AWAY from the setpoint — TICK forensics: e grew
                // 0.447 -> 0.486 deg across consecutive storm commits),
                // while genuine rescale LADDERS (dive convergence,
                // meridian Jacobian ramps) walk the error DOWN rung
                // after rung. The regime silent-hold therefore only
                // applies once the error is no longer improving against
                // its value at the last rescale commit on this axis
                // (ec): without this, the regime gate held the
                // convergence/climb ladders of clean passes (m-1hz
                // 0.21 -> 0.29, m-meridional 0.31 -> 0.38 deg).
                double& ec = (axis == AxisRA) ? rescale_err_ra_ : rescale_err_de_;
                const bool error_growing = std::fabs(e_ax) >= ec;
                if (!out_of_band || escape)
                {
                    // WOBBLE-REGIME MAGNITUDE GATE (acceleration-only —
                    // deceleration rescales are the real work of meridian
                    // Jacobian ramps; silently holding them reproduced
                    // the 7-deg m-meridional overshoot of the first
                    // persistence variant): inside a regime a same-dir
                    // ACCELERATION demand that touches HIGH and swings
                    // less than high_recommit_regime_db is held at the
                    // running program, exactly like the economy zones do
                    // outside a regime. Rationale: the SO-50 worst-minute
                    // demand PERSISTS out-of-band at the 0.25 db for
                    // > 1 s (real Jacobian swings of 30-40 % + push
                    // quantization), so time confirmation alone cannot
                    // absorb it (115 :K/min survived the window law);
                    // each such commit costs ~0.7 deg coast + ~2.5 s
                    // issuance starvation, MORE than the swing corrects.
                    // An acceleration hold only LAGS (position-loop
                    // escape at high_recommit_escape_deg bounds it); the
                    // global-db widening this mirrors was measured
                    // error-neutral.
                    if (regime && error_growing && !escape &&
                        cfg_.high_recommit_regime_db > 0.0 &&
                        same_dir && (sent_high || want_high) &&
                        std::fabs(x) > std::fabs(sent_cache) &&
                        std::fabs(std::fabs(x) - std::fabs(sent_cache)) >=
                            6.0 &&
                        swing < cfg_.high_recommit_regime_db)
                    {
                        x = sent_cache; // silent hold: economy-zone
                                        // fall-through below sees an
                                        // identical program -> no wire
                        rc += 1.0;      // storm evidence (keeps the
                                        // regime alive while it holds)
                        sp = 0.0;
                    }
                    else
                    {
                        sp = 0.0; // no episode (the rescale-direction ledger
                                  // KEEPS: it is what the alternation reads)
                        // Error ledger: this demand (beyond the 6x wire
                        // quantum, same-dir, touches HIGH) will commit on
                        // the economy path below unless a silent zone
                        // swallows it — stamp the error it commits at.
                        if (!escape && same_dir && (sent_high || want_high) &&
                            std::fabs(std::fabs(x) - std::fabs(sent_cache)) >=
                                6.0)
                            ec = std::fabs(e_ax);
                    }
                }
                else
                {
                    const int d = std::fabs(x) > std::fabs(sent_cache)
                                      ? 1 : -1; // magnitude direction
                    // HOLD ONLY ACCELERATION FLIPS. A deceleration hold
                    // keeps the FASTER program running past the setpoint:
                    // overshoot -> flip -> reversal coupling, and every
                    // meridian-crossing suite scenario (m-1hz, m-
                    // meridional, scenario 3) lives in exactly that
                    // decel-before-reversal transient at the 1/sin(az)
                    // lever. An acceleration hold only lags (bounded by
                    // the budget), never overshoots.
                    // Flip candidates: same as outside a regime (only
                    // ACCELERATION flips against the rescale ledger arm
                    // an episode — regime authority is the magnitude
                    // gate, never deceleration holds). Inside a regime
                    // the armed episode gets the full window + regime
                    // hold budget (see budget below).
                    const bool flip_cand = sd != 0 && d != sd && d > 0;
                    if (flip_cand)
                    {
                        double win = cfg_.high_recommit_s;
                        // Single hold budget (regime-specific hold knob
                        // was tried and removed: the clean-pass meridian
                        // episodes arm on REAL flips whose holds must
                        // stay budget-bounded regardless of regime).
                        const double budget = cfg_.high_recommit_hold_deg;
                        const double mismatch_dps = std::fabs(
                            std::fabs(x) - std::fabs(sent_cache)) *
                            SIDEREAL_DEG_PER_S;
                        if (budget > 0.0 && mismatch_dps > 1e-9)
                            win = std::min(win, budget / mismatch_dps);
                        if (sp == 0.0) sp = now_d; // episode begins
                        if (now_d - sp < win)
                            x = sent_cache; // confirming: hold the program
                        else
                        {
                            sp = 0.0; // persisted: commit, adopt direction
                            sd = d;
                            rc += 1.0; // storm evidence
                            ec = std::fabs(e_ax); // error ledger (above)
                        }
                    }
                    else
                    {
                        sp = 0.0; // continuation or first rescale: commit
                        sd = d;   // now (legacy path, zero added delay)
                        rc += 1.0;
                        ec = std::fabs(e_ax); // error ledger (above)
                    }
                }
            }
            const double x_cmd = x; // final post-confirm demand (the wire
                                    // program + band grace classify on this)
            if (x != 0 && sent_cache != 0 && (x > 0) == (sent_cache > 0))
            {
                const double ax = std::fabs(x), as = std::fabs(sent_cache);
                // boundary transition zone: [104x -6 %, 104x +6 %]
                const auto near104 =
                    [&](double v) {
                        return v >= 104.0 * (1.0 - cfg_.high_band_demote_db) &&
                               v <= 104.0 * (1.0 + cfg_.high_band_promote_db);
                    };
                if (!sent_high && !want_high)
                {
                    if (std::fabs(ax - as) < 0.05)
                        return; // sub-quantization: :I would change nothing
                }
                else if (sent_high && want_high)
                {
                    if (std::fabs(ax - as) < 6.0 ||
                        std::fabs(ax / as - 1.0) < cfg_.high_reprogram_db)
                        return; // keep the running high rate (25 %/6x)
                }
                else if (near104(as) && near104(ax))
                {
                    // Band-crossing demand INSIDE the boundary zone: hold
                    // the running state. Crossing costs a stop+restart in
                    // either direction, and both rates here are within ~6 %
                    // of each other and of the boundary, so staying put is
                    // nearly free and kills the promote/demote flip that an
                    // ISS-class az rate sweeping through 104x produces
                    // (replay t=1920 s window). OUTSIDE the zone we fall
                    // through and reprogram for real — a 101x demand must
                    // never hold a 600x rate, only a ~108x one.
                    return;
                }
            }
            // NO time-based reprogram cooldown here (tried 2026-11-16 and
            // rejected): with a closed position loop, holding a stale HIGH
            // rate for a fixed hold-off either freezes a rate error the
            // nudge cannot exit quickly enough (fixed 2.5 s froze
            // transients at the 104-135x boundary in test_tracker) or is
            // escaped within a second and changes nothing. The deadband
            // above is the error-bounded economy; the board's coast is
            // paid once per genuine rate change.
            board_.slew_rate(axis, x_cmd);
            // Band-aware ramp grace. Wire program changed (cold :J or any
            // in-place :I): classify the commanded rate into board
            // acceleration bands and restamp the skip-grace on entry.
            // Evidence (Phase-1 + first live Phase-4 sweep 2026-11-17, where
            // DEC latched at t=24 s while an I-driven climb entered the slow
            // band via free in-place :I on the old cold-:J-only stamp):
            //   LOW <~55x  delivers commanded issuance immediately
            //               (Phase-1: 0.999-1.018 delivered at 10-50x)
            //   SLOW 55-104x starves/phase-losses 10-25 % converging over
            //               ~15-25 s (Phase-1 0.902 at 80x, 0.776 at 90x)
            //   HIGH >=104x ~1 s acceleration (Phase-1, promoted band)
            // Hysteresis (5x below the entry edge while armed) stops glide
            // dither at the boundary from re-arming the grace repeatedly —
            // each arm blanks the skip detector for its whole window.
            {
                int& band = (axis == AxisRA) ? ramp_ra_band_ : ramp_de_band_;
                const double axb = std::fabs(x_cmd);
                const double lo  = cfg_.safety.skip_ramp_grace_low_min_x;
                const bool slow =
                    axb < 104.0 && (band == 1 ? axb >= lo - 5.0 : axb >= lo);
                const int c = axb >= 104.0 ? 2 : (slow ? 1 : 0);
                if (sent_cache == 0.0 || c != band) // cold :J or band entry
                {
                    band = c;
                    (axis == AxisRA ? ramp_ra_mono_ : ramp_de_mono_) = mono_now;
                    (axis == AxisRA ? ramp_ra_grace_s_ : ramp_de_grace_s_) =
                        cold_start_grace_s(cfg_.safety, x_cmd);
                    // Cold starts were assumed to accelerate from t=0 with no
                    // blind — live AO-123 2026-11-20 false-latched a skip on a
                    // cold +600x dive (delivery 0.36 vs linear 0.67 at t=2 s):
                    // the board's cold issuance is dead early, then ramps.
                    (axis == AxisRA ? ramp_ra_blind_s_ : ramp_de_blind_s_) =
                        c == 2 ? cfg_.safety.skip_cold_blind_high_s
                               : cfg_.safety.skip_cold_blind_low_s;
                }
            }
            sent_cache = x_cmd;
            if (x_cmd != 0) want_run = true;
        };
        // e_ax is the setpoint error in COUNTER space (axis-angle error
        // times the axis wire sign: lines above x=rate*sign/SID; counter
        // coordinate = (axis − ref)·sign, so counter-space error = err·sign
        // — the sign the board and the coast direction live in).
        apply_axis(AxisRA, x_ra, sent_ra_x_, err_ra * cfg_.ra_sign);
        apply_axis(AxisDE, x_de, sent_de_x_, err_de * cfg_.dec_sign);
        // A forced :K/restart inside slew_rate puts a decelerate+hold gap
        // into the very motion window our skip detector measures against the
        // pre-stop rate; restart that assessment from a clean baseline
        // (same semantics as a deliberate stop, see halt lambda above) and
        // restamp the restart axis's acceleration grace.
        const bool rst_ra = board_.take_stop_restart(AxisRA);
        const bool rst_de = board_.take_stop_restart(AxisDE);
        if (rst_ra || rst_de)
        {
            have_prev_cur_ = false;
            skip_ra_ = skip_de_ = skip_exp_ra_ = skip_exp_de_ = 0;
            char srbuf[64];
            if (rst_ra)
            {
                ++stop_restarts_ra_;
                snprintf(srbuf, sizeof(srbuf), "RA %+.0fx -> %+.0fx",
                         pre_sent_ra_, sent_ra_x_);
                state_.last_stop_restart = srbuf;
                ramp_ra_mono_  = mono_now;
                ramp_ra_grace_s_ = restart_grace_s(cfg_.safety, sent_ra_x_);
                ramp_ra_blind_s_ = restart_blind_s(cfg_.safety, sent_ra_x_);
            }
            if (rst_de)
            {
                ++stop_restarts_de_;
                snprintf(srbuf, sizeof(srbuf), "DEC %+.0fx -> %+.0fx",
                         pre_sent_de_, sent_de_x_);
                state_.last_stop_restart = srbuf;
                ramp_de_mono_  = mono_now;
                ramp_de_grace_s_ = restart_grace_s(cfg_.safety, sent_de_x_);
                ramp_de_blind_s_ = restart_blind_s(cfg_.safety, sent_de_x_);
            }
        }
        any_axis_running_ = want_run || sent_ra_x_ != 0 || sent_de_x_ != 0;
        state_.rate_ra_x = sent_ra_x_;
        state_.rate_dec_x = sent_de_x_;
        state_.stop_restart_ra = stop_restarts_ra_;
        state_.stop_restart_de = stop_restarts_de_;
    }
    catch (const BoardError& e)
    {
        // comms failure while attached to the mount: kill motion, latch.
        stop_quiet();
        safety_.trip(std::string("board I/O failure: ") + e.what());
        state_.halted = true;
        state_.halt_reason = safety_.latch_reason();
    }
}

// ---------------- --debug tick tracing ----------------

// Called from the step() wrapper with mu_ held, AFTER step_impl on EVERY
// path that produced a real tick (calibrated, dt > 0), including all the
// early safety returns and the BoardError catch — that is the point: a halt
// row is the row the autopsy needs. Never throws: it is an observer.
void Tracker::emit_debug_tick(std::chrono::steady_clock::time_point mono_now,
                              double dt)
{
    (void)mono_now; // timestamps live in the wire latencies + dt; the caller
                    // (main's log_line) prefixes the UTC stamp
    // Observer effect (documented at set_debug): one :f poll per axis. The
    // frames it generates are queries (write-gate safe) and land in this
    // tick's own wire segment. If the board is dead, status() throws and the
    // failure shows up in the wire segment of this same row instead.
    char fra[8] = "err", frde[8] = "err";
    try
    {
        auto fword = [](const AxisStatus& st) {
            return std::string() + static_cast<char>('0' + ((st.slew_mode ? 1 : 0) |
                                                            (st.backward ? 2 : 0) |
                                                            (st.high_speed ? 4 : 0)))
                                 + static_cast<char>('0' + (st.running ? 1 : 0))
                                 + static_cast<char>('0' + (st.initialized ? 1 : 0));
        };
        std::string wa = fword(board_.status(AxisRA));
        std::string wd = fword(board_.status(AxisDE));
        snprintf(fra, sizeof(fra), "%s", wa.c_str());
        snprintf(frde, sizeof(frde), "%s", wd.c_str());
    }
    catch (const BoardError&) { /* row still emitted; wire shows the failure */ }

    const std::vector<WireExchange> wire = board_.take_trace();

    std::string s;
    s.reserve(320 + wire.size() * 48);
    app(s, "[TICK] seq=%llu dt=%.3f", static_cast<unsigned long long>(++dbg_seq_), dt);
    app(s, " cur=%06X,%06X ref=%06X,%06X",
        last_raw_ra_, last_raw_de_, ref_count_ra_, ref_count_de_);
    app(s, " ax=%.3f,%.3f", dbg_cur_.ra_deg, dbg_cur_.dec_deg);
    app(s, " vm=%.4f,%.4f", vel_est_.ra_deg, vel_est_.dec_deg);
    app(s, " hs=%.2f,%.2f", state_.current.az_deg, state_.current.el_deg);
    if (target_.valid)
        app(s, " tg=%.2f,%.2f age=%.1f", target_.az, target_.el, dbg_target_age_);
    else
        app(s, " tg=none");
    if (have_vel_)
        app(s, " pat=%.4f,%.4f", pattern_hs_vel_.az_deg, pattern_hs_vel_.el_deg);
    else
        app(s, " pat=none");
    app(s, " des=%.3f,%.3f err=%.3f flip=%d",
        dbg_des_.ra_deg, dbg_des_.dec_deg, dbg_err_, des_on_flip_ ? 1 : 0);
    app(s, " x=%+.1f,%+.1f sent=%+.1f,%+.1f band=%d,%d",
        dbg_x_ra_, dbg_x_de_, sent_ra_x_, sent_de_x_, ramp_ra_band_, ramp_de_band_);
    // wobble-regime leaky counters + demand-LPF escalation state (the
    // 0a4(h) law): reg= is the rc ledger, lrpf flag = escalated tau live.
    app(s, " reg=%.1f,%.1f", dbg_reg_ra_, dbg_reg_de_);
    app(s, " sk=%+.3f,%+.3f", skip_ra_, skip_de_);
    app(s, " f=%s,%s", fra, frde);

    // active-state flags: only the engaged ones, comma-joined
    {
        s += " flags=";
        bool first = true;
        auto flag = [&](const char* name, bool on) {
            if (!on) return;
            s += first ? "" : ",";
            s += name;
            first = false;
        };
        flag("trk", state_.tracking);
        flag("glide", state_.glide);
        flag("floor", state_.floor_guard);
        flag("cap", state_.rate_limited);
        flag("ret", state_.travel_returning);
        flag("halt", state_.halted);
        flag("latch", safety_.latched());
        flag("lrpf", dem_esc_ra_ || dem_esc_de_);
        if (first) s += "-";
    }
    // reasons (quoted): halt trumps travel trumps latch — they co-occur on a
    // skip latch, and the halt_reason IS the latch text there.
    if (state_.halted && !state_.halt_reason.empty())
    {
        s += " halt=\"";
        append_wire_token(s, state_.halt_reason, true);
        s += '"';
    }
    if (state_.travel_returning && !state_.travel_reason.empty())
    {
        s += " ret=\"";
        append_wire_token(s, state_.travel_reason, true);
        s += '"';
    }
    // stop/restart event edge since the previous row (cumulative always,
    // delta + evidence only on change: the :K storm forensics counts these)
    const int sr_ra = state_.stop_restart_ra, sr_de = state_.stop_restart_de;
    app(s, " srk=%d,%d", sr_ra, sr_de);
    if (sr_ra != dbg_sr_ra_ || sr_de != dbg_sr_de_)
    {
        app(s, " srkd=+%d,+%d ev=\"", sr_ra - dbg_sr_ra_, sr_de - dbg_sr_de_);
        append_wire_token(s, state_.last_stop_restart, true);
        s += '"';
    }
    dbg_sr_ra_ = sr_ra;
    dbg_sr_de_ = sr_de;

    // the wire: every raw exchange this tick produced, verbatim, in order.
    // One token per exchange: [tx rx latency_ms], rx marker substitutions:
    //   (held) write-gated, never hit the wire; (timeout) nothing arrived;
    //   a trailing "?" marks a malformed (unparseable) frame.
    app(s, " wire=%zu:", wire.size());
    constexpr size_t WIRE_SHOW = 64;
    for (size_t i = 0; i < wire.size() && i < WIRE_SHOW; ++i)
    {
        const WireExchange& w = wire[i];
        s += " [";
        append_wire_token(s, w.tx);
        s += ' ';
        if (w.held)
            s += "(held)";
        else if (w.err == "timeout")
            s += "(timeout)";
        else
        {
            append_wire_token(s, w.rx);
            if (w.err == "malformed")
                s += '?';
            else if (w.err == "nak")
                app(s, "!%d", w.nak);
        }
        if (!w.held)
            app(s, " %.1f", w.ms);
        s += ']';
    }
    if (wire.size() > WIRE_SHOW)
        app(s, " +%zu more", wire.size() - WIRE_SHOW);

    dbg_sink_(s);
}

std::string Tracker::on_status()
{
    // Single lock pass composes straight from the members: snapshot()/
    // current_axes() re-take the same mutex, so calling them here would
    // deadlock. dbg_cur_ is stored unconditionally per tick (the --debug
    // capture is branch-free), so the axis-space pose + cord ledger are
    // always current between ticks.
    std::lock_guard<std::mutex> lk(mu_);
    std::ostringstream o;
    o << "sateqd " << version_string() << "\n";
    o << std::fixed << std::setprecision(3);
    if (board_reset_)
        o << "state: BOARD RESET latched (counters reset mid-session): re-home the"
             " mount, then restart sateqd with --no-resume\n";
    else if (safety_.latched())
        o << "state: LATCHED: " << safety_.latch_reason()
          << " - clear with rotctld 'R' (re-arm at CURRENT pose)\n";
    else if (state_.halted)
        o << "state: HALTED: "
          << (state_.halt_reason.empty() ? std::string("halted") : state_.halt_reason)
          << "\n";
    else if (state_.tracking)
        o << "state: TRACK (" << (state_.glide ? "glide" : "dive")
          << (state_.floor_guard ? ", floor-guard" : "")
          << (state_.rate_limited ? ", rate-cap" : "") << ")\n";
    else
        o << "state: IDLE (no target)\n";
    if (state_.travel_returning)
        o << "travel: returning to datum - " << state_.travel_reason << "\n";
    o << "pointing: az=" << state_.current.az_deg
      << " el=" << state_.current.el_deg << "\n";
    if (state_.tracking || !state_.halted)
        o << "target:   az=" << state_.target.az_deg
          << " el=" << state_.target.el_deg << "\n";
    o << "rates:    RA=" << std::showpos << std::setprecision(1)
      << state_.rate_ra_x << "x DEC=" << state_.rate_dec_x << "x"
      << std::noshowpos << std::setprecision(3)
      << "  branch=flip" << (des_on_flip_ ? "1" : "0") << "\n";
    o << "cord:     DEC wrap " << std::showpos << std::setprecision(1)
      << (dbg_cur_.dec_deg - ref_axes_.dec_deg) << std::noshowpos
      << " deg (window +/-" << cfg_.wrap_window_deg
      << "; datum flight unwinds to 0)\n" << std::setprecision(3);
    o << "axes:     RA " << std::showpos << (dbg_cur_.ra_deg - ref_axes_.ra_deg)
      << " dec " << (dbg_cur_.dec_deg - ref_axes_.dec_deg) << std::noshowpos
      << " deg from datum (unfolded)\n";
    o << "wire:     :K reprograms RA=" << state_.stop_restart_ra
      << " DEC=" << state_.stop_restart_de << "\n";
    o << "clients:  " << live_clients_ << "\n";
    return o.str();
}

TrackerState Tracker::snapshot()
{
    std::lock_guard<std::mutex> lk(mu_);
    return state_;
}

// ---------------- RotctldHandler ----------------

void Tracker::on_clients_changed(int live_clients)
{
    std::lock_guard<std::mutex> lk(mu_);
    live_clients_ = live_clients;
    if (live_clients > 0)
        return;
    // Last rotctld client gone (operator disconnected — GT2 un-checks
    // Connect by closing the socket): drop the target so the next tick
    // halts. Do NOT latch: a reconnect plus fresh P simply resumes tracking.
    target_.valid = false;
}

void Tracker::on_set_target(double az_deg, double el_deg)
{
    set_target_at(az_deg, el_deg, std::chrono::steady_clock::now());
}

void Tracker::set_target_at(double az_deg, double el_deg,
                            std::chrono::steady_clock::time_point now)
{
    std::lock_guard<std::mutex> lk(mu_);

    // A REAL client push always wins over (and cancels) the daemon-owned
    // re-home maneuver, and re-arms it for the next idle period.
    rehome_active_ = false;
    rehome_done_   = false;
    rehome_idle_mono_ = 0;
    home_no_route_ = false;
    // el-gate (see TrackerConfig::rehome_min_el_deg): only a push that
    // actually points above the horizon at a real target ARMS the idle
    // homeflight. Low-el pushes (GT2 el-0 pre-aims, post-LOS ghosts)
    // still CANCEL an in-flight homeflight (above) but never arm one —
    // and never DISARM either: the ghosts that trail a real LOS must not
    // veto the unwind flight the real pass earned.
    if (el_deg > cfg_.rehome_min_el_deg)
        rehome_armed_ = true;

    // raw client-push recorder tap (main.cpp --record); log exactly what
    // arrived, before any normalization, so replays are faithful
    if (target_tap_)
        target_tap_(now, az_deg, el_deg);

    // Target velocity estimation in continuous axis-angle space: EMA of
    // d(desired)/dt between rotctld updates (GPredict sends ~1 Hz). The
    // az/el→axes map is time-free (LST cancels between close-spaced
    // samples), so this is the true setpoint rate, no propagation needed.
    // Used for the extrapolated setpoint (dive mode + state reporting);
    // glide mode runs on the reference trajectory below.
    AxisAngles d = hs_to_axes(az_deg, el_deg, cfg_.site.lat_deg, cfg_.orientation);
    if (have_upd_des_)
    {
        // unwrap against the previous update so long passes never jump 360°
        d.ra_deg  += 360.0 * std::round((last_upd_des_.ra_deg - d.ra_deg) / 360.0);
        d.dec_deg += 360.0 * std::round((last_upd_des_.dec_deg - d.dec_deg) / 360.0);
    }
    last_upd_des_ = d;
    have_upd_des_ = true;

    // --- pattern estimator: alpha-beta filter in AZ/EL ------------------
    // Filters az and el independently (ScalarABFilter docs in tracker.h
    // cover why LSQ window fits and a CA-Kalman were both wrong HERE).
    // The satellite's AZ/EL motion is the smooth quantity; GT2's 0.1 deg
    // quantization is white noise in that space, and the az/el -> axis
    // map's Jacobian at the filtered point converts state and derivatives
    // to continuous axis angles (map is time-free: LST cancels between
    // close-spaced samples). Acceleration is NOT filter state — the alpha-
    // beta velocity is ramp-unbiased, so a genuine az/el acceleration
    // shows up as a slow velocity drift; a long-tau EMA of that drift
    // gives a low-noise accel for the age^2 setpoint extrapolation and
    // the innovation gate without letting quantization noise into the
    // rate path (the glide reference NEVER sees it: pattern_vel_ only).
    // Restart rules: client gap (pattern_gap_reset_s) or innovation gate
    // (a new sample pattern_gate_deg off the filter's own prediction —
    // satellite switch-over in one rotctld stream, GT2 FO-29->ISS live
    // 2026-11-16, or a tracker-app mode change).
    HsCoord hs_u{az_deg, el_deg};
    if (have_upd_hs_)
        hs_u.az_deg += 360.0 * std::round((last_upd_hs_.az_deg - hs_u.az_deg) / 360.0);
    last_upd_hs_ = hs_u;
    have_upd_hs_ = true;

    const double dt_push = std::chrono::duration<double>(now - last_push_t_).count();
    const double dt_ab = std::chrono::duration<double>(now - last_ab_t_).count();
    bool restart = !pat_ab_ready_ || dt_push <= 1e-4 ||
                   dt_ab > cfg_.pattern_gap_reset_s;
    // Min-cadence hold (see TrackerConfig::pattern_min_dt_s): a push
    // arriving inside the window keeps target freshness (below:
    // target_/watchdog/ref_target_time_ still advance) but the estimator
    // keeps its previous state; the NEXT real update runs against
    // last_ab_t_ carrying the latest sample, so correlated push pairs
    // (GT2 double-fire) cannot amplify beta*r/dt by 1/dt_push.
    const bool pending = !restart && dt_ab < cfg_.pattern_min_dt_s;
    if (!restart && !pending)
    {
        // innovation gate: newest sample vs the filter's own prediction
        // (state is held at the last estimator update, so predict it
        // forward). dt-scaled allowance: after a reset the filter's v = 0,
        // so ANY genuinely moving target misses a bare-prediction gate and
        // would deadlock the estimator in restart-forever (reproduced in
        // test_tracker 1 Hz staircase, 2026-11-17; same deadlock at
        // satellite switch-over onto a fast target, which likely
        // contributed to the replay ISS-acquisition :K storm). The
        // allowance rate (2 deg/s) exceeds max apparent LEO rate (~1.2),
        // while real switch-overs land degrees away and still trip.
        const double ga = pat_ab_[0].pred_pos(dt_ab) + 0.5 * pat_a_ema_[0] * dt_ab * dt_ab;
        const double ge = pat_ab_[1].pred_pos(dt_ab) + 0.5 * pat_a_ema_[1] * dt_ab * dt_ab;
        const double res = std::hypot(hs_u.az_deg - ga, hs_u.el_deg - ge);
        if (res > cfg_.pattern_gate_deg + cfg_.pattern_gate_rate_deg_s * dt_ab)
            restart = true;
    }
    if (restart)
    {
        pat_ab_[0].reset(hs_u.az_deg);
        pat_ab_[1].reset(hs_u.el_deg);
        pat_a_ema_[0] = pat_a_ema_[1] = 0.0;
        pat_ab_ready_ = true;
        have_vel_ = false;
        last_ab_t_ = now;
    }
    else if (pending)
    {
        // freshness-only push: estimator state (and the derived filt_pos_,
        // pattern_vel_, pattern_acc2_ below) are the PREVIOUS update's —
        // exactly what a re-read of the same sky position should produce.
    }
    else
    {
        // slow acceleration channel: EMA of the velocity correction rate
        // (the alpha-beta v-step's own increment IS an accel sample:
        // dv = beta*r/dt, so a_inst = dv/dt), half-weighted into the
        // age^2 COEFFICIENT convention pattern_acc2_ carries (des +=
        // vel*age + acc2*age^2). tau = 6 pushes at the live 2 s cadence;
        // LEO az/el acceleration at the ground peaks ~0.006 deg/s^2, so
        // the clamp leaves a 3x engineering margin.
        constexpr double ACC_TAU_S = 12.0;
        constexpr double ACC_CLAMP = 0.02;  // deg/s^2, true accel magnitude
        for (int c = 0; c < 2; ++c)
        {
            const double z = (c == 0) ? hs_u.az_deg : hs_u.el_deg;
            ScalarABFilter& f = pat_ab_[c];
            const double v_prev = f.v;
            f.predict_update(dt_ab, z, cfg_.pattern_ab_alpha, cfg_.pattern_ab_beta);
            const double a_inst = (f.v - v_prev) / dt_ab;
            const double k = dt_ab / (ACC_TAU_S + dt_ab);
            pat_a_ema_[c] += (a_inst - pat_a_ema_[c]) * k;
            pat_a_ema_[c] = clampd(pat_a_ema_[c], -ACC_CLAMP, ACC_CLAMP);
        }
        last_ab_t_ = now;
    }
    last_push_t_ = now;
    {
        // filtered state (az, el, rates, slow accel) -> continuous axes
        const double pa = pat_ab_[0].p, pe = pat_ab_[1].p;
        const double va = pat_ab_[0].v, ve = pat_ab_[1].v;
        const double aa = pat_a_ema_[0], ae = pat_a_ema_[1];
        // age^2 COEFFICIENT convention (des += vel*age + acc2*age^2):
        // half the true acceleration, everywhere it is stored.
        pattern_hs_vel_.az_deg  = va;  pattern_hs_vel_.el_deg  = ve;
        pattern_hs_acc2_.az_deg = 0.5 * aa;
        pattern_hs_acc2_.el_deg = 0.5 * ae;
        auto A = [&](double az, double el) {
            return hs_to_axes(az, el, cfg_.site.lat_deg, cfg_.orientation);
        };
        constexpr double E = 0.005; // deg, central difference
        const AxisAngles A0  = A(pa, pe);
        const AxisAngles Arp = A(pa + E, pe), Arm = A(pa - E, pe);
        const AxisAngles Aep = A(pa, pe + E), Aem = A(pa, pe - E);
        const double dra_daz  = (Arp.ra_deg  - Arm.ra_deg)  / (2 * E);
        const double ddec_daz = (Arp.dec_deg - Arm.dec_deg) / (2 * E);
        const double dra_del  = (Aep.ra_deg  - Aem.ra_deg)  / (2 * E);
        const double ddec_del = (Aep.dec_deg - Aem.dec_deg) / (2 * E);
        // FILTERED setpoint base for the glide reference: raw quantized
        // pushes must not step ref_target_ (each 0.1 deg step is a conv
        // velocity spike; see the estimator comment on :K storms).
        filt_pos_ = A0;
        filt_pos_.ra_deg  += 360.0 * std::round((d.ra_deg  - filt_pos_.ra_deg) / 360.0);
        filt_pos_.dec_deg += 360.0 * std::round((d.dec_deg - filt_pos_.dec_deg) / 360.0);
        // Pattern velocity: Jacobian at the FILTERED point x filtered az/el
        // rates. (Tried and rejected 2026-11-17, live-replay A/B: replacing
        // this with axis-space chords of filt_pos_ + Holt double-EMA to
        // capture map curvature — the chord is the span-AVERAGE rate, and
        // any smoothing freezes in its half-span lag; with the accel-aware
        // reference (ref_advance) and the HIGH-band escape ramp the plain
        // Jacobian mapping is both more accurate AND cheaper on the wire:
        // replay :K 948 vs 1232-1690, test metrics equal or better.)
        pattern_vel_.ra_deg  = dra_daz * va + dra_del * ve;
        pattern_vel_.dec_deg = ddec_daz * va + ddec_del * ve;
        pattern_acc2_.ra_deg  = dra_daz  * (0.5 * aa) + dra_del  * (0.5 * ae);
        pattern_acc2_.dec_deg = ddec_daz * (0.5 * aa) + ddec_del * (0.5 * ae);
        have_vel_ = true;
#ifdef GLIDE_DEBUG
        printf("      push ab hs=(%.3f,%.3f) vel_hs=(%.4f,%.4f) "
               "acc_hs=(%.5f,%.5f) vel=(%.4f,%.4f)\n",
               pa, pe, va, ve, aa, ae,
               pattern_vel_.ra_deg, pattern_vel_.dec_deg);
#endif
    }

    // Glide reference bookkeeping: store the FILTERED sample (ref_advance
    // does the rest; filt_pos_ = the filter state at this push's time, see
    // the estimator block — raw quantized pushes must not step the
    // reference target). While glide is engaged, bring it into the
    // reference's own 360°-representation space by taking the turn NEAREST
    // the current reference point — RA is physically periodic, and the
    // per-update unwrapping above only bounds jumps across updates, it does
    // not know which turn the reference lives on.
    const double now_d =
        std::chrono::duration<double>(now.time_since_epoch()).count();
    AxisAngles f = have_vel_ ? filt_pos_ : d;
    if (have_ref_ && ref_flip_)   // glide lives in the flip branch space:
        f = flip_map(f, true, true);  // store the sample in ref space
    if (have_ref_) {
        f.ra_deg  += 360.0 * std::round((ref_pos_.ra_deg  - f.ra_deg) / 360.0);
        f.dec_deg += 360.0 * std::round((ref_pos_.dec_deg - f.dec_deg) / 360.0);
    }
    ref_target_ = f;
    // Anchor the reference's extrapolation clock to the FILTER update time,
    // not the push time: ref_advance projects ref_target_ forward by
    // pattern_vel, and that velocity (and the filter position itself) is
    // anchored at last_ab_t_. With a min-cadence hold in effect the push
    // anchor would silently subtract the pending interval from the glide
    // projection (a constant v x pending lag for sub-second clients).
    ref_target_time_ = have_vel_
        ? std::chrono::duration<double>(last_ab_t_.time_since_epoch()).count()
        : now_d;
    have_ref_target_ = true;

    Target t;
    t.az = az_deg;
    t.el = el_deg;
    t.t = now;
    t.valid = true;
    target_ = t;
    state_.target = HsCoord{az_deg, el_deg};
    safety_.note_client_command(now);
}

void Tracker::on_get_position(double& az_deg, double& el_deg)
{
    HsCoord hs = current_pointing(std::chrono::system_clock::now());
    az_deg = hs.az_deg;
    el_deg = hs.el_deg;
    // A `p` poll IS a rotctld command: the client is present, it is just
    // not pushing a target. GT2 polls every 2 s (rotor.js ROTOR_POLL_MS)
    // even while its follow throttle suppresses pushes — near-horizon
    // passes push only every 20-56 s at 1 deg/2 s, and LOS gaps run
    // minutes: feeding the absent watchdog on pushes ALONE tripped it
    // with GT2 connected and polling (live-session replay 2026-11-17).
    // Trade-off (documented): a monitor client keeps the transport
    // watchdog fed while the primary tracker is gone; target-stale (30 s)
    // and last-client-disconnect target-drop remain the guards there.
    std::lock_guard<std::mutex> lk(mu_);
    safety_.note_client_command(client_stamp());
}

void Tracker::on_stop()
{
    std::lock_guard<std::mutex> lk(mu_);
    try { board_.stop_all(true); } catch (...) {}
    any_axis_running_ = false;
    sent_ra_x_ = sent_de_x_ = 0;
    state_.halted = true;
    state_.halt_reason = "stopped by client";
    safety_.note_client_command(client_stamp());
    // Explicit stop/park (rotctld S/K) means STAY PUT: suppress the re-home
    // maneuver until the next real target push re-arms it. The el-gate is
    // DISARMED too: without that, an operator stop after a real pass plus
    // the next pass's el-0 pre-aim push (which clears rehome_done_) would
    // re-open the idle clock on the stale arm from the old pass.
    rehome_active_ = false;
    rehome_done_   = true;
    rehome_armed_  = false;
    home_no_route_ = false;
    rehome_idle_mono_ = 0;
}

void Tracker::on_park()
{
    on_stop();
    std::lock_guard<std::mutex> lk(mu_);
    target_ = Target{}; // forget target entirely: park = idle
    state_.halt_reason = "parked";
}

void Tracker::on_reset()
{
    std::lock_guard<std::mutex> lk(mu_);
    safety_.clear();
    // the operator pressing R IS the client: re-arm the transport watchdog
    // (clear() does not touch last_client_, so a stale stamp would
    // immediately re-trip the hold after an honest latch-clear)
    safety_.note_client_command(client_stamp());
    // R is the operator saying "clear and continue": re-arm the re-home
    // maneuver (latches had suppressed it), el-gate included — an
    // operator at the controls gets normal post-pass behavior back even
    // if a stop or a mid-LOS latch had disarmed it.
    rehome_done_ = false;
    rehome_armed_ = true;
    rehome_idle_mono_ = 0;
    have_prev_desired_ = false;
    have_prev_cur_ = false;
    have_vel_ = false; // re-derive setpoint velocity from next updates
    pat_ab_ready_ = false;   // pattern estimator restarts on next push
    have_upd_hs_ = false;
    have_ref_ = have_ref_target_ = glide_active_ = slide_latch_ = false;
    ref_flip_ = false;
    nudge_err_ = {};
    vel_seeded_ = false; // re-anchored mapping: rate history invalid
    dem_seeded_ = false; //  "   "  demand-filter history invalid
    skip_ra_ = skip_de_ = skip_exp_ra_ = skip_exp_de_ = 0;
    state_.halted = false;
    state_.halt_reason.clear();
}

// ---------------- Runner ----------------

Runner::Runner(Tracker& tracker) : tr_(tracker) {}

void Runner::start()
{
    running_ = true;
    th_ = std::thread(&Runner::loop, this);
}
void Runner::stop()
{
    running_ = false;
    if (th_.joinable())
        th_.join();
}
void Runner::loop()
{
    using namespace std::chrono;
    const auto period = duration_cast<steady_clock::duration>(duration<double>(1.0 / 10.0));
    auto next = steady_clock::now();
    while (running_)
    {
        next += period;
        tr_.step(system_clock::now(), steady_clock::now());
        auto now = steady_clock::now();
        if (next > now)
            std::this_thread::sleep_until(next);
        else
            next = now;
    }
}

} // namespace sateq
