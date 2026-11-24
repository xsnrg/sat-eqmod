#include "sateq/safety.h"

#include <cmath>

namespace sateq {

static double fold180(double v);

SafetyMonitor::SafetyMonitor(SafetyConfig cfg) : cfg_(cfg) {}

GuardDecision SafetyMonitor::check_pointing(double el_deg) const
{
    GuardDecision d;
    if (latched_)
    {
        d.result = GuardResult::Latched;
        d.reason = latch_reason_;
        return d;
    }
    if (el_deg < cfg_.min_elevation_deg)
    {
        d.result = GuardResult::Blocked;
        d.reason = "target below mechanical floor " + std::to_string((int)cfg_.min_elevation_deg) + " deg";
        return d;
    }
    return d;
}

GuardDecision SafetyMonitor::check_rate(double rate_x_sidereal) const
{
    GuardDecision d;
    if (latched_)
    {
        d.result = GuardResult::Latched;
        d.reason = latch_reason_;
        return d;
    }
    double absrate = std::fabs(rate_x_sidereal);
    if (absrate > cfg_.max_rate_x_sidereal)
    {
        d.result = GuardResult::Blocked;
        d.reason = "rate " + std::to_string(absrate) + "x exceeds safety max";
        return d;
    }
    return d;
}

void SafetyMonitor::set_reference(const AxisAngles& axes_now)
{
    ref_ = axes_now;
    has_ref_ = true;
    latched_ = false;
    latch_reason_.clear();
}

GuardDecision SafetyMonitor::check_cumulative(const AxisAngles& axes_now) const
{
    GuardDecision d;
    if (latched_)
    {
        d.result = GuardResult::Latched;
        d.reason = latch_reason_;
        return d;
    }
    if (!has_ref_)
        return d;
    double dra = std::fabs(axes_now.ra_deg - ref_.ra_deg);
    // DEC folds like the travel envelope (fold180 below, and the tracker's
    // box): the flip branch (a+180, 180-d) reaches DEC past +-180 while
    // pointing IDENTICALLY to the folded offset — live 2026-11-20, the
    // over-the-top route home reported DEC 270 (= -90 physically, tube at
    // home) and the unfolded compare false-latched at the fusion crossing.
    // The DEC cord-wrap backstop therefore bounds the FOLDED offset; the
    // feedline constraint that actually matters lives on RA (below).
    double dde = std::fabs(fold180(axes_now.dec_deg - ref_.dec_deg));
    if (dra > cfg_.max_ra_cumulative_deg)
    {
        d.result = GuardResult::Latched; // cable wrap is a latch: needs human
        d.reason = "RA cumulative rotation " + std::to_string((int)dra) +
                   "deg exceeds limit — manual unwrap required";
        return d;
    }
    if (dde > cfg_.max_dec_cumulative_deg)
    {
        d.result = GuardResult::Latched;
        d.reason = "DEC cumulative rotation " + std::to_string((int)dde) +
                   "deg exceeds limit — check counterweight/OTA";
        return d;
    }
    return d;
}

// Fold into (-180, 180]: axis offsets from the datum are taken the SHORT
// way round; a pose cannot leave the box without crossing a wall, so a
// fold can never mask a violation. (RA is physically periodic; DEC past
// +-180 is reached only by the flip branch (a+180, 180-d), which points
// IDENTICALLY to the folded offset — see check_cumulative.)
static double fold180(double v)
{
    while (v > 180.0) v -= 360.0;
    while (v < -180.0) v += 360.0;
    return v;
}

GuardDecision SafetyMonitor::check_travel(const AxisAngles& current,
                                          const AxisAngles& target,
                                          double margin) const
{
    GuardDecision d;
    if (latched_)
    {
        d.result = GuardResult::Latched;
        d.reason = latch_reason_;
        return d;
    }
    if (!has_ref_)
        return d;
    // Mechanical travel envelope: datum-relative per-axis limits
    // (max_{ra,dec}_travel_deg). This is STAND CLEARANCE — the tube
    // physically hits the stand — which the cumulative guards above
    // (cabling budget, 540/270 deg) never sees: live 2026-11-19 a GT2
    // pre-aim wound RA ~160 deg from the datum, the tube stopped at the
    // stand, and the stalled 600x slew pulled the 12 V rail down and
    // killed the serial link. Non-latching BY DESIGN (unlike the wrap
    // guard): a refusal must not poison later legal targets; the caller
    // returns home and any new legal command resumes motion.
    // limit <= 0 disables the axis; >= 180 is the whole travel range.
    auto limit = [&](double lim) {
        if (lim <= 0.0) return 0.0;   // 0 => disabled (caller skips)
        return std::max(0.0, std::min(lim, 180.0) - margin);
    };
    auto off = [&](double now, double ref, double lim, double& out) {
        double o = std::fabs(fold180(now - ref));
        out = (o > lim) ? o : 0.0;
    };
    // current-pose violation first: we are ALREADY against the stand,
    // the mount must leave no matter what the target says.
    if (cfg_.max_ra_travel_deg > 0.0)
    {
        double lim = limit(cfg_.max_ra_travel_deg), o = 0.0;
        off(current.ra_deg, ref_.ra_deg, lim, o);
        if (o > 0.0)
        {
            d.result = GuardResult::Blocked;
            d.reason = "RA travel envelope: current pose " + std::to_string((int)o) +
                       "deg from datum exceeds limit " +
                       std::to_string((int)cfg_.max_ra_travel_deg) + "deg";
            return d;
        }
        off(target.ra_deg, ref_.ra_deg, lim, o);
        if (o > 0.0)
        {
            d.result = GuardResult::Blocked;
            d.reason = "RA travel envelope: target " + std::to_string((int)o) +
                       "deg from datum exceeds limit " +
                       std::to_string((int)cfg_.max_ra_travel_deg) + "deg";
            return d;
        }
    }
    if (cfg_.max_dec_travel_deg > 0.0)
    {
        double lim = limit(cfg_.max_dec_travel_deg), o = 0.0;
        off(current.dec_deg, ref_.dec_deg, lim, o);
        if (o > 0.0)
        {
            d.result = GuardResult::Blocked;
            d.reason = "DEC travel envelope: current pose " + std::to_string((int)o) +
                       "deg from datum exceeds limit " +
                       std::to_string((int)cfg_.max_dec_travel_deg) + "deg";
            return d;
        }
        off(target.dec_deg, ref_.dec_deg, lim, o);
        if (o > 0.0)
        {
            d.result = GuardResult::Blocked;
            d.reason = "DEC travel envelope: target " + std::to_string((int)o) +
                       "deg from datum exceeds limit " +
                       std::to_string((int)cfg_.max_dec_travel_deg) + "deg";
            return d;
        }
    }
    return d;
}

void SafetyMonitor::trip(const std::string& reason)
{
    latched_ = true;
    latch_reason_ = reason;
}

void SafetyMonitor::clear()
{
    latched_ = false;
    latch_reason_.clear();
    client_tripped_ = board_tripped_ = false;
}

void SafetyMonitor::note_client_command(std::chrono::steady_clock::time_point t)
{
    last_client_ = t;
    client_seen_ = true;
    client_tripped_ = false;   // the returning client clears the hold
}

void SafetyMonitor::note_board_ok(std::chrono::steady_clock::time_point t)
{
    last_board_ = t;
}

GuardDecision SafetyMonitor::check_watchdog(std::chrono::steady_clock::time_point t,
                                            bool motors_may_be_running) const
{
    GuardDecision d;
    if (!motors_may_be_running)
        return d;
    if (client_seen_)
    {
        double dt = std::chrono::duration<double>(t - last_client_).count();
        if (dt > cfg_.watchdog_timeout_s && !client_tripped_)
        {
            d.result = GuardResult::Latched;
            d.reason = "watchdog: no rotctld command for " + std::to_string((int)dt) + "s";
            client_tripped_ = true;
            watchdog_reason_ = d.reason;
            return d;
        }
    }
    if (last_board_ != std::chrono::steady_clock::time_point{})
    {
        double dt = std::chrono::duration<double>(t - last_board_).count();
        if (dt > cfg_.serial_stale_s && !board_tripped_)
        {
            d.result = GuardResult::Latched;
            d.reason = "watchdog: no board I/O for " + std::to_string((int)dt) + "s";
            board_tripped_ = true;
            watchdog_reason_ = d.reason;
            return d;
        }
    }
    return d;
}

} // namespace sateq
