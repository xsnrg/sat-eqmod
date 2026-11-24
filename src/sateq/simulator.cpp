#include "sateq/simulator.h"
#include "sateq/byteorder.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace sateq {

namespace {
constexpr double MICROSECONDS = 1e6;
constexpr uint32_t U24_MAX    = 0xFFFFFF;

std::string u24_frame(uint32_t n)
{
    char buf[7];
    revu24_format(n & U24_MAX, buf);
    return std::string("=") + buf;
}
} // namespace

// INTERNAL status bit layout — sim physics bookkeeping only. The ":f" wire
// response is built from these in live-board digit form (see case 'f').
enum SimBits : unsigned {
    S_INITIALIZED = 0x0100, // armed with :F since (sim) power-on; :J needs it
    S_RUNNING     = 0x0001,
    S_SLEW        = 0x0010,
    S_BACKWARD    = 0x0020,
    S_HIGHSPEED   = 0x0040,
};

SimulatorTransport::SimulatorTransport(SimProfile profile)
    : prof_(std::move(profile))
{
    now_ = std::chrono::steady_clock::now();
    for (AxisSim* a : {&ax1_, &ax2_})
    {
        a->position   = 0x80; // live Sirius counters read 000080 at connect
        a->period_us  = 598;  // vendor sim default 0x256
        a->last_tick  = now_;
        recenter_play(*a);
        // AxisSim default status: S_SLEW set, INIT clear — live boards report
        // "=100" at power-on and NAK ":J" with code 4 until ":F" arms them.
    }
}

void SimulatorTransport::open()
{
    std::lock_guard<std::mutex> lk(mu_);
    open_ = true;
}
void SimulatorTransport::close()
{
    std::lock_guard<std::mutex> lk(mu_);
    open_ = false;
}

void SimulatorTransport::write(const std::string& bytes)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (!open_)
        throw BoardError(BoardError::Serial, "simulator closed");
    rx_ += bytes;
    size_t pos;
    while ((pos = rx_.find('\r')) != std::string::npos)
    {
        std::string cmd = rx_.substr(0, pos);
        rx_.erase(0, pos + 1);
        if (cmd.empty())
            continue; // stray CR noise, like the real link tolerates
        log_.push_back(cmd);
        // fault injection first: consume the command without a real response
        auto sit = silence_pending_.find(cmd[1]);
        if (sit != silence_pending_.end() && sit->second > 0)
        {
            sit->second--;
            continue;
        }
        auto nit = nak_pending_.find(cmd[1]);
        if (nit != nak_pending_.end() && nit->second > 0)
        {
            nit->second--;
            out_.push_back('!');
            out_.push_back('0');
            out_.push_back('\r');
            continue;
        }
        std::string resp = handle(cmd);
        auto tit = trunc_pending_.find(cmd[1]);
        if (tit != trunc_pending_.end() && tit->second > 0)
        {
            tit->second--;
            if (resp.size() > 1)
                resp.pop_back();
            if (resp.size() > 1)
                resp.pop_back();
        }
        for (char c : resp)
            out_.push_back(c);
        out_.push_back('\r');
    }
}

std::optional<std::string> SimulatorTransport::read_frame(int /*timeout_ms*/)
{
    std::lock_guard<std::mutex> lk(mu_);
    if (out_.empty())
        return std::nullopt; // instant: tests never sleep waiting for timeouts
    ResponseAccumulator acc;
    std::optional<std::string> frame;
    while (!out_.empty())
    {
        char c = out_.front();
        out_.pop_front();
        if (auto f = acc.feed(static_cast<unsigned char>(c)))
        {
            frame = f;
            break; // one frame per read, like the byte-wise serial path
        }
    }
    return frame;
}

void SimulatorTransport::flush_input()
{
    std::lock_guard<std::mutex> lk(mu_);
    out_.clear();
}

std::vector<std::string> SimulatorTransport::command_log() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return log_;
}
std::vector<std::string> SimulatorTransport::command_log_tail(size_t offset) const
{
    std::lock_guard<std::mutex> lk(mu_);
    if (offset >= log_.size())
        offset = 0;
    return {log_.begin() + static_cast<long>(offset), log_.end()};
}
void SimulatorTransport::clear_log()
{
    std::lock_guard<std::mutex> lk(mu_);
    log_.clear();
}

void SimulatorTransport::advance(double seconds)
{
    std::lock_guard<std::mutex> lk(mu_);
    tick();
    auto d = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(seconds));
    now_ += d;
    tick();
}

void SimulatorTransport::set_wall_clock(bool on)
{
    std::lock_guard<std::mutex> lk(mu_);
    wall_clock_ = on;
    if (!on)
        return;
    // Re-anchor the virtual clock and every ramp/coast stamp to "now" so
    // switching mid-session cannot teleport axes through a huge elapsed gap.
    now_ = std::chrono::steady_clock::now();
    ax1_.last_tick = ax2_.last_tick = now_;
    ax1_.accel_from = ax1_.blind_from = ax1_.slow_from = now_;
    ax2_.accel_from = ax2_.blind_from = ax2_.slow_from = now_;
}

uint32_t SimulatorTransport::sim_position(char axis)
{
    std::lock_guard<std::mutex> lk(mu_);
    tick();
    return static_cast<uint32_t>((axis == '1' ? ax1_ : ax2_).position) & U24_MAX;
}

bool SimulatorTransport::sim_running(char axis)
{
    std::lock_guard<std::mutex> lk(mu_);
    tick();
    return (((axis == '1' ? ax1_ : ax2_).status & S_RUNNING) != 0);
}

// ---- backlash play model ---------------------------------------------------
// Classical play operator with half-width h = S/2 on the unwrapped counter:
//   phys = clamp(phys, unwrapped − h, unwrapped + h)
// Sustained motion saturates the counter-vs-physical lag at ±h (the clamp
// edge drags phys 1:1); a reversal holds phys while the counter backs through
// the full S = 2h, then phys tracks 1:1 from the other flank. h == 0 clamps
// to a point: phys == unwrapped bit-exactly, so an unset axis (every pre-
// existing test) sees the counter itself.
void SimulatorTransport::sync_play(AxisSim& a)
{
    const double c24 = (double)U24_MAX + 1.0;
    double d = std::fmod(a.position - a.prev_pos + c24 * 16.0, c24);
    if (d > c24 * 0.5) d -= c24; // signed delta in (−2^23, 2^23]
    a.unwrapped += d;
    a.prev_pos = a.position;
    const double h = a.backlash * 0.5;
    if (h <= 0.0)
        a.phys = a.unwrapped;
    else
        a.phys = std::min(std::max(a.phys, a.unwrapped - h), a.unwrapped + h);
}

void SimulatorTransport::recenter_play(AxisSim& a)
{
    a.unwrapped = a.position;
    a.phys      = a.position;
    a.prev_pos  = a.position;
}

void SimulatorTransport::set_backlash(char axis, double slop_steps)
{
    std::lock_guard<std::mutex> lk(mu_);
    AxisSim& a = (axis == '2') ? ax2_ : ax1_;
    a.backlash = slop_steps > 0.0 ? slop_steps : 0.0;
    recenter_play(a); // slop unknown until now: assume mesh centered
}

double SimulatorTransport::sim_physical(char axis)
{
    std::lock_guard<std::mutex> lk(mu_);
    tick();
    const double c24 = (double)U24_MAX + 1.0;
    // truncate like the :j wire (uint counter): with backlash 0 this is
    // bit-identical to sim_position()/Board::position.
    const double w = std::fmod((axis == '1' ? ax1_ : ax2_).phys + c24, c24);
    return static_cast<double>(static_cast<uint32_t>(w) & U24_MAX);
}

void SimulatorTransport::fail_next(char cmd_char, int n)
{
    std::lock_guard<std::mutex> lk(mu_);
    nak_pending_[cmd_char] += n;
}
void SimulatorTransport::silence_next(char cmd_char, int n)
{
    std::lock_guard<std::mutex> lk(mu_);
    silence_pending_[cmd_char] += n;
}

void SimulatorTransport::truncate_next(char cmd_char, int n)
{
    std::lock_guard<std::mutex> lk(mu_);
    trunc_pending_[cmd_char] += n;
}

void SimulatorTransport::set_stalled(char axis, bool stalled)
{
    std::lock_guard<std::mutex> lk(mu_);
    (axis == '1' ? ax1_ : ax2_).stalled = stalled;
}

void SimulatorTransport::clear_armed()
{
    std::lock_guard<std::mutex> lk(mu_);
    ax1_.status &= ~S_INITIALIZED;
    ax2_.status &= ~S_INITIALIZED;
}

void SimulatorTransport::set_accel(char axis, double seconds)
{
    std::lock_guard<std::mutex> lk(mu_);
    AxisSim& a = axis == '1' ? ax1_ : ax2_;
    a.accel_s = seconds;
    a.accel_from = now_;
}

void SimulatorTransport::set_slow_ramp(char axis, double seconds, double start_ratio,
                                       double min_x_x)
{
    std::lock_guard<std::mutex> lk(mu_);
    AxisSim& a = axis == '1' ? ax1_ : ax2_;
    a.slow_ramp_s = seconds;
    a.slow_ramp_floor = start_ratio;
    a.slow_ramp_min_x = min_x_x;
}

void SimulatorTransport::set_restart_blind(char axis, double seconds)
{
    std::lock_guard<std::mutex> lk(mu_);
    AxisSim& a = axis == '1' ? ax1_ : ax2_;
    a.restart_blind_s = seconds;
    a.blind_armed = a.blind_active = false;
}

void SimulatorTransport::set_cold_blind(char axis, double seconds)
{
    std::lock_guard<std::mutex> lk(mu_);
    AxisSim& a = axis == '1' ? ax1_ : ax2_;
    a.cold_blind_s = seconds;
}

void SimulatorTransport::set_coast(char axis, double seconds)
{
    std::lock_guard<std::mutex> lk(mu_);
    AxisSim& a = axis == '1' ? ax1_ : ax2_;
    a.coast_s    = seconds;
    a.coast_vel0 = 0;
}

// caller must hold mu_
void SimulatorTransport::tick()
{
    if (wall_clock_)
        now_ = std::chrono::steady_clock::now();
    for (AxisSim* a : {&ax1_, &ax2_})
    {
        double elapsed_us = std::chrono::duration<double, std::micro>(now_ - a->last_tick).count();
        if (elapsed_us <= 0)
        {
            a->last_tick = now_;
            continue;
        }
        if ((a->status & S_RUNNING) && !a->stalled)
        {
            unsigned stepmul = (a->status & S_HIGHSPEED) ? prof_.hs_ratio : 1;
            // vendor physics: period ticks at (wormperiod*1e6/steps_worm) us
            // per step, scaled by hs_ratio in high-speed mode.
            unsigned mul = static_cast<unsigned>(MICROSECONDS / prof_.steps_worm);
            unsigned rem = static_cast<unsigned>(MICROSECONDS) % prof_.steps_worm;
            double timer_us = a->period_us * static_cast<double>(mul) +
                              (a->period_us * static_cast<double>(rem)) / prof_.steps_worm;
            double deltastep = elapsed_us * stepmul / timer_us;
            // optional issuance-accel ramp (set_accel): counters follow
            // issuance, so they lag the commanded rate during the ramp
            if (a->accel_s > 0.0)
            {
                double age = std::chrono::duration<double>(now_ - a->accel_from).count();
                if (age < a->accel_s)
                    deltastep *= std::max(0.0, age) / a->accel_s;
            }
            // :K-restart starvation (set_restart_blind): zero issuance for
            // the coast+re-accel window after the first :J that follows a
            // :K on a running axis (sweep-6: ~2 s at HIGH, ~1 deg lost).
            if (a->blind_active)
            {
                double age = std::chrono::duration<double>(now_ - a->blind_from).count();
                if (age < a->blind_dur_s)
                    deltastep = 0;
                else
                    a->blind_active = false;
            }
            // Slow-ramp LOW band (Phase-1 Sirius): LOW programs commanding
            // [min_x, 104x) issue ~0.8-0.9 of commanded at band entry,
            // converging over tens of seconds. The transient clock restarts
            // every time the program ENTERS the band (:J from stopped or an
            // in-place :I arrival — live sweep 2026-11-17 starved ~10 % for
            // ~20 s after an in-place entry at ~60x).
            if (a->slow_ramp_s > 0.0 && !(a->status & S_HIGHSPEED) && timer_us > 0.0)
            {
                const double x_sid =
                    (MICROSECONDS / timer_us) * STELLAR_DAY_SEC /
                    static_cast<double>(prof_.steps_360);
                const bool in_band =
                    x_sid >= a->slow_ramp_min_x && x_sid < 104.0;
                if (in_band && !a->slow_active)
                {
                    a->slow_active = true;
                    a->slow_from   = now_;
                }
                else if (!in_band)
                    a->slow_active = false;
                if (a->slow_active)
                {
                    double age =
                        std::chrono::duration<double>(now_ - a->slow_from).count();
                    if (age < a->slow_ramp_s)
                        deltastep *= a->slow_ramp_floor +
                                     (1.0 - a->slow_ramp_floor) * age / a->slow_ramp_s;
                }
            }
            bool backward    = (a->status & S_BACKWARD) != 0;

            if (!(a->status & S_SLEW) && a->goto_active)
            {
                // GOTO: clamp motion at absolute target
                double cur = a->position;
                double tgt = static_cast<double>(a->goto_target);
                // 24-bit shortest? No: motion is in commanded direction only.
                double dist = backward ? (cur >= tgt ? cur - tgt : (cur + (double)U24_MAX + 1 - tgt))
                                       : (tgt >= cur ? tgt - cur : ((double)U24_MAX + 1 - cur + tgt));
                if (deltastep >= dist)
                {
                    a->position = tgt;
                    a->status &= ~S_RUNNING;
                    a->goto_active = false;
                }
                else
                    a->position = backward ? cur - deltastep : cur + deltastep;
            }
            else
            {
                a->position = backward ? a->position - deltastep : a->position + deltastep;
            }
            // 24-bit counter wrap, like the board
            a->position = std::fmod(a->position + (double)U24_MAX + 1, (double)U24_MAX + 1);
        }
        else if (a->coast_vel0 != 0.0 && !a->stalled)
        {
            // :K coast (set_coast): the board's step timer winds down after
            // the stop command and the counter keeps advancing — exactly
            // what :j reports on metal. v(t) = v0·(1 − t/T), T = 2·coast_s:
            // total displacement v0·coast_s, matching the tracker's
            // high_coast_s landing model. Integrated CLOSED-FORM over the
            // tick span (a single advance() larger than the whole coast
            // window must still produce the full displacement).
            const double T = 2.0 * a->coast_s;
            const double dt = elapsed_us * 1e-6;
            // age measured at the START of this tick span (now_ is the
            // tick END); the coast may live entirely inside one big advance()
            const double age =
                std::chrono::duration<double>(now_ - a->coast_from).count() - dt;
            if (age >= T || dt <= 0.0)
            {
                a->coast_vel0 = 0;
            }
            else
            {
                const double t0 = age > 0.0 ? age : 0.0;
                const double t1 = std::min(age + dt, T);
                const double v0 = a->coast_vel0;
                if (t1 > t0)
                {
                    // ∫ v0(1 − t/T) dt from t0 to t1
                    a->position += v0 * ((t1 - t0) - (t1 * t1 - t0 * t0) / (2.0 * T));
                    a->position = std::fmod(a->position + (double)U24_MAX + 1,
                                            (double)U24_MAX + 1);
                }
                if (t1 >= T)
                    a->coast_vel0 = 0;
            }
        }
        sync_play(*a);
        a->last_tick = now_;
    }
}

// Build the response frame for one complete command (no CR). mu_ held.
std::string SimulatorTransport::handle(const std::string& cmd)
{
    tick();
    if (cmd.size() < 3 || cmd[0] != ':')
        return "!";
    char c = cmd[1];
    char ax = cmd[2];
    if (ax != '1' && ax != '2' && !(c == 'e' && ax == '1'))
    {
        // :e is axis-agnostic on real boards but probed on 1; :P/:V/:O accept args
        if (c != 'P' && c != 'V' && c != 'O')
            return "!0";
    }
    AxisSim& a = (ax == '2') ? ax2_ : ax1_;

    auto arg_u24 = [&](size_t off) -> uint32_t {
        if (cmd.size() < off + 6)
            throw std::runtime_error("sim: short arg for " + cmd);
        return revu24_parse(cmd.data() + off);
    };

    switch (c)
    {
        case 'e': return "=" + prof_.version_payload;
        case 'a': return u24_frame(prof_.steps_360);
        case 'b': return u24_frame(prof_.steps_worm);
        case 'g': return "=10"; // 16, matches both boards as plain 2-hex
        case 'D': return u24_frame(a.period_us);
        case 'f':
        {
            // ":f" wire digits — LIVE 2026-11-16 (controlled Sirius tests):
            // power-on idle "100" (or "700" with stale mode bits), after :F
            // idle "101", running fwd "111", back "311", hi fwd "511", hi
            // back "711"; :K from a backward run leaves idle "301" — digit1
            // (mode) PERSISTS across stops, digit3 = :F-initialized flag.
            unsigned s = a.status;
            char buf[4];
            buf[0] = char('0' | ((s & S_SLEW)      ? 1 : 0) |
                               ((s & S_BACKWARD)   ? 2 : 0) |
                               ((s & S_HIGHSPEED)  ? 4 : 0));
            buf[1] = char('0' | ((s & S_RUNNING)     ? 1 : 0));
            buf[2] = char('0' | ((s & S_INITIALIZED) ? 1 : 0));
            return std::string("=") + buf;
        }
        case 'j': return u24_frame(static_cast<uint32_t>(a.position));
        case 'E':
            // absolute counter write: physical follows, contact unknown
            a.position = static_cast<double>(arg_u24(3));
            recenter_play(a);
            return "=";
        case 'F':
            if (ax == '1' || ax == '3') ax1_.status |= S_INITIALIZED;
            if (ax == '2' || ax == '3') ax2_.status |= S_INITIALIZED;
            return "=";
        case 'B': return "="; // sleep/wake: ACK (never sent by sateqd)
        case 'I':
            // LIVE: accepted while running in LOW speed (immediately
            // effective); NAK 2 while running in high speed
            if ((a.status & S_RUNNING) && (a.status & S_HIGHSPEED))
                return "!2";
            a.period_us = arg_u24(3);
            return "=";
        case 'G':
        {
            if (cmd.size() < 5) return "!0";
            // LIVE: :G is NAK 2 while the axis runs, even for an identical
            // program; the host must :K and wait for the running bit first
            if (a.status & S_RUNNING)
                return "!2";
            unsigned mode = static_cast<unsigned>(hex_nibble(cmd[3]));
            unsigned dir  = static_cast<unsigned>(hex_nibble(cmd[4]));
            a.status &= ~(S_SLEW | S_BACKWARD | S_HIGHSPEED);
            if (dir & 1) a.status |= S_BACKWARD;
            if (mode == 0) a.status |= S_HIGHSPEED;              // goto high
            else if (mode == 1) { /* slew low: both clear */ }
            else if (mode == 2) { /* goto low */ }
            else if (mode == 3) a.status |= (S_SLEW | S_HIGHSPEED);
            if (mode == 0 || mode == 2)
                ; // goto: SLEW stays clear
            else
                a.status |= S_SLEW;
            return "=";
        }
        case 'J':
            // LIVE: NAK code 4 = axis not armed (":F" never sent since board
            // power-on). Arms are per-axis; :G and :I still ACK unarmed.
            if (!(a.status & S_INITIALIZED))
                return "!4";
            {
                const bool was_running = (a.status & S_RUNNING) != 0;
                a.status |= S_RUNNING;
                a.coast_vel0 = 0; // :J program wins over a running coast
                a.accel_from = now_; // issuance ramp restarts at every :J
                if (a.restart_blind_s > 0 && a.blind_armed)
                {
                    // first :J after a :K-on-running: board coasts + re-accelerates
                    a.blind_armed  = false;
                    a.blind_active = true;
                    a.blind_dur_s  = a.restart_blind_s;
                    a.blind_from   = now_;
                }
                else if (!was_running && a.cold_blind_s > 0.0)
                {
                    // set_cold_blind: a :J from STOPPED starves issuance —
                    // models the AO-123 cold-dive delivery (0.36 at t=2 s).
                    a.blind_active = true;
                    a.blind_dur_s  = a.cold_blind_s;
                    a.blind_from   = now_;
                }
            }
            if (!(a.status & S_SLEW) && a.goto_set)
            {
                a.goto_active = true;
            }
            return "=";
        case 'K': // non-instant stop: captures the coast when modeled (below)
        case 'L':
            // live boards clear only the running bit on stop: mode digits
            // (digit1) persist — e.g. ":K" after a backward run idles at "301"
            if (a.status & S_RUNNING) a.blind_armed = true;
            if (c == 'K' && a.coast_s > 0.0 && (a.status & S_RUNNING) && !a.stalled)
            {
                // capture the issuance rate at the :K instant (same formula
                // as tick()); the counter then decays to rest over 2·coast_s
                unsigned stepmul = (a.status & S_HIGHSPEED) ? prof_.hs_ratio : 1;
                unsigned mul = static_cast<unsigned>(MICROSECONDS / prof_.steps_worm);
                unsigned rem = static_cast<unsigned>(MICROSECONDS) % prof_.steps_worm;
                double timer_us = a.period_us * static_cast<double>(mul) +
                                  (a.period_us * static_cast<double>(rem)) / prof_.steps_worm;
                double v = timer_us > 0.0 ? 1e6 * static_cast<double>(stepmul) / timer_us : 0.0;
                a.coast_vel0 = (a.status & S_BACKWARD) ? -v : v;
                a.coast_from = now_;
            }
            else
                a.coast_vel0 = 0; // :L is instant; :K unmodeled/idle = stop
            a.status &= ~S_RUNNING;
            a.goto_active = false;
            return "=";
        case 'H': // goto increment (vendor sim semantics: absolute target count)
            a.goto_target = arg_u24(3);
            a.goto_set = true;
            return "=";
        case 'S': // absolute goto target (real boards)
            a.goto_target = arg_u24(3);
            a.goto_set = true;
            return "=";
        case 'M': // break-point increment
        case 'U': // break step
            return "=";
        case 'P': return "="; // guide rate
        case 'V': return "="; // polar LED
        case 'd':
            if (!prof_.supports_home) return "!0";
            return u24_frame(prof_.home_count);
        case 'q':
            if (!prof_.eq8_features) return "!0";
            return "=050000"; // matches HDX accepted feature word 0x05
        case 'W':
            if (!prof_.eq8_features) return "!0";
            return "=";
        case 'O':
            if (!prof_.eq8_features) return "!0";
            return "=";
        default:
            return "!"; // unknown: bare NAK like the vendor simulator
    }
}

} // namespace sateq
