// In-process emulator of the SkyWatcher/Synta motor board behind the
// Transport interface. Behavior transcribed from indi-eqmod's
// simulator/skywatcher-simulator.cpp plus the live board traces recorded
// 2026-11-15 (Sirius =020401 HEQ5, HDX =020B04 EQ8). Positions animate on a
// virtual clock (advance()) so motion/safety logic is unit-testable without
// sleeping or hardware.
#pragma once
#include "sateq/board.h"

#include <chrono>
#include <deque>
#include <map>
#include <mutex>
#include <vector>

namespace sateq {

struct SimProfile {
    std::string name = "HEQ5";
    std::string version_payload = "020401"; // exact :e payload, incl. mount code
    uint32_t steps_360  = 9024000; // live Sirius :a "00B289" nibble-pair-swapped
    uint32_t steps_worm = 64935;   // live Sirius :b "A7FD00" nibble-pair-swapped
    uint32_t hs_ratio   = 16;      // live :g "10"
    bool eq8_features   = false;
    bool supports_home  = false;
    uint32_t home_count = 0x7BAA; // arbitrary; EQ8 boards report an encoder count
};

inline SimProfile sim_heq5() { return SimProfile{}; }
inline SimProfile sim_eq8()
{
    SimProfile p;
    p.name          = "EQ8";
    p.version_payload = "020B04"; // live HDX-110 payload
    p.eq8_features  = true;
    p.supports_home = true;
    return p;
}

class SimulatorTransport : public Transport {
public:
    explicit SimulatorTransport(SimProfile profile = sim_heq5());

    void open() override;
    void close() override;
    bool is_open() const override { return open_; }
    void write(const std::string& bytes) override;
    std::optional<std::string> read_frame(int timeout_ms) override;
    void flush_input() override;

    // Test inspection / injection -------------------------------------
    std::vector<std::string> command_log() const; // e.g. ":I1002D39"
    // Entries from offset on (whole log if offset > size, e.g. after
    // clear_log); avoids O(n) copies per poll in long replays.
    std::vector<std::string> command_log_tail(size_t offset) const;
    void clear_log();
    void advance(double seconds);        // move virtual clock, animate axes
    // Wall-clock mode: tick() stamps now_ from steady_clock — lets a REAL-TIME
    // driver (the eqtool --sim wizard) exercise the sim without a manual
    // advance() pump: real sleeps produce real simulated motion.
    void set_wall_clock(bool on);
    uint32_t sim_position(char axis);    // animated position, no wire roundtrip
    bool sim_running(char axis);         // motor-running bit, no roundtrip
    void fail_next(char cmd_char, int n = 1);    // NAK "!0" next n commands
    void silence_next(char cmd_char, int n = 1); // no reply (timeout path)
    // reply with the frame's last two chars dropped (valid ACK, short body):
    // reproduces live wire byte-loss, e.g. "=1008\r" for a :j position
    void truncate_next(char cmd_char, int n = 1);
    // Mechanical jam: axis keeps its running bit and answers commands, but the
    // step counter does not advance — models "board commands, nothing moves".
    void set_stalled(char axis, bool stalled);
    // Model the board losing power and coming back: axes disarm (:J NAKs 4
    // again until :F). Positions persist here; a real board also resets
    // counters, which resume-state tests cover separately.
    void clear_armed();
    // Model board issuance acceleration (live Sirius 2026-11-16: after :J the
    // step-issue rate ramps; the :j counters follow issuance, so they lag the
    // commanded rate during the ramp — ~1 s in high speed). While running,
    // issuance scales linearly 0->1 over `seconds` since :J. 0 = ideal.
    void set_accel(char axis, double seconds);
    // Slow-ramp LOW band (Phase-1 Sirius 2026-11-15/16): a LOW-speed program
    // commanding [min_x_x, 104x) sidereal delivers only `start_ratio`
    // (~0.8-0.9 measured at 80-100x) of the commanded issuance at band entry
    // and converges to 1.0 over `seconds` (~25 s); the ramp clock restarts
    // each time the program ENTERS the band (via :J or an in-place :I — the
    // first live Phase-4 sweep 2026-11-17 starved ~10 % for ~20 s on an
    // in-place arrival). >=104x programs HIGH (covered by set_accel).
    // Phase-1 brackets the physical lower edge between 60x (delivered 1.018)
    // and 80x (0.902), but the live sweep latched at ~62x -> min_x_x is
    // injectable; default 70 keeps the model inside hard Phase-1 evidence.
    void set_slow_ramp(char axis, double seconds, double start_ratio = 0.85,
                       double min_x_x = 70.0);
    // :K-restart starvation (live sweep-6 2026-11-18): after a :K/:L that
    // catches a RUNNING axis, the NEXT :J delivers ZERO issuance for
    // `seconds` — the real board free-wheels its deceleration tail and then
    // re-accelerates (~2 s at HIGH; coast alone ~1 deg at 600x). Issuance
    // resumes at full commanded rate when the window closes. The running
    // bit is NOT held during the window (the sim still stops immediately;
    // only the counter effect is modeled).
    void set_restart_blind(char axis, double seconds);
    // COLD-start issuance blind (live AO-123 acquisition 2026-11-20 03:23):
    // the FIRST MOTION after the board has idled is far more starved than
    // Phase-1's consecutive warm runs suggested (measured delivered 0.36 of
    // commanded at t=2.0 s of a cold +600x dive, vs that profile's 0.67) —
    // dead issuance for `seconds`, then commanded rate resumes (the board's
    // HIGH accel itself is ~1 s, Phase-1). Arms at a :J on an axis that was
    // NOT running (a cold start; a :K-restart uses set_restart_blind
    // instead). 0 = ideal (cold == warm).
    void set_cold_blind(char axis, double seconds);
    // HIGH-speed :K deceleration-coast model (set_coast(axis, s)): :K on a
    // RUNNING axis keeps advancing the counter as the board's step timer
    // winds down — displacement |rate|·s, linear velocity decay over 2·s
    // (calibrated to the live Sirius 600x tail: ~1.0 s, ~18–19k steps, so
    // s ≈ 0.3 reproduces the ~0.74° coast). :L stays instant (it is on
    // metal); :J during a coast overrides it. The running bit clears
    // immediately as before (board-layer wait_running_clear semantics of
    // every existing test unchanged); the coast moves only the COUNTER,
    // which is exactly what :j reports on metal. 0 (default) = instant
    // stop, bit-exact pre-fixture behavior. This fixture closes the replay
    // blind spot that hid the AO-73 coast-pumped ping-pong (sim could not
    // see that every HIGH flip spends ~1 deg of coast — the tracker's
    // high_coast_s landing model and this fixture must agree).
    void set_coast(char axis, double seconds);
    // Gear-mesh slop (backlash) play model: set_backlash(axis, S) inserts
    // S ISSUED steps of lost motion per direction reversal between the
    // :j-visible counter and sim_physical(), the hidden DELIVERED position.
    // Model = symmetric play, half-width h = S/2: sustained motion lags the
    // counter by h; after a reversal physical HOLDS while the counter backs
    // through the full S, then tracks 1:1 on the other flank. 0 (default) =
    // ideal: physical IS the counter, bit-exact — no existing test sees it.
    void set_backlash(char axis, double slop_steps);
    double sim_physical(char axis);  // delivered position (wrapped), no roundtrip

private:
    struct AxisSim {
        double position  = 0x80;   // live counters read 000080 after connect
        unsigned status  = 0x0010; // S_SLEW set, INIT clear: live boards idle-report "=100"
        uint32_t period_us = 598;  // vendor sim default 0x256
        std::chrono::steady_clock::time_point last_tick{};
        uint32_t goto_target = 0;
        bool goto_set    = false;
        bool goto_active = false;
        bool stalled     = false; // jam: powered but counter frozen
        double accel_s   = 0;     // >0: issuance ramp duration after :J (set_accel)
        double slow_ramp_s = 0;   // >0: slow-ramp band model (set_slow_ramp)
        double slow_ramp_floor = 0.85; // ratio delivered at band entry
        double slow_ramp_min_x = 70.0; // band lower edge (x sidereal)
        bool slow_active = false;      // in-band edge detection (tick)
        // restart-blind state (set_restart_blind): armed by :K on a running
        // axis, consumed by the next :J; issuance zero until blind_from+blind_s
        double restart_blind_s = 0;
        double cold_blind_s    = 0; // set_cold_blind: zero issuance after a
                                     // cold (not-running) :J
        bool blind_armed  = false;
        bool blind_active = false;
        double blind_dur_s = 0;     // active window length (restart or cold)
        std::chrono::steady_clock::time_point blind_from{};
        // :K coast model (set_coast): coast_vel0 = signed steps/s at the
        // :K instant; counter integrates v0*(1 − t/(2·coast_s)) to zero.
        double coast_s = 0;
        double coast_vel0 = 0;
        std::chrono::steady_clock::time_point coast_from{};
        std::chrono::steady_clock::time_point slow_from{};
        std::chrono::steady_clock::time_point accel_from{};
        // backlash play model (set_backlash): phys lives in the unwrapped
        // counter space and follows the classical play operator
        //   phys = clamp(phys, unwrapped − h, unwrapped + h),  h = S/2,
        // kept current by sync_play() after every position mutation.
        double backlash  = 0.0;  // full slop S: issued steps per reversal
        double unwrapped = 0x80; // position un-wrapped via signed deltas
        double phys      = 0x80; // delivered position (same unwrapped space)
        double prev_pos  = 0x80; // last wrapped position (delta source)
    };

    void tick();                                   // animate to now_ (mu_ held)
    void sync_play(AxisSim& a);                    // advance play model to position (mu_ held)
    void recenter_play(AxisSim& a);                // seat phys at the counter (mu_ held)
    std::string handle(const std::string& cmd);    // full frame incl ACK/NAK

    SimProfile prof_;
    bool open_ = false;
    mutable std::mutex mu_;
    std::string rx_;                          // partial inbound command
    std::deque<char> out_;                    // pending response bytes
    std::vector<std::string> log_;
    std::chrono::steady_clock::time_point now_{}; // virtual clock
    bool wall_clock_ = false;          // tick() stamps now_ from steady_clock
    AxisSim ax1_, ax2_;
    std::map<char, int> nak_pending_;
    std::map<char, int> silence_pending_;
    std::map<char, int> trunc_pending_;
};

} // namespace sateq
