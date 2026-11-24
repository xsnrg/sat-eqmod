// Board: protocol client over an abstract Transport, plus the POSIX serial
// transport. Retry/dispatch semantics mirror indi-eqmod (3 attempts, CR
// framing, ACK/NAK). All motor-board knowledge lives here; nothing above
// this layer knows the wire format.
#pragma once
#include <atomic>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "sateq/framing.h"
#include "sateq/protocol.h"

namespace sateq {

class Transport {
public:
    virtual ~Transport() = default;
    virtual void open()             = 0;
    virtual void close()            = 0;
    virtual bool is_open() const    = 0;
    virtual void write(const std::string& bytes)  = 0;
    // Read until CR or timeout; returns complete frame (no CR) or nullopt.
    virtual std::optional<std::string> read_frame(int timeout_ms) = 0;
    virtual void flush_input()      = 0;
};

// 9600 8N1 serial port (POSIX termios).
class SerialTransport : public Transport {
public:
    explicit SerialTransport(std::string devpath);
    ~SerialTransport() override;
    void open() override;
    void close() override;
    bool is_open() const override { return fd_ >= 0; }
    void write(const std::string& bytes) override;
    std::optional<std::string> read_frame(int timeout_ms) override;
    void flush_input() override;

private:
    std::string path_;
    int fd_ = -1;
};

struct BoardError : std::runtime_error {
    enum Kind { Timeout, Nak, Malformed, Serial } kind;
    int nak_code = -1;
    BoardError(Kind k, const std::string& msg, int nak = -1)
        : std::runtime_error(msg), kind(k), nak_code(nak) {}
};

struct AxisInfo {
    uint32_t steps_360  = 0;  // grids per revolution   (:a)
    uint32_t steps_worm = 0;  // steps per worm         (:b)
    uint32_t hs_ratio   = 0;  // high-speed ratio       (:g)
    double arcsec_per_step() const { return 1296000.0 / steps_360; }
};

struct AxisStatus {
    bool initialized = false; // :f digit3 bit0
    bool running     = false; // :f digit2 bit0
    bool slew_mode   = false; // :f digit1 bit0: true=SLEW, false=GOTO
    bool backward    = false; // :f digit1 bit1
    bool high_speed  = false; // :f digit1 bit2
};

// ---- wire trace (--debug) -------------------------------------------------
// One raw tx/rx exchange recorded at the Board::dispatch choke point, which
// every command funnels through regardless of Transport (live serial or the
// in-process simulator): exactly the record a wire forensics session needs
// ("what did we send, what came back, how long did it take"), including the
// frames a link failure swallowed (timeouts) and garbage frames (rx stored
// verbatim; the consumer sanitizes non-printables).
struct WireExchange {
    std::string tx;    // frame sent, minus CR (":j1")
    std::string rx;    // frame received, minus CR ("=000080", "!3", or raw
                       // garbage on malformed); "" when none arrived
    std::string err;   // "" = clean ACK; "timeout"; "malformed"; "nak"; "held"
    int nak = -1;      // NAK code when err == "nak"
    bool held = false; // write-gated: the frame NEVER reached the wire
    double ms = 0.0;   // wall-clock tx->rx latency (0 for held)
};

class Board {
public:
    explicit Board(std::shared_ptr<Transport> t);

    void connect();     // opens transport (idempotent)
    void disconnect();

    // Identity / capabilities (handshake validated live on both boards)
    uint8_t mount_code();                      // :e1 (cached)
    bool probe_eq8_features();                 // :q1010000: ACK=true, NAK=false
    AxisInfo inquire_axis(char axis);          // :a :b :g
    uint32_t home_sensor(char axis);           // :d (EQ8 class only)

    // Counters
    uint32_t position(char axis);              // :j
    void set_position(char axis, uint32_t v);  // :E
    void initialize(char axis);                // :F  (zero counters)

    // Other inquiries
    std::string inquire_pec(char axis);        // :s  raw payload (units vendor-unclear)

    // Status
    AxisStatus status(char axis);              // :f

    // Motion primitives (mode-then-go, exactly like the vendor driver)
    void set_step_period(char axis, uint32_t period_us);          // :I
    uint32_t get_step_period(char axis);                          // :D
    void set_motion_mode(char axis, int mode, int dir);           // :G
    void start_motion(char axis);                                 // :J
    void stop_axis(char axis, bool instant = false);              // :K / :L
    void stop_all(bool instant = true);                           // both axes

    // Goto machinery
    void set_goto_target(char axis, uint32_t abs_target);         // :S
    void set_goto_increment(char axis, uint32_t inc);             // :H
    void set_break_increment(char axis, uint32_t inc);            // :M
    void set_break_step(char axis, uint32_t step);                // :U

    // Higher-level: continuous slew at rate (x sidereal, signed: +=forward).
    // Uses cached inquire_axis() info (fetches once per axis on first use).
    // LIVE VERIFIED (Sirius 2026-11-16) while-running rules, implemented
    // transparently here: :G NAKs (code 2) whenever the axis is running,
    // even for an identical program; :I is accepted AND immediately
    // effective while running in LOW speed but NAKs 2 in high speed. A
    // program change the running axis will not accept is performed as
    // :K -> wait for decel -> reprogram -> :J, and take_stop_restart()
    // then returns true so callers can exempt that window from skip
    // accounting. Auto-stops when rate == 0.
    void slew_rate(char axis, double rate_x_sidereal);
    bool take_stop_restart(char axis); // read-and-clear per-axis event flag
    // Re-read and cache the per-axis constants (call after a power cycle).
    void prime_axis(char axis);
    const AxisInfo& axis_info(char axis); // primed info (primes if needed)
    void stop_tracking() { stop_all(false); }

    // Raw dispatch with retries (exposed for tools/tests).
    // Returns payload after '='; throws BoardError on NAK/timeout/serial.
    std::string dispatch(char cmd, char axis, const std::string& arg = "");

    // ---- wire trace (--debug) -----------------------------------------
    // Record every dispatch exchange (per retry attempt, so a timeout +
    // reissue sequence shows each wire frame) into a drainable queue.
    // Zero cost with tracing off: no record is built, no timestamp read.
    // The queue is capped (drop-oldest) so an undrained trace can never
    // grow without bound; take_trace() drains.
    void set_trace(bool on);
    bool tracing() const { return trace_on_.load(std::memory_order_relaxed); }
    std::vector<WireExchange> take_trace();

    // ---- write gate (bench safety) ------------------------------------
    // While blocked, any non-query command (anything failing
    // is_query_command, including stops) is NOT sent: its wire frame is
    // queued in held_frames() and dispatch returns a synthetic ACK with an
    // empty payload, so callers behave as if the board accepted it. Nothing
    // can physically move the mount until block_writes(false). Held frames
    // keep the exact bytes they WOULD have sent (minus CR), oldest first;
    // the queue is capped (drop-oldest) and drops are counted.
    void block_writes(bool on = true);
    bool writes_blocked() const { return block_writes_; }
    std::vector<std::string> held_frames();   // returns and clears the queue
    size_t held_count() const;
    size_t held_dropped() const { return held_dropped_; }

    // Transport timing knobs. Defaults (3 retries, 1000 ms) suit live motion;
    // the identity probe below uses 1 retry and a short timeout so a dead
    // port does not stall discovery.
    void set_io_timeouts(int retries, int timeout_ms)
    {
        retries_ = retries;
        timeout_ms_ = timeout_ms;
    }

private:
    std::shared_ptr<Transport> t_;
    int retries_ = 3;
    int timeout_ms_ = 1000;
    int mount_code_ = -1;
    // ":F" arming cache per axis (board NAKs ":J" code 4 until armed; see
    // start_motion). A NAK-4 on :J re-arms regardless of this cache.
    bool armed_ra_ = false;
    bool armed_de_ = false;

    // Last motion program commanded per axis, to decide which wire updates
    // the running board will actually accept (see slew_rate rules above).
    struct Prog
    {
        bool valid     = false;
        int  mode      = 0;    // SLEW_LOW / SLEW_HIGH
        int  dir       = 0;    // FORWARD / BACKWARD
        uint32_t period_us = 0;
    };
    std::map<char, Prog> prog_;
    bool stop_ra_ = false, stop_de_ = false; // per-axis stop/restart events
    void wait_running_clear(char axis, int timeout_ms);
    std::map<char, AxisInfo> axis_cache_;
    bool block_writes_ = false;
    std::vector<std::string> held_;
    size_t held_dropped_ = 0;
    static constexpr size_t HELD_CAP = 4096;

    // --debug wire trace state (see set_trace; mutex only ever taken while
    // tracing, so an off trace is two relaxed-atomic loads and nothing more).
    // Dispatch may run on the tracker thread and a rotctld p-poll thread
    // concurrently, hence the mutex.
    std::atomic<bool> trace_on_{false};
    std::mutex trace_mu_;
    std::vector<WireExchange> trace_;
    static constexpr size_t TRACE_CAP = 8192;
};

// protocol.h helpers
inline std::string motion_mode_arg(int mode, int dir)
{
    return std::string() + static_cast<char>(mode) + static_cast<char>(dir);
}

// Open devpath briefly and ask the board who it is (:e1). Returns the mount
// code (0x01 HEQ5, 0x04 EQ8, ...) or -1 for any failure: open error, port
// busy, timeout, NAK, garbage. Writes are gated for the whole probe, so the
// only frame that can reach the wire is the identity query itself. Feeds
// load_session's auto-discovery (--mount auto).
int probe_mount_code(const std::string& devpath, int timeout_ms = 1500);

} // namespace sateq
