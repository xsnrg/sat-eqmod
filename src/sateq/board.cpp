#include "sateq/board.h"
#include "sateq/byteorder.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

#include <cstring>
#include <chrono>
#include <map>

namespace sateq {

// ---------- protocol math ----------

SlewPeriod period_for_rate(double rate_x_sidereal, uint32_t steps_worm,
                           uint32_t steps_360, uint32_t hs_ratio)
{
    double absrate = rate_x_sidereal < 0 ? -rate_x_sidereal : rate_x_sidereal;
    bool high = false;
    if (absrate > LOWSPEED_RATE)
    {
        absrate /= static_cast<double>(hs_ratio);
        high = true;
    }
    double period = (STELLAR_DAY_SEC * static_cast<double>(steps_worm)) /
                    static_cast<double>(steps_360) / absrate;
    return SlewPeriod{static_cast<uint32_t>(period), high};
}

// ---------- SerialTransport ----------

SerialTransport::SerialTransport(std::string devpath) : path_(std::move(devpath)) {}
SerialTransport::~SerialTransport() { if (fd_ >= 0) ::close(fd_); }

void SerialTransport::open()
{
    if (fd_ >= 0) return;
    fd_ = ::open(path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0)
    {
        // Release-quality failure modes: name the fix, not just errno.
        std::string hint;
        if (errno == EBUSY)
            hint = " — held by another process; find it with `fuser -v "
                   + path_ + "` (gpsd? a second sateqd?) and stop it first";
        else if (errno == EACCES)
            hint = " — permission denied; add your user to the dialout/uucp group";
        else if (errno == ENOENT)
            hint = " — no such device; check the cable and pick the path from `ls -l /dev/serial/by-id/`";
        throw BoardError(BoardError::Serial,
                         "cannot open " + path_ + ": " + std::strerror(errno) + hint);
    }
    // Kernel-enforced single ownership: with the exclusive lock held, any
    // later open() of this tty by another process (a second sateqd, eqtool
    // against a live daemon — the 2026-11-16/11-20 counter-corruption
    // races) fails EBUSY instead of sharing the wire. Best-effort: some
    // drivers lack TIOCEXCL; losing it must not break a good port.
    if (::ioctl(fd_, TIOCEXCL) != 0 && errno == EBUSY)
    {
        ::close(fd_);
        fd_ = -1;
        throw BoardError(BoardError::Serial,
                         "cannot lock " + path_ + " exclusively: another sateqd/eqtool "
                         "instance holds it (`fuser -v " + path_ + "`)");
    }
    termios tio{};
    if (::tcgetattr(fd_, &tio) != 0)
        throw BoardError(BoardError::Serial, "tcgetattr failed on " + path_);
    ::cfmakeraw(&tio);
    ::cfsetispeed(&tio, B9600);
    ::cfsetospeed(&tio, B9600);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~static_cast<tcflag_t>(CSTOPB | PARENB | CRTSCTS);
    tio.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO | ECHOE | ISIG);
    tio.c_iflag &= ~static_cast<tcflag_t>(IXON | IXOFF | IXANY | INLCR | ICRNL | IGNCR);
    tio.c_oflag &= ~static_cast<tcflag_t>(OPOST);
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 0;
    if (::tcsetattr(fd_, TCSANOW, &tio) != 0)
        throw BoardError(BoardError::Serial, "tcsetattr failed on " + path_);
    // Like the vendor driver: keep DTR/RTS asserted.
    int mcs = TIOCM_DTR | TIOCM_RTS;
    ::ioctl(fd_, TIOCMBIS, &mcs);
}

void SerialTransport::close()
{
    if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
}

void SerialTransport::write(const std::string& bytes)
{
    size_t off = 0;
    while (off < bytes.size())
    {
        ssize_t n = ::write(fd_, bytes.data() + off, bytes.size() - off);
        if (n < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK) { continue; }
            throw BoardError(BoardError::Serial, std::string("serial write: ") + std::strerror(errno));
        }
        off += static_cast<size_t>(n);
    }
}

std::optional<std::string> SerialTransport::read_frame(int timeout_ms)
{
    ResponseAccumulator acc;
    for (;;)
    {
        struct pollfd pfd{fd_, POLLIN, 0};
        int pr = ::poll(&pfd, 1, timeout_ms);
        if (pr == 0)
            return std::nullopt;
        if (pr < 0)
        {
            if (errno == EINTR) continue;
            throw BoardError(BoardError::Serial, "poll: " + std::string(std::strerror(errno)));
        }
        unsigned char c;
        ssize_t n = ::read(fd_, &c, 1);
        if (n <= 0)
            continue;
        if (auto f = acc.feed(c))
            return f;
    }
}

void SerialTransport::flush_input()
{
    if (fd_ >= 0) ::tcflush(fd_, TCIOFLUSH);
}

// ---------- Board ----------

Board::Board(std::shared_ptr<Transport> t) : t_(std::move(t)) {}

void Board::connect()    { t_->open(); }
void Board::disconnect() { t_->close(); }

std::string Board::dispatch(char cmd, char axis, const std::string& arg)
{
    if (!t_->is_open())
        throw BoardError(BoardError::Serial, "transport not open");
    std::string frame = build_command(cmd, axis, arg);

    // --debug wire trace: a per-attempt record of everything that goes near
    // the wire. Cost when off: one relaxed load per dispatch (the tx string
    // cut and the timestamps sit behind the flag).
    const bool dbg = trace_on_.load(std::memory_order_relaxed);
    const std::string tx = dbg ? frame.substr(0, frame.size() - 1) : std::string();
    auto rec = [&](WireExchange w) {
        if (!dbg) return;
        std::lock_guard<std::mutex> lk(trace_mu_);
        if (trace_.size() >= TRACE_CAP)
            trace_.erase(trace_.begin());
        trace_.push_back(std::move(w));
    };

    if (block_writes_ && !is_query_command(cmd))
    {
        // bench consent gate: record exact wire bytes, fabricate an ACK
        if (held_.size() >= HELD_CAP)
        {
            held_.erase(held_.begin());
            ++held_dropped_;
        }
        held_.push_back(tx.empty() ? frame.substr(0, frame.size() - 1) : tx);
        rec({tx, "", "held", -1, true, 0.0}); // never reached the wire
        return std::string(); // write commands only ever ACK ("=" => empty payload)
    }
    std::string last_err = "no response";
    for (int attempt = 0; attempt < retries_; ++attempt)
    {
        t_->flush_input();
        const auto t0 = std::chrono::steady_clock::now();
        t_->write(frame);
        auto resp = t_->read_frame(timeout_ms_);
        const double ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - t0).count();
        if (!resp)
        {
            last_err = "timeout after " + std::to_string(timeout_ms_) + " ms";
            rec({tx, "", "timeout", -1, false, ms});
            continue;  // vendor driver retries
        }
        ParsedResponse p = parse_response(*resp);
        if (p.ok)
        {
            rec({tx, *resp, "", -1, false, ms});
            return p.payload;
        }
        if (!p.malformed && (*resp)[0] == NAK)
        {
            rec({tx, *resp, "nak", p.nak_code, false, ms});
            throw BoardError(BoardError::Nak, "NAK for " + frame.substr(0, frame.size() - 1) +
                                          " code " + std::to_string(p.nak_code), p.nak_code);
        }
        rec({tx, *resp, "malformed", -1, false, ms});
        last_err = "malformed response: " + *resp;
    }
    throw BoardError(BoardError::Timeout, "no valid response to " +
                     frame.substr(0, frame.size() - 1) + ": " + last_err);
}

uint8_t Board::mount_code()
{
    if (mount_code_ < 0)
    {
        std::string p = dispatch(InquireMCVersion, AxisRA);
        if (p.size() < 6)
            throw BoardError(BoardError::Malformed, "bad :e payload: " + p +
                             " (board answered the handshake with garbage — bus "
                             "contention or wiring noise; a second process on the "
                             "port now fails EBUSY, so suspect cabling)");
        mount_code_ = mount_code_from_version_payload(p.data());
    }
    return static_cast<uint8_t>(mount_code_);
}

bool Board::probe_eq8_features()
{
    try
    {
        dispatch(GetFeatureCmd, AxisRA, "010000");
        return true;
    }
    catch (const BoardError& e)
    {
        if (e.kind == BoardError::Nak)
            return false;
        throw;
    }
}

AxisInfo Board::inquire_axis(char axis)
{
    AxisInfo info;
    {
        std::string p = dispatch(InquireGridPerRevolution, axis);
        if (p.size() < 6) throw BoardError(BoardError::Malformed, "bad :a payload: " + p);
        info.steps_360 = revu24_parse(p.data());
    }
    {
        std::string p = dispatch(InquireTimerInterruptFreq, axis);
        if (p.size() < 6) throw BoardError(BoardError::Malformed, "bad :b payload: " + p);
        info.steps_worm = revu24_parse(p.data());
    }
    {
        std::string p = dispatch(InquireHighSpeedRatio, axis);
        if (p.size() < 2) throw BoardError(BoardError::Malformed, "bad :g payload: " + p);
        info.hs_ratio = highstr_parse(p.data());
    }
    return info;
}

uint32_t Board::home_sensor(char axis)
{
    std::string p = dispatch(GetHomePosition, axis);
    if (p.size() < 6) throw BoardError(BoardError::Malformed, "bad :d payload: " + p);
    return revu24_parse(p.data());
}

std::string Board::inquire_pec(char axis)
{
    // ':s' PEC cycle inquiry. Units/semantics are vendor-unclear (cells?
    // steps?), so the library surfaces the raw payload; tools decide.
    return dispatch(InquirePecCycle, axis);
}

void Board::block_writes(bool on) { block_writes_ = on; }

void Board::set_trace(bool on)
{
    trace_on_.store(on, std::memory_order_relaxed);
    if (!on)
    {
        std::lock_guard<std::mutex> lk(trace_mu_);
        trace_.clear(); // an off trace must not carry stale records across
    }
}

std::vector<WireExchange> Board::take_trace()
{
    std::lock_guard<std::mutex> lk(trace_mu_);
    std::vector<WireExchange> out;
    out.swap(trace_);
    return out;
}

std::vector<std::string> Board::held_frames()
{
    std::vector<std::string> out;
    out.swap(held_);
    return out;
}

size_t Board::held_count() const { return held_.size(); }

uint32_t Board::position(char axis)
{
    // A truncated ACK (e.g. "=1008" — two hex chars lost on the wire) looks
    // like a clean reply to dispatch, so the Malformed throw below must
    // retry here, mirroring dispatch's own retry-on-garbage policy.
    // (Live incident 2026-11-16: one such frame escaped through a rotctld
    // p-poll, whose thread had no handler, and killed the daemon.)
    for (int attempt = 0, n = retries_ > 0 ? retries_ : 1; attempt < n; ++attempt)
    {
        std::string p = dispatch(GetAxisPosition, axis);
        if (p.size() >= 6)
            return revu24_parse(p.data());
    }
    throw BoardError(BoardError::Malformed, "bad :j payload (retried)");
}

void Board::set_position(char axis, uint32_t v)
{
    char buf[7]; revu24_format(v, buf);
    dispatch(SetAxisPositionCmd, axis, buf);
}

void Board::initialize(char axis) { dispatch(Initialize, axis); }

AxisStatus Board::status(char axis)
{
    // Wire format observed: "=100\r" = three hex nibble-chars.
    // digit1: bit0 SLEW, bit1 BACKWARD, bit2 HIGHSPEED
    // digit2: bit0 running;  digit3: bit0 initialized
    std::string p = dispatch(GetAxisStatus, axis);
    if (p.size() < 3) throw BoardError(BoardError::Malformed, "bad :f payload: " + p);
    unsigned d1 = static_cast<unsigned>(hex_nibble(p[0]));
    unsigned d2 = static_cast<unsigned>(hex_nibble(p[1]));
    unsigned d3 = static_cast<unsigned>(hex_nibble(p[2]));
    AxisStatus s;
    s.slew_mode  = (d1 & 0x1) != 0;
    s.backward   = (d1 & 0x2) != 0;
    s.high_speed = (d1 & 0x4) != 0;
    s.running    = (d2 & 0x1) != 0;
    s.initialized = (d3 & 0x1) != 0;
    return s;
}

void Board::set_step_period(char axis, uint32_t period_us)
{
    char buf[7]; revu24_format(period_us, buf);
    dispatch(SetStepPeriod, axis, buf);
}

uint32_t Board::get_step_period(char axis)
{
    std::string p = dispatch(GetStepPeriod, axis);
    if (p.size() < 6) throw BoardError(BoardError::Malformed, "bad :D payload: " + p);
    return revu24_parse(p.data());
}

void Board::set_motion_mode(char axis, int mode, int dir)
{
    dispatch(SetMotionMode, axis, motion_mode_arg(mode, dir));
}

void Board::start_motion(char axis)
{
    // LIVE 2026-11-16 Sirius: ":J" NAKs with code 4 on any axis that has not
    // been armed with ":F" since the board was powered on. ":F" does NOT
    // disturb the step counter (verified: count unchanged around :F), and
    // arming is per axis. The vendor driver arms at Connect(); we arm lazily
    // at the single motion-start choke point and cache the result.
    bool& armed = (axis == AxisDE) ? armed_de_ : armed_ra_;
    if (!armed)
    {
        if (!status(axis).initialized)
            initialize(axis);
        armed = true;
    }
    try
    {
        dispatch(StartMotion, axis);
    }
    catch (const BoardError& e)
    {
        // NAK 4 => axis disarmed under us (board power-cycled mid-session).
        // Re-arm once; a second NAK propagates.
        if (e.nak_code != 4)
            throw;
        initialize(axis);
        dispatch(StartMotion, axis);
    }
}

void Board::stop_axis(char axis, bool instant)
{
    dispatch(instant ? AxisStopInstant : NotInstantAxisStop, axis);
}

void Board::stop_all(bool instant)
{
    stop_axis(AxisRA, instant);
    stop_axis(AxisDE, instant);
}

void Board::set_goto_target(char axis, uint32_t abs_target)
{
    char buf[7]; revu24_format(abs_target, buf);
    dispatch(SetGotoTarget, axis, buf);
}

void Board::set_goto_increment(char axis, uint32_t inc)
{
    char buf[7]; revu24_format(inc, buf);
    dispatch(SetGotoTargetIncrement, axis, buf);
}

void Board::set_break_increment(char axis, uint32_t inc)
{
    char buf[7]; revu24_format(inc, buf);
    dispatch(SetBreakPointIncrement, axis, buf);
}

void Board::set_break_step(char axis, uint32_t step)
{
    char buf[7]; revu24_format(step, buf);
    dispatch(SetBreakStep, axis, buf);
}

void Board::prime_axis(char axis) { axis_cache_[axis] = inquire_axis(axis); }

const AxisInfo& Board::axis_info(char axis)
{
    auto it = axis_cache_.find(axis);
    if (it == axis_cache_.end())
    {
        prime_axis(axis);
        it = axis_cache_.find(axis);
    }
    return it->second;
}

void Board::slew_rate(char axis, double rate_x_sidereal)
{
    if (rate_x_sidereal == 0.0)
    {
        stop_axis(axis, false);
        return;
    }
    double absrate = rate_x_sidereal < 0 ? -rate_x_sidereal : rate_x_sidereal;
    if (absrate < MIN_RATE_X_SIDEREAL || absrate > MAX_RATE_X_SIDEREAL)
        throw BoardError(BoardError::Malformed, "rate out of board limits");
    const AxisInfo info = axis_info(axis);
    SlewPeriod sp = period_for_rate(rate_x_sidereal, info.steps_worm,
                                    info.steps_360, info.hs_ratio);
    // Deliberate deviation from the vendor driver: at 104..128x sidereal its
    // formula emits periods below the board's physical minimum (~6 us), which
    // the stepper timer cannot honor. Promote that band into the high-speed
    // path instead — same commanded rate, legal period. Above 128x the vendor
    // already folds, so this never fires there.
    if (!sp.use_high_speed && sp.period_us < MIN_PERIOD_US_DEFAULT)
    {
        double absrate = rate_x_sidereal < 0 ? -rate_x_sidereal : rate_x_sidereal;
        double period = STELLAR_DAY_SEC * static_cast<double>(info.steps_worm) /
                        static_cast<double>(info.steps_360) / (absrate / info.hs_ratio);
        sp = SlewPeriod{static_cast<uint32_t>(period), true};
    }
    if (sp.period_us < MIN_PERIOD_US_DEFAULT)
        throw BoardError(BoardError::Malformed, "rate exceeds achievable period");
    int mode = sp.use_high_speed ? SLEW_HIGH : SLEW_LOW;
    int dir  = rate_x_sidereal >= 0 ? FORWARD : BACKWARD;

    // Wire-level rules measured on the live Sirius (2026-11-16):
    //   :G while axis runs        -> NAK 2 (even for the identical program)
    //   :I while running LOW      -> ACK, immediately effective
    //   :I while running HIGH     -> NAK 2
    //   :J while running          -> ACK (idempotent; we avoid re-sending)
    // An axis that never got :J (armed but stopped) accepts the full cold
    // sequence :G/:I/:J, so only a live running bit forces extra care.
    const bool running_now = status(axis).running;
    Prog old = prog_[axis];
    const bool same_prog = old.valid && old.mode == mode && old.dir == dir &&
                           old.period_us == sp.period_us;
    if (same_prog && running_now)
        return; // already doing exactly this: no wire traffic at all

    if (!running_now)
    {
        set_motion_mode(axis, mode, dir);
        set_step_period(axis, sp.period_us);
        start_motion(axis);
    }
    else if (old.valid && old.mode == SLEW_LOW && mode == SLEW_LOW &&
             old.dir == dir)
    {
        set_step_period(axis, sp.period_us); // low-speed in-place speed change
    }
    else
    {
        // direction flip, mode-class change, or high-speed period change:
        // the board demands a stop first (decel tail ~0.03 s low, ~1.0 s high)
        stop_axis(axis, false);
        wait_running_clear(axis, mode == SLEW_HIGH ? 3000 : 1000);
        set_motion_mode(axis, mode, dir);
        set_step_period(axis, sp.period_us);
        start_motion(axis);
        (axis == AxisRA ? stop_ra_ : stop_de_) = true;
    }
    prog_[axis] = Prog{true, mode, dir, sp.period_us};
}

void Board::wait_running_clear(char axis, int timeout_ms)
{
    // poll :f running bit; the deceleration tail is board physics we cannot
    // shorten, so this simply blocks until the axis answers "stopped"
    for (int waited = 0; waited < timeout_ms; waited += 20)
    {
        if (!status(axis).running)
            return;
        ::usleep(20 * 1000);
    }
    throw BoardError(BoardError::Timeout,
                     std::string("axis ") + axis + " still running after :K (stop-restart)");
}

bool Board::take_stop_restart(char axis)
{
    bool& f = axis == AxisRA ? stop_ra_ : stop_de_;
    bool v = f;
    f = false;
    return v;
}

int probe_mount_code(const std::string& devpath, int timeout_ms)
{
    // Discovery must never be the thing that moves a mount: the write gate
    // stays pinned for the whole probe, so dispatch can only ever put the
    // ":e1" query on the wire. Any failure is simply "not this board".
    try
    {
        auto t = std::make_shared<SerialTransport>(devpath);
        Board board(t);
        board.set_io_timeouts(1, timeout_ms);
        board.block_writes(true);
        board.connect();
        return static_cast<int>(board.mount_code());
    }
    catch (const std::exception&)
    {
        return -1;
    }
}

} // namespace sateq
