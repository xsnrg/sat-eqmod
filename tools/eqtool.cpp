// eqtool — board-level diagnostic CLI for hardware bring-up and bench tests.
// Deliberately dumb: it only ever does what the subcommand says, so it is
// safe to keep next to a powered mount. Motion subcommands require the
// explicit --yes-i-want-motion flag because things will physically move.
//
// Usage:
//   eqtool --port /dev/serial/by-id/... info
//   eqtool --port ... query            (full read-only battery, raw payloads)
//   eqtool --port ... poll [--hz 5]
//   eqtool --port ... stop
//   eqtool --port ... home            (EQ8 home encoder; HEQ5 replies with a
//                                     :j-like value — trust the :q probe, not
//                                     this, for home-encoder presence)
//   eqtool --port ... rate --axis 1 --x 1.0 --seconds 2 --force-motion
//   eqtool --port ... rate ... --hold-writes   (build+print frames, send none)
#include "sateq/board.h"
#include "sateq/version.h"
#include "sateq/byteorder.h"
#include "sateq/session.h"
#include "sateq/simulator.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <unistd.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>

using namespace sateq;

namespace {

struct Options
{
    std::string port;
    std::string sim;  // "heq5" | "eq8": drive the built-in board simulator (no hardware)
    std::string host = "127.0.0.1";  // status: sateqd rotctld host
    int rtport = 4533;               // status: rotctld TCP port
    bool watch = false;              // status: refresh once per second
    std::string command;
    char axis = '1';
    bool axis_given = false;  // backlash defaults to BOTH axes when --axis absent
    double x = 1.0;
    bool x_given = false;  // backlash overrides the default (see run_backlash)
    double seconds = 1.0;
    double hz = 5.0;
    bool force_motion = false;
    bool hold_writes = false;
    // backlash wizard
    std::string mount_name;    // which <name>.cal file to reference/write
    std::string state_dir;     // override state home (tests)
    double burst_arcsec = 15.0;
    int reps = 3;
    double runin_s = 2.0;
    std::string eyes = "ask";  // ask | cmd
    std::string eyes_cmd;
    bool no_save = false;
};

[[noreturn]] void die(const std::string& msg)
{
    std::fprintf(stderr, "error: %s\n", msg.c_str());
    std::exit(2);
}

void usage()
{
    std::fprintf(stderr,
        "usage: eqtool --port DEV <command> [options]   (or --sim heq5|eq8, no hardware)\n"
        "commands: info | query | poll | stop | home | rate | backlash | status\n"
        "  query = full read-only battery (safe on a powered mount)\n"
        "  status = operator panel from a RUNNING sateqd (rotctld TCP, no --port):\n"
        "           eqtool status [--host H] [--rtport N] [--watch]\n"
        "  backlash = measure gear-mesh slop (see --help detail below)\n"
        "options:  --axis 1|2  --x RATE  --seconds N  --hz N  --force-motion\n"
        "          --hold-writes  queue writes instead of sending (dry run)\n"
        "backlash options: --x RATE (default 10x) --burst-arcsec N (15) --reps N (3)\n"
        "          --runin S (2.0)  --mount NAME (cal file name)  --state-dir DIR\n"
        "          --eyes ask|cmd  --eyes-cmd PROG  --no-save\n"
        "  eyes=cmd contract: PROG reset   at each reversal (exit 0);\n"
        "                     PROG query   after each burst (exit 0=MOVED, 1=still)\n");
    std::exit(2);
}

Options parse(int argc, char** argv)
{
    Options o;
    std::vector<std::string> args(argv + 1, argv + argc);
    for (size_t i = 0; i < args.size(); ++i)
    {
        const std::string& a = args[i];
        auto next = [&](const char* opt) -> std::string {
            if (++i >= args.size())
                die(std::string("missing value for ") + opt);
            return args[i];
        };
        if (a == "--version")
        {
            std::printf("eqtool %s\n", sateq::version_string().c_str());
            std::exit(0);
        }
        else if (a == "--port") o.port = next("--port");
        else if (a == "--host") o.host = next("--host");
        else if (a == "--rtport") o.rtport = std::stoi(next("--rtport"));
        else if (a == "--watch") o.watch = true;
        else if (a == "--sim") o.sim = next("--sim");
        else if (a == "--axis") { o.axis = next("--axis")[0]; o.axis_given = true; }
        else if (a == "--x") { o.x = std::stod(next("--x")); o.x_given = true; }
        else if (a == "--seconds") o.seconds = std::stod(next("--seconds"));
        else if (a == "--hz") o.hz = std::stod(next("--hz"));
        else if (a == "--force-motion" || a == "--yes-i-want-motion") o.force_motion = true;
        else if (a == "--hold-writes") o.hold_writes = true;
        else if (a == "--mount") o.mount_name = next("--mount");
        else if (a == "--state-dir") o.state_dir = next("--state-dir");
        else if (a == "--burst-arcsec") o.burst_arcsec = std::stod(next("--burst-arcsec"));
        else if (a == "--reps") o.reps = std::stoi(next("--reps"));
        else if (a == "--runin") o.runin_s = std::stod(next("--runin"));
        else if (a == "--eyes") o.eyes = next("--eyes");
        else if (a == "--eyes-cmd") o.eyes_cmd = next("--eyes-cmd");
        else if (a == "--no-save") o.no_save = true;
        else if (a == "info" || a == "query" || a == "poll" || a == "stop" || a == "home" ||
                 a == "rate" || a == "backlash" || a == "status")
            o.command = a;
        else usage();
    }
    if (o.command.empty() ||
        (o.command != "status" && o.port.empty() && o.sim.empty()))
        usage();
    if (o.command == "status" && o.port.empty() && o.sim.empty() && o.watch)
    {
        // Attaching as a client has motion semantics (last-client-drop =
        // operator gone): state the contract instead of letting it surprise.
        std::printf("# NOTE: while attached, this connection counts as an operator"
                    " client; on exit, if it was the LAST client the daemon drops"
                    " the target (mount coasts to rest)\n");
    }
    if (!o.sim.empty() && o.sim != "heq5" && o.sim != "eq8")
        die("--sim must be heq5 or eq8");
    if (!o.sim.empty() && o.command == "backlash")
        std::printf("# NOTE: simulator session — no real mount; backlash numbers are "
                    "meaningless (the sim has no gear slop); exercise the wizard only\n");
    if (o.axis != '0' && o.axis != '1' && o.axis != '2')
        die("--axis must be 1 (RA/AZ), 2 (DEC/ALT), or 0 (both; backlash only)");
    return o;
}

const char* axis_name(char a) { return a == '1' ? "RA" : "DE"; }

std::string hex6(uint32_t v)
{
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%06X", v & 0xFFFFFF);
    return std::string(buf);
}

void print_info(Board& b)
{
    uint8_t code = b.mount_code();
    std::printf("mount code    : 0x%02X (%s)\n", code, mount_code_name(code));
    bool eq8 = b.probe_eq8_features();
    std::printf("eq8 features  : %s\n", eq8 ? "yes (:d/:W/:O available)" : "no (classic path)");
    for (char ax : {'1', '2'})
    {
        AxisInfo i = b.inquire_axis(ax);
        double arcsec_per_step = 1296000.0 / static_cast<double>(i.steps_360);
        AxisStatus s = b.status(ax);
        std::printf("%s axis       : steps/360=%u steps/worm=%u hs_ratio=%u\n",
                    axis_name(ax), i.steps_360, i.steps_worm, i.hs_ratio);
        std::printf("               arcsec/step=%.4f  step period=%u us\n",
                    arcsec_per_step, b.get_step_period(ax));
        std::printf("               status: %s%s%s%s  position=%s\n",
                    s.running ? "running " : "", s.slew_mode ? "slew " : "slew-low ",
                    s.backward ? "backward " : "forward ", s.initialized ? "init" : "uninit",
                    hex6(b.position(ax)).c_str());
    }
}

void print_pointing_line(Board& b)
{
    AxisStatus s1 = b.status(AxisRA), s2 = b.status(AxisDE);
    std::printf("%s=%s%s %s=%s%s\n",
                axis_name(AxisRA), hex6(b.position(AxisRA)).c_str(),
                s1.running ? " RUN" : "",
                axis_name(AxisDE), hex6(b.position(AxisDE)).c_str(),
                s2.running ? " RUN" : "");
    std::fflush(stdout);
}

// ---- full read-only battery ------------------------------------------------
// Sends ONLY commands passing is_query_command(); the Board gate is additionally
// forced ON as a belt-and-braces guarantee (queries still work when blocked).
// One line per inquiry so the output doubles as documentation of the live
// board command set (payload formats, NAK codes).

// Run one raw query, print "-> payload" or "-> NAK/err". Returns payload
// or empty on failure.
std::string probe(Board& b, char cmd, char ax, const std::string& arg, const char* what)
{
    std::printf("%s %-16s ->", (std::string(":") + cmd + ax + arg).c_str(), what);
    try
    {
        std::string p = b.dispatch(cmd, ax, arg);
        if (p.size() == 6)
        {
            // hex payload: also show its revu24 value (that is how the
            // library reads steps/period/position — useful for spot checks)
            uint32_t v = revu24_parse(p.c_str());
            std::printf(" %s (u24=%u)\n", p.c_str(), v);
        }
        else
            std::printf(" %s\n", p.c_str());
        std::fflush(stdout);
        return p;
    }
    catch (const BoardError& e)
    {
        std::printf(" %s\n", e.what());
        std::fflush(stdout);
        return {};
    }
}

void run_query_battery(Board& b)
{
    std::printf("# read-only query battery (no writes can escape the wire)\n");
    std::fflush(stdout);
    std::string e = probe(b, InquireMCVersion, '1', "", "mount id");
    if (e.size() == 6)
        std::printf("#   mount code 0x%02X (%s)\n", mount_code_from_version_payload(e.c_str()),
                    mount_code_name(mount_code_from_version_payload(e.c_str())));
    probe(b, GetFeatureCmd, '1', "010000", "eq8 features probe");
    for (char ax : {'1', '2'})
    {
        probe(b, InquireGridPerRevolution, ax, "", "steps/rev");
        probe(b, InquireTimerInterruptFreq, ax, "", "steps/worm");
        probe(b, InquireHighSpeedRatio, ax, "", "hs ratio");
        probe(b, InquirePecCycle, ax, "", "PEC cycle");
        probe(b, GetAxisStatus, ax, "", "axis status");
        probe(b, GetAxisPosition, ax, "", "axis position");
        probe(b, GetStepPeriod, ax, "", "step period");
        probe(b, GetHomePosition, ax, "", "home encoder");
    }
}

} // namespace

// ---- backlash measurement wizard -------------------------------------------
// The :j counter counts issued pulses, NEVER delivered motion, so gear-mesh
// backlash is invisible from the wire: the one sensor we do not have is
// "physically moving". The wizard therefore asks an ORACLE — the operator's
// eyes on a laser-dot-on-the-wall lever (AskEyes) or a program inspecting the
// dot with a webcam (CmdEyes; the seam a CV probe plugs into). Protocol per
// (axis, direction):
//   1. RUN-IN: drive +direction for --runin s so the mesh seats hard on that
//      flank; oracle VERIFIES the scope actually moved (catches dead axes).
//   2. REVERSE, then issue bursts in the opposite direction with geometric
//      growth; after each burst, read ISSUED steps from :j (:K coast included).
//   3. Oracle answers the CUMULATIVE question "moved since reversal?" —
//      "no": accumulate, retry bigger; "yes": backlash lies inside
//      (cum_before_burst, cum_after_burst]: report the window midpoint.
// Window is bounded at <= 2x; median over reps; both directions per axis;
// record the worst (compensation must cover the worst flank pair). Results
// merge into <state_dir>/<name>.cal, which sateqd auto-loads at startup.

struct Eyes
{
    virtual ~Eyes() = default;
    virtual void reset(const char* axis_name) = 0;  // new observation window
    virtual bool moved() = 0;                       // moved since reset?
};

struct AskEyes : Eyes
{
    void reset(const char* ax) override
    {
        std::printf("\n  WATCH THE %s AXIS (laser dot) until this window's question.\n", ax);
        std::fflush(stdout);
    }
    bool moved() override
    {
        std::printf("  did the scope physically MOVE since this window opened? [y/N] ");
        std::fflush(stdout);
        std::string line;
        if (!std::getline(std::cin, line))
            die("stdin closed — run interactively, or use --eyes cmd --eyes-cmd PROG");
        return line == "y" || line == "Y" || line == "yes";
    }
};

struct CmdEyes : Eyes
{
    explicit CmdEyes(std::string prog) : prog_(std::move(prog)) {}
    void reset(const char*) override
    {
        if (std::system((prog_ + " reset").c_str()) != 0)
            die("eyes program failed on 'reset'");
    }
    bool moved() override
    {
        const int rc = std::system((prog_ + " query").c_str());
        if (rc == 0)
            return true;
        if (rc == 256)  // exit(1)
            return false;
        die("eyes program 'query' must exit 0 (MOVED) or 1 (still)");
    }
    std::string prog_;
};

int32_t wrap24(uint32_t raw)  // signed 24-bit counter delta (same as `rate`)
{
    int32_t d = static_cast<int32_t>(raw);
    if (d > 0x800000) d -= 0x1000000;
    if (d < -0x800000) d += 0x1000000;
    return d;
}

bool wait_clear(Board& b, char ax, int ms_budget)
{
    for (int waited = 0; waited < ms_budget; waited += 20)
    {
        if (!b.status(ax).running)
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return !b.status(ax).running;
}

struct BlWindow
{
    bool ok = false;
    double lo = 0, hi = 0;  // issued-step window containing the backlash
};

BlWindow measure_one_dir(Board& b, char ax, int dir, double rate_x, double burst_arcsec,
                         double runin_s, double arcsec_per_step, Eyes& eyes, double hint_steps)
{
    const double sidereal_arcsec_s = 1296000.0 / STELLAR_DAY_SEC;
    const auto ms = [](double seconds) {
        return std::chrono::milliseconds(static_cast<long>(seconds * 1000.0));
    };

    // 1. run-in (+dir), oracle-verified: continuous drive far beyond typical slop
    eyes.reset(axis_name(ax));
    b.slew_rate(ax, dir * rate_x);
    std::this_thread::sleep_for(ms(runin_s));
    b.stop_axis(ax, false);
    wait_clear(b, ax, 5000);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));  // coast + settle
    if (!eyes.moved())
    {
        std::printf("  run-in drove %s for %.1fs yet NOTHING MOVED — axis not connected, "
                    "mount unpowered, or oracle not observing? Aborting.\n",
                    axis_name(ax), runin_s);
        return {};
    }

    // 2./3. reversal search
    eyes.reset(axis_name(ax));
    const double ceiling = hint_steps > 0 ? 16.0 * hint_steps : 50000.0;
    double burst = burst_arcsec, cum = 0.0;
    for (int trial = 0; trial < 40 && cum < ceiling; ++trial)
    {
        const double dur = std::max(0.12, burst / (rate_x * sidereal_arcsec_s));
        const uint32_t p0 = b.position(ax);
        b.slew_rate(ax, -dir * rate_x);
        std::this_thread::sleep_for(ms(dur));
        b.stop_axis(ax, false);
        wait_clear(b, ax, 5000);
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        const double d = std::fabs(static_cast<double>(wrap24(b.position(ax) - p0)));
        const double lo_before = cum;
        cum += d;
        std::printf("  trial %2d: ~%5.1f arcsec commanded -> %4.0f steps issued "
                    "(cum %6.0f = %7.1f arcsec)\n",
                    trial + 1, burst, d, cum, cum * arcsec_per_step);
        std::fflush(stdout);
        if (eyes.moved())
            return {true, lo_before, cum};
        burst *= 2.0;
    }
    std::printf("  NO motion within %.0f issued steps — oracle still saying no? "
                "(check laser dot geometry / oracle program)\n", cum);
    return {};
}

double median_of(std::vector<double> v)
{
    if (v.empty())
        return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

void run_backlash(Board& b, Options& o)
{
    if (!o.force_motion)
        die("backlash physically moves the mount: re-run with --force-motion");
    if (o.hold_writes)
        die("backlash cannot dry-run: the measurement IS physical motion");
    if (o.eyes == "ask" && !isatty(STDIN_FILENO))
        die("--eyes ask needs an interactive terminal (or --eyes cmd --eyes-cmd PROG)");
    if (o.eyes == "cmd" && o.eyes_cmd.empty())
        die("--eyes cmd requires --eyes-cmd PROG (contract: PROG reset / PROG query)");
    if (!o.x_given)
        o.x = 10.0;  // documented default; the shared 1x rate default would give a
                     // first-run (no cal hint) 30" run-in that cannot seat large slop
    if (o.reps < 1 || o.burst_arcsec <= 0.0 || o.runin_s < 0.5 || o.x <= 0.0 || o.x > 100.0)
        die("--reps>=1, --burst-arcsec>0, --runin>=0.5, --x in (0,100] (low band, instant)");

    std::unique_ptr<Eyes> eyes =
        o.eyes == "cmd"
            ? std::unique_ptr<Eyes>(new CmdEyes("'" + o.eyes_cmd + "'"))
            : std::unique_ptr<Eyes>(new AskEyes);

    Paths paths = default_paths();
    if (!o.state_dir.empty())
        paths.state_dir = o.state_dir;

    // A previous measurement seeds the burst budget (first trial size hint)
    // and the sanity check; a corrupt/absent cal simply means "no hint".
    BacklashCal prev;
    if (!o.mount_name.empty())
        prev.load(backlash_cal_path(paths, o.mount_name));

    std::vector<char> axes = (o.axis == '0' || !o.axis_given)
                                 ? std::vector<char>{'1', '2'}
                                 : std::vector<char>{o.axis};

    BacklashCal out;
    out.mount_code = static_cast<int>(b.mount_code());
    out.utc_unix = static_cast<uint64_t>(std::time(nullptr));
    char mbuf[64];
    std::snprintf(mbuf, sizeof mbuf, "burst@%.0fx res%.0f\" midpoint", o.x, o.burst_arcsec);
    out.method = mbuf;

    for (char ax : axes)
    {
        const AxisInfo info = b.axis_info(ax);
        const double arcsec_per_step = 1296000.0 / static_cast<double>(info.steps_360);
        const double hint = ax == '1' ? prev.ra_steps : prev.dec_steps;
        if (hint > 0 && o.runin_s * o.x * (1296000.0 / STELLAR_DAY_SEC) < 2.0 * hint * arcsec_per_step)
        {
            // previous backlash needs more run-in travel to guarantee seating
            o.runin_s = 2.5 * hint * arcsec_per_step / (o.x * (1296000.0 / STELLAR_DAY_SEC));
            std::printf("NOTE: raising run-in to %.1fs for the previously measured slop\n",
                        o.runin_s);
        }
        std::printf("\n=== %s axis backlash: rate %.0fx, run-in %.1fs, bursts from %.0f arcsec, "
                    "%d reps, oracle=%s ===\n",
                    axis_name(ax), o.x, o.runin_s, o.burst_arcsec, o.reps, o.eyes.c_str());
        std::fflush(stdout);
        std::vector<double> fwd_mid, rev_mid;
        std::string samples;
        for (int rep = 0; rep < o.reps; ++rep)
        {
            BlWindow f = measure_one_dir(b, ax, +1, o.x, o.burst_arcsec, o.runin_s,
                                         arcsec_per_step, *eyes, hint);
            if (!f.ok)
                die("measurement aborted (see above)");
            BlWindow r = measure_one_dir(b, ax, -1, o.x, o.burst_arcsec, o.runin_s,
                                         arcsec_per_step, *eyes, hint);
            if (!r.ok)
                die("measurement aborted (see above)");
            fwd_mid.push_back(0.5 * (f.lo + f.hi));
            rev_mid.push_back(0.5 * (r.lo + r.hi));
            char sb[96];
            std::snprintf(sb, sizeof sb, "%s%d:%.0f-%.0f/%.0f-%.0f", samples.empty() ? "" : ",",
                          rep + 1, f.lo, f.hi, r.lo, r.hi);
            samples += sb;
            std::printf(" rep %d medians: fwd %.0f rev %.0f steps\n", rep + 1,
                        median_of(fwd_mid), median_of(rev_mid));
            std::fflush(stdout);
        }
        const double worst = std::max(median_of(fwd_mid), median_of(rev_mid));
        std::printf("%s backlash: fwd %.0f, rev %.0f -> RECORD %.0f steps = %.1f arcsec\n",
                    axis_name(ax), median_of(fwd_mid), median_of(rev_mid), worst,
                    worst * arcsec_per_step);
        if (hint > 0 && (worst > 8.0 * hint || worst < hint / 8.0))
            std::printf("WARNING: far from previous measurement (%.0f steps): geartrain "
                        "changed, or the oracle missed motion?\n", hint);
        if (ax == '1') { out.ra_steps = worst; out.ra_samples = samples; }
        else           { out.dec_steps = worst; out.dec_samples = samples; }
    }

    std::printf("\ntotal travel this procedure: a few arcmin — re-home the mount "
                "(level bubble ritual) before resuming sessions\n");
    if (o.no_save)
        return;
    if (o.mount_name.empty())
    {
        std::printf("note: pass --mount NAME to persist the result to <state_dir>/<NAME>.cal\n");
        return;
    }
    // merge into any existing cal so single-axis re-measurement keeps the other axis
    BacklashCal merged = prev.valid ? prev : BacklashCal();
    merged.utc_unix = out.utc_unix;
    merged.mount_code = out.mount_code;
    merged.method = out.method;
    if (out.ra_steps >= 0) { merged.ra_steps = out.ra_steps; merged.ra_samples = out.ra_samples; }
    if (out.dec_steps >= 0) { merged.dec_steps = out.dec_steps; merged.dec_samples = out.dec_samples; }
    const std::string p = backlash_cal_path(paths, o.mount_name);
    std::printf(merged.save(p) ? "saved: %s\n" : "ERROR: could not write %s\n", p.c_str());
}

// ---- status: thin rotctld TCP client (never opens the serial port) ------
// Sends the sat-eqmod `status` extension and prints the panel. Motion
// semantics of a monitor connection are stated at startup (see parse()).
std::string rotctld_query(const std::string& host, int port, const std::string& q)
{
    addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    if (::getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res)
        die("cannot resolve " + host + ":" + std::to_string(port));
    std::unique_ptr<addrinfo, void (*)(addrinfo*)> freeme(res, freeaddrinfo);
    int fd = -1;
    for (addrinfo* a = res; a; a = a->ai_next)
    {
        fd = ::socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0)
            continue;
        if (::connect(fd, a->ai_addr, a->ai_addrlen) == 0)
            break;
        ::close(fd);
        fd = -1;
    }
    if (fd < 0)
        die("cannot connect to rotctld at " + host + ":" + std::to_string(port) +
            " (is sateqd running?)");
    timeval tv{3, 0};
    ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    std::string out;
    ::send(fd, q.data(), q.size(), MSG_NOSIGNAL);
    char buf[1024];
    while (out.find("RPRT") == std::string::npos && out.size() < 65536)
    {
        ssize_t r = ::recv(fd, buf, sizeof(buf), 0);
        if (r <= 0)
            break;
        out.append(buf, static_cast<size_t>(r));
    }
    ::close(fd);
    return out;
}

int run_status(const Options& o)
{
    for (int i = 0;; ++i)
    {
        std::string r = rotctld_query(o.host, o.rtport, "status\n");
        if (r.find("RPRT -1") != std::string::npos)
            die("daemon rejected 'status' (RPRT -1): sateqd predates the status "
                "command — update it, or use rotctld 'p' for position only");
        if (o.watch && i > 0)
            std::printf("---\n");
        std::fputs(r.c_str(), stdout);
        std::fflush(stdout);
        if (!o.watch)
        {
            // One-shot: this connection just closed. If it was the only
            // client AND the mount was actively tracking, that close just
            // dropped the target (operator-gone semantics). Warn loudly.
            const bool tracking = r.find("state: TRACK") != std::string::npos;
            size_t cp = r.find("clients:");
            int clients = cp == std::string::npos ? -1 : std::atoi(r.c_str() + cp + 8);
            if (tracking && clients == 1)
                std::printf("NOTE: this status connection was the ONLY rotctld client and the"
                            " mount was TRACKING: closing it drops the target and the mount"
                            " coasts to rest (last-client = operator-gone design).\n");
            return 0;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

int main(int argc, char** argv)
try
{
    Options o = parse(argc, argv);
    if (o.command == "status")
        return run_status(o);
    std::shared_ptr<Transport> t;
    if (!o.sim.empty())
    {
        auto sim = std::make_shared<SimulatorTransport>(o.sim == "eq8" ? sim_eq8() : sim_heq5());
        sim->set_wall_clock(true);  // wizard sleeps real time: animate in real time
        t = sim;
        std::printf("# simulator profile %s (wall clock) — NO REAL MOUNT on the wire\n",
                    o.sim.c_str());
    }
    else
        t = std::make_shared<SerialTransport>(o.port);
    Board b(t);
    b.connect();
    if (o.hold_writes)
    {
        b.block_writes(true);
        std::printf("# write gate ON: writes are queued, never sent\n");
        std::fflush(stdout);
    }
    else if (o.command == "query" || o.command == "info")
    {
        // Belt-and-braces: these subcommands are read-only BY DESIGN, so
        // engage the gate regardless — even a future edit that adds a write
        // here could not put it on the wire. (is_query_command already
        // restricts what run_query_battery dispatches.)
        b.block_writes(true);
    }

    if (o.command == "info")
    {
        print_info(b);
    }
    else if (o.command == "query")
    {
        run_query_battery(b);
    }
    else if (o.command == "poll")
    {
        std::printf("polling at %.1f Hz (Ctrl-C to exit)\n", o.hz);
        for (;;)
        {
            print_pointing_line(b);
            std::this_thread::sleep_for(std::chrono::milliseconds(
                static_cast<long>(1000.0 / o.hz)));
        }
    }
    else if (o.command == "stop")
    {
        b.stop_all(true);
        std::printf("instant-stopped both axes (:L1 :L2)\n");
        print_pointing_line(b);
    }
    else if (o.command == "home")
    {
        // :d home-sensor encoder (EQ8-class boards; NAKs on HEQ5).
        for (char ax : {'1', '2'})
        {
            try
            {
                std::printf("%s home encoder : %s\n", axis_name(ax), hex6(b.home_sensor(ax)).c_str());
            }
            catch (const BoardError& e)
            {
                std::printf("%s home encoder : unavailable (%s)\n", axis_name(ax), e.what());
            }
        }
    }
    else if (o.command == "rate")
    {
        if (!o.force_motion && !o.hold_writes)
            die("rate physically moves the mount: re-run with --force-motion "
                "(or --hold-writes to dry-run the frame sequence)");
        const char ax = o.axis;
        const double s360 = static_cast<double>(b.axis_info(ax).steps_360);
        std::printf("slewing %s at %+.2fx sidereal for %.2f s, then stopping\n",
                    axis_name(ax), o.x, o.seconds);
        std::fflush(stdout);
        const uint32_t p0 = b.position(ax);
        const auto t0 = std::chrono::steady_clock::now();
        b.slew_rate(ax, o.x);
        const auto t_run = std::chrono::steady_clock::now();
        // sample loop: position + RAW :f words (characterizing accel/slew bits)
        const long poll_ms = static_cast<long>(1000.0 / o.hz);
        std::vector<std::pair<double, std::string>> fsamples;
        while (std::chrono::steady_clock::now() - t_run <
               std::chrono::milliseconds(static_cast<long>(o.seconds * 1000.0)))
        {
            const double tsec =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            const std::string fraw = b.dispatch('f', ax);
            fsamples.emplace_back(tsec, fraw);
            std::printf("t=%6.2f pos=%s f=%s\n", tsec, hex6(b.position(ax)).c_str(),
                        fraw.empty() ? "(no reply)" : fraw.c_str());
            std::fflush(stdout);
            std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
        }
        const uint32_t p_run = b.position(ax); // before decel tail pollutes
        const auto t_stop_cmd = std::chrono::steady_clock::now();
        b.stop_axis(ax, false); // :K decelerating stop (vendor-like)
        // wait for the board to report not-running (decel tail), max 3 s
        bool stopped = false;
        for (int i = 0; i < 300 && !stopped; ++i)
        {
            stopped = !b.status(ax).running;
            if (!stopped) std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        const auto t1 = std::chrono::steady_clock::now();
        const uint32_t p1 = b.position(ax);
        // signed 24-bit wrap deltas
        auto wrap_delta = [](uint32_t raw_u) -> int32_t {
            int32_t d = static_cast<int32_t>(raw_u);
            if (d > 0x800000) d -= 0x1000000;
            if (d < -0x800000) d += 0x1000000;
            return d;
        };
        const int32_t d_run = wrap_delta(p_run - p0);   // commanded-window motion
        const int32_t d_dec = wrap_delta(p1 - p_run);   // deceleration tail (coast)
        const double run_s = std::chrono::duration<double>(t_stop_cmd - t_run).count();
        const double exp_steps = std::fabs(o.x) * s360 / STELLAR_DAY_SEC * run_s;
        const double ratio = std::labs(static_cast<long>(d_run)) / (exp_steps > 0 ? exp_steps : 1.0);
        std::printf("result: %s run_steps=%+d coast_steps=%+d (raw %06X->%06X->%06X) run=%.2fs\n",
                    axis_name(ax), d_run, d_dec, p0, p_run, p1, run_s);
        std::printf("        commanded %+.2fx -> delivered ratio %.4f of |x| over run window\n",
                    o.x, ratio);
        std::printf("        :f transitions:");
        std::string last;
        for (auto& s : fsamples)
            if (s.second != last)
            {
                std::printf(" %s@%.2fs", s.second.c_str(), s.first);
                last = s.second;
            }
        std::printf("\n");
        std::printf("stopped after %.3f s (running-bit clear: %s)\n",
                    std::chrono::duration<double>(t1 - t_stop_cmd).count(),
                    stopped ? "yes" : "NO — check mount!");
    }
    else if (o.command == "backlash")
    {
        run_backlash(b, o);
    }
    if (o.hold_writes)
    {
        auto held = b.held_frames();
        std::printf("# write gate: %zu write frame(s) queued, NONE sent:\n", held.size());
        for (const auto& f : held)
            std::printf("#   %s\n", f.c_str());
        if (b.held_dropped())
            std::printf("#   (%zu dropped from capped queue)\n", b.held_dropped());
        std::fflush(stdout);
    }
    return 0;
}
catch (const std::exception& e)
{
    die(e.what());
}
