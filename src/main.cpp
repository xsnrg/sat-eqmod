// sateqd — satellite-tracking daemon for SkyWatcher/Synta EQ mounts.
// Owns the motor-board serial port and exposes a Hamlib rotctld server so
// stock trackers (GPredict, rotctl -m 2) drive the mount. All safety gating
// lives in sateq::SafetyMonitor; this file is wiring only.
//
// Startup: connect → identify board → prime axis constants → (EQ8-class)
// log home-sensor counts → calibrate against the operator-stated home
// pointing → serve.
#include "sateq/rotctld.h"
#include "sateq/session.h"
#include "sateq/version.h"
#include "sateq/simulator.h"
#include "sateq/tracker.h"

#include <atomic>
#include <cstdarg>
#include <cmath>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace sateq;

namespace {

std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

struct Options
{
    std::string port;                    // serial device (ignored with --sim)
    std::string sim;                     // "", "heq5", or "eq8"
    std::string orientation = "polar";   // "polar" | "altaz" (head config, not mount)
    int rotctld_port = 4533;
    double lat = 0, lon = 0;
    bool have_site = false;
    double home_az = 180.0, home_el = 45.0;
    bool have_home = false;
    double ra_sign = 1.0, dec_sign = 1.0;
    double update_hz = 10.0;
    double target_stale_s = 90.0;        // rotctld target older than this =>
                                         // hold. Backstop only: the real
                                         // proxies are last-client-
                                         // disconnect (target drop) and the
                                         // poll-fed transport watchdog. GT2
                                         // iq-work's follow throttle can
                                         // legitimately gap 20-60 s on slow
                                         // near-horizon targets (1 deg/2 s
                                         // push rule), so this must clear
                                         // the worst throttle gap.
    double min_elevation_deg = -15.0;    // hard mechanical floor (below horizon)
    bool have_min_el = false;
    // Datum-relative mechanical travel envelope per axis (see safety.h
    // max_{ra,dec}_travel_deg): |axis offset from the calibrated datum| may
    // not exceed this IN EITHER DIRECTION, on either kinematic branch. The
    // default 105 is the bench stand's cable/stand-over limit (2026-11-19
    // incident: a GT2 pre-aim wound RA ~150 deg and the tube hit the stand);
    // 0 disables one axis.
    double ra_travel_deg = 105.0, dec_travel_deg = 105.0;
    bool have_travel = false;
    double high_coast_s = 0.0;           // HIGH :K coast landing model (s);
                                         // 0 = landing test disabled
    double coast_margin_deg = 0.3;       // landing tolerance around setpoint
    double rehome_idle_s = 0.0;          // auto fly-home after target gone
                                         // this long (0 = disabled)
    double rehome_arrive_deg = 0.5;      // re-home arrival window per axis
    double rehome_min_el_deg = 2.0;      // el gate for arming (0 = legacy)
    bool have_coast = false;             // high_coast_s/coast_margin_deg
    bool have_rehome = false;            // rehome_idle_s/rehome_arrive_deg
    double glide_promote_holdoff_s = 0.0; // glide LOW->HIGH promotion hold-off
                                         // (OFF default, see TrackerConfig)
                                         // (0 = promote instantly, legacy;
                                         // live LUCA pass 2026-11-20)
    double high_gate_moving_x = 90.0;  // pattern x alone authorizing HIGH in
                                         // glide (90 = band-edge relay fix;
                                         // 3.0 = legacy any-moving-pattern)
    double skip_cold_blind_high_s = 2.0; // cold-HIGH skip grace blind (the
                                         // board's cold issuance is dead
                                         // early: AO-123 false-latch 11-20)
    double skip_supply_slow_band = 0.75; // sustained slow-ramp-band issue
                                         // fraction (RS-44 apex latch 11-20)
    double skip_restart_grace_high_s = 4.0; // HIGH stop/restart skip-grace
                                         // window END; must exceed the 3.0 s
                                         // restart blind (double-latch 11-22)
    double branch_handover_margin_deg = 5.0; // branch-membership hysteresis
                                         // (rs44 LOS teleport 11-20; 0 = legacy)
    bool   wrap_home_unwind = true;    // home lands cord-free (DEC wrap 0);
    bool   cord_unwind_route = true;   // deep-ledger rewind RA-leg first
    bool   have_cord_route = false;    // (azimuth-first cord unwind)
                                       // false = legacy nearest-image landing
    double wrap_window_deg = 270.0;    // |DEC wrap| branch picks may land in
    double wrap_tiebreak_margin_deg = 90.0; // cord saved to take the
    double wrap_tiebreak_extra_deg  = 30.0; // slightly-farther cord-light branch
    bool have_wrap_home = false;
    bool have_wrap_window = false;
    bool have_wrap_tiebreak = false;
    double wall_strain_deg = 10.0;     // wall-strain watchdog (0 disables)
    double wall_strain_s   = 5.0;
    bool have_wall_strain = false;
    double high_recommit_s = 1.0;      // HIGH rescale persistence confirm
                                       // (0 = legacy immediate commit)
    double high_recommit_hold_deg = 0.1;
    double high_recommit_regime_s = 4.0;
    double high_recommit_regime_age_s = 1.5;
    double high_recommit_regime_db = 1e9;
    double high_recommit_escape_deg = 2.0;
    double demand_lpf_regime_tau_s = 1.0;
    double demand_lpf_regime_min_rc = 12.0;
    double demand_lpf_regime_release_err_deg = 1.0;
    double high_gate_err_deg = 2.0;
    bool have_recommit = false;
    bool have_lpfreg = false;
    bool have_promote = false;
    bool have_mgate = false;
    bool have_coldblind = false;
    bool have_supply = false;
    bool have_rgrace = false;
    bool have_branch_margin = false;
    double track_stop_el = -5.0;         // operational track gate: below this the
                                         // tracker holds (arm -5 rising, stop at LOS)
    bool have_track_stop = false;
    double max_rate_x_sidereal = 600.0;
    // measured gear slop from the session (.cal or profile override); no CLI
    // flag on purpose — it is machine-measured data, not operator tuning.
    double backlash_ra_steps = 0.0, backlash_dec_steps = 0.0;
    bool verbose = false;
    bool debug = false;                  // --debug: one key=value autopsy row
                                         // per control tick (full wire trace
                                         // incl. :f poll per tick; large log)
    bool read_only = false;              // gate all writes (bench consent mode)
    std::string record_file;             // --record FILE: CSV of every target
                                         // push (epoch_s,mono_s,az,el), for
                                         // replaying real client cadence offline
    std::string mount;                   // --mount NAME: resolve station +
                                         // mount profile (see sateq/session.h)
    bool no_resume = false;              // ignore a persisted session state
    int expected_mount_code = -1;        // resolved from profile; refuse mismatch
    // Presence flags: only EXPLICIT settings (CLI or --config) override the
    // mount profile; profile values fill everything else (profile itself
    // already merged the station config underneath).
    bool have_port = false, have_orientation = false, have_rotctld_port = false;
    bool have_ra_sign = false, have_dec_sign = false, have_update_hz = false;
    bool have_max_rate = false;
};

void log_line(const char* fmt, ...)
{
    char ts[32];
    std::time_t now = std::time(nullptr);
    std::tm tm{};
    gmtime_r(&now, &tm);
    std::strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%SZ", &tm);
    char buf[8192]; // generous: --debug emits one long key=value row per tick
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    std::printf("%s %s\n", ts, buf);
    std::fflush(stdout);
}

[[noreturn]] void die(const std::string& msg)
{
    log_line("FATAL: %s", msg.c_str());
    std::exit(2);
}

void usage()
{
    std::fprintf(stderr,
        "sateqd — satellite tracker daemon for SkyWatcher EQ mounts (rotctld server)\n"
        "usage: sateqd --mount NAME [options]  |  sateqd --port DEV [options]\n"
        "  --mount NAME         load station + mount profile (see sateq/session.h):\n"
        "                       resolves port, expected mount code, park pose, signs,\n"
        "                       rate caps; persists rotation budget across restarts\n"
        "  --mount auto         probe every profile's port (read-only :e1 query) and\n"
        "                       use the single profile whose mount_code answers there\n"
        "  --no-resume          ignore any persisted session state (start fresh budget)\n"
        "  --config FILE        key=value file (keys: port mount no_resume rotctld_port\n"
        "                       lat lon home_az home_el ra_sign dec_sign update_hz\n"
        "                       orientation min_elevation_deg track_stop_el\n"
        "                       max_rate_x_sidereal read_only debug record_file\n"
        "                       glide_promote_holdoff_s high_gate_moving_x skip_cold_blind_high_s\n"
        "                       skip_supply_slow_band skip_restart_grace_high_s branch_handover_margin_deg\n"
        "                       wrap_home_unwind cord_unwind_route wrap_window_deg\n"
        "                       wrap_tiebreak_margin_deg wrap_tiebreak_extra_deg\n"
        "                       wall_strain_deg wall_strain_s high_recommit_s\n"
        "                       high_recommit_hold_deg high_recommit_regime_s\n"
        "                       high_recommit_regime_db high_recommit_regime_age_s\n"
        "                       high_recommit_escape_deg high_gate_err_deg\n"
        "                       demand_lpf_regime_tau_s demand_lpf_regime_min_rc\n"
        "                       demand_lpf_regime_release_err_deg)\n"
        "  --port PATH          serial device, prefer /dev/serial/by-id/...\n"
        "  --sim heq5|eq8       run against the built-in board simulator\n"
        "  --read-only          consent gate: queue all write commands, send none\n"
        "  --debug              one key=value trace row per control tick: every raw\n"
        "                       wire frame sent/received (incl. timeouts, NAKs, garbage),\n"
        "                       counters, rates, flags. Adds :f polling; large logs\n"
        "  --orientation O      head configuration: altaz | polar (default polar)\n"
        "                       altaz: RA shaft horizontal due NORTH, tube level;\n"
        "                       session start pose = tube level EAST unless --home-*\n"
        "  --rotctld-port N     TCP port (default 4533)\n"
        "  --lat X --lon Y      observer site, degrees east-positive\n"
        "  --home-az A --home-el E   REAL sky pointing at startup (required)\n"
        "  --ra-sign N --dec-sign N  +1/-1 motor direction convention\n"
        "  --verbose            per-tick rate logging while moving\n"
        "  --version            print build identity and exit\n");
    std::exit(2);
}

bool parse_config_line(Options& c, const std::string& kv, const char* src)
{
    auto eq = kv.find('=');
    if (eq == std::string::npos)
        return false;
    auto trim = [](std::string s) {
        size_t a = s.find_first_not_of(" \t\r\n");
        size_t b = s.find_last_not_of(" \t\r\n");
        return a == std::string::npos ? std::string() : s.substr(a, b - a + 1);
    };
    std::string k = trim(kv.substr(0, eq)), v = trim(kv.substr(eq + 1));
    auto num = [&](double& dst) {
        try { dst = std::stod(v); }
        catch (...) { die(std::string("bad value '") + v + "' for " + k + " in " + src); }
    };
    if (k == "port") { c.port = v; c.have_port = true; }
    else if (k == "mount") c.mount = v;
    else if (k == "no_resume") c.no_resume = (v == "yes" || v == "true" || v == "1");
    else if (k == "orientation") { c.orientation = v; c.have_orientation = true; }
    else if (k == "rotctld_port") { double d; num(d); c.rotctld_port = static_cast<int>(d); c.have_rotctld_port = true; }
    else if (k == "lat") { num(c.lat); c.have_site = true; }
    else if (k == "lon") { num(c.lon); c.have_site = true; }
    else if (k == "home_az") { num(c.home_az); c.have_home = true; }
    else if (k == "home_el") { num(c.home_el); c.have_home = true; }
    else if (k == "ra_sign") { num(c.ra_sign); c.have_ra_sign = true; }
    else if (k == "dec_sign") { num(c.dec_sign); c.have_dec_sign = true; }
    else if (k == "update_hz") { num(c.update_hz); c.have_update_hz = true; }
    else if (k == "target_stale_s") { num(c.target_stale_s); }
    else if (k == "min_elevation_deg") { num(c.min_elevation_deg); c.have_min_el = true; }
    else if (k == "ra_travel_deg") { num(c.ra_travel_deg); c.have_travel = true; }
    else if (k == "dec_travel_deg") { num(c.dec_travel_deg); c.have_travel = true; }
    else if (k == "high_coast_s") { num(c.high_coast_s); c.have_coast = true; }
    else if (k == "coast_margin_deg") { num(c.coast_margin_deg); c.have_coast = true; }
    else if (k == "rehome_idle_s") { num(c.rehome_idle_s); c.have_rehome = true; }
    else if (k == "rehome_arrive_deg") { num(c.rehome_arrive_deg); c.have_rehome = true; }
    else if (k == "rehome_min_el_deg") { num(c.rehome_min_el_deg); c.have_rehome = true; }
    else if (k == "glide_promote_holdoff_s") { num(c.glide_promote_holdoff_s); c.have_promote = true; }
    else if (k == "high_gate_moving_x") { num(c.high_gate_moving_x); c.have_mgate = true; }
    else if (k == "skip_cold_blind_high_s") { num(c.skip_cold_blind_high_s); c.have_coldblind = true; }
    else if (k == "skip_supply_slow_band") { num(c.skip_supply_slow_band); c.have_supply = true; }
    else if (k == "skip_restart_grace_high_s") { num(c.skip_restart_grace_high_s); c.have_rgrace = true; }
    else if (k == "branch_handover_margin_deg") { num(c.branch_handover_margin_deg); c.have_branch_margin = true; }
    else if (k == "wrap_home_unwind") { c.wrap_home_unwind = (v == "yes" || v == "true" || v == "1"); c.have_wrap_home = true; }
    else if (k == "cord_unwind_route") { c.cord_unwind_route = (v == "yes" || v == "true" || v == "1"); c.have_cord_route = true; }
    else if (k == "wrap_window_deg") { num(c.wrap_window_deg); c.have_wrap_window = true; }
    else if (k == "wrap_tiebreak_margin_deg") { num(c.wrap_tiebreak_margin_deg); c.have_wrap_tiebreak = true; }
    else if (k == "wrap_tiebreak_extra_deg") { num(c.wrap_tiebreak_extra_deg); c.have_wrap_tiebreak = true; }
    else if (k == "wall_strain_deg") { num(c.wall_strain_deg); c.have_wall_strain = true; }
    else if (k == "wall_strain_s") { num(c.wall_strain_s); c.have_wall_strain = true; }
    else if (k == "high_recommit_s") { num(c.high_recommit_s); c.have_recommit = true; }
    else if (k == "high_recommit_hold_deg") { num(c.high_recommit_hold_deg); c.have_recommit = true; }
    else if (k == "high_recommit_regime_s") { num(c.high_recommit_regime_s); c.have_recommit = true; }
    else if (k == "high_recommit_regime_age_s") { num(c.high_recommit_regime_age_s); c.have_recommit = true; }
    else if (k == "high_recommit_regime_db") { num(c.high_recommit_regime_db); c.have_recommit = true; }
    else if (k == "high_recommit_escape_deg") { num(c.high_recommit_escape_deg); c.have_recommit = true; }
    else if (k == "demand_lpf_regime_tau_s") { num(c.demand_lpf_regime_tau_s); c.have_lpfreg = true; }
    else if (k == "demand_lpf_regime_min_rc") { num(c.demand_lpf_regime_min_rc); c.have_lpfreg = true; }
    else if (k == "demand_lpf_regime_release_err_deg") { num(c.demand_lpf_regime_release_err_deg); c.have_lpfreg = true; }
    else if (k == "high_gate_err_deg") { num(c.high_gate_err_deg); c.have_recommit = true; }
    else if (k == "track_stop_el") { num(c.track_stop_el); c.have_track_stop = true; }
    else if (k == "max_rate_x_sidereal") { num(c.max_rate_x_sidereal); c.have_max_rate = true; }
    else if (k == "read_only") c.read_only = (v == "yes" || v == "true" || v == "1");
    else if (k == "debug") c.debug = (v == "yes" || v == "true" || v == "1");
    else if (k == "record_file") c.record_file = v;
    else if (!k.empty() && k[0] != '#')
        std::fprintf(stderr, "warning: unknown config key '%s' in %s\n", k.c_str(), src);
    return true;
}

Options parse(int argc, char** argv)
{
    Options c;
    std::vector<std::string> args(argv + 1, argv + argc);

    // pass 1: config file (so command line always wins)
    for (size_t i = 0; i < args.size(); ++i)
        if (args[i] == "--config" && i + 1 < args.size())
        {
            const std::string& path = args[i + 1];
            std::ifstream f(path);
            if (!f)
                die("cannot open config " + path);
            std::string line;
            while (std::getline(f, line))
            {
                size_t a = line.find_first_not_of(" \t");
                if (a == std::string::npos || line[a] == '#')
                    continue;
                parse_config_line(c, line, path.c_str());
            }
        }

    // pass 2: command line
    for (size_t i = 0; i < args.size(); ++i)
    {
        auto next = [&](const char* opt) -> std::string {
            if (++i >= args.size())
                die(std::string("missing value for ") + opt);
            return args[i];
        };
        const std::string& a = args[i];
        if (a == "--config") next("--config"); // already applied
        else if (a == "--port") { c.port = next("--port"); c.have_port = true; }
        else if (a == "--mount") c.mount = next("--mount");
        else if (a == "--no-resume") c.no_resume = true;
        else if (a == "--sim") c.sim = next("--sim");
        else if (a == "--read-only") c.read_only = true;
        else if (a == "--debug") c.debug = true;
        else if (a == "--record") c.record_file = next("--record");
        else if (a == "--orientation") { c.orientation = next("--orientation"); c.have_orientation = true; }
        else if (a == "--rotctld-port") { c.rotctld_port = std::stoi(next("--rotctld-port")); c.have_rotctld_port = true; }
        else if (a == "--lat") { c.lat = std::stod(next("--lat")); c.have_site = true; }
        else if (a == "--lon") { c.lon = std::stod(next("--lon")); c.have_site = true; }
        else if (a == "--home-az") { c.home_az = std::stod(next("--home-az")); c.have_home = true; }
        else if (a == "--home-el") { c.home_el = std::stod(next("--home-el")); c.have_home = true; }
        else if (a == "--ra-sign") { c.ra_sign = std::stod(next("--ra-sign")); c.have_ra_sign = true; }
        else if (a == "--dec-sign") { c.dec_sign = std::stod(next("--dec-sign")); c.have_dec_sign = true; }
        else if (a == "--verbose") c.verbose = true;
        else if (a == "--version")
        {
            std::printf("sateqd %s\n", sateq::version_string().c_str());
            std::exit(0);
        }
        else usage();
    }
    if (std::fabs(c.lat) > 90 || std::fabs(c.lon) > 180)
        die("site out of range: lat in [-90,90], lon in [-180,180] east-positive");
    if (c.ra_sign == 0) c.ra_sign = 1;
    if (c.dec_sign == 0) c.dec_sign = 1;
    return c;
}

} // namespace

int main(int argc, char** argv)
try
{
    Options c = parse(argc, argv);
    log_line("sateqd %s", sateq::version_string().c_str());

    // ---- session layer: --mount NAME resolves station + mount profile ----
    // Precedence: explicit CLI/--config > mount profile > station config
    // (load_session already merged station underneath the profile).
    Paths paths;
    std::string state_path;
    SessionState prior;                 // persisted state, if any (needs --mount)
    if (!c.mount.empty())
    {
        paths = default_paths();
        if (c.mount == "auto")
        {
            // Which mount is wired up? Ask the boards instead of operator
            // memory: probe every profile's own port with the read-only :e1
            // identity query and keep the single profile whose expected
            // mount_code answers there. Query-only by construction (the
            // probe pins the write gate); with both mounts powered the
            // ambiguity is an error, not a coin flip.
            if (!c.sim.empty())
                die("--mount auto needs a real board; with --sim name the profile explicitly");
            std::string detected, derr;
            std::vector<MountCandidate> inv;
            if (!detect_mount(paths,
                              [](const std::string& port) { return probe_mount_code(port); },
                              detected, inv, derr))
                die("--mount auto: " + derr);
            for (const auto& cand : inv)
            {
                if (cand.name != detected && cand.live_code >= 0)
                    log_line("mount auto: also heard 0x%02X on %s (profile %s)",
                             cand.live_code, cand.port.c_str(), cand.name.c_str());
            }
            log_line("mount auto: selected profile '%s' (board answered on its port)",
                     detected.c_str());
            c.mount = detected;
        }
        SessionConfig sc;
        std::string err;
        if (!load_session(paths, c.mount, c.have_orientation ? c.orientation : "", sc, err))
            die("mount '" + c.mount + "': " + err);
        if (!c.have_port)          c.port = sc.port;
        if (!c.have_orientation)   c.orientation =
            (sc.orientation == Orientation::AltAzLevel) ? "altaz" : "polar";
        if (!c.have_rotctld_port)  c.rotctld_port = sc.rotctld_port;
        if (!c.have_site && sc.have_site)
        {
            c.lat = sc.site.lat_deg; c.lon = sc.site.lon_east_deg; c.have_site = true;
        }
        if (!c.have_home)
        {
            HsCoord pk = sc.park_pose();
            c.home_az = pk.az_deg; c.home_el = pk.el_deg; c.have_home = true;
        }
        if (!c.have_ra_sign)   c.ra_sign = sc.ra_sign;
        if (!c.have_dec_sign)  c.dec_sign = sc.dec_sign;
        if (!c.have_update_hz) c.update_hz = sc.update_hz;
        if (!c.have_min_el)    c.min_elevation_deg = sc.min_elevation_deg;
        if (!c.have_travel)   { c.ra_travel_deg = sc.ra_travel_deg;
                                c.dec_travel_deg = sc.dec_travel_deg; }
        if (!c.have_coast)    { c.high_coast_s = sc.high_coast_s;
                                c.coast_margin_deg = sc.coast_margin_deg; }
        if (!c.have_rehome)   { c.rehome_idle_s = sc.rehome_idle_s;
                                c.rehome_arrive_deg = sc.rehome_arrive_deg;
                                c.rehome_min_el_deg = sc.rehome_min_el_deg; }
        if (!c.have_promote)   c.glide_promote_holdoff_s = sc.glide_promote_holdoff_s;
        if (!c.have_mgate)     c.high_gate_moving_x = sc.high_gate_moving_x;
        if (!c.have_coldblind) c.skip_cold_blind_high_s = sc.skip_cold_blind_high_s;
        if (!c.have_supply)      c.skip_supply_slow_band = sc.skip_supply_slow_band;
        if (!c.have_rgrace)      c.skip_restart_grace_high_s = sc.skip_restart_grace_high_s;
        if (!c.have_branch_margin) c.branch_handover_margin_deg = sc.branch_handover_margin_deg;
        if (!c.have_wrap_home)     c.wrap_home_unwind = sc.wrap_home_unwind;
        if (!c.have_cord_route)    c.cord_unwind_route = sc.cord_unwind_route;
        if (!c.have_wrap_window)   c.wrap_window_deg = sc.wrap_window_deg;
        if (!c.have_wrap_tiebreak) { c.wrap_tiebreak_margin_deg = sc.wrap_tiebreak_margin_deg;
                                     c.wrap_tiebreak_extra_deg = sc.wrap_tiebreak_extra_deg; }
        if (!c.have_wall_strain)  { c.wall_strain_deg = sc.wall_strain_deg;
                                    c.wall_strain_s = sc.wall_strain_s; }
        if (!c.have_recommit) {
            c.high_recommit_s = sc.high_recommit_s;
            c.high_recommit_hold_deg = sc.high_recommit_hold_deg;
            c.high_recommit_regime_s = sc.high_recommit_regime_s;
            c.high_recommit_regime_age_s = sc.high_recommit_regime_age_s;
            c.high_recommit_regime_db = sc.high_recommit_regime_db;
            c.high_recommit_escape_deg = sc.high_recommit_escape_deg;
            c.high_gate_err_deg = sc.high_gate_err_deg;
        }
        if (!c.have_lpfreg) { c.demand_lpf_regime_tau_s = sc.demand_lpf_regime_tau_s;
                              c.demand_lpf_regime_min_rc = sc.demand_lpf_regime_min_rc;
                              c.demand_lpf_regime_release_err_deg = sc.demand_lpf_regime_release_err_deg; }
        if (!c.have_track_stop) c.track_stop_el = sc.track_stop_el;
        if (!c.have_max_rate)  c.max_rate_x_sidereal = sc.max_rate_x_sidereal;
        c.backlash_ra_steps = sc.ra_backlash_steps;
        c.backlash_dec_steps = sc.dec_backlash_steps;
        c.expected_mount_code = sc.expected_mount_code;
        state_path = session_state_path(paths, c.mount);
        prior.load(state_path);         // absence is fine: fresh session
        char codebuf[16];
        if (sc.expected_mount_code >= 0)
            std::snprintf(codebuf, sizeof codebuf, "0x%02X", sc.expected_mount_code);
        else
            std::snprintf(codebuf, sizeof codebuf, "any");
        log_line("mount profile '%s': port=%s expected code=%s orientation=%s",
                 c.mount.c_str(), c.port.c_str(), codebuf, c.orientation.c_str());
        if (!sc.backlash_note.empty())
            log_line("%s", sc.backlash_note.c_str());  // measured cal / override / ignored
    }

    if (!c.have_site)
        die("--lat/--lon required (or lat=,lon= in --config)");
    if (c.orientation != "polar" && c.orientation != "altaz")
        die("--orientation must be polar or altaz");
    if (c.orientation == "altaz" && !c.have_home)
    {
        // session start pose (agreed): tube level EAST — the level bubble
        // makes it unambiguous, no home sensor needed
        c.home_az = 90.0;
        c.home_el = 0.0;
        c.have_home = true;
        log_line("orientation=altaz: assuming session start pose (tube level EAST); "
                 "override with --home-az/--home-el");
    }
    if (!c.have_home)
        die("--home-az/--home-el required: state where the scope ACTUALLY points at startup");
    if (c.port.empty() && c.sim.empty())
        die("--port DEV or --sim heq5|eq8 required");

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    TrackerConfig tc;
    tc.site = LatLon{c.lat, c.lon};
    tc.orientation = (c.orientation == "altaz") ? Orientation::AltAzLevel : Orientation::Polar;
    tc.home_az_deg = c.home_az;
    tc.home_el_deg = c.home_el;
    tc.ra_sign = c.ra_sign;
    tc.dec_sign = c.dec_sign;
    tc.update_hz = c.update_hz;
    tc.target_stale_s = c.target_stale_s;
    tc.writes_gated = c.read_only;   // skip detection off while gated (main gate below)
    tc.safety.min_elevation_deg = c.min_elevation_deg;
    tc.safety.track_stop_el = c.track_stop_el;
    tc.safety.max_rate_x_sidereal = c.max_rate_x_sidereal;
    // travel envelope (mechanical): refuse targets whose axis solution would
    // wind past the datum box (the 2026-11-19 stand-strike class of failure).
    if ((c.ra_travel_deg != 0.0 && (c.ra_travel_deg < 5.0 || c.ra_travel_deg > 180.0)) ||
        (c.dec_travel_deg != 0.0 && (c.dec_travel_deg < 5.0 || c.dec_travel_deg > 180.0)))
        die("ra_travel_deg/dec_travel_deg must be 0 (axis disabled) or 5..180");
    tc.safety.max_ra_travel_deg = c.ra_travel_deg;
    tc.safety.max_dec_travel_deg = c.dec_travel_deg;
    if (c.high_coast_s < 0.0 || c.coast_margin_deg < 0.0 ||
        c.rehome_idle_s < 0.0 || c.rehome_arrive_deg < 0.0 ||
        c.rehome_min_el_deg < 0.0)
        die("high_coast_s/coast_margin_deg/rehome_* must be >= 0");
    tc.high_coast_s = c.high_coast_s;
    tc.glide_promote_holdoff_s = c.glide_promote_holdoff_s;
    tc.high_gate_moving_x = c.high_gate_moving_x;
    tc.safety.skip_cold_blind_high_s = c.skip_cold_blind_high_s;
    tc.safety.skip_supply_slow_band = c.skip_supply_slow_band;
    tc.safety.skip_restart_grace_high_s = c.skip_restart_grace_high_s;
    if (c.branch_handover_margin_deg < 0.0)
        die("branch_handover_margin_deg must be >= 0");
    tc.branch_handover_margin_deg = c.branch_handover_margin_deg;
    if (c.wrap_window_deg < 0.0 || c.wrap_tiebreak_margin_deg < 0.0 ||
        c.wrap_tiebreak_extra_deg < 0.0)
        die("wrap_* knobs must be >= 0");
    tc.wrap_home_unwind = c.wrap_home_unwind;
    tc.cord_unwind_route = c.cord_unwind_route;
    tc.wrap_window_deg = c.wrap_window_deg;
    tc.wrap_tiebreak_margin_deg = c.wrap_tiebreak_margin_deg;
    tc.wrap_tiebreak_extra_deg = c.wrap_tiebreak_extra_deg;
    if (c.wall_strain_deg < 0.0 || c.wall_strain_s < 0.0)
        die("wall_strain_* must be >= 0");
    tc.wall_strain_deg = c.wall_strain_deg;
    tc.wall_strain_s = c.wall_strain_s;
    if (c.high_recommit_s < 0.0 || c.high_recommit_hold_deg < 0.0 ||
        c.high_recommit_regime_s < 0.0 || c.high_recommit_regime_age_s < 0.0 ||
        c.high_recommit_regime_db < 0.0 || c.high_recommit_escape_deg < 0.0 ||
        c.high_gate_err_deg < 0.0 || c.demand_lpf_regime_tau_s < 0.0 ||
        c.demand_lpf_regime_min_rc < 0.0 || c.demand_lpf_regime_release_err_deg < 0.0)
        die("high_recommit_*/high_gate_err_deg/demand_lpf_regime_* must be >= 0");
    tc.high_recommit_s = c.high_recommit_s;
    tc.high_recommit_hold_deg = c.high_recommit_hold_deg;
    tc.high_recommit_regime_s = c.high_recommit_regime_s;
    tc.high_recommit_regime_age_s = c.high_recommit_regime_age_s;
    tc.high_recommit_regime_db = c.high_recommit_regime_db;
    tc.high_recommit_escape_deg = c.high_recommit_escape_deg;
    tc.high_gate_err_deg = c.high_gate_err_deg;
    tc.demand_lpf_regime_tau_s = c.demand_lpf_regime_tau_s;
    tc.demand_lpf_regime_min_rc = c.demand_lpf_regime_min_rc;
    tc.demand_lpf_regime_release_err_deg = c.demand_lpf_regime_release_err_deg;
    tc.coast_margin_deg = c.coast_margin_deg;
    tc.rehome_idle_s = c.rehome_idle_s;
    tc.rehome_arrive_deg = c.rehome_arrive_deg;
    tc.rehome_min_el_deg = c.rehome_min_el_deg;
    // Backlash calibration is LOADED (session .cal / profile overrides in
    // c.backlash_*_steps) but NOT yet consumed: the 2026-11-18 measurement-
    // side play model limit-cycled live (AO-73 pass forensics) and was
    // reverted. Redesign = demand-side take-up (pattern/estimators stay
    // backlash-free); re-wire tc.{ra,dec}_backlash_steps when it lands.

    // ---- board bring-up ------------------------------------------------
    std::shared_ptr<Transport> t;
    std::shared_ptr<SimulatorTransport> sim; // non-null only with --sim
    if (!c.sim.empty())
    {
        SimProfile prof = (c.sim == "eq8") ? sim_eq8() : sim_heq5();
        if (c.sim != "eq8" && c.sim != "heq5")
            die("--sim must be heq5 or eq8");
        sim = std::make_shared<SimulatorTransport>(prof);
        t = sim;
        log_line("using built-in simulator profile %s (NO REAL MOUNT)", prof.name.c_str());
    }
    else
    {
        t = std::make_shared<SerialTransport>(c.port);
    }
    Board board(t);
    board.connect();
    if (c.read_only)
    {
        board.block_writes(true);
        log_line("READ-ONLY session: write commands (motion, stops, counter "
                 "writes) are queued and logged, NEVER sent to the board");
    }
    uint8_t code = board.mount_code();
    log_line("connected: mount code 0x%02X (%s)", code, mount_code_name(code));
    if (c.expected_mount_code >= 0 && static_cast<int>(code) != c.expected_mount_code)
    {
        // Wrong hardware on the port: refusing to drive an unexpected mount
        // (its rates/signs/gearing differ). Bypass by fixing the profile's
        // mount_code or not selecting it.
        char want[16], got[16];
        std::snprintf(want, sizeof want, "0x%02X", c.expected_mount_code);
        std::snprintf(got, sizeof got, "0x%02X", code);
        die("mount '" + c.mount + "' expects mount_code " + want +
            " but the board reported " + got + " (" + mount_code_name(code) +
            "): wrong device or stale profile — refusing to move it");
    }
    log_line("orientation: %s (park pose az=%.1f el=%.1f)",
             c.orientation.c_str(), c.home_az, c.home_el);
    {
        char ra[32], de[32];
        if (tc.safety.max_ra_travel_deg > 0.0)
            std::snprintf(ra, sizeof ra, "RA +-%.0f deg", tc.safety.max_ra_travel_deg);
        else std::snprintf(ra, sizeof ra, "RA unlimited");
        if (tc.safety.max_dec_travel_deg > 0.0)
            std::snprintf(de, sizeof de, "DEC +-%.0f deg", tc.safety.max_dec_travel_deg);
        else std::snprintf(de, sizeof de, "DEC unlimited");
        log_line("travel envelope: %s %s (datum-relative, both branches; "
                 "illegal targets return to park, resume automatically)", ra, de);
    }
    log_line("coast gate: %s (HIGH :K landing model tau=%.2f s, margin=%.2f deg)",
             tc.high_coast_s > 0.0 ? "ON" : "off",
             tc.high_coast_s, tc.coast_margin_deg);
    log_line("re-home: %s (fly datum after %.0f s target-gone, arrive %.2f deg, "
             "arm only past el %.1f deg)",
             tc.rehome_idle_s > 0.0 ? "ON" : "off",
             tc.rehome_idle_s, tc.rehome_arrive_deg, tc.rehome_min_el_deg);
    if (tc.orientation == Orientation::Polar)
        log_line("note: polar orientation — prograde satellites cross the meridian "
                 "mid-pass; expect a go-around (~1 min off-target) near transit");
    bool eq8 = board.probe_eq8_features();
    log_line("eq8-class features: %s", eq8 ? "yes" : "no");
    if (!c.sim.empty())
        eq8 = (c.sim == "eq8"); // sim profile decides

    for (char ax : {'1', '2'})
    {
        AxisInfo i = board.inquire_axis(ax);
        log_line("axis %c: steps/360=%u steps/worm=%u hs_ratio=%u (%.4f arcsec/step)",
                 ax, i.steps_360, i.steps_worm, i.hs_ratio, i.arcsec_per_step());
    }
    if (eq8)
    {
        // HDX-110: home-sensor encoder counts are reference telemetry only —
        // we NEVER zero counters with them, we only log for the operator.
        try
        {
            log_line("home sensor: RA=0x%06X DEC=0x%06X",
                     board.home_sensor(AxisRA), board.home_sensor(AxisDE));
        }
        catch (const std::exception& e)
        {
            log_line("home sensor unavailable: %s", e.what());
        }
    }

    // ---- tracker + reference pointing -----------------------------------
    // Resume iff a prior session state exists AND the live counters still sit
    // at the persisted reference (the mount never moved while we were down):
    // then the cord-wrap rotation budget continues. Otherwise calibrate
    // fresh — the operator-stated park pose IS the re-reference ritual.
    const uint64_t started_unix = static_cast<uint64_t>(std::time(nullptr));
    Tracker tracker(board, tc);
    bool resumed = false;
    if (!c.no_resume && prior.valid)
    {
        if (!prior.orientation.empty() && prior.orientation != c.orientation)
        {
            // axes↔counter mapping differs between orientations: the saved
            // reference axes would be interpreted wrongly. Re-reference.
            log_line("prior state was orientation '%s', this session is '%s': "
                     "starting fresh budget at park pose",
                     prior.orientation.c_str(), c.orientation.c_str());
        }
        else
        {
            const uint32_t live_ra = board.position(AxisRA);
            const uint32_t live_de = board.position(AxisDE);
            if (prior.resumable(live_ra, live_de))
            {
                tracker.restore(prior.ra_ref, prior.de_ref,
                                AxisAngles{prior.ref_axes_ra_deg, prior.ref_axes_dec_deg},
                                std::chrono::system_clock::now(), std::chrono::steady_clock::now());
                resumed = true;
                log_line("resumed session: counters RA=0x%06X DE=0x%06X unchanged since "
                         "last run — rotation budget continues",
                         prior.ra_ref, prior.de_ref);
            }
            else
            {
                log_line("prior state present but mount moved while down (live RA=0x%06X "
                         "DE=0x%06X vs saved RA=0x%06X DE=0x%06X): starting fresh budget",
                         live_ra, live_de, prior.ra_ref, prior.de_ref);
            }
        }
    }
    if (!resumed)
        tracker.calibrate(std::chrono::system_clock::now(), std::chrono::steady_clock::now());
    {
        HsCoord p = tracker.current_pointing(std::chrono::system_clock::now());
        log_line("%s: reference pointing az=%.2f el=%.2f (stated home az=%.2f el=%.2f)",
                 resumed ? "resumed" : "calibrated",
                 p.az_deg, p.el_deg, c.home_az, c.home_el);
    }

    // ---- target-push recorder (--record) ----------------------------------
    // Append-only CSV of every rotctld `P` (raw az/el + timestamps) so a
    // real client's push cadence (GT2 throttle etc.) can be replayed
    // against the tracker in simulation. Flush per line: a pass is short
    // and a crash must not eat the dataset.
    std::ofstream record_out;
    if (!c.record_file.empty())
    {
        record_out.open(c.record_file, std::ios::app);
        if (!record_out)
            die("cannot open --record file " + c.record_file);
        if (record_out.tellp() == 0)
            record_out << "epoch_s,mono_s,az_deg,el_deg\n";
        const auto mono_t0 = std::chrono::steady_clock::now();
        tracker.set_target_tap([&record_out, mono_t0](
                                   std::chrono::steady_clock::time_point mono,
                                   double az, double el) {
            const double epoch = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();
            const double mono_s = std::chrono::duration<double>(mono - mono_t0).count();
            record_out << std::fixed << std::setprecision(3)
                       << epoch << ',' << mono_s << ',' << az << ',' << el << '\n';
            record_out.flush();
        });
        log_line("recording target pushes to %s", c.record_file.c_str());
    }

    // ---- tick tracing (--debug) -------------------------------------------
    // One [TICK] key=value row per control tick carrying the full wire
    // exchange (raw frames, timeouts, NAKs, garbage, write-gate holds),
    // counters, control state and event edges. The sink runs under the
    // tracker lock; log_line serializes, truncates and flushes per line.
    if (c.debug)
    {
        tracker.set_debug_sink(
            [](const std::string& row) { log_line("%s", row.c_str()); });
        tracker.set_debug(true);
        log_line("tick tracing ON: one [TICK] row per control tick (full wire "
                 "trace; adds one :f poll per axis per tick; expect ~170 MB/h)");
    }

    // control-law narrative lines (branch handover census) into the main
    // log: grep "branch handover" post-pass.
    tracker.set_log_sink([](const std::string& s) { log_line("%s", s.c_str()); });

    // ---- rotctld server --------------------------------------------------
    RotctldServer server(c.rotctld_port, tracker);
    server.set_client_observer([&tracker](int live) {
        log_line("rotctld clients connected: %d", live);
        tracker.on_clients_changed(live);  // last client gone => drop target
    });
    server.start();
    log_line("rotctld server listening on port %d", c.rotctld_port);

    // ---- control loop ----------------------------------------------------
    // Same 10 Hz cadence as Runner; with --sim we also drive the virtual
    // clock from the wall clock so the simulated mount animates in real time.
    const auto tick = std::chrono::duration<double>(1.0 / c.update_hz);
    auto wall_prev = std::chrono::steady_clock::now();
    std::string last_logged_state;
    std::string last_rl_logged;
    auto last_rl_time = std::chrono::steady_clock::now() - std::chrono::seconds(60);
    std::string last_fg_logged;
    auto last_fg_time = last_rl_time;
    std::string last_tr_logged;   // travel-envelope refusal reason
    int last_sr_total = 0;
    auto last_sr_time = last_rl_time;
    int status_counter = 0;
    bool clean_exit = false;
    while (!g_stop.load())
    {
        auto t_start = std::chrono::steady_clock::now();
        double sim_dt = std::chrono::duration<double>(t_start - wall_prev).count();
        wall_prev = t_start;
        if (sim)
            sim->advance(sim_dt);
        try
        {
            tracker.step(std::chrono::system_clock::now(), t_start);
        }
        catch (const std::exception& e)
        {
            log_line("control tick failed: %s", e.what());
        }
        auto s = tracker.snapshot();
        std::ostringstream st;
        st << (s.halted ? "HALTED" : s.tracking ? "TRACK" : "IDLE");
        if (s.halted)
            st << ": " << s.halt_reason;
        std::string stx = st.str();
        if (stx != last_logged_state)
        {
            log_line("state: %s", stx.c_str());
            last_logged_state = stx;
            last_rl_logged.clear();
        }
        // rate-limit is a transient condition during acquisition; log its
        // rising edge at most every 10 s instead of every flip-flop.
        if (s.rate_limited && last_rl_logged != stx &&
            std::chrono::steady_clock::now() - last_rl_time > std::chrono::seconds(10))
        {
            log_line("rate limit engaged while %s (demanded rate clamped)", stx.c_str());
            last_rl_logged = stx;
            last_rl_time = std::chrono::steady_clock::now();
        }
        // trajectory floor guard: same rising-edge treatment — during a legal
        // floor slide it is active on most ticks and must not spam.
        if (s.floor_guard && last_fg_logged != stx &&
            std::chrono::steady_clock::now() - last_fg_time > std::chrono::seconds(10))
        {
            log_line("floor guard engaged while %s: setpoint shortened to keep "
                     "el >= %.1f deg (target=(%.2f,%.2f) current=(%.2f,%.2f))",
                     stx.c_str(), c.min_elevation_deg,
                     s.target.az_deg, s.target.el_deg, s.current.az_deg, s.current.el_deg);
            last_fg_logged = stx;
            last_fg_time = std::chrono::steady_clock::now();
        }
        // travel-envelope refusal: log the rising edge of the reason text;
        // while returning home the target keeps streaming but the reference
        // is HOME, and tracking resumes automatically once it is legal again.
        if (s.travel_returning && last_tr_logged != s.travel_reason)
        {
            log_line("travel envelope: returning to park — %s", s.travel_reason.c_str());
            last_tr_logged = s.travel_reason;
        }
        else if (!s.travel_returning && !last_tr_logged.empty())
        {
            log_line("travel envelope: target legal again, resuming tracking");
            last_tr_logged.clear();
        }
        if (c.verbose && (s.rate_ra_x != 0 || s.rate_dec_x != 0))
            log_line("rates: RA=%+.1fx DEC=%+.1fx target=(%.2f,%.2f) current=(%.2f,%.2f)",
                     s.rate_ra_x, s.rate_dec_x,
                     s.target.az_deg, s.target.el_deg, s.current.az_deg, s.current.el_deg);
        // board :K stop/restart reprograms (band/dir changes in slew_rate):
        // each one serializes the tick on the deceleration tail and blanks
        // the skip detector — during dives they must be visible in the log.
        // 2 s throttle keeps the forensics (which from->to transitions fire)
        // dense enough to attribute storms without flooding normal passes.
        {
            int sr = s.stop_restart_ra + s.stop_restart_de;
            if (sr != last_sr_total &&
                std::chrono::steady_clock::now() - last_sr_time > std::chrono::seconds(2))
            {
                log_line("wire stop/restart: RA=%d DEC=%d (last: %s) :K reprograms",
                         s.stop_restart_ra, s.stop_restart_de,
                         s.last_stop_restart.c_str());
                last_sr_total = sr;
                last_sr_time = std::chrono::steady_clock::now();
            }
        }
        if (++status_counter >= 600) // ~minute summary
        {
            status_counter = 0;
            log_line("status: %s target=(%.2f,%.2f) current=(%.2f,%.2f) rates=(%+.1fx,%+.1fx)",
                     stx.c_str(), s.target.az_deg, s.target.el_deg,
                     s.current.az_deg, s.current.el_deg, s.rate_ra_x, s.rate_dec_x);
        }
        std::this_thread::sleep_for(tick - (std::chrono::steady_clock::now() - t_start));
    }

    // ---- shutdown: stop motors, then tear down ----------------------------
    log_line("shutting down");
    try
    {
        board.stop_all(true);
        clean_exit = true;
    }
    catch (const std::exception& e)
    {
        log_line("WARNING: stop failed: %s (mount may still be commanded!)", e.what());
    }
    (void)clean_exit;
    // Persist the session reference (counters + axes at calibration) so a
    // restart can resume the rotation budget if the mount never moved. Read-
    // only bench sessions never move the mount, so we leave state untouched —
    // resuming from a stale real-world pose would be wrong here.
    if (!c.mount.empty() && !c.read_only && clean_exit && tracker.calibrated())
    {
        if (tracker.board_reset_latched())
        {
            // The board was power-cycled during this session: the tracker
            // anchor (and thus any (counters, pose) pair we could persist) is
            // a lie. A 2026-11-20 live session lived the alternative: an idle daemon
            // watched a power cycle at exit, re-anchored the fresh counters
            // against the stale anchor, and the next daemon resumed 159 deg
            // away from the tube. Drop the state entirely: the next start
            // fresh-calibrates at the operator-homed park pose.
            std::remove(state_path.c_str());
            log_line("board was reset mid-session: DISCARDED session state %s "
                     "(anchor invalid) — re-home the mount; next start "
                     "calibrates fresh at the park pose", state_path.c_str());
        }
        else
        {
        // Re-anchor to the CURRENT (counters, pose) pair, not the calibration
        // one: a daemon restart (mount stays powered, nothing moves) must be
        // resumable from ANY pose, and "live == anchor" then remains exactly
        // the right "has it moved since we last watched it" test for BOTH the
        // daemon-restart and the power-cycle case (a power cycle resets the
        // counters to 0x800000, which fails the slop check unless the session
        // ended at that very anchor — same detection the calibration anchor
        // gave, without false-rejecting every dive-then-restart session).
        // Storing the calibration pair instead made every mid-sky restart look
        // like "mount moved while down" (live bug 2026-11-18: restart after a
        // dive fresh-calibrated at home while the tube sat 80 deg away).
        AxisAngles cur = tracker.current_axes(); // axes match live counters
        SessionState st;
        st.ra_ref = tracker.live_count_ra();
        st.de_ref = tracker.live_count_de();
        st.ref_axes_ra_deg = cur.ra_deg;
        st.ref_axes_dec_deg = cur.dec_deg;
        st.ref_utc_unix = std::chrono::duration_cast<std::chrono::seconds>(
                              std::chrono::system_clock::now().time_since_epoch())
                              .count();
        st.started_unix = started_unix;
        st.orientation = c.orientation;
        if (st.save(state_path))
            log_line("saved session state: %s (RA=0x%06X DE=0x%06X)",
                     state_path.c_str(), st.ra_ref, st.de_ref);
        else
            log_line("WARNING: could not write session state %s", state_path.c_str());
        }
    }
    if (c.read_only)
        log_line("read-only session: %zu write frame(s) queued at exit, %zu dropped, NONE sent",
                 board.held_count(), board.held_dropped());
    return 0;
}
catch (const std::exception& e)
{
    die(e.what());
}
