#include "sateq/session.h"

#include <cerrno>
#include <cstdlib>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <grp.h>
#include <pwd.h>
#include <sys/stat.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace sateq {

std::string Paths::config_dir() const
{
    // See declaration: $SATEQ_CONFIG_DIR > %APPDATA%\sateq (Windows) >
    // $XDG_CONFIG_HOME/sateq > <home>/.config/sateq.
    if (const char* s = std::getenv("SATEQ_CONFIG_DIR"))
        if (*s)
            return s;
#ifdef _WIN32
    if (const char* a = std::getenv("APPDATA"))
        if (*a)
            return std::string(a) + "\\sateq";
#endif
    std::string base;
    if (const char* x = std::getenv("XDG_CONFIG_HOME"))
        if (*x)
            base = x;
    if (base.empty())
        base = home + "/.config";
    return base + "/sateq";
}

namespace {
// State home: $SATEQ_STATE_DIR > %LOCALAPPDATA%\sateq > $XDG_STATE_HOME/sateq
// (default ~/.local/state/sateq). Machine-generated persistent data —
// session state and measured calibration — lives here, never in config/.
std::string state_dir_default(const std::string& home)
{
    if (const char* s = std::getenv("SATEQ_STATE_DIR"))
        if (*s)
            return s;
#ifdef _WIN32
    if (const char* a = std::getenv("LOCALAPPDATA"))
        if (*a)
            return std::string(a) + "\\sateq";
#endif
    std::string base;
    if (const char* x = std::getenv("XDG_STATE_HOME"))
        if (*x)
            base = x;
    if (base.empty())
        base = home + "/.local/state";
    return base + "/sateq";
}
} // namespace

Paths default_paths()
{
    Paths p;
    if (p.home.empty())
        if (const char* h = std::getenv("HOME"))
            p.home = h;
    if (p.home.empty())
    {
        if (passwd* pw = getpwuid(getuid()))
            p.home = pw->pw_dir;
    }
    if (p.station_file.empty())
        p.station_file = p.config_dir() + "/station.conf";
    if (p.mounts_dir.empty())
        p.mounts_dir = p.config_dir() + "/mounts";
    if (p.state_dir.empty())
        p.state_dir = state_dir_default(p.home);
    return p;
}

HsCoord SessionConfig::park_pose() const
{
    if (have_park_pose)
        return HsCoord{park_az_deg, park_el_deg};
    if (orientation == Orientation::AltAzLevel)
        return HsCoord{90.0, 0.0}; // agreed session start: tube level EAST
    return HsCoord{park_az_deg, park_el_deg}; // polar: validation ensures stated
}

namespace {

bool parse_code(const std::string& s, int& out)
{
    // accepts "0x01", "1", "01" — profile-friendly hex with 0x, decimal too
    try
    {
        size_t used = 0;
        long v = std::stol(s, &used, 0);
        if (used != s.size() || v < 0 || v > 255)
            return false;
        out = static_cast<int>(v);
        return true;
    }
    catch (const std::exception&)
    {
        return false;
    }
}

bool parse_orientation(const std::string& s, Orientation& o)
{
    if (s == "polar") { o = Orientation::Polar; return true; }
    if (s == "altaz") { o = Orientation::AltAzLevel; return true; }
    return false;
}

} // namespace

bool load_session(const Paths& paths, const std::string& mount_name,
                  const std::string& orientation_override,
                  SessionConfig& out, std::string& err)
{
    if (mount_name.empty())
    {
        err = "no mount selected (use --mount NAME or port= in station.conf)";
        return false;
    }

    // precedence: CLI handles itself above this layer; here profile > station
    fs::path prof_path = fs::path(paths.mounts_dir) / (mount_name + ".conf");
    std::error_code ec;
    if (!fs::exists(prof_path, ec))
    {
        err = "mount profile not found: " + prof_path.string();
        return false;
    }

    Config profile, station;
    if (!profile.load(prof_path.string()))
    {
        err = "cannot read mount profile: " + prof_path.string();
        return false;
    }
    bool have_station = station.load(paths.station_file); // optional at this stage
    profile.merge_defaults(station);
    Config& c = profile;

    out.name = mount_name;
    out.port = c.str("port");
    if (c.has("mount_code"))
    {
        if (!parse_code(c.str("mount_code"), out.expected_mount_code))
        {
            err = prof_path.string() + ": bad mount_code value '" + c.str("mount_code") + "'";
            return false;
        }
    }
    std::string ori = c.str("orientation", "");
    if (!orientation_override.empty())
        ori = orientation_override; // CLI wins (caller pre-applies, but honor here too)
    if (!ori.empty() && !parse_orientation(ori, out.orientation))
    {
        err = prof_path.string() + ": orientation must be polar or altaz, got '" + ori + "'";
        return false;
    }
    if (c.has("park_az") || c.has("park_el"))
    {
        if (!(c.has("park_az") && c.has("park_el")))
        {
            err = prof_path.string() + ": park_az and park_el must be given together";
            return false;
        }
        out.park_az_deg = c.num("park_az", 0);
        out.park_el_deg = c.num("park_el", 0);
        out.have_park_pose = true;
    }
    out.ra_sign = c.num("ra_sign", 1.0);
    out.dec_sign = c.num("dec_sign", 1.0);
    out.max_rate_x_sidereal = c.num("max_rate_x_sidereal", 600.0);
    out.min_elevation_deg = c.num("min_elevation_deg", -15.0);
    out.ra_travel_deg = c.num("ra_travel_deg", 105.0);
    out.dec_travel_deg = c.num("dec_travel_deg", 105.0);
    out.high_coast_s = c.num("high_coast_s", 0.0);
    out.coast_margin_deg = c.num("coast_margin_deg", 0.3);
    out.rehome_idle_s = c.num("rehome_idle_s", 0.0);
    out.rehome_arrive_deg = c.num("rehome_arrive_deg", 0.5);
    out.rehome_min_el_deg = c.num("rehome_min_el_deg", 2.0);
    out.glide_promote_holdoff_s = c.num("glide_promote_holdoff_s", 0.0);
    out.high_gate_moving_x = c.num("high_gate_moving_x", 90.0);
    out.skip_cold_blind_high_s = c.num("skip_cold_blind_high_s", 2.0);
    out.skip_supply_slow_band = c.num("skip_supply_slow_band", 0.75);
    out.skip_restart_grace_high_s = c.num("skip_restart_grace_high_s", 4.0);
    out.branch_handover_margin_deg = c.num("branch_handover_margin_deg", 5.0);
    out.wrap_home_unwind = c.flag("wrap_home_unwind", true);
    out.cord_unwind_route = c.flag("cord_unwind_route", true);
    out.wrap_window_deg = c.num("wrap_window_deg", 270.0);
    out.wrap_tiebreak_margin_deg = c.num("wrap_tiebreak_margin_deg", 90.0);
    out.wrap_tiebreak_extra_deg = c.num("wrap_tiebreak_extra_deg", 30.0);
    out.wall_strain_deg = c.num("wall_strain_deg", 10.0);
    out.wall_strain_s = c.num("wall_strain_s", 5.0);
    out.high_recommit_s = c.num("high_recommit_s", 1.0);
    out.high_recommit_hold_deg = c.num("high_recommit_hold_deg", 0.1);
    out.high_recommit_regime_s = c.num("high_recommit_regime_s", 4.0);
    out.high_recommit_regime_age_s = c.num("high_recommit_regime_age_s", 1.5);
    out.high_recommit_regime_db = c.num("high_recommit_regime_db", 1e9);
    out.high_recommit_escape_deg = c.num("high_recommit_escape_deg", 2.0);
    out.demand_lpf_regime_tau_s = c.num("demand_lpf_regime_tau_s", 1.0);
    out.demand_lpf_regime_min_rc = c.num("demand_lpf_regime_min_rc", 12.0);
    out.demand_lpf_regime_release_err_deg = c.num("demand_lpf_regime_release_err_deg", 1.0);
    out.high_gate_err_deg = c.num("high_gate_err_deg", 2.0);
    out.track_stop_el = c.num("track_stop_el", -5.0);
    out.home_sensor = c.flag("home_sensor", false);
    out.rotctld_port = c.integer("rotctld_port", 4533);
    out.update_hz = c.num("update_hz", 10.0);
    // explicit operator override of the measured backlash cal (see below)
    const double bl_ra = c.num("backlash_ra_steps", -1.0);
    const double bl_de = c.num("backlash_dec_steps", -1.0);
    if (c.has("lat") && c.has("lon"))
    {
        out.site = LatLon{c.num("lat", 0), c.num("lon", 0)};
        out.have_site = true;
    }

    // ---- validation ------------------------------------------------------
    if (out.port.empty())
    {
        err = prof_path.string() + ": port= is required (use /dev/serial/by-id/...)";
        return false;
    }
    if (std::fabs(out.ra_sign) != 1.0 || std::fabs(out.dec_sign) != 1.0)
    {
        err = prof_path.string() + ": ra_sign/dec_sign must be +1 or -1";
        return false;
    }
    if (out.max_rate_x_sidereal <= 0 || out.max_rate_x_sidereal > 800)
    {
        err = prof_path.string() + ": max_rate_x_sidereal must be in (0, 800]";
        return false;
    }
    if (out.track_stop_el < out.min_elevation_deg || out.track_stop_el > 90.0)
    {
        err = prof_path.string() +
              ": track_stop_el must be >= min_elevation_deg and <= 90 (mechanical "
              "floor is the hard limit; the track gate must never allow lower)";
        return false;
    }
    if ((out.ra_travel_deg != 0.0 && (out.ra_travel_deg < 5.0 || out.ra_travel_deg > 180.0)) ||
        (out.dec_travel_deg != 0.0 && (out.dec_travel_deg < 5.0 || out.dec_travel_deg > 180.0)))
    {
        err = prof_path.string() +
              ": ra_travel_deg/dec_travel_deg must be 0 (axis disabled) or 5..180 "
              "(datum-relative mechanical envelope half-width per axis)";
        return false;
    }
    if (out.orientation == Orientation::Polar && !out.have_park_pose)
    {
        err = std::string(prof_path.string()) +
              ": polar profiles must state park_az/park_el (altaz defaults to tube level EAST)";
        return false;
    }
    if (out.have_park_pose &&
        (out.park_az_deg < 0 || out.park_az_deg >= 360 || std::fabs(out.park_el_deg) > 90))
    {
        err = prof_path.string() + ": park_az out of [0,360) or park_el out of [-90,90]";
        return false;
    }
    if (out.have_site && (std::fabs(out.site.lat_deg) > 90 ||
                          std::fabs(out.site.lon_east_deg) > 180))
    {
        err = "station config: lat/lon out of range";
        return false;
    }
    if ((c.has("backlash_ra_steps") && bl_ra < 0.0) ||
        (c.has("backlash_dec_steps") && bl_de < 0.0))
    {
        err = prof_path.string() + ": backlash_ra_steps/backlash_dec_steps must be >= 0";
        return false;
    }

    // ---- backlash slop: explicit profile keys win; else load the measured
    // cal file written by `eqtool backlash` on this machine. A cal taken on a
    // different board (mount_code guard) is ignored, loudly — never silently
    // applied to the wrong hardware.
    if (bl_ra >= 0.0 || bl_de >= 0.0)
    {
        out.ra_backlash_steps = bl_ra < 0.0 ? 0.0 : bl_ra;   // one-sided override: other = 0
        out.dec_backlash_steps = bl_de < 0.0 ? 0.0 : bl_de;
        char onbuf[160];
        std::snprintf(onbuf, sizeof onbuf,
                      "backlash override from profile: RA=%.0f DEC=%.0f steps",
                      out.ra_backlash_steps, out.dec_backlash_steps);
        out.backlash_note = onbuf;
    }
    else
    {
        BacklashCal cal;
        const std::string calp = backlash_cal_path(paths, mount_name);
        if (cal.load(calp))
        {
            char guard[160];
            if (out.expected_mount_code >= 0 && cal.mount_code >= 0 &&
                cal.mount_code != out.expected_mount_code)
            {
                std::snprintf(guard, sizeof guard,
                              "backlash cal IGNORED (%s): taken on mount code 0x%02X, "
                              "profile expects 0x%02X — re-measure with eqtool backlash",
                              mount_name.c_str(), cal.mount_code, out.expected_mount_code);
                out.backlash_note = guard;
            }
            else
            {
                out.ra_backlash_steps = cal.ra_steps < 0.0 ? 0.0 : cal.ra_steps;
                out.dec_backlash_steps = cal.dec_steps < 0.0 ? 0.0 : cal.dec_steps;
                std::snprintf(guard, sizeof guard,
                              "measured backlash: RA=%.0f DEC=%.0f steps (%s%s)",
                              out.ra_backlash_steps, out.dec_backlash_steps,
                              cal.method.empty() ? "measured" : cal.method.c_str(),
                              cal.ra_steps < 0.0 || cal.dec_steps < 0.0 ? ", partial" : "");
                out.backlash_note = guard;
            }
        }
    }
    (void)have_station; // station optionality surfaced by caller via station_file existence
    for (const std::string& w : profile.warnings())
        std::fprintf(stderr, "warning: %s\n", w.c_str());
    return true;
}

// ---- mount auto-detection ---------------------------------------------------

bool detect_mount(const Paths& paths, const MountProbe& probe,
                  std::string& mount_name_out,
                  std::vector<MountCandidate>& inventory_out,
                  std::string& err)
{
    inventory_out.clear();
    mount_name_out.clear();

    std::error_code ec;
    fs::directory_iterator it(fs::path(paths.mounts_dir), ec);
    if (ec)
    {
        err = "cannot list mounts directory " + paths.mounts_dir + ": " + ec.message();
        return false;
    }
    std::vector<fs::path> files;
    for (const auto& e : it)
    {
        if (!ec && e.is_regular_file() && e.path().extension() == ".conf")
            files.push_back(e.path());
    }
    std::sort(files.begin(), files.end());   // deterministic probe order

    // One probe per distinct port: two profiles describing the same cable must
    // not produce two different answers, and probing twice only doubles the
    // timeout the user waits on when nothing answers.
    std::map<std::string, int> probed;
    for (const auto& f : files)
    {
        Config c;
        if (!c.load(f.string()))
            continue;                       // unreadable profile: not a candidate
        MountCandidate cand;
        cand.name = f.stem().string();
        cand.port = c.str("port");
        if (!c.has("mount_code") || !parse_code(c.str("mount_code"), cand.expected_code))
            continue;                       // cannot identify: skipped below
        if (cand.port.empty())
        {
            cand.live_code = -2;
            inventory_out.push_back(cand);
            continue;
        }
        auto known = probed.find(cand.port);
        if (known == probed.end())
        {
            int code = -1;
            try { code = probe(cand.port); }
            catch (...) { code = -1; }      // probe must not throw; belt and braces
            known = probed.emplace(cand.port, code).first;
        }
        cand.live_code = known->second;
        inventory_out.push_back(cand);
    }

    std::vector<const MountCandidate*> matches;
    for (const MountCandidate& cand : inventory_out)
        if (cand.is_match())
            matches.push_back(&cand);

    auto describe = [](const MountCandidate& cand) {
        const char* state = cand.live_code == -2 ? "skipped (no port)"
                            : cand.live_code == -1 ? "no answer"
                            : "answered";
        char buf[256];
        std::snprintf(buf, sizeof buf, "  %-12s port=%-38s expects=0x%02X %s",
                      cand.name.c_str(), cand.port.c_str(), cand.expected_code, state);
        std::string line(buf);
        if (cand.live_code >= 0)
        {
            char cb[24];
            std::snprintf(cb, sizeof cb, " 0x%02X", cand.live_code);
            line += cb;
            if (!cand.is_match())
                line += " (MISMATCH)";
        }
        return line;
    };

    if (matches.size() == 1)
    {
        mount_name_out = matches.front()->name;
        return true;
    }
    std::string inv;
    for (const MountCandidate& cand : inventory_out)
        inv += "\n" + describe(cand);
    if (matches.empty())
    {
        err = "no mount profile matched a board on its own port. Probed:" +
              (inv.empty() ? std::string("  (no usable profiles; each needs port"
                                         " and mount_code)")
                           : inv) +
              "\nCheck mount power/cable, or select a profile explicitly with --mount NAME.";
    }
    else
    {
        err = "several mount profiles matched a board on their own port";
        for (const MountCandidate* cand : matches)
            err += "\n  " + cand->name + " (port " + cand->port + ")";
        err += "\nBoth mounts are likely powered and wired: pick one with --mount NAME.";
    }
    return false;
}

// ---- persisted session state ---------------------------------------------

bool SessionState::load(const std::string& path)
{
    Config c;
    if (!c.load(path))
        return false;
    if (!(c.has("ra_ref") && c.has("de_ref")))
        return false; // not a state file we understand
    try
    {
        ra_ref = static_cast<uint32_t>(c.num("ra_ref", -1));
        de_ref = static_cast<uint32_t>(c.num("de_ref", -1));
        ref_axes_ra_deg = c.num("ref_axes_ra_deg", 0);
        ref_axes_dec_deg = c.num("ref_axes_dec_deg", 0);
        ref_utc_unix = static_cast<uint64_t>(c.num("ref_utc_unix", 0));
        started_unix = static_cast<uint64_t>(c.num("started_unix", 0));
        orientation = c.str("orientation", "");
    }
    catch (const std::exception&)
    {
        return false; // corrupt state: treat as absent (fresh session)
    }
    if (ra_ref > 0xFFFFFF || de_ref > 0xFFFFFF)
        return false;
    valid = true;
    return true;
}

bool SessionState::save(const std::string& path) const
{
    std::error_code ec;
    fs::path p(path);
    if (p.has_parent_path())
        fs::create_directories(p.parent_path(), ec);
    std::ofstream f(path, std::ios::trunc);
    if (!f)
        return false;
    f << "# sateq session state — written by sateqd; delete to reset budget\n"
      << "ra_ref = 0x" << std::hex << (ra_ref & 0xFFFFFF) << "\n"
      << "de_ref = 0x" << std::hex << (de_ref & 0xFFFFFF) << "\n"
      << std::dec
      << "ref_axes_ra_deg = " << ref_axes_ra_deg << "\n"
      << "ref_axes_dec_deg = " << ref_axes_dec_deg << "\n"
      << "ref_utc_unix = " << ref_utc_unix << "\n"
      << "started_unix = " << started_unix << "\n"
      << "orientation = " << orientation << "\n";
    return static_cast<bool>(f);
}

bool SessionState::resumable(uint32_t live_ra, uint32_t live_de, int slop) const
{
    if (!valid)
        return false;
    auto plausible = [slop](uint32_t saved, uint32_t live) {
        // Directed delta mod 2^24: powered-off motion is impossible, so the
        // live counter must sit within `slop` steps AHEAD of the saved one on
        // the ring (behind = something dragged the train; huge forward = the
        // mount was re-homed or powered through a wrap).
        uint32_t d = (live - saved) & 0xFFFFFF;
        return d <= static_cast<uint32_t>(slop);
    };
    return plausible(ra_ref, live_ra) && plausible(de_ref, live_de);
}

std::string session_state_path(const Paths& paths, const std::string& mount_name)
{
    return paths.state_dir + "/" + mount_name + ".state";
}

// ---- measured backlash calibration -----------------------------------------

std::string backlash_cal_path(const Paths& paths, const std::string& mount_name)
{
    return paths.state_dir + "/" + mount_name + ".cal";
}

bool BacklashCal::load(const std::string& path)
{
    Config c;
    if (!c.load(path))
        return false;
    try
    {
        ra_steps = c.num("ra_backlash_steps", -1.0);
        dec_steps = c.num("dec_backlash_steps", -1.0);
    }
    catch (const std::exception&)
    {
        return false;  // a corrupt axis value is a corrupt cal
    }
    if (ra_steps < 0.0 && dec_steps < 0.0)
        return false;  // nothing measured in here
    if (c.has("mount_code"))
    {
        std::string s = c.str("mount_code");
        try
        {
            size_t used = 0;
            long v = std::stol(s, &used, 0);
            if (used == s.size() && v >= 0 && v <= 255)
                mount_code = static_cast<int>(v);
        }
        catch (const std::exception&)
        {
        }
    }
    if (c.has("utc_unix"))
    {
        try
        {
            utc_unix = std::stoull(c.str("utc_unix"));
        }
        catch (const std::exception&)
        {
        }
    }
    method = c.str("method");
    ra_samples = c.str("ra_samples");
    dec_samples = c.str("dec_samples");
    valid = true;
    return true;
}

bool BacklashCal::save(const std::string& path) const
{
    std::error_code ec;
    fs::path p(path);
    if (!p.parent_path().empty())
        fs::create_directories(p.parent_path(), ec);
    std::ofstream f(path);
    if (!f)
        return false;
    f << "# sateq backlash calibration — written by eqtool backlash\n"
      << "# issued steps from a direction reversal until physical motion resumes.\n"
      << "# delete this file to declare the measurement void.\n"
      << "utc_unix = " << utc_unix << "\n";
    if (mount_code >= 0)
        f << "mount_code = 0x" << std::hex << mount_code << std::dec << "\n";
    if (!method.empty())
        f << "method = " << method << "\n";
    if (ra_steps >= 0.0)
        f << "ra_backlash_steps = " << static_cast<long long>(ra_steps) << "\n";
    if (dec_steps >= 0.0)
        f << "dec_backlash_steps = " << static_cast<long long>(dec_steps) << "\n";
    if (!ra_samples.empty())
        f << "ra_samples = " << ra_samples << "\n";
    if (!dec_samples.empty())
        f << "dec_samples = " << dec_samples << "\n";
    return static_cast<bool>(f);
}

} // namespace sateq
