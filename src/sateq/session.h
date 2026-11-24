// Session configuration layers for sateqd (design agreed 2026-11):
//
//  1. STATION config  ~/.config/sateq/station.conf  — site lat/lon, rotctld
//     port, where mount profiles live. One per physical station.
//  2. MOUNT profile   <mounts_dir>/<name>.conf      — one per mount: serial
//     by-id path, expected mount code (refuse wrong hardware), park pose,
//     axis signs, rate caps, home-sensor availability. Profiles live in
//     <mounts_dir> (default ~/.config/sateq/mounts). A profile never fixes
//     the head ORIENTATION — one mount can run polar or alt-az sessions.
//  3. SESSION state   <state_dir>/<name>.state      — written by the daemon
//     and read at startup: the reference step counters taken at calibration
//     plus the rotation budget accumulated since. On restart: live counters
//     still matching the persisted reference (mount never moved) => resume
//     the budget; anything else => budget restarts from the new pose (the
//     session's stated park pose IS the operator's re-reference ritual).
//
// Precedence everywhere: CLI > mount profile > station config (first-wins
// merge, see sateq/config.h).
#pragma once
#include "sateq/config.h"
#include "sateq/tracker.h"   // Orientation

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace sateq {

// Where each layer lives. Test seams: every path is overridable; empty
// fields fall back to platform defaults (below).
struct Paths
{
    std::string home;        // "" -> getuid() passwd entry $HOME
    std::string station_file;    // "" -> <config_dir>/station.conf
    std::string mounts_dir;      // "" -> <config_dir>/mounts
    std::string state_dir;       // "" -> platform state home (see .cpp)
    // Platform CONFIG home, in priority order:
    //   $SATEQ_CONFIG_DIR  (explicit escape hatch, incl. tests)
    //   Windows: %APPDATA%\sateq
    //   XDG:     $XDG_CONFIG_HOME/sateq, default <home>/.config/sateq
    // std::filesystem has no portable config-dir API; this is the de-facto
    // convention on every OS we target (Linux primary, macOS honors XDG).
    std::string config_dir() const;
};

Paths default_paths();

// A mount profile after resolution (defaults applied, cross-field checks
// done). Field names mirror the profile keys, so a profile line "port = ..."
// feeds SessionConfig::port directly.
struct SessionConfig
{
    std::string name;              // profile name (sirius, hdx, ...)
    std::string port;              // serial device; prefer by-id paths
    int expected_mount_code = -1;  // from "mount_code = 0x01"; refuse mismatch
    Orientation orientation = Orientation::Polar;
    bool have_park_pose = false;   // park_az/park_el given (else orientation default)
    double park_az_deg = 180.0, park_el_deg = 45.0;
    double ra_sign = 1.0, dec_sign = 1.0;
    double max_rate_x_sidereal = 600.0;
    double min_elevation_deg = -15.0;  // hard mechanical floor (below horizon)
    double ra_travel_deg = 105.0;      // datum-relative mechanical travel
    double dec_travel_deg = 105.0;     // envelope per axis (0 = axis disabled)
    double high_coast_s = 0.0;         // HIGH :K coast landing model s
                                       // (0 = landing test disabled)
    double coast_margin_deg = 0.3;     // landing tolerance around setpoint
    double rehome_idle_s = 0.0;        // auto fly-home after this much
                                       // target-gone idle (0 = disabled)
    double rehome_arrive_deg = 0.5;    // re-home arrival window per axis
    double rehome_min_el_deg = 2.0;    // el gate: only a real pass (push
                                       // above this el) arms the homeflight
    double glide_promote_holdoff_s = 0.0; // glide LOW->HIGH promotion
                                       // hold-off (0 = promote instantly;
                                       // OFF default, see TrackerConfig —
                                       // the mgate tightening is the fix)
    double high_gate_moving_x = 90.0;  // pattern rate (x sidereal) that
                                       // alone authorizes HIGH in glide
                                       // (legacy 3.0 = any moving pattern;
                                       // 90 = only beyond-LOW patterns —
                                       // band-edge relay fix, live LUCA)
    double skip_cold_blind_high_s = 2.0; // cold-HIGH skip grace blind
                                       // (skip-detector SafetyConfig)
    double skip_supply_slow_band = 0.75; // sustained LOW slow-ramp-band issue
                                       // fraction (1.0 = legacy: expect full)
    double skip_restart_grace_high_s = 4.0;
                                       // HIGH stop/restart skip-grace window
                                       // END (must exceed the 3.0 s restart
                                       // blind — 2026-11-22 double-latch fix;
                                       // SafetyConfig)
    double branch_handover_margin_deg = 5.0;
                                       // branch-membership hysteresis band
                                       // (rs44 2026-11-20 LOS teleport fix;
                                       // 0 = legacy nearest-tie-break)
    bool   wrap_home_unwind = true;    // homeward flights land on the
                                       // CORD-FREE datum image (DEC offset
                                       // exactly 0) — always unwind (see
                                       // TrackerConfig; false = legacy
                                       // nearest-image landing)
    bool   cord_unwind_route = true;   // deep-ledger rewind flies the RA
                                       // leg first (azimuth-first cord
                                       // unwind, 2026-11-22; false =
                                       // legacy straight-line rewind)
    double wrap_window_deg = 270.0;    // |DEC wrap| a branch pick may land
                                       // in (0 = window disabled)
    double wrap_tiebreak_margin_deg = 90.0; // cord saved to out-vote a
                                       // nearer branch at branch selection
    double wrap_tiebreak_extra_deg  = 30.0; // travel we pay for that vote
    double wall_strain_deg = 10.0;     // wall-strain watchdog: axis err
                                       // past which a vetoed demand means
                                       // the wall cannot hold the target
                                       // (0 disables; see TrackerConfig)
    double wall_strain_s   = 5.0;      // sustained duration to trip
    double high_recommit_s = 1.0;      // HIGH rescale direction-persistence
                                       // confirm (0 = legacy immediate
                                       // commit; see TrackerConfig)
    double high_recommit_hold_deg = 0.1;        // hold budget (0 = no cap)
    double high_recommit_regime_s = 4.0;        // wobble-regime window
    double high_recommit_regime_db = 1e9;         // regime accel freeze (0 = off)
    double high_recommit_regime_age_s = 1.5;    // regime client-staleness gate
    double high_recommit_escape_deg = 2.0;      // recommit-law position-loop escape
    double demand_lpf_regime_tau_s = 1.0;       // wobble-regime demand LPF escalation (0 = off)
    double demand_lpf_regime_min_rc = 12.0;     // escalation storm-grade bar
    double demand_lpf_regime_release_err_deg = 1.0; // escalation error release (0 = off)
    double high_gate_err_deg   = 2.0;           // glide HIGH permission gate (axis deg; bench profile 2.5)
    double track_stop_el = -5.0;       // operational track gate: arm on the way
                                       // up at/below this, stop tracking below
    bool home_sensor = false;          // ":d" available (EQ8-class; verified at connect)
    int rotctld_port = 4533;
    LatLon site{};
    bool have_site = false;
    double update_hz = 10.0;

    // Gear-mesh slop (steps issued after a direction reversal before the
    // physical train moves), measured by `eqtool backlash`. Auto-loaded from
    // <state_dir>/<name>.cal unless the profile states backlash_ra_steps /
    // backlash_dec_steps explicitly (explicit wins: operator override).
    // 0 = unknown/uncompensated. Consumed by the tracker take-up model
    // (phase-5) once real numbers exist.
    double ra_backlash_steps = 0.0, dec_backlash_steps = 0.0;
    std::string backlash_note;  // human sentence logged at startup (may be empty)

    // The reference pose for this session's orientation: explicit park pose
    // or the orientation's agreed default (polar: profile must state one;
    // altaz: tube level EAST).
    HsCoord park_pose() const;
};

// Resolve --mount NAME: loads <mounts_dir>/<name>.conf, merges the station
// config underneath, applies orientation/park defaults, runs cross-field
// validation. On failure returns false with `err` set for the user.
bool load_session(const Paths& paths, const std::string& mount_name,
                  const std::string& orientation_override, // "altaz"|"polar"|""
                  SessionConfig& out, std::string& err);

// ---- mount auto-detection (--mount auto) ----------------------------------
// The boards answer the read-only identity query :e1 with their mount code
// (Sirius/HEQ5-class 0x01, HDX-110/EQ8 0x04), so "which mount is wired up?"
// is answerable from the wire instead of operator memory. Strategy: probe
// every profile's OWN port, and accept exactly the profiles whose expected
// mount_code matches what answered on that port. Profiles lacking port or
// mount_code cannot take part and are listed as skipped. Exactly one match
// wins; zero or several matches are an error with a full inventory, which
// also covers both mounts being powered at once (refuse to guess).
struct MountCandidate
{
    std::string name;          // profile name
    std::string port;          // profile's serial path
    int expected_code = -1;    // profile mount_code
    int live_code = -2;        // probe result: code, -1 = no answer, -2 = not probed
    bool is_match() const { return live_code >= 0 && live_code == expected_code; }
};

// probe(port) -> live mount code, or -1 on any failure (open, timeout,
// garbage). Injected so this layer is unit-testable without hardware; the
// daemon passes sateq::probe_mount_code (query-only, write gate pinned on).
using MountProbe = std::function<int(const std::string& port)>;

bool detect_mount(const Paths& paths, const MountProbe& probe,
                  std::string& mount_name_out,
                  std::vector<MountCandidate>& inventory_out,
                  std::string& err);

// ---- persisted session state ---------------------------------------------
// What a session is: reference step counters + the axis angles assigned to
// them at calibration. The rotation budget does NOT need its own snapshot:
// it is (live counters − reference counters) against those axes, so resume
// is exact as long as the counters survived (resumable() below).
struct SessionState
{
    bool valid = false;            // file existed and parsed
    uint32_t ra_ref = 0, de_ref = 0;  // counters at last calibration
    double ref_axes_ra_deg = 0, ref_axes_dec_deg = 0; // axes angles at those counters
    uint64_t ref_utc_unix = 0;     // wall time the reference was established
    uint64_t started_unix = 0;     // when this daemon session booted
    std::string orientation;       // "polar" | "altaz" (diagnostics)

    bool load(const std::string& path);
    bool save(const std::string& path) const;

    // Counters at restart still matching the persisted reference (within a
    // half step-period of slop) means the mount never moved while we were
    // down: the rotation budget can be resumed.
    bool resumable(uint32_t live_ra, uint32_t live_de, int slop = 1) const;
};

std::string session_state_path(const Paths& paths, const std::string& mount_name);

// ---- measured backlash calibration ----------------------------------------
// Physical measurements belong to the machine that is WIRED to the mount,
// and they are machine-generated, not hand-written — so per the XDG spec
// they are STATE, not CONFIG: <state_dir>/<name>.cal (same directory and
// lifecycle as <name>.state). key=value format like every other layer.
//
//   # sateq backlash calibration — written by eqtool backlash
//   utc_unix = 1795000000
//   mount_code = 0x01           # guard: ignored when it differs from the
//                               # profile's expected code (cal from wrong board)
//   method = burst@10x
//   ra_backlash_steps = 842     # slop per axis: issued steps from reversal
//   dec_backlash_steps = 1023   # until physical motion resumes (max over dirs)
//   ra_samples = 812/860,824/874,836/886  # per rep: forward/reverse windows
//   dec_samples = ...
struct BacklashCal
{
    bool valid = false;                 // file existed with >=1 usable axis value
    double ra_steps = -1.0, dec_steps = -1.0;  // -1 = not measured (0 = measured none)
    int mount_code = -1;
    uint64_t utc_unix = 0;
    std::string method;
    std::string ra_samples, dec_samples;  // raw windows, documentation only

    bool load(const std::string& path);   // false: absent/unreadable/no axis value
    bool save(const std::string& path) const;
};

std::string backlash_cal_path(const Paths& paths, const std::string& mount_name);

} // namespace sateq
