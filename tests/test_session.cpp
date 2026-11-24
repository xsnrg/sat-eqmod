// Session layer: profile+station resolution with precedence, orientation
// defaults and validation, and persisted session-state resume detection.
#include "testkit.h"
#include "sateq/session.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

using namespace sateq;

static const std::string TMP = "/tmp/sateq_test_session";

static void write_file(const std::string& path, const std::string& body)
{
    std::ofstream f(path);
    f << body;
}

static Paths test_paths()
{
    Paths p;
    p.home = TMP;
    p.station_file = TMP + "/station.conf";
    p.mounts_dir = TMP + "/mounts";
    p.state_dir = TMP + "/state";
    return p;
}

int main()
{
    system(("rm -rf " + TMP + " && mkdir -p " + TMP + "/mounts").c_str());
    Paths P = test_paths();

    TEST("station config supplies site + defaults; profile supplies hardware");
    {
        write_file(P.station_file, "lat = 44.95\nlon = -93.28\nrotctld_port = 4540\n");
        write_file(P.mounts_dir + "/sirius.conf",
                   "# Sirius bench rig\n"
                   "port = /dev/serial/by-id/usb-FTDI_TTL232R_FTA67MMH-if00-port0\n"
                   "mount_code = 0x01\n"
                   "orientation = polar\n"
                   "park_az = 180\npark_el = 60\n");
        SessionConfig sc;
        std::string err;
        CHECK(load_session(P, "sirius", "", sc, err));
        CHECK(err.empty());
        CHECK(sc.have_site);
        CHECK_NEAR(sc.site.lat_deg, 44.95, 1e-12);
        CHECK_NEAR(sc.site.lon_east_deg, -93.28, 1e-12);
        CHECK_EQ(sc.expected_mount_code, 0x01);
        CHECK_EQ(sc.rotctld_port, 4540);            // from station
        CHECK(sc.orientation == Orientation::Polar);
        CHECK(sc.have_park_pose);
        CHECK_NEAR(sc.track_stop_el, -5.0, 1e-12);    // default track gate
        HsCoord pp = sc.park_pose();
        CHECK_NEAR(pp.az_deg, 180.0, 1e-12);
        CHECK_NEAR(pp.el_deg, 60.0, 1e-12);
    }

    TEST("profile overrides station values (first-wins: profile loaded first)");
    {
        write_file(P.mounts_dir + "/sirius2.conf",
                   "port = /dev/ttyUSB9\nmount_code = 1\norientation = polar\n"
                   "park_az = 0\npark_el = 90\nrotctld_port = 4600\n");
        SessionConfig sc;
        std::string err;
        CHECK(load_session(P, "sirius2", "", sc, err));
        CHECK_EQ(sc.rotctld_port, 4600); // profile beats station's 4540
    }

    TEST("altaz defaults: park pose = tube level EAST, no park_* needed");
    {
        write_file(P.mounts_dir + "/altazm.conf",
                   "port = /dev/ttyUSB0\norientation = altaz\n");
        SessionConfig sc;
        std::string err;
        CHECK(load_session(P, "altazm", "", sc, err));
        CHECK(sc.orientation == Orientation::AltAzLevel);
        CHECK(!sc.have_park_pose);
        HsCoord pp = sc.park_pose();
        CHECK_NEAR(pp.az_deg, 90.0, 1e-12);
        CHECK_NEAR(pp.el_deg, 0.0, 1e-12);
    }

    TEST("orientation override switches a polar profile to altaz for one session");
    {
        SessionConfig sc;
        std::string err;
        CHECK(load_session(P, "sirius", "altaz", sc, err)); // sirius.conf is polar WITH park
        CHECK(sc.orientation == Orientation::AltAzLevel);
        // explicit park pose stays authoritative for any orientation
        HsCoord pp = sc.park_pose();
        CHECK_NEAR(pp.az_deg, 180.0, 1e-12);
    }

    TEST("validation errors");
    {
        // fresh SessionConfig per call: a loaded config carries park-pose
        // state that would mask the very defaults we're testing
        std::string err;
        SessionConfig sc;
        sc = SessionConfig{};
        CHECK(!load_session(P, "nope", "", sc, err));
        CHECK(err.find("not found") != std::string::npos);

        write_file(P.mounts_dir + "/bad1.conf", "mount_code = 0x01\norientation = polar\n"
                                                "park_az = 180\npark_el = 60\n");
        sc = SessionConfig{};
        CHECK(!load_session(P, "bad1", "", sc, err));
        CHECK(err.find("port") != std::string::npos);

        write_file(P.mounts_dir + "/bad2.conf", "port = /dev/x\norientation = polar\n");
        sc = SessionConfig{};
        CHECK(!load_session(P, "bad2", "", sc, err)); // polar without park pose
        CHECK(err.find("park") != std::string::npos);

        write_file(P.mounts_dir + "/bad3.conf",
                   "port = /dev/x\norientation = wobble\npark_az = 1\npark_el = 2\n");
        sc = SessionConfig{};
        CHECK(!load_session(P, "bad3", "", sc, err));
        CHECK(err.find("orientation") != std::string::npos);

        write_file(P.mounts_dir + "/bad4.conf",
                   "port = /dev/x\norientation = polar\npark_az = 180\npark_el = 60\nmax_rate_x_sidereal = 5000\n");
        sc = SessionConfig{};
        CHECK(!load_session(P, "bad4", "", sc, err));
        CHECK(err.find("max_rate") != std::string::npos);

        // track gate below the mechanical floor would silently disable it
        write_file(P.mounts_dir + "/bad5.conf",
                   "port = /dev/x\norientation = polar\npark_az = 180\npark_el = 60\n"
                   "min_elevation_deg = -15\ntrack_stop_el = -16\n");
        sc = SessionConfig{};
        CHECK(!load_session(P, "bad5", "", sc, err));
        CHECK(err.find("track_stop_el") != std::string::npos);

        // legitimate: high local horizon (treeline) raises the track gate
        write_file(P.mounts_dir + "/trees.conf",
                   "port = /dev/x\norientation = altaz\ntrack_stop_el = 7.5\n");
        sc = SessionConfig{};
        CHECK(load_session(P, "trees", "", sc, err));
        CHECK_NEAR(sc.track_stop_el, 7.5, 1e-12);
    }

    TEST("session state save/load round-trip + resume detection");
    {
        SessionState st;
        st.ra_ref = 0x8000A7;
        st.de_ref = 0xAA7B01;
        st.ref_axes_ra_deg = 12.5;
        st.ref_axes_dec_deg = -40.25;
        st.ref_utc_unix = 1794700000;
        st.started_unix = 1794700800;
        st.orientation = "altaz";
        std::string path = session_state_path(P, "sirius");
        CHECK(st.save(path));

        SessionState back;
        CHECK(back.load(path));
        CHECK(back.valid);
        CHECK_EQ(back.ra_ref, 0x8000A7u);
        CHECK_EQ(back.de_ref, 0xAA7B01u);
        CHECK_NEAR(back.ref_axes_ra_deg, 12.5, 1e-9);
        CHECK_NEAR(back.ref_axes_dec_deg, -40.25, 1e-9);
        CHECK_EQ(back.ref_utc_unix, 1794700000ull);
        CHECK_EQ(back.started_unix, 1794700800ull);
        CHECK_EQ(back.orientation, std::string("altaz"));

        CHECK(back.resumable(0x8000A7, 0xAA7B01));       // never moved
        CHECK(back.resumable(0x8000A8, 0xAA7B01));       // slop 1 tolerated
        CHECK(!back.resumable(0x8000A9, 0xAA7B01));      // beyond slop
        CHECK(!back.resumable(0x800000, 0xAA7B00));      // re-homed to park
        // wrap ring: saved near 0xFFFFFF, live just rolled past zero = tiny
        // FORWARD motion (fine); backward motion = someone moved the train
        SessionState wrap;
        wrap.valid = true;
        wrap.ra_ref = 0xFFFFFF; wrap.de_ref = 5;
        CHECK(wrap.resumable(0x000000, 0x000006));  // +1 each, wrapped
        CHECK(wrap.resumable(0xFFFFFF, 0x000005));  // unmoved
        CHECK(!wrap.resumable(0xFFFFFE, 0x000005)); // one step BACKWARD: refuse
        CHECK(!wrap.resumable(0x000100, 5));        // dragged far forward
    }

    TEST("missing/corrupt state files are simply absent");
    {
        SessionState st;
        CHECK(!st.load(TMP + "/state/absent.state"));
        write_file(TMP + "/state/junk.state", "not a state file at all\n");
        CHECK(!st.load(TMP + "/state/junk.state"));
        write_file(TMP + "/state/nanbad.state", "ra_ref = abc\nde_ref = 1\n");
        SessionState st2;
        CHECK(!st2.load(TMP + "/state/nanbad.state"));
    }

    // ---- --mount auto detection ------------------------------------------
    // Same two-mount bench we actually have: Sirius (HEQ5-class 0x01) and
    // HDX-110 (EQ8 0x04), one FTDI each. The probe is faked; the wire-level
    // probe_mount_code has its own failure tests in test_board_sim.
    TEST("detect_mount: sole profile whose board answers on its own port wins");
    {
        Paths D = test_paths();
        system(("rm -rf " + TMP + "/mounts && mkdir -p " + TMP + "/mounts").c_str());
        write_file(D.mounts_dir + "/sirius.conf", "port = /dev/ttySIRIUS\nmount_code = 0x01\n");
        write_file(D.mounts_dir + "/hdx.conf",    "port = /dev/ttyHDX\nmount_code = 0x04\n");
        // only HDX powered up:
        std::string name;
        std::vector<MountCandidate> inv;
        std::string err;
        CHECK(detect_mount(D, [](const std::string& p) { return p == "/dev/ttyHDX" ? 0x04 : -1; },
                           name, inv, err));
        CHECK_EQ(name, std::string("hdx"));
        CHECK(err.empty());
        CHECK_EQ(inv.size(), size_t(2));
        int silent = 0, answered = 0;
        for (const auto& cand : inv)
        {
            if (cand.live_code == -1) ++silent;
            if (cand.live_code == 0x04) ++answered;
        }
        CHECK_EQ(silent, 1);
        CHECK_EQ(answered, 1);
    }

    TEST("detect_mount: code mismatch is a MISMATCH, not a match");
    {
        Paths D = test_paths();
        system(("rm -rf " + TMP + "/mounts && mkdir -p " + TMP + "/mounts").c_str());
        std::string name, err;
        std::vector<MountCandidate> inv;
        // HDX port answers but the profile claiming it expects 0x01 (wrong
        // profile written for that cable): no profile may be selected.
        write_file(D.mounts_dir + "/sirius.conf", "port = /dev/ttyHDX\nmount_code = 0x01\n");
        CHECK(!detect_mount(D, [](const std::string&) { return 0x04; }, name, inv, err));
        CHECK(name.empty());
        CHECK(err.find("MISMATCH") != std::string::npos);
    }

    TEST("detect_mount: both mounts powered => refuse to guess");
    {
        Paths D = test_paths();
        system(("rm -rf " + TMP + "/mounts && mkdir -p " + TMP + "/mounts").c_str());
        write_file(D.mounts_dir + "/sirius.conf", "port = /dev/ttySIRIUS\nmount_code = 0x01\n");
        write_file(D.mounts_dir + "/hdx.conf",    "port = /dev/ttyHDX\nmount_code = 0x04\n");
        std::string name, err;
        std::vector<MountCandidate> inv;
        CHECK(!detect_mount(D, [](const std::string& p)
                            { return p == "/dev/ttyHDX" ? 0x04 : p == "/dev/ttySIRIUS" ? 0x01 : -1; },
                            name, inv, err));
        CHECK(name.empty());
        CHECK(err.find("sirius") != std::string::npos);
        CHECK(err.find("hdx") != std::string::npos);
        CHECK(err.find("--mount NAME") != std::string::npos);
    }

    TEST("detect_mount: profiles missing port/mount_code cannot take part; one probe per port");
    {
        Paths D = test_paths();
        system(("rm -rf " + TMP + "/mounts && mkdir -p " + TMP + "/mounts").c_str());
        write_file(D.mounts_dir + "/nocode.conf", "port = /dev/ttyFREE\n");          // no role
        write_file(D.mounts_dir + "/noport.conf", "mount_code = 0x01\n");            // skipped
        write_file(D.mounts_dir + "/a.conf", "port = /dev/ttyDUP\nmount_code = 0x01\n");
        write_file(D.mounts_dir + "/b.conf", "port = /dev/ttyDUP\nmount_code = 0x01\n");
        write_file(D.mounts_dir + "/hdx.conf", "port = /dev/ttyHDX\nmount_code = 0x04\n");
        std::map<std::string, int> calls;
        std::string name, err;
        std::vector<MountCandidate> inv;
        bool ok = detect_mount(D, [&](const std::string& p) { calls[p]++; return p == "/dev/ttyHDX" ? 0x04 : -1; },
                               name, inv, err);
        CHECK(ok);
        CHECK_EQ(name, std::string("hdx"));            // dup profiles: port answered nothing,
        CHECK_EQ(calls["/dev/ttyDUP"], 1);             // both claim it; probed ONCE anyway
        // (they just both failed to match — port /dev/ttyDUP never answered 0x01)
        CHECK_EQ(calls.size(), size_t(2));             // nocode had no role: never probed
        bool noport_listed_skipped = false;
        for (const MountCandidate& cand : inv)
            if (cand.name == "noport")
                noport_listed_skipped = (cand.live_code == -2);
        CHECK(noport_listed_skipped);                  // listed as skipped (no port)
    }

    TEST("detect_mount: two profiles on ONE port (cable shared between mounts) still disambiguate");
    {
        Paths D = test_paths();
        system(("rm -rf " + TMP + "/mounts && mkdir -p " + TMP + "/mounts").c_str());
        write_file(D.mounts_dir + "/sirius.conf", "port = /dev/ttySHARED\nmount_code = 0x01\n");
        write_file(D.mounts_dir + "/hdx.conf",    "port = /dev/ttySHARED\nmount_code = 0x04\n");
        std::string name, err;
        std::vector<MountCandidate> inv;
        CHECK(detect_mount(D, [](const std::string&) { return 0x01; }, name, inv, err));
        CHECK_EQ(name, std::string("sirius"));   // the board wired to that cable speaks HEQ5
    }

    TEST("detect_mount: empty/missing mounts dir is an error, not a crash");
    {
        Paths D = test_paths();
        system(("rm -rf " + TMP + "/mounts").c_str());
        std::string name, err;
        std::vector<MountCandidate> inv;
        CHECK(!detect_mount(D, [](const std::string&) { return 0x01; }, name, inv, err));
        CHECK(!err.empty());
    }

    // ---- platform paths (XDG + overrides) ---------------------------------
    TEST("platform paths: SATEQ_* and XDG env overrides");
    {
        // save/restore the environment the rest of the process sees
        struct EnvGuard
        {
            std::map<std::string, std::string> saved;
            std::vector<std::string> was_absent;
            ~EnvGuard()
            {
                for (auto& kv : saved)
                    setenv(kv.first.c_str(), kv.second.c_str(), 1);
                for (const std::string& k : was_absent)
                    unsetenv(k.c_str());
            }
        } guard;
        for (const char* k : {"SATEQ_CONFIG_DIR", "SATEQ_STATE_DIR", "XDG_CONFIG_HOME",
                              "XDG_STATE_HOME", "HOME"})
            if (const char* v = getenv(k))
                guard.saved[k] = v;
            else
                guard.was_absent.push_back(k);

        setenv("HOME", "/tmp/fakehome", 1);
        unsetenv("SATEQ_CONFIG_DIR");
        unsetenv("SATEQ_STATE_DIR");
        unsetenv("XDG_CONFIG_HOME");
        unsetenv("XDG_STATE_HOME");
        Paths p = default_paths();
        CHECK_EQ(p.config_dir(), std::string("/tmp/fakehome/.config/sateq"));   // XDG default
        CHECK_EQ(p.state_dir, std::string("/tmp/fakehome/.local/state/sateq")); // XDG default
        CHECK_EQ(p.station_file, std::string("/tmp/fakehome/.config/sateq/station.conf"));

        setenv("XDG_CONFIG_HOME", "/tmp/xdgcfg", 1);
        setenv("XDG_STATE_HOME", "/tmp/xdgstate", 1);
        p = default_paths();
        CHECK_EQ(p.config_dir(), std::string("/tmp/xdgcfg/sateq"));
        CHECK_EQ(p.state_dir, std::string("/tmp/xdgstate/sateq"));

        setenv("SATEQ_CONFIG_DIR", "/tmp/sateq-cfg", 1);   // explicit escape hatch wins
        setenv("SATEQ_STATE_DIR", "/tmp/sateq-state", 1);
        p = default_paths();
        CHECK_EQ(p.config_dir(), std::string("/tmp/sateq-cfg"));
        CHECK_EQ(p.state_dir, std::string("/tmp/sateq-state"));
        CHECK_EQ(backlash_cal_path(p, "sirius"), std::string("/tmp/sateq-state/sirius.cal"));
    }

    // ---- measured backlash calibration ------------------------------------
    TEST("backlash cal: save/load round-trip, partial cals, corrupt files");
    {
        BacklashCal c;
        c.ra_steps = 842; c.dec_steps = 1023; c.mount_code = 0x01;
        c.utc_unix = 1795000000; c.method = "burst@10x res15\" midpoint";
        c.ra_samples = "1:800-900/810-890,2:820-880/815-885";
        c.dec_samples = "1:1000-1050/995-1045";
        CHECK(c.save(TMP + "/state/x.cal"));
        BacklashCal r;
        CHECK(r.load(TMP + "/state/x.cal"));
        CHECK(r.valid);
        CHECK_EQ(r.ra_steps, 842.0);
        CHECK_EQ(r.dec_steps, 1023.0);
        CHECK_EQ(r.mount_code, 0x01);
        CHECK_EQ(r.utc_unix, uint64_t(1795000000));
        CHECK_EQ(r.method, std::string("burst@10x res15\" midpoint"));
        CHECK_EQ(r.ra_samples, std::string("1:800-900/810-890,2:820-880/815-885"));

        // partial: only RA measured — DEC stays -1 ("not measured"), still valid
        BacklashCal half;
        half.ra_steps = 500;
        CHECK(half.save(TMP + "/state/half.cal"));
        BacklashCal rh;
        CHECK(rh.load(TMP + "/state/half.cal"));
        CHECK_EQ(rh.ra_steps, 500.0);
        CHECK_EQ(rh.dec_steps, -1.0);

        BacklashCal none;                     // measured nothing: not a cal
        CHECK(none.save(TMP + "/state/none.cal"));
        BacklashCal rn;
        CHECK(!rn.load(TMP + "/state/none.cal"));

        BacklashCal rb;
        CHECK(!rb.load(TMP + "/state/absent.cal"));
        write_file(TMP + "/state/bad.cal", "ra_backlash_steps = garbage\n");
        CHECK(!rb.load(TMP + "/state/bad.cal"));
        write_file(TMP + "/state/bad.cal", "# empty of axes\nmethod = x\n");
        CHECK(!rb.load(TMP + "/state/bad.cal"));
    }

    TEST("load_session: backlash auto-loads from cal file; profile overrides; board guard");
    {
        Paths D = test_paths();
        system(("rm -rf " + TMP + "/mounts && mkdir -p " + TMP + "/mounts").c_str());
        write_file(D.station_file, "lat = 44.95\nlon = -93.28\n");
        write_file(D.mounts_dir + "/blmount.conf",
                   "port = /dev/ttyBL\nmount_code = 0x01\norientation = altaz\n");
        SessionConfig sc;
        std::string err;

        // no cal file, no keys: zeros, silent
        CHECK(load_session(D, "blmount", "", sc, err));
        CHECK_EQ(sc.ra_backlash_steps, 0.0);
        CHECK_EQ(sc.dec_backlash_steps, 0.0);
        CHECK(sc.backlash_note.empty());

        // cal file matching the profile's board: auto-applied + logged note
        BacklashCal c;
        c.ra_steps = 842; c.dec_steps = 1023; c.mount_code = 0x01; c.method = "burst@10x";
        CHECK(c.save(backlash_cal_path(D, "blmount")));
        SessionConfig s2;
        CHECK(load_session(D, "blmount", "", s2, err));
        CHECK_EQ(s2.ra_backlash_steps, 842.0);
        CHECK_EQ(s2.dec_backlash_steps, 1023.0);
        CHECK(s2.backlash_note.find("measured backlash") != std::string::npos);

        // cal taken on the OTHER board: refused with a loud note, values stay 0
        BacklashCal wrong;
        wrong.ra_steps = 999; wrong.mount_code = 0x04;
        CHECK(wrong.save(backlash_cal_path(D, "blmount")));
        SessionConfig s3;
        CHECK(load_session(D, "blmount", "", s3, err));
        CHECK_EQ(s3.ra_backlash_steps, 0.0);
        CHECK(s3.backlash_note.find("IGNORED") != std::string::npos);

        // explicit profile keys override the cal file (operator veto)
        wrong.mount_code = 0x01;  // make the cal valid again so override is what wins
        CHECK(wrong.save(backlash_cal_path(D, "blmount")));
        write_file(D.mounts_dir + "/blmount.conf",
                   "port = /dev/ttyBL\nmount_code = 0x01\norientation = altaz\n"
                   "backlash_ra_steps = 7\nbacklash_dec_steps = 9\n");
        SessionConfig s4;
        CHECK(load_session(D, "blmount", "", s4, err));
        CHECK_EQ(s4.ra_backlash_steps, 7.0);
        CHECK_EQ(s4.dec_backlash_steps, 9.0);
        CHECK(s4.backlash_note.find("override from profile") != std::string::npos);

        // one-sided override: other axis defaults to 0 (never a negative leak)
        write_file(D.mounts_dir + "/blmount.conf",
                   "port = /dev/ttyBL\nmount_code = 0x01\norientation = altaz\n"
                   "backlash_ra_steps = 7\n");
        SessionConfig s5;
        CHECK(load_session(D, "blmount", "", s5, err));
        CHECK_EQ(s5.ra_backlash_steps, 7.0);
        CHECK_EQ(s5.dec_backlash_steps, 0.0);

        // negative values are a validation error, not silent nonsense
        write_file(D.mounts_dir + "/blmount.conf",
                   "port = /dev/ttyBL\nmount_code = 0x01\norientation = altaz\n"
                   "backlash_ra_steps = -5\n");
        SessionConfig s6;
        CHECK(!load_session(D, "blmount", "", s6, err));
        system(("rm -f " + TMP + "/mounts/blmount.conf").c_str());
    }

    system(("rm -rf " + TMP).c_str());
    return testkit::summary("session");
}
