// Safety gate: elevation floor, cumulative-rotation latch, rate cap,
// watchdog trips, and the rule that a latch only opens via clear().
#include "testkit.h"
#include "sateq/safety.h"

using namespace sateq;
using SC = std::chrono::steady_clock;

int main()
{
    TEST("mechanical elevation floor at -15 deg");
    {
        // SafetyMonitor knows ONLY the mechanical floor; the operational
        // track gate (track_stop_el) lives in the tracker, which knows
        // whether a target is merely invalid or a pass that has set.
        SafetyMonitor s;
        CHECK(s.check_pointing(-14.9).result == GuardResult::Ok);
        CHECK(s.check_pointing(-15.1).result == GuardResult::Blocked);
        CHECK(std::string(s.check_pointing(-15.1).reason).find("mechanical") != std::string::npos);
        CHECK(s.check_pointing(90).result == GuardResult::Ok);
    }

    TEST("rate cap");
    {
        SafetyMonitor s;
        CHECK(s.check_rate(599).result == GuardResult::Ok);
        CHECK(s.check_rate(-599).result == GuardResult::Ok);
        CHECK(s.check_rate(601).result == GuardResult::Blocked);
    }

    TEST("cumulative rotation latches and stays latched");
    {
        SafetyMonitor s;
        AxisAngles ref{10.0, 45.0};
        s.set_reference(ref);
        CHECK(s.check_cumulative(AxisAngles{100.0, 45.0}).result == GuardResult::Ok);
        CHECK(s.check_cumulative(AxisAngles{10.0 + 541, 45.0}).result == GuardResult::Latched);
        // even back inside limits, latched stays latched until clear()
        s.trip("manual e-stop");
        CHECK(s.check_cumulative(ref).result == GuardResult::Latched);
        CHECK(s.check_pointing(45).result == GuardResult::Latched);
        CHECK(s.check_rate(1).result == GuardResult::Latched);
        s.clear();
        CHECK(s.check_cumulative(ref).result == GuardResult::Ok);
    }

    TEST("dec cumulative limit folds like the travel envelope");
    {
        // Live 2026-11-20: the over-the-top flip route home reported DEC
        // 270.07 (= -89.93 folded, tube physically AT home, az 180/el 0)
        // and the unfolded |dec-ref| compare false-latched the session at
        // the fusion crossing, just as ISS was coming up. DEC is bounded
        // at ~+-182 from the datum on BOTH branches, so an offset past
        // +-180 is always the flip image of the folded pose — bounding
        // the FOLDED offset is the only honest measure. Consequence (by
        // design, operator guidance 2026-11-19: DEC is the free axis, the
        // feedline constraint lives on RA): with the default 270 limit a
        // folded DEC offset (max 180) never latches; the RA 540 guard is
        // the operative cable backstop.
        SafetyMonitor s;
        s.set_reference(AxisAngles{0, 0});
        CHECK(s.check_cumulative(AxisAngles{0, 270.07}).result == GuardResult::Ok);
        CHECK(s.check_cumulative(AxisAngles{0, 280}).result == GuardResult::Ok);  // = -80
        CHECK(s.check_cumulative(AxisAngles{0, -180}).result == GuardResult::Ok);
        SafetyConfig tight = {};
        tight.max_dec_cumulative_deg = 170; // folded offsets still latch a tight limit
        SafetyMonitor t(tight);
        t.set_reference(AxisAngles{0, 0});
        CHECK(t.check_cumulative(AxisAngles{0, 160}).result == GuardResult::Ok);
        CHECK(t.check_cumulative(AxisAngles{0, 175}).result == GuardResult::Latched);
        CHECK(t.check_cumulative(AxisAngles{0, 200}).result == GuardResult::Ok); // fold -> 160
    }

    TEST("travel envelope: datum-relative arc guard (stand limit)");
    {
        // The Sirius stand limits each axis to +/-105 deg of its datum
        // pose (15 deg below horizon on the last degrees of either flank
        // — live freeze 2026-11-19: a GT2 pre-aim whose axis solution was
        // RA -152 wound the tube into the stand; the cumulative 540/270
        // cable limits never see it). Non-latching: unlike the cumulative
        // wrap guard the recovery is automatic (auto-return-home), so a
        // refusal must NOT poison subsequent legal targets.
        SafetyMonitor s;
        s.set_reference(AxisAngles{0, 0});
        CHECK(s.check_travel(AxisAngles{100, 0}, AxisAngles{0, 0}).result == GuardResult::Ok);
        CHECK(s.check_travel(AxisAngles{106, 0}, AxisAngles{0, 0}).result == GuardResult::Blocked);
        CHECK(s.check_travel(AxisAngles{0, 0}, AxisAngles{106, 0}).result == GuardResult::Blocked);
        CHECK(s.check_travel(AxisAngles{0, 0}, AxisAngles{-106, 0}).result == GuardResult::Blocked);
        // dec envelope separate
        CHECK(s.check_travel(AxisAngles{0, 104}, AxisAngles{0, 0}).result == GuardResult::Ok);
        CHECK(s.check_travel(AxisAngles{0, 0}, AxisAngles{0, 106}).result == GuardResult::Blocked);
        // reasons name the axis and the offending pose
        CHECK(std::string(s.check_travel(AxisAngles{-150, 0}, AxisAngles{0, 0}).reason).find("RA") != std::string::npos);
        CHECK(std::string(s.check_travel(AxisAngles{0, 0}, AxisAngles{0, 200}).reason).find("target") != std::string::npos);
        // margin (hysteresis) shrinks the box for re-acquisition
        CHECK(s.check_travel(AxisAngles{0, 0}, AxisAngles{103, 0}).result == GuardResult::Ok);
        CHECK(s.check_travel(AxisAngles{0, 0}, AxisAngles{103, 0}, 5.0).result == GuardResult::Blocked);
        // non-latching: one tick later a legal pair is Ok again
        CHECK(s.check_travel(AxisAngles{0, 0}, AxisAngles{0, 0}).result == GuardResult::Ok);
        // current-pose violation blocks regardless of target (must leave),
        // and takes reporting priority over a simultaneous target violation
        {
            auto d = s.check_travel(AxisAngles{200, 0}, AxisAngles{300, 0});
            CHECK(d.result == GuardResult::Blocked);
            CHECK(std::string(d.reason).find("current") != std::string::npos);
        }
        // limit 0 disables that axis entirely
        {
            SafetyConfig c;
            c.max_ra_travel_deg = 0;   // RA unlimited (e.g. polar pier profile)
            SafetyMonitor u{c};
            u.set_reference(AxisAngles{0, 0});
            CHECK(u.check_travel(AxisAngles{1e5, 0}, AxisAngles{0, 0}).result == GuardResult::Ok);
            CHECK(u.check_travel(AxisAngles{0, 0}, AxisAngles{0, 106}).result == GuardResult::Blocked);
        }
    }

    TEST("reference re-set opens wrap latch (unwrap acknowledged)");
    {
        SafetyMonitor s;
        s.set_reference(AxisAngles{});
        CHECK(s.check_cumulative(AxisAngles{600, 0}).result == GuardResult::Latched);
        s.trip("ra wrap"); // tracker latches on that decision
        CHECK(s.latched());
        s.set_reference(AxisAngles{600, 0}); // operator unwrapped, new ref
        CHECK(!s.latched());
    }

    TEST("watchdog: stale rotctld link trips once while motors may run");
    {
        SafetyMonitor s;
        auto t0 = SC::now();
        s.note_client_command(t0);
        CHECK(s.check_watchdog(t0 + std::chrono::seconds(2), true).result == GuardResult::Ok);
        auto d = s.check_watchdog(t0 + std::chrono::seconds(6), true);
        CHECK(d.result == GuardResult::Latched);
        // trips only once (the caller latches on first report)
        CHECK(s.check_watchdog(t0 + std::chrono::seconds(7), true).result == GuardResult::Ok);
        // fresh client command re-arms
        s.note_client_command(t0 + std::chrono::seconds(8));
        CHECK(s.check_watchdog(t0 + std::chrono::seconds(9), true).result == GuardResult::Ok);
        // and does nothing when motors are known-stopped
        CHECK(s.check_watchdog(t0 + std::chrono::seconds(99), false).result == GuardResult::Ok);
    }

    TEST("watchdog: stale serial link trips");
    {
        SafetyMonitor s;
        auto t0 = SC::now();
        s.note_board_ok(t0);
        CHECK(s.check_watchdog(t0 + std::chrono::milliseconds(1500), true).result == GuardResult::Ok);
        CHECK(s.check_watchdog(t0 + std::chrono::milliseconds(2500), true).result == GuardResult::Latched);
    }

    TEST("latch reason is human-readable and preserved");
    {
        SafetyMonitor s;
        s.trip("e-stop from rotctld Q");
        CHECK(std::string(s.latch_reason()).find("e-stop") != std::string::npos);
        auto d = s.check_pointing(0);
        CHECK(d.reason == s.latch_reason());
    }

    return testkit::summary("safety");
}
