// dome_coverage: enumerate the sky reachable by the AltAzLevel mount under
// the datum-relative travel envelope, using the PRODUCTION astro math and
// the tracker's branch/box logic (select_branch semantics from tracker.cpp,
// replicated here as pure functions).
//
//   g++ -O2 -I src tools/analysis/dome_coverage.cpp build/libsateq.a -o /tmp/cov
//   /tmp/cov <home_az> <home_el> <ra_lim> <dec_lim> [--grid|--solve|--pct]
//
// Limits are STRICT bounds in deg from the datum axes (fold180 space, so
// 180 == unlimited; >180 cannot be expressed — that is the cumulative
// guard's job). The production 0=disable convention is NOT replicated.
//
// --grid (default): print, per azimuth, the lowest reachable elevation
//          (0 = fully reachable from the horizon; 91 = never).
// --solve: for each azimuth print the MINIMUM dec limit (ra_lim fixed) that
//          makes it horizon-reachable, and the RA offset used there.
// --pct  : solid-angle coverage % of the upper hemisphere + worst floor.
//
// Bench context (2026-11-19): RA +-105 (cable/stand-limited axis) + DEC 180
// (shaft axis, mechanically ~182 per operator) covers 100% of the sky.
#include "sateq/astro.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

using namespace sateq;

static double fold180(double x)
{
    x = std::fmod(x + 180.0, 360.0);
    if (x < 0.0) x += 360.0;
    return x - 180.0;
}

struct Off { double ra, dec; };
static Off rel(const AxisAngles& ax, const AxisAngles& ref)
{
    return {fold180(ax.ra_deg - ref.ra_deg), fold180(ax.dec_deg - ref.dec_deg)};
}
// NOTE: unlike the production profile convention (0 = axis disabled), this
// tool treats limits as STRICT bounds (fold180 output lives in (-180, 180],
// so 180 is already "unlimited" and a limit > 180 cannot be expressed —
// over-travel guards beyond 180 are the cumulative guard's job).
static bool in_box(const Off& o, double ra_lim, double dec_lim)
{
    return std::fabs(o.ra) <= ra_lim && std::fabs(o.dec) <= dec_lim;
}

// Both axis solutions of a pointing (canonical + flip), as offsets from the
// datum axes (= the axes of the home pointing).
static bool reachable(double az, double el, const AxisAngles& ref,
                      double ra_lim, double dec_lim, Off* used = nullptr)
{
    AxisAngles can = hs_to_axes(az, el, 0.0, Orientation::AltAzLevel);
    AxisAngles flip{can.ra_deg + 180.0, 180.0 - can.dec_deg};
    Off c = rel(can, ref), f = rel(flip, ref);
    if (in_box(c, ra_lim, dec_lim)) { if (used) *used = c; return true; }
    if (in_box(f, ra_lim, dec_lim)) { if (used) *used = f; return true; }
    return false;
}

int main(int argc, char** argv)
{
    if (argc < 5) { std::printf("usage: cov HOME_AZ HOME_EL RA_LIM DEC_LIM [--grid|--solve|--pct]\n"); return 2; }
    double haz = std::atof(argv[1]), hel = std::atof(argv[2]);
    double ra_lim = std::atof(argv[3]), dec_lim = std::atof(argv[4]);
    bool solve = argc > 5 && std::string(argv[5]) == "--solve";
    AxisAngles ref = hs_to_axes(haz, hel, 0.0, Orientation::AltAzLevel);
    std::printf("home (%.1f, %.1f) -> datum axes (%.2f, %.2f); limits RA %.0f DEC %.0f\n",
                haz, hel, ref.ra_deg, ref.dec_deg, ra_lim, dec_lim);

    if (solve) {
        std::printf("az  min_dec_lim_for_horizon  RA_offset_at_that_pointing\n");
        for (int az = 0; az < 360; az += 5) {
            double lo = 0, hi = 180;
            if (reachable(az, 0.0, ref, ra_lim, 0.0)) { std::printf("%3d  0\n", az); continue; }
            while (hi - lo > 0.5) {
                double mid = 0.5 * (lo + hi);
                if (reachable(az, 0.0, ref, ra_lim, mid)) hi = mid; else lo = mid;
            }
            Off u; reachable(az, 0.0, ref, ra_lim, hi, &u);
            std::printf("%3d  %6.1f   RA %+7.1f\n", az, hi, u.ra);
        }
        return 0;
    }
    if (argc > 5 && std::string(argv[5]) == "--pct") {
        // Solid-angle coverage of the upper hemisphere on a 1x1 deg grid,
        // ring-area weighted by d(sin el).
        double tot = 0.0, got = 0.0;
        for (int az = 0; az < 360; ++az)
            for (int el = 0; el <= 90; ++el) {
                double cell = std::sin((el + 1) * M_PI / 180.0) -
                              std::sin(el * M_PI / 180.0);
                tot += cell;
                if (reachable(double(az), double(el), ref, ra_lim, dec_lim))
                    got += cell;
            }
        std::printf("hemisphere coverage: %.1f%%   (RA %.0f, DEC %.0f)\n",
                    100.0 * got / tot, ra_lim, dec_lim);
        int hi = 0, hiaz = 0;
        for (int az = 0; az < 360; ++az) {
            int best = 91;
            for (int el = 0; el <= 90; ++el)
                if (reachable(double(az), double(el), ref, ra_lim, dec_lim)) { best = el; break; }
            if (best > hi) { hi = best; hiaz = az; }
        }
        std::printf("worst azimuth floor: az %3d -> cannot point below el %d there\n", hiaz, hi);
        return 0;
    }
    std::printf("az  lowest_reachable_el  (91 = never)\n");
    for (int az = 0; az < 360; az += 5) {
        int best = 91;
        for (int el = 0; el <= 90; ++el)
            if (reachable(az, el, ref, ra_lim, dec_lim)) { best = el; break; }
        std::printf("%3d  %d\n", az, best);
    }
    return 0;
}
