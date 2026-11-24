// wind_probe: from the home datum, enumerate candidate push poses that wind
// the DEC cord (|folded dec offset| past 90 = upper ledger band) while the
// RA offset stays meaningfully away from datum — the material for the
// az-first cord-unwind live drive test. Branch choice = nearest in-box
// (select_branch semantics replicated as in dome_coverage).
//   g++ -O2 -I src tools/analysis/wind_probe.cpp build/libsateq.a -o /tmp/wp
#include "sateq/astro.h"
#include <cmath>
#include <cstdio>
using namespace sateq;
static double fold180(double x)
{
    x = std::fmod(x + 180.0, 360.0);
    if (x < 0.0) x += 360.0;
    return x - 180.0;
}
int main()
{
    const double ra_lim = 115.0, dec_lim = 180.0;
    AxisAngles ref = hs_to_axes(90.0, 0.0, 0.0, Orientation::AltAzLevel);
    std::printf("az   el | branch  RAoff   DEoff  ledger-band  axis-dist\n");
    for (int el : {3, 5, 8, 12, 20, 30, 45})
        for (int az = 0; az < 360; az += 10)
        {
            AxisAngles can = hs_to_axes((double)az, (double)el, 0.0, Orientation::AltAzLevel);
            AxisAngles flip{can.ra_deg + 180.0, 180.0 - can.dec_deg};
            double c_r = fold180(can.ra_deg - ref.ra_deg), c_d = fold180(can.dec_deg - ref.dec_deg);
            double f_r = fold180(flip.ra_deg - ref.ra_deg), f_d = fold180(flip.dec_deg - ref.dec_deg);
            bool cb = std::fabs(c_r) <= ra_lim && std::fabs(c_d) <= dec_lim;
            bool fb = std::fabs(f_r) <= ra_lim && std::fabs(f_d) <= dec_lim;
            if (!cb && !fb) continue;
            bool use_flip = fb && (!cb || std::hypot(f_r, f_d) < std::hypot(c_r, c_d));
            double r = use_flip ? f_r : c_r, d = use_flip ? f_d : c_d;
            double band = std::fmod(d < 0 ? d + 360.0 : d, 360.0);
            bool upper = band >= 90.0 && band <= 270.0;
            double dist = std::hypot(r, d);
            if (upper && std::fabs(r) >= 45.0 && dist >= 90.0 && dist <= 190.0)
                std::printf("%3d %3d | flip%d %+7.1f %+7.1f  %s  %6.1f\n",
                            az, el, use_flip ? 1 : 0, r, d, upper ? "YES" : "no ", dist);
        }
    return 0;
}
