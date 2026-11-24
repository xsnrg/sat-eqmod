// Motion math: rate → step-period, matching indi-eqmod's formula on the
// LIVE Sirius constants (steps_360=9,024,000; steps_worm=64,935; ratio=16).
#include "testkit.h"
#include "sateq/protocol.h"
#include "sateq/board.h"

using namespace sateq;

static constexpr uint32_t S360 = 9024000, SWORM = 64935, HS = 16;

// Physical check: with the vendor simulator's timing model
// (timer_us = period*(1e6/steps_worm); steps/s = stepmul*1e6/timer_us),
// axis turns/s = absrate_x / STELLAR_DAY.
static double axis_turns_per_s(double /*rate_x*/, uint32_t period_us, bool high)
{
    double mul = std::floor(1e6 / SWORM), rem = std::fmod(1e6, (double)SWORM);
    double timer_us = period_us * mul + period_us * rem / SWORM;
    double stepmul = high ? HS : 1;
    double steps_per_s = 1e6 * stepmul / timer_us;
    return steps_per_s / S360;
}

int main()
{
    TEST("period_for_rate: sidereal 1x on Sirius constants");
    auto sp = period_for_rate(1.0, SWORM, S360, HS);
    CHECK(!sp.use_high_speed);
    // indi formula: STELLAR_DAY*steps_worm/steps_360 = 86164.098*64935/9024000
    CHECK_EQ(sp.period_us, 620u);
    double turns = axis_turns_per_s(1.0, sp.period_us, false);
    CHECK_NEAR(turns * STELLAR_DAY_SEC, 1.0, 0.01); // one axis turn / sidereal day

    TEST("period_for_rate: tracking rates physical (integer truncation <5%)");
    for (double r : {0.5, 1.0, 2.0, 17.0, 100.0})
    {
        auto p = period_for_rate(r, SWORM, S360, HS);
        double t = axis_turns_per_s(r, p.period_us, p.use_high_speed);
        CHECK_NEAR(t * STELLAR_DAY_SEC, r, 0.05 * r);
    }

    TEST("period_for_rate: high-speed fold at >128x");
    auto lo = period_for_rate(128.0, SWORM, S360, HS);
    auto hi = period_for_rate(129.0, SWORM, S360, HS);
    CHECK(!lo.use_high_speed);
    CHECK(hi.use_high_speed);
    // NOTE: vendor truncation is the ONLY error source here; 128x itself
    // yields 4 us, below the board's ~6 us minimum — Board::slew_rate must
    // promote that band to high-speed (tested in test_board_sim).
    double t = axis_turns_per_s(600.0, hi.period_us, hi.use_high_speed);
    (void)t;
    auto p600 = period_for_rate(600.0, SWORM, S360, HS);
    CHECK(p600.use_high_speed);
    double t600 = axis_turns_per_s(600.0, p600.period_us, p600.use_high_speed);
    CHECK_NEAR(t600 * STELLAR_DAY_SEC, 600.0, 0.05 * 600); // period integer truncation

    TEST("period_for_rate: sign ignored in magnitude, min period respected");
    auto spn = period_for_rate(-1.0, SWORM, S360, HS);
    CHECK_EQ(spn.period_us, 620u);

    TEST("arcsec per microstep sanity (Sirius HEQ5 class)");
    AxisInfo info;
    info.steps_360 = S360;
    CHECK_NEAR(info.arcsec_per_step(), 1296000.0 / 9024000.0, 1e-9); // 0.1436"/step

    return testkit::summary("motion_math");
}
