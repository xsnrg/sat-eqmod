// Astronomy: JD/GMST against published values, coordinate round-trips,
// and a hard-coded reference check (an independent implementation's output).
#include "testkit.h"
#include "sateq/astro.h"

#include <ctime>

using namespace sateq;

// 45N 93W-like reference site (Minneapolis-area bench): lat 44.95, lon -93.28
static constexpr double LAT = 44.95, LON = -93.28;

int main()
{
    TEST("julian_date against published epoch values");
    {
        using namespace std::chrono;
        // 2000-01-01T12:00:00Z == JD 2451545.0 (J2000.0)
        // Unix time for that instant: 946728000
        auto j2000 = system_clock::from_time_t(946728000);
        CHECK_NEAR(julian_date(j2000), 2451545.0, 1e-6);
        // 1970-01-01T00:00:00Z == JD 2440587.5
        auto epoch = system_clock::from_time_t(0);
        CHECK_NEAR(julian_date(epoch), 2440587.5, 1e-6);
        // 2026-11-15T00:00:00Z (unix 1794700800) == JD 2461359.5
        auto d = system_clock::from_time_t(1794700800);
        CHECK_NEAR(julian_date(d), 2461359.5, 1e-4);
    }

    TEST("gmst known-good value");
    {
        // 2026-11-15 00:00 UT via Meeus 12.2: 54.10°. Independent sanity:
        // sun RA ~23h20m on Nov 15 → GMST at 0h UT = sunRA+12h ≈ 9h20m? no —
        // GMST(0h UT) = 280.46+360.9856*d model IS the NOAA form; cross-checked
        // against rough planetarium reasoning (LST Kansas City 00UT ~324°).
        // Tolerance covers omitted nutation/equation-of-equinoxes (<0.3°).
        using namespace std::chrono;
        double jd = julian_date(system_clock::from_time_t(1794700800));
        double g = gmst_deg(jd);
        CHECK_NEAR(g, 54.10, 0.3);
    }

    TEST("lst = gmst + lon");
    {
        using namespace std::chrono;
        double jd = julian_date(system_clock::from_time_t(1763164800));
        CHECK_NEAR(deg_norm360(lst_deg(jd, LON) - (gmst_deg(jd) + LON)), 0.0, 1e-9);
    }

    TEST("equator object rises due east, sets due west");
    {
        // dec=0, lat=44.95: at HA=-90 (6h east) az should be ~90 (east)
        HsCoord e = ha_dec_to_hs(360.0 - 90.0, 0.0, LAT);
        CHECK_NEAR(e.el_deg, 0.0, 0.02);
        CHECK_NEAR(deg_norm180(e.az_deg - 90.0), 0.0, 0.5); // circular diff
        HsCoord w = ha_dec_to_hs(90.0, 0.0, LAT);
        CHECK_NEAR(w.el_deg, 0.0, 0.02);
        CHECK_NEAR(deg_norm180(w.az_deg - 270.0), 0.0, 0.5);
    }

    TEST("transit: HA=0 → due south, el = 90-lat+dec");
    {
        HsCoord t = ha_dec_to_hs(0.0, 20.0, LAT);
        CHECK_NEAR(t.az_deg, 180.0, 1e-6);
        CHECK_NEAR(t.el_deg, 90.0 - LAT + 20.0, 1e-6);
    }

    TEST("circumpolar: never sets when dec > 90-lat");
    {
        for (double ha = 0; ha < 360; ha += 7.0)
        {
            HsCoord p = ha_dec_to_hs(ha, 80.0, LAT);
            CHECK(p.el_deg > 0.0);
        }
    }

    TEST("hs <-> ha/dec round-trip (grid)");
    {
        for (double lat : {-33.0, 0.0, 44.95, 65.0})
            for (double ha = 0; ha < 360.0; ha += 23.0)
                for (double dec = -80.0; dec <= 80.0; dec += 31.0)
                {
                    HsCoord hs = ha_dec_to_hs(ha, dec, lat);
                    double ha2, dec2;
                    hs_to_ha_dec(hs.az_deg, hs.el_deg, lat, ha2, dec2);
                    CHECK_NEAR(dec2, dec, 1e-6);
                    double dha = std::fabs(deg_norm180(ha2 - ha));
                    CHECK_NEAR(dha, 0.0, 1e-6);
                }
    }

    TEST("hs -> eq uses LST: ra = lst - ha");
    {
        double lst = 100.0;
        HsCoord hs = ha_dec_to_hs(30.0, 15.0, LAT);
        EqCoord eq = hs_to_eq(hs, LAT, lst);
        CHECK_NEAR(eq.ra_deg, deg_norm360(lst - 30.0), 1e-6);
        CHECK_NEAR(eq.dec_deg, 15.0, 1e-6);
    }

    TEST("sky_to_axes / axes_to_ha_dec round-trip");
    {
        AxisAngles a = sky_to_axes(123.4, 56.7);
        CHECK_NEAR(a.ra_deg, -123.4, 1e-12);
        CHECK_NEAR(a.dec_deg, 90.0 - 56.7, 1e-12);
        double ha, dec;
        axes_to_ha_dec(a, ha, dec);
        CHECK_NEAR(ha, 123.4, 1e-9);
        CHECK_NEAR(dec, 56.7, 1e-9);
    }

    TEST("AltAzLevel golden poses");
    {
        // session start pose: tube level EAST -> a=+90, d=0
        AxisAngles e = hs_to_axes(90, 0, LAT, Orientation::AltAzLevel);
        CHECK_NEAR(e.ra_deg, 90.0, 1e-9);
        CHECK_NEAR(e.dec_deg, 0.0, 1e-9);
        // level west -> a=-90; north horizon (along shaft) d=+90; zenith d=0,a=0
        AxisAngles w = hs_to_axes(270, 0, LAT, Orientation::AltAzLevel);
        CHECK_NEAR(w.ra_deg, -90.0, 1e-9);
        CHECK_NEAR(w.dec_deg, 0.0, 1e-9);
        AxisAngles n = hs_to_axes(0, 0, LAT, Orientation::AltAzLevel);
        CHECK_NEAR(n.dec_deg, 90.0, 1e-9);
        AxisAngles z = hs_to_axes(0, 90, LAT, Orientation::AltAzLevel);
        CHECK_NEAR(z.ra_deg, 0.0, 1e-9);
        CHECK_NEAR(z.dec_deg, 0.0, 1e-9);
        // forward from motors: (a=0,d=90) must be due north horizon
        HsCoord hn = axes_to_hs(AxisAngles{0, 90}, LAT, Orientation::AltAzLevel);
        CHECK_NEAR(hn.az_deg, 0.0, 1e-9);
        CHECK_NEAR(hn.el_deg, 0.0, 1e-9);
        // (a=0,d=-90) due south horizon
        HsCoord hs = axes_to_hs(AxisAngles{0, -90}, LAT, Orientation::AltAzLevel);
        CHECK_NEAR(hs.az_deg, 180.0, 1e-9);
        CHECK_NEAR(hs.el_deg, 0.0, 1e-9);
    }

    TEST("AltAzLevel sky->axes->sky round-trip (full sky grid)");
    {
        for (double el = 0.5; el <= 90.0; el += 0.5)
            for (double az = 0; az < 360; az += 1.0)
            {
                // skip the two horizon singular points (az N/S at el~0)
                if (el < 1.0 && (az < 1.0 || (az > 179.0 && az < 181.0))) continue;
                AxisAngles ax = hs_to_axes(az, el, LAT, Orientation::AltAzLevel);
                // RA motor never exceeds +-90 over the whole sky hemisphere
                CHECK(ax.ra_deg >= -90.0 - 1e-6 && ax.ra_deg <= 90.0 + 1e-6);
                HsCoord hs = axes_to_hs(ax, LAT, Orientation::AltAzLevel);
                CHECK_NEAR(std::fabs(deg_norm180(hs.az_deg - az)), 0.0, 1e-6);
                CHECK_NEAR(hs.el_deg, el, 1e-6);
            }
    }

    TEST("AltAzLevel ignores latitude (pure head-vs-horizon geometry)");
    {
        AxisAngles a = hs_to_axes(123.0, 40.0, 44.95, Orientation::AltAzLevel);
        AxisAngles b = hs_to_axes(123.0, 40.0, -12.0, Orientation::AltAzLevel);
        CHECK_NEAR(a.ra_deg, b.ra_deg, 1e-12);
        CHECK_NEAR(a.dec_deg, b.dec_deg, 1e-12);
    }

    TEST("AltAzLevel: satellite-like track stays in motor travel envelope");
    {
        // a low pass rising E, transiting S, setting W: axes must move
        // smoothly through the central +-90 travel band (no wrap surprises)
        double prev_a = 0, prev_d = 0;
        bool first = true;
        for (double t = 0; t <= 1.0; t += 0.01)
        {
            double az = 90 + t * 180.0;            // E(90) -> S(180) -> W(270)
            double el = 10 + 70 * std::sin(M_PI * t); // rise to ~80 set
            if (el <= 0) continue;
            AxisAngles ax = hs_to_axes(az, el, LAT, Orientation::AltAzLevel);
            if (!first)
            {
                CHECK_NEAR(ax.ra_deg - prev_a, 0.0, 25.0);  // smooth, no jumps
                CHECK_NEAR(ax.dec_deg - prev_d, 0.0, 25.0);
            }
            prev_a = ax.ra_deg; prev_d = ax.dec_deg; first = false;
        }
    }

    return testkit::summary("astro");
}
