#include "sateq/astro.h"

#include <algorithm>

namespace sateq {

using Clock = std::chrono::system_clock;

double julian_date(Clock::time_point utc)
{
    // days since 1970-01-01T00:00:00Z; Unix epoch == JD 2440587.5
    double days = std::chrono::duration<double>(utc.time_since_epoch()).count() / 86400.0;
    return days + 2440587.5;
}

double gmst_deg(double jd_utc)
{
    // Meeus 12.2 low-order form; quadratic term negligible over decades.
    double t = (jd_utc - 2451545.0) / 36525.0;
    double g = 280.46061837 +
               360.98564736629 * (jd_utc - 2451545.0) +
               0.000387933 * t * t - t * t * t / 38710000.0;
    return deg_norm360(g);
}

double lst_deg(double jd_utc, double lon_east_deg)
{
    return deg_norm360(gmst_deg(jd_utc) + lon_east_deg);
}

HsCoord ha_dec_to_hs(double ha_deg, double dec_deg, double lat_deg)
{
    double ha = ha_deg * DEG2RAD;
    double dec = dec_deg * DEG2RAD;
    double lat = lat_deg * DEG2RAD;

    double sin_el = std::sin(dec) * std::sin(lat) +
                    std::cos(dec) * std::cos(lat) * std::cos(ha);
    sin_el = std::clamp(sin_el, -1.0, 1.0);
    double el = std::asin(sin_el);

    double y = std::sin(ha);
    double x = std::cos(ha) * std::sin(lat) - std::tan(dec) * std::cos(lat);
    double az = std::atan2(y, x) * RAD2DEG + 180.0; // az from N through E

    return HsCoord{deg_norm360(az), el * RAD2DEG};
}

void hs_to_ha_dec(double az_deg, double el_deg, double lat_deg,
                  double& ha_deg_out, double& dec_deg_out)
{
    double az = deg_norm360(az_deg) * DEG2RAD;
    double el = el_deg * DEG2RAD;
    double lat = lat_deg * DEG2RAD;

    double sin_dec = std::sin(el) * std::sin(lat) +
                     std::cos(el) * std::cos(lat) * std::cos(az);
    sin_dec = std::clamp(sin_dec, -1.0, 1.0);
    double dec = std::asin(sin_dec);

    double cd = std::cos(dec);
    double sin_h = -std::cos(el) * std::sin(az) / cd;
    double cos_h = (std::sin(el) - sin_dec * std::sin(lat)) / (cd * std::cos(lat));
    double ha = std::atan2(sin_h, cos_h) * RAD2DEG;

    ha_deg_out = deg_norm360(ha);
    dec_deg_out = dec * RAD2DEG;
}

EqCoord hs_to_eq(const HsCoord& hs, double lat_deg, double lst)
{
    double ha, dec;
    hs_to_ha_dec(hs.az_deg, hs.el_deg, lat_deg, ha, dec);
    return EqCoord{deg_norm360(lst - ha), dec};
}

AxisAngles sky_to_axes(double ha_deg, double dec_deg)
{
    return AxisAngles{-ha_deg, 90.0 - dec_deg};
}

void axes_to_ha_dec(const AxisAngles& ax, double& ha_deg_out, double& dec_deg_out)
{
    ha_deg_out = deg_norm360(-ax.ra_deg);
    dec_deg_out = 90.0 - ax.dec_deg;
}

AxisAngles hs_to_axes(double az_deg, double el_deg, double lat_deg, Orientation o)
{
    if (o == Orientation::Polar)
    {
        double ha, dec;
        hs_to_ha_dec(az_deg, el_deg, lat_deg, ha, dec);
        return sky_to_axes(ha, dec);
    }
    // AltAzLevel: closed form from t = sin d * N + cos d * (sin a * E + cos a * U)
    double az = deg_norm360(az_deg) * DEG2RAD;
    double el = el_deg * DEG2RAD;
    double sin_d = std::cos(el) * std::cos(az);
    sin_d = std::clamp(sin_d, -1.0, 1.0);
    double d = std::asin(sin_d) * RAD2DEG;
    double a = std::atan2(std::cos(el) * std::sin(az), std::sin(el)) * RAD2DEG;
    return AxisAngles{a, d};
}

HsCoord axes_to_hs(const AxisAngles& ax, double lat_deg, Orientation o)
{
    if (o == Orientation::Polar)
    {
        double ha, dec;
        axes_to_ha_dec(ax, ha, dec);
        return ha_dec_to_hs(ha, dec, lat_deg);
    }
    double a = ax.ra_deg * DEG2RAD;
    double d = ax.dec_deg * DEG2RAD;
    double cd = std::cos(d);
    double e = cd * std::sin(a);
    double u = cd * std::cos(a);
    double n = std::sin(d);
    return HsCoord{deg_norm360(std::atan2(e, n) * RAD2DEG),
                   std::asin(std::clamp(u, -1.0, 1.0)) * RAD2DEG};
}

} // namespace sateq
