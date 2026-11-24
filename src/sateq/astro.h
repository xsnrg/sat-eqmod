// Minimal astronomy for equatorial satellite tracking. Hand-rolled (no
// libnova on the dev box); Meeus-quality for our needs (<<0.01°). All angles
// degrees unless noted. UTC throughout; local time/DST never enters.
//
// Conventions
//  - azimuth: from true North through East (0..360)
//  - elevation: +up (deg)
//  - hour angle H: grows westward (0..360)
//  - longitude: +east
//  - "physical" mount axes: see sky_to_axes() below.
#pragma once
#include <chrono>
#include <cmath>

namespace sateq {

constexpr double DEG2RAD = M_PI / 180.0;
constexpr double RAD2DEG = 180.0 / M_PI;

inline double deg_norm360(double d)
{
    d = std::fmod(d, 360.0);
    if (d < 0) d += 360.0;
    return d;
}
inline double deg_norm180(double d)
{
    return std::fmod(d + 540.0, 360.0) - 180.0;
}

double julian_date(std::chrono::system_clock::time_point utc);
double gmst_deg(double jd_utc);                 // mean sidereal time at Greenwich
double lst_deg(double jd_utc, double lon_east_deg);

struct HsCoord { double az_deg, el_deg; };
struct EqCoord { double ra_deg, dec_deg; };
struct LatLon  { double lat_deg, lon_east_deg; };

// Sky <-> sky
HsCoord ha_dec_to_hs(double ha_deg, double dec_deg, double lat_deg);
// returns HA (0..360) and DEC from topocentric az/el
void    hs_to_ha_dec(double az_deg, double el_deg, double lat_deg,
                     double& ha_deg_out, double& dec_deg_out);
EqCoord hs_to_eq(const HsCoord&, double lat_deg, double lst_deg);

// Sky -> physical mount axes (standard form = west pier for a northern site).
// The daemon drives the two motor counters directly; the mapping is affine:
//   RA axis  physical angle = -HA          (forward motor dir increases pos;
//                                           verified in Phase-1 motion test)
//   DEC axis physical angle = 90 - DEC     (0 = polaris-side home)
// The tracker keeps RA "physical" UNWRAPPED (continuous) so cumulative
// rotation — the cord-wrap budget — is exactly the accumulated delta.
struct AxisAngles { double ra_deg, dec_deg; };

AxisAngles sky_to_axes(double ha_deg, double dec_deg);
void       axes_to_ha_dec(const AxisAngles&, double& ha_deg_out, double& dec_deg_out);

// Session orientation: how the head is physically configured. The motor
// axes are always (axis1 = RA-shaft motor, axis2 = DEC-shaft motor); only
// the sky<->motor-angle map changes. Motor angles below are the CONTINUOUS
// physical angles tracked by counters + unwrapping, same units as the
// polar AxisAngles above.
enum class Orientation {
    Polar,       // wedge/pier polar aligned (HA/DEC kinematics; meridian flips)
    AltAzLevel,  // equator-style pose: RA shaft HORIZONTAL due north, tube
                 // level. No meridian constraint; works for satellite work.
};

// Topocentric pointing -> continuous motor angles (a = axis1 deg, d = axis2
// deg) for the session orientation. AltAzLevel ignores lat_deg: the
// geometry is pure head-vs-horizon.
AxisAngles hs_to_axes(double az_deg, double el_deg, double lat_deg, Orientation o);
HsCoord    axes_to_hs(const AxisAngles&, double lat_deg, Orientation o);

// AltAzLevel motor definitions (tube unit vector
//   t = sin d * N_hat + cos d * (sin a * E_hat + cos a * U_hat)):
//   d (DEC motor): tube angle from the RA shaft axis; d=+90 tube level
//     north (along shaft), d=0 tube perpendicular to the shaft (a=90 level
//     east = SESSION START POSE), d=-90 tube level south.
//   a (RA motor): rotation about the horizontal north axis; a=0 tube in
//     meridian plane (zenith at d=0), a=+90 level east, a=-90 level west.
// Full-sky bijection except the north/south horizon POINTS (el=0, az=0/180),
// where the RA motor stops moving the tube; satellites never operate there.

} // namespace sateq
