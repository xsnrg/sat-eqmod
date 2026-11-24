#include "sateq/astro.h"
#include <cmath>
#include <cstdio>
using namespace sateq;
static void unwrap(double& cur, double prev)
{ while (cur - prev > 180) cur -= 360; while (cur - prev < -180) cur += 360; }
int main() {
    double a0=0,d0=0,pa=0,pd=0,t,az,el; bool first=true;
    double last_rep_a=0, rep_t=0, rep_az=0, rep_el=0;
    while (std::scanf("%lf %lf %lf", &t,&az,&el)==3) {
        AxisAngles x = hs_to_axes(az, el, 44.95, Orientation::AltAzLevel);
        double a=x.ra_deg,d=x.dec_deg;
        if (!first){unwrap(a,pa);unwrap(d,pd);} else {a0=a;d0=d;}
        pa=a;pd=d;first=false;
        double da=a-a0;
        if (std::fabs(da-last_rep_a)>90){ last_rep_a=da; 
            std::printf("t=%7.1f dRA=%7.1f dDE=%7.1f az=%7.2f el=%6.2f\n",t,da,d-d0,az,el);}
        (void)rep_t;(void)rep_az;(void)rep_el;
    }
    return 0;
}
