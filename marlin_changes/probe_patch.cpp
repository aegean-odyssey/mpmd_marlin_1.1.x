/* PROBE_PATCH.CPP
 * probe error compensation for the Monoprice MP Mini Delta 3d printer
 * Copyright (c) 2020 Aegean Associates, Inc. All rights reserved.
 */

/*
  The Monoprice MP Mini Delta 3d printer employs three (3) mechanical 
  switches under its build plate to detect the nozzle's contact with
  the build plate. This ability gives the Mini Delta a means for auto-
  matic calibration and bed-leveling, by "tapping" the build plate to
  determine a height (z-axis) value. The technique is flawed, though,
  as the build plate tilts during the measurement process.

  Here we provide an extremely naive model of the measurement error
  associated with the "tapping" process. The reasoning is as follows:

  + Normally, the build plate rests on top of three micro-switches.

  + The nozzle presses on the build plate until any one of the three
    switches makes contact. At this point, the current z-axis position
    is the build plate height (for a given x and y position).

  + There are seven possible orientations of the build plate in which
    at least one switch is depressed. We make the (probably wrong)
    simplifying assumption that any swith is at one end of its travel
    or the other (i.e. the z-axis position at the location of the switch
    is either zero or its full travel position, nothing in-between).

  + All three switches may make contact at the center (x,y = 0,0) of the
    build plate. The code below uses the radius of the probe's position 
    to the center of the build plate to identify the "all three switches
    depressed" orientation of the build plate. Otherwise, the angle of the
    probe's position around the z-axis determines which of the remaining 
    six orientations to use. There are three small regions where two of
    the three switches are depressed, and three remaining regions where
    only one of the three switches is depressed.

  + The code computes a plane for each orientation. Once the orientation
    is selected (from the probe's x, y position), the code computes the
    "expected" z-axis value, z = f(x,y), where f() is the equation of a 
    plane for the selected orientation. This z value represents the 
    measurement (height) error due to the tilt of the build plate.

  + Finally, we remove the switch travel from the result to normalize
    the (height) error to zero at (x:0,y:0). This last step is to mimic
    an "ideal" probe where the offset is uniform across the bed *AND* in
    such a way that is backward compatible with the existing calibration
    procedure for our firmware.
*/

#include <math.h>

#define SWITCH_TRAVEL  -0.5
#define SWITCH_RADIUS  50.0
#define ENDCAP_RADIUS  65.0

// area where 3 switches are depressed (mm)
#define CENTER_RADIUS  (SWITCH_RADIUS/2.0)
// area where 2 switches are depressed (degrees)
#define SWITCHx2_AREA  10.0

/* coordinates for each switch in opened and closed positions */

// tower/switch A, R:r θ:90°
#define Ax(r)  (0.0)
#define Ay(r)  (r)
#define A0  Ax(ENDCAP_RADIUS),Ay(ENDCAP_RADIUS),0.0
#define A1  Ax(SWITCH_RADIUS),Ay(SWITCH_RADIUS),SWITCH_TRAVEL

// tower/switch B, R:r, θ:210°
#define Bx(r)  (-(r) * sqrt(3.0) /2.0)
#define By(r)  (-(r) /2.0)
#define B0  Bx(ENDCAP_RADIUS),By(ENDCAP_RADIUS),0.0
#define B1  Bx(SWITCH_RADIUS),By(SWITCH_RADIUS),SWITCH_TRAVEL

// tower/switch C, R:r, θ:330°
#define Cx(r)  (+(r) * sqrt(3.0) /2.0)
#define Cy(r)  (-(r) /2.0)
#define C0  Cx(ENDCAP_RADIUS),Cy(ENDCAP_RADIUS),0.0
#define C1  Cx(SWITCH_RADIUS),Cy(SWITCH_RADIUS),SWITCH_TRAVEL


/* plane equation
   A x + B y + C z + D = 0
   where
   A = (Ay*Cz - By*Cz + Bz*Cy - Az*Cy - Ay*Bz + Az*By)
   B = (Bx*Cz - Ax*Cz - Bz*Cx + Az*Cx + Ax*Bz - Az*Bx)
   C = (Ax*Cy - Bx*Cy + By*Cx - Ay*Cx - Ax*By + Ay*Bx)
   D = (Ax*By*Cz - Ay*Bx*Cz - Ax*Bz*Cy + Az*Bx*Cy + Ay*Bz*Cx - Az*By*Cx)
*/

// A x + B y + C z + D = 0
#define A_(a,b,c,d,e,f,g,h,i)  (b*i - e*i + f*h - c*h - b*f + c*e)
#define B_(a,b,c,d,e,f,g,h,i)  (d*i - a*i - f*g + c*g + a*f - c*d)
#define C_(a,b,c,d,e,f,g,h,i)  (a*h - d*h + e*g - b*g - a*e + b*d)
#define D_(a,b,c,d,e,f,g,h,i)  (a*e*i - b*d*i - a*f*h + c*d*h + b*f*g - c*e*g)

// z = f(x,y) = MX x + MY y + ZO
#define MX_(a,b,c,d,e,f,g,h,i)  (-A_(a,b,c,d,e,f,g,h,i)/C_(a,b,c,d,e,f,g,h,i))
#define MY_(a,b,c,d,e,f,g,h,i)  (-B_(a,b,c,d,e,f,g,h,i)/C_(a,b,c,d,e,f,g,h,i))
#define ZO_(a,b,c,d,e,f,g,h,i)  (-D_(a,b,c,d,e,f,g,h,i)/C_(a,b,c,d,e,f,g,h,i))

// a little hack here to go from 3 arguments to 9 arguments
#define A_xyz  double a, double b, double c
#define B_xyz  double d, double e, double f
#define C_xyz  double g, double h, double i
constexpr float MX(A_xyz, B_xyz, C_xyz) { return MX_(a,b,c,d,e,f,g,h,i); }
constexpr float MY(A_xyz, B_xyz, C_xyz) { return MY_(a,b,c,d,e,f,g,h,i); }
constexpr float ZO(A_xyz, B_xyz, C_xyz) { return ZO_(a,b,c,d,e,f,g,h,i); }

typedef struct {
    float Mx, My, Zo;
} plane_t;

constexpr plane_t ao_plane_coefficients[] =
    {{ MX(A1,B1,C1), MY(A1,B1,C1), ZO(A1,B1,C1) - SWITCH_TRAVEL },
     { MX(A1,B0,C0), MY(A1,B0,C0), ZO(A1,B0,C0) - SWITCH_TRAVEL },
     { MX(A1,B1,C0), MY(A1,B1,C0), ZO(A1,B1,C0) - SWITCH_TRAVEL },
     { MX(A0,B1,C0), MY(A0,B1,C0), ZO(A0,B1,C0) - SWITCH_TRAVEL },
     { MX(A0,B1,C1), MY(A0,B1,C1), ZO(A0,B1,C1) - SWITCH_TRAVEL },
     { MX(A0,B0,C1), MY(A0,B0,C1), ZO(A0,B0,C1) - SWITCH_TRAVEL },
     { MX(A1,B0,C1), MY(A1,B0,C1), ZO(A1,B0,C1) - SWITCH_TRAVEL }};

#ifndef RADIANS
#define RADIANS(a)  ((a) * M_PI /180.0)
#endif

int ao_choose_plane(float x, float y)
{
    // zone considered as the center area
    float r = sqrtf(x*x + y*y);
    if (r <= CENTER_RADIUS) return 0;

    // sector angle w.r.t switch/tower A
    float a = atan2f(x, y); // swap x,y to rotate 90°
    if (a <= RADIANS(-(180 - SWITCHx2_AREA))) return 4;
    if (a <= RADIANS(-( 60 + SWITCHx2_AREA))) return 3;
    if (a <= RADIANS(-( 60 - SWITCHx2_AREA))) return 2;
    if (a <= 0.0) return 1;
    if (a >= RADIANS(+(180 - SWITCHx2_AREA))) return 4;
    if (a >= RADIANS(+( 60 + SWITCHx2_AREA))) return 5;
    if (a >= RADIANS(+( 60 - SWITCHx2_AREA))) return 6;
    if (a >= 0.0) return 1;

    return 0; // should never happen
}

float ao_z_correction(float x, float y)
{
    plane_t p = ao_plane_coefficients[ao_choose_plane(x, y)];
    return p.Mx * x + p.My * y + p.Zo;
}
