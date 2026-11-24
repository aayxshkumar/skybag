// Take-off and landing performance by integrating the equations of motion
// along the runway (fixed time step, semi-implicit Euler), not by table
// lookup. Speeds the crew sees are calibrated airspeed in knots; internally
// everything is true airspeed in m/s along the runway.
#pragma once

#include <string>

#include "skybag/aircraft.hpp"

namespace skybag {

struct RunwayConditions {
  double elevation_ft = 0;
  double qnh_hpa = 1013.25;
  double oat_c = 15;
  double headwind_kt = 0;  // negative = tailwind (already the component along the runway)
  double slope_pct = 0;    // positive = uphill
  bool wet = false;
};

struct TakeoffInput {
  double mass_kg = 70000;
  RunwayConditions rw;
  double tora_m = 3000, toda_m = 3000, asda_m = 3000;
  double dt = 0.05;  // integration step, s
};

struct TakeoffSpeeds {
  double vs_kt = 0, vr_kt = 0, v2_kt = 0;
};

struct TakeoffResult {
  TakeoffSpeeds speeds;
  double v1_kt = 0;
  double tod_all_engines_m = 0;  // to 35 ft with all engines
  double tod_oei_m = 0;          // engine failure at V1, continued to 35 ft
  double asd_m = 0;              // engine failure at V1, stopped
  double required_m = 0;         // max(1.15 x all engines, OEI, ASD): the runway the take-off needs
  bool balanced = false;         // V1 found where OEI distance = ASD (not clamped to VMCG or VR)
  bool ok = false;               // fits the declared distances
  std::string limit;             // what limits it, if not ok
};

TakeoffSpeeds takeoff_speeds(const Aircraft& ac, double mass_kg, const RunwayConditions& rw);
// Engine failure at vef_kt (CAS), take-off continued to 35 ft (vef_kt < 0: no failure).
double continued_distance(const Aircraft& ac, const TakeoffInput& in, double vef_kt);
// Engine failure at vef_kt, reaction time, then idle + max braking to a stop.
double accelerate_stop_distance(const Aircraft& ac, const TakeoffInput& in, double vef_kt);
// Balanced V1 by bisection between VMCG and VR.
TakeoffResult takeoff(const Aircraft& ac, const TakeoffInput& in);

struct LandingInput {
  double mass_kg = 60000;
  RunwayConditions rw;
  double lda_m = 3000;
  double dt = 0.05;
};

struct LandingResult {
  double vref_kt = 0;
  double air_m = 0, ground_m = 0, actual_m = 0;
  double required_m = 0;  // actual / 0.6 dry, x 1.15 more when wet
  bool ok = false;
  std::string limit;      // what limits it, if not ok
};

LandingResult landing(const Aircraft& ac, const LandingInput& in);

}  // namespace skybag
