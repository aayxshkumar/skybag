// Flight plan: route legs on the WGS84 geodesic, a climb / cruise / descent
// profile, the wind triangle on each leg, and the fuel plan (trip,
// contingency, final reserve, alternate, taxi -> block fuel), iterated until
// the take-off mass and the fuel agree.
#pragma once

#include <string>
#include <vector>

#include "skybag/aircraft.hpp"
#include "skybag/geo.hpp"
#include "skybag/wind.hpp"

namespace skybag {

struct Waypoint {
  std::string name;
  LatLon pos;
};

struct PlanInput {
  std::vector<Waypoint> route;  // at least two points
  double cruise_ft = 35000;
  double mach = 0.78;
  double zfw_kg = 58000;
  double taxi_kg = 200;
  double contingency_pct = 5;
  double final_reserve_min = 30;
  double alternate_kg = 0;
};

struct LegPlan {
  std::string from, to;
  int leg = 0;        // index of the route leg this piece belongs to
  std::string phase;  // "climb", "cruise", "descent": legs are split at TOC and TOD
  double dist_nm = 0, course_deg = 0, heading_deg = 0;
  double wind_from_deg = 0, wind_kt = 0, tas_kt = 0, gs_kt = 0;
  double alt_start_ft = 0, alt_end_ft = 0;
  double time_min = 0, fuel_kg = 0;
};

struct Plan {
  bool ok = false;
  std::string error;
  std::vector<LegPlan> legs;
  double cruise_ft = 0;  // may be lower than asked if the route is too short
  double dist_nm = 0, time_min = 0;
  double toc_nm = 0, tod_nm = 0;  // along-track distance of top of climb / descent
  double trip_kg = 0, contingency_kg = 0, reserve_kg = 0, alternate_kg = 0, taxi_kg = 0, block_kg = 0;
  double tow_kg = 0, ldw_kg = 0;
  std::vector<std::string> warnings;  // MTOW / MLW / tank capacity
};

// The planner evaluates the wind once per leg piece (at the start of the
// piece, at the piece's mean altitude) and holds course and mass constant
// across the piece. Fine for short legs; on a long direct leg the wind and
// course change underneath it. KNOWN GAP (see --check fuel-along-leg).
Plan plan(const Aircraft& ac, const WindField& wind, const PlanInput& in);

// Reference trip fuel for a given take-off mass: the same profile flown in
// small steps along the geodesic with the local course, the local wind at the
// actual altitude, and the mass updated every step.
double reference_trip_fuel(const Aircraft& ac, const WindField& wind, const PlanInput& in, double cruise_ft,
                           double tow_kg, double step_nm, double* time_min = nullptr);

// Fuel flow in cruise, kg/h, at a mass, pressure altitude, Mach and ISA deviation.
double cruise_fuel_flow(const Aircraft& ac, double mass_kg, double alt_ft, double mach, double isa_dev);
// Holding fuel flow at 1500 ft, kg/h (used for the final reserve).
double holding_fuel_flow(const Aircraft& ac, double mass_kg);

}  // namespace skybag
