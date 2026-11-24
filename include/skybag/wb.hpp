// Weight and balance: masses and arms per station, CG in %MAC, and the
// envelope check at zero-fuel, take-off and landing mass.
#pragma once

#include <string>
#include <utility>
#include <vector>

#include "skybag/aircraft.hpp"

namespace skybag {

// Point in a closed polygon (vertices in order, either winding). Points on an
// edge or a vertex count as inside: the limit lines are certified values.
bool point_in_polygon(const std::vector<std::pair<double, double>>& poly, double x, double y);

struct LoadItem {
  std::string station;
  double kg = 0;
};

struct WbPoint {
  std::string name;  // "ZFW", "TOW", "LDW"
  double mass_kg = 0, cg_mac = 0;
  bool inside = false;
};

struct WbResult {
  std::vector<WbPoint> points;
  bool ok = false;
  std::vector<std::string> problems;  // station overloads, structural limits, envelope
};

WbResult weight_and_balance(const Aircraft& ac, const std::vector<LoadItem>& load, double block_fuel_kg,
                            double taxi_kg, double trip_kg);

double cg_mac(const Aircraft& ac, double moment_kgm, double mass_kg);

}  // namespace skybag
