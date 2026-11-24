#include "skybag/wb.hpp"

#include <algorithm>
#include <cmath>

namespace skybag {

bool point_in_polygon(const std::vector<std::pair<double, double>>& poly, double x, double y) {
  const std::size_t n = poly.size();
  if (n < 3) return false;
  bool inside = false;
  for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
    const double xi = poly[i].first, yi = poly[i].second, xj = poly[j].first, yj = poly[j].second;
    // On this edge? Collinear and within the bounding box.
    const double cross = (xj - xi) * (y - yi) - (yj - yi) * (x - xi);
    const double scale = std::max({std::abs(xj - xi), std::abs(yj - yi), 1.0});
    if (std::abs(cross) <= 1e-9 * scale * scale && x >= std::min(xi, xj) && x <= std::max(xi, xj) &&
        y >= std::min(yi, yj) && y <= std::max(yi, yj))
      return true;
    // Crossing test on a horizontal ray to +x, half-open in y.
    if ((yi > y) != (yj > y)) {
      const double xc = xi + (y - yi) * (xj - xi) / (yj - yi);
      if (x < xc) inside = !inside;
    }
  }
  return inside;
}

double cg_mac(const Aircraft& ac, double moment, double mass) {
  return (moment / mass - ac.lemac_m) / ac.mac_m * 100.0;
}

WbResult weight_and_balance(const Aircraft& ac, const std::vector<LoadItem>& load, double block_fuel_kg,
                            double taxi_kg, double trip_kg) {
  WbResult r;
  double mass = ac.oew, moment = ac.oew * ac.oew_arm_m;
  for (const LoadItem& item : load) {
    const auto& st = sb20_stations();
    const auto it = std::find_if(st.begin(), st.end(), [&](const Station& s) { return s.name == item.station; });
    if (it == st.end()) {
      r.problems.push_back("unknown station " + item.station);
      continue;
    }
    if (item.kg < 0) r.problems.push_back(item.station + " has a negative load");
    if (item.kg > it->max_kg) r.problems.push_back(item.station + " over its limit");
    mass += item.kg;
    moment += item.kg * it->arm_m;
  }
  const auto& env = sb20_envelope();
  auto add = [&](const char* name, double m, double mom) {
    WbPoint p{name, m, cg_mac(ac, mom, m), false};
    p.inside = point_in_polygon(env, p.mass_kg, p.cg_mac);
    if (!p.inside) r.problems.push_back(std::string(name) + " outside the CG envelope");
    r.points.push_back(p);
  };
  add("ZFW", mass, moment);
  const double tow_fuel = block_fuel_kg - taxi_kg;
  add("TOW", mass + tow_fuel, moment + tow_fuel * ac.fuel_arm_m);
  const double ldw_fuel = tow_fuel - trip_kg;
  add("LDW", mass + ldw_fuel, moment + ldw_fuel * ac.fuel_arm_m);
  if (mass > ac.mzfw) r.problems.push_back("zero-fuel mass above MZFW");
  if (mass + tow_fuel > ac.mtow) r.problems.push_back("take-off mass above MTOW");
  if (mass + ldw_fuel > ac.mlw) r.problems.push_back("landing mass above MLW");
  if (block_fuel_kg > ac.max_fuel) r.problems.push_back("fuel above tank capacity");
  if (ldw_fuel < 0) r.problems.push_back("trip fuel larger than the fuel on board");
  r.ok = r.problems.empty();
  return r;
}

}  // namespace skybag
