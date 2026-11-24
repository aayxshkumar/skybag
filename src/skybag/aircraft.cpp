#include "skybag/aircraft.hpp"

#include <algorithm>
#include <cmath>

#include "skybag/units.hpp"

namespace skybag {

double Aircraft::thrust(double v, double rho, double a, double delta_t, int operating) const {
  // Density lapse sigma^0.75 and a linear-in-Mach drop: a common textbook
  // shape for a high-bypass turbofan at take-off power.
  const double sigma = rho / kIsaRho0;
  const double mach = v / a;
  double t = static_thrust * std::pow(sigma, 0.75) * (1.0 - 0.35 * mach);
  // Flat rating: sigma already lowers thrust on a hot day; above the
  // flat-rating temperature the engine also hits its EGT limit.
  if (delta_t > flat_rating_dt) t *= std::max(0.0, 1.0 - hot_thrust_loss * (delta_t - flat_rating_dt));
  return t * operating;
}

const Aircraft& sb20() {
  static const Aircraft ac;
  return ac;
}

const std::vector<Station>& sb20_stations() {
  static const std::vector<Station> s = {
      {"crew", 4.0, 400},          {"pax-forward", 12.5, 50 * 84.0}, {"pax-mid", 18.0, 50 * 84.0},
      {"pax-aft", 24.5, 50 * 84.0}, {"cargo-forward", 10.5, 3400},   {"cargo-aft", 26.5, 4500},
  };
  return s;
}

const std::vector<std::pair<double, double>>& sb20_envelope() {
  // Narrow at low mass, widest in the middle, forward limit moves aft at MTOW.
  static const std::vector<std::pair<double, double>> e = {
      {40000, 17.0}, {40000, 36.0}, {53000, 39.0}, {66000, 38.0}, {74000, 33.5},
      {74000, 23.0}, {64000, 18.0}, {53000, 16.0},
  };
  return e;
}

}  // namespace skybag
