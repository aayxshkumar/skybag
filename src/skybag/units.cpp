#include "skybag/units.hpp"

#include <cmath>

namespace skybag {

Atmosphere isa(double h, double delta_t) {
  Atmosphere at;
  double t_isa, p;
  if (h <= kTropopause) {
    t_isa = kIsaT0 - kIsaLapse * h;
    p = kIsaP0 * std::pow(t_isa / kIsaT0, kG0 / (kIsaLapse * kR));
  } else {
    const double t11 = kIsaT0 - kIsaLapse * kTropopause;
    const double p11 = kIsaP0 * std::pow(t11 / kIsaT0, kG0 / (kIsaLapse * kR));
    t_isa = t11;
    p = p11 * std::exp(-kG0 * (h - kTropopause) / (kR * t11));
  }
  at.T = t_isa + delta_t;
  at.p = p;
  at.rho = p / (kR * at.T);
  at.a = std::sqrt(kGamma * kR * at.T);
  return at;
}

double pressure_altitude_m(double elevation_m, double qnh_hpa) {
  // Altitude in the standard atmosphere where pressure equals QNH, used as
  // the offset between elevation and pressure altitude.
  const double p = qnh_hpa * 100.0;
  const double h_qnh = kIsaT0 / kIsaLapse * (1.0 - std::pow(p / kIsaP0, kIsaLapse * kR / kG0));
  return elevation_m + h_qnh;
}

}  // namespace skybag
