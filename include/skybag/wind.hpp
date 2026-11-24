// Synthetic winds and temperatures aloft. A westerly jet stream in each
// hemisphere (strongest near FL340 around 40 degrees of latitude, wavy in
// longitude) plus smooth noise. Seeded and deterministic: the same seed is the
// same "forecast". Stands in for a real GRIB feed, which the desktop can't
// download.
#pragma once

#include <cstdint>

#include "skybag/geo.hpp"

namespace skybag {

struct Wind {
  double from_deg = 0;   // direction the wind blows from, true
  double speed_kt = 0;
  double isa_dev_c = 0;  // temperature deviation from ISA
  double u_kt = 0, v_kt = 0;  // eastward, northward components
};

class WindField {
 public:
  explicit WindField(std::uint32_t seed = 1, double strength = 1.0) : seed_(seed), strength_(strength) {}
  Wind at(LatLon p, double alt_ft) const;
  std::uint32_t seed() const { return seed_; }

 private:
  std::uint32_t seed_;
  double strength_;
};

// Wind triangle. Given true course, true airspeed and the wind, the heading
// to fly and the resulting ground speed (kt). ok = false when the crosswind
// is stronger than the airspeed.
struct WindTriangle {
  double heading_deg = 0, groundspeed_kt = 0, wca_deg = 0;
  bool ok = false;
};
WindTriangle wind_triangle(double course_deg, double tas_kt, const Wind& w);

// Headwind and crosswind components on a runway heading (positive headwind,
// positive crosswind from the right).
void runway_components(double runway_heading_deg, double wind_from_deg, double wind_kt, double* headwind,
                       double* crosswind);

}  // namespace skybag
