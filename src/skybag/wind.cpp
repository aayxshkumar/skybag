#include "skybag/wind.hpp"

#include <cmath>

#include "terra/terrain.hpp"
#include "terra/vec.hpp"

namespace skybag {

namespace {
constexpr double kPi = 3.14159265358979323846;
double rad(double d) { return d * kPi / 180.0; }
}  // namespace

Wind WindField::at(LatLon p, double alt_ft) const {
  const double lat = rad(p.lat), lon = rad(p.lon);
  const terra::dvec3 unit{std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
  // Jet core latitude wanders +-8 degrees with longitude.
  const double core = 40.0 + 8.0 * terra::value_noise(unit * 1.3, seed_ ^ 0x9e37u);
  const double alat = std::abs(p.lat);
  const double band = std::exp(-((alat - core) * (alat - core)) / (2 * 9.0 * 9.0));
  const double fl = alt_ft / 100.0;
  const double vertical = std::exp(-((fl - 340.0) * (fl - 340.0)) / (2 * 110.0 * 110.0));
  const double jet = (25.0 + 115.0 * band) * vertical * strength_;
  const double nu = 18.0 * terra::value_noise(unit * 2.1 + terra::dvec3{0, 0, fl / 400.0}, seed_ ^ 0x51edu);
  const double nv = 18.0 * terra::value_noise(unit * 2.1 + terra::dvec3{0, fl / 400.0, 0}, seed_ ^ 0x2c1bu);
  Wind w;
  w.u_kt = jet + nu * strength_;
  w.v_kt = nv * strength_ + 20.0 * strength_ * terra::value_noise(unit * 0.9, seed_ ^ 0x77u) * band;
  w.speed_kt = std::hypot(w.u_kt, w.v_kt);
  // "From" direction: opposite of the vector the air moves along.
  w.from_deg = wrap360(std::atan2(-w.u_kt, -w.v_kt) * 180.0 / kPi);
  w.isa_dev_c = 6.0 * terra::value_noise(unit * 1.7 + terra::dvec3{fl / 300.0, 0, 0}, seed_ ^ 0x1234u);
  return w;
}

WindTriangle wind_triangle(double course_deg, double tas_kt, const Wind& w) {
  WindTriangle r;
  // Wind vector the air moves along (to-direction), in course-aligned axes.
  const double crs = rad(course_deg);
  const double along = w.u_kt * std::sin(crs) + w.v_kt * std::cos(crs);   // tailwind +
  const double cross = w.u_kt * std::cos(crs) - w.v_kt * std::sin(crs);   // pushes to the right of course +
  if (std::abs(cross) >= tas_kt) return r;
  const double wca = std::asin(-cross / tas_kt);  // crab into the wind
  r.wca_deg = wca * 180.0 / kPi;
  r.heading_deg = wrap360(course_deg + r.wca_deg);
  r.groundspeed_kt = tas_kt * std::cos(wca) + along;
  r.ok = r.groundspeed_kt > 0;
  return r;
}

void runway_components(double rwy_hdg, double wind_from, double wind_kt, double* headwind, double* crosswind) {
  const double d = rad(wind_from - rwy_hdg);
  if (headwind) *headwind = wind_kt * std::cos(d);
  if (crosswind) *crosswind = wind_kt * std::sin(d);
}

}  // namespace skybag
