#include "skybag/geo.hpp"

#include <algorithm>
#include <cmath>

#include "terra/ellipsoid.hpp"

namespace skybag {

namespace {
constexpr double kPi = 3.14159265358979323846;
double rad(double d) { return d * kPi / 180.0; }
double deg(double r) { return r * 180.0 / kPi; }
}  // namespace

double wrap180(double d) {
  d = std::fmod(d + 180.0, 360.0);
  if (d < 0) d += 360.0;
  return d - 180.0;
}

double wrap360(double d) {
  d = std::fmod(d, 360.0);
  if (d < 0) d += 360.0;
  return d;
}

Inverse inverse(LatLon a, LatLon b) {
  const terra::GeodesicResult g =
      terra::vincenty_inverse(terra::Ellipsoid::wgs84(), rad(a.lat), rad(a.lon), rad(b.lat), rad(b.lon));
  return {g.distance, wrap360(deg(g.az1)), wrap360(deg(g.az2)), g.converged};
}

// T. Vincenty (1975), direct problem.
LatLon direct(LatLon p, double az_deg, double s, double* az2_deg) {
  const terra::Ellipsoid e = terra::Ellipsoid::wgs84();
  const double a = e.a(), b = e.b(), f = e.f();
  const double alpha1 = rad(az_deg);
  const double sa1 = std::sin(alpha1), ca1 = std::cos(alpha1);
  const double tan_u1 = (1 - f) * std::tan(rad(p.lat));
  const double cu1 = 1 / std::sqrt(1 + tan_u1 * tan_u1), su1 = tan_u1 * cu1;
  const double sigma1 = std::atan2(tan_u1, ca1);
  const double sin_alpha = cu1 * sa1;
  const double cos2_alpha = 1 - sin_alpha * sin_alpha;
  const double u_sq = cos2_alpha * (a * a - b * b) / (b * b);
  const double A = 1 + u_sq / 16384 * (4096 + u_sq * (-768 + u_sq * (320 - 175 * u_sq)));
  const double B = u_sq / 1024 * (256 + u_sq * (-128 + u_sq * (74 - 47 * u_sq)));

  double sigma = s / (b * A), sigma_prev = 0, cos_2sm = 0, sin_sigma = 0, cos_sigma = 0;
  for (int it = 0; it < 200; ++it) {
    cos_2sm = std::cos(2 * sigma1 + sigma);
    sin_sigma = std::sin(sigma);
    cos_sigma = std::cos(sigma);
    const double d_sigma =
        B * sin_sigma *
        (cos_2sm + B / 4 *
                       (cos_sigma * (-1 + 2 * cos_2sm * cos_2sm) -
                        B / 6 * cos_2sm * (-3 + 4 * sin_sigma * sin_sigma) * (-3 + 4 * cos_2sm * cos_2sm)));
    sigma_prev = sigma;
    sigma = s / (b * A) + d_sigma;
    if (std::abs(sigma - sigma_prev) < 1e-12) break;
  }
  cos_2sm = std::cos(2 * sigma1 + sigma);
  sin_sigma = std::sin(sigma);
  cos_sigma = std::cos(sigma);

  const double x = su1 * sin_sigma - cu1 * cos_sigma * ca1;
  const double lat2 = std::atan2(su1 * cos_sigma + cu1 * sin_sigma * ca1, (1 - f) * std::hypot(sin_alpha, x));
  const double lambda = std::atan2(sin_sigma * sa1, cu1 * cos_sigma - su1 * sin_sigma * ca1);
  const double C = f / 16 * cos2_alpha * (4 + f * (4 - 3 * cos2_alpha));
  const double L =
      lambda - (1 - C) * f * sin_alpha *
                   (sigma + C * sin_sigma * (cos_2sm + C * cos_sigma * (-1 + 2 * cos_2sm * cos_2sm)));
  if (az2_deg) *az2_deg = wrap360(deg(std::atan2(sin_alpha, -x)));
  return {deg(lat2), wrap180(p.lon + deg(L))};
}

std::vector<LatLon> densify(LatLon a, LatLon b, double step_m) {
  const Inverse inv = inverse(a, b);
  const int n = std::max(1, static_cast<int>(std::ceil(inv.dist_m / std::max(step_m, 1.0))));
  std::vector<LatLon> pts;
  pts.reserve(static_cast<std::size_t>(n) + 1);
  pts.push_back(a);
  for (int k = 1; k < n; ++k) pts.push_back(direct(a, inv.az1_deg, inv.dist_m * k / n));
  pts.push_back(b);
  return pts;
}

}  // namespace skybag
