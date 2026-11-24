// Geodesy for flight planning on WGS84, in degrees and metres. The inverse
// problem is terra's Vincenty solver; the direct problem is Vincenty's direct
// formula. Both are Vincenty (1975), so both inherit its known weakness:
// the inverse does not converge for nearly antipodal points.
#pragma once

#include <vector>

namespace skybag {

struct LatLon {
  double lat = 0, lon = 0;  // degrees, lon in [-180, 180)
};

struct Inverse {
  double dist_m = 0;
  double az1_deg = 0;  // initial true course, [0, 360)
  double az2_deg = 0;  // final true course at the second point
  bool ok = false;     // false when Vincenty failed to converge
};

double wrap180(double deg);
double wrap360(double deg);

Inverse inverse(LatLon a, LatLon b);
// Point reached from a after dist_m along the geodesic with initial course
// az_deg. *az2_deg (if given) receives the course at the end point.
LatLon direct(LatLon a, double az_deg, double dist_m, double* az2_deg = nullptr);

// Points along the geodesic a -> b, about step_m apart, both ends included.
std::vector<LatLon> densify(LatLon a, LatLon b, double step_m);

}  // namespace skybag
