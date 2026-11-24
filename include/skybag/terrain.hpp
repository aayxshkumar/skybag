// Terrain model and route terrain clearance.
//
// The terrain is synthetic: terra's procedural planet (land only, the sea
// floor is clamped to 0 m), flattened to field elevation within 4 km of every
// airport and blended back to the procedural height by 20 km. That keeps
// departures and arrivals sane on real airport data without shipping a DEM.
// A real elevation model is on the roadmap; everything below works on any
// height function.
#pragma once

#include <cstdint>
#include <vector>

#include "skybag/geo.hpp"
#include "skybag/navdb.hpp"

namespace skybag {

class Terrain {
 public:
  Terrain(const NavDb* db, std::uint32_t seed = 7);
  // Height above mean sea level, metres.
  double height_m(LatLon p) const;
  // Procedural height alone (no airport flattening), for tests.
  double raw_height_m(LatLon p) const;

 private:
  const NavDb* db_;
  std::uint32_t seed_;
};

struct Clearance {
  double max_terrain_m = 0;  // highest terrain found inside the corridor
  LatLon where;              // where it is
  int leg = -1;              // which leg (0-based), -1 if none
  double msa_ft = 0;         // minimum safe altitude: terrain + 1000 ft (+2000 ft above 5000 ft terrain), rounded up to 100
};

double msa_ft(double terrain_m);

// The clearance the planner uses. It looks at the terrain under each route
// waypoint only, so it misses a ridge between two waypoints and ignores the
// corridor width. KNOWN GAP (see --check terrain-corridor).
Clearance route_clearance(const Terrain& t, const std::vector<LatLon>& route, double half_width_m);

// Reference: dense sampling along each geodesic leg and across the corridor.
// Slow, used by the checks and the tests.
Clearance brute_force_clearance(const Terrain& t, const std::vector<LatLon>& route, double half_width_m,
                                double along_step_m, int across_samples);

}  // namespace skybag
