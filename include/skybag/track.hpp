// Simulated ownship for the moving map: fixes flown along a plan at the
// plan's ground speeds, with seeded GPS-like position noise. Same plan + same
// seed = same track, bit for bit (the replay hash).
#pragma once

#include <cstdint>
#include <vector>

#include "skybag/planner.hpp"

namespace skybag {

struct Fix {
  double t_s = 0;
  LatLon pos;
  double alt_ft = 0, gs_kt = 0, track_deg = 0;
};

std::vector<Fix> simulate_track(const Plan& plan, const std::vector<Waypoint>& route, double interval_s,
                                std::uint64_t seed, double noise_m = 5.0);

std::uint64_t track_hash(const std::vector<Fix>& fixes);

}  // namespace skybag
