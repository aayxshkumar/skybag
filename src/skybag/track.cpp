#include "skybag/track.hpp"

#include <cmath>

#include "skybag/units.hpp"

namespace skybag {

namespace {

// xorshift64*: tiny, portable, and the same sequence everywhere (unlike the
// <random> distributions, which differ between standard libraries).
struct Rng {
  std::uint64_t s;
  explicit Rng(std::uint64_t seed) : s(seed ? seed : 0x9e3779b97f4a7c15ull) {}
  std::uint64_t next() {
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return s * 0x2545f4914f6cdd1dull;
  }
  double uniform() { return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0); }
};

}  // namespace

std::vector<Fix> simulate_track(const Plan& plan, const std::vector<Waypoint>& route, double interval_s,
                                std::uint64_t seed, double noise_m) {
  std::vector<Fix> out;
  if (!plan.ok || route.size() < 2 || interval_s <= 0) return out;
  Rng rng(seed);
  std::vector<double> leg_course(route.size() - 1);
  for (std::size_t k = 0; k + 1 < route.size(); ++k) leg_course[k] = inverse(route[k].pos, route[k + 1].pos).az1_deg;

  double t0 = 0;       // time at the start of the current piece, s
  double offset = 0;   // along-leg distance at the start of the current piece, m
  int leg = -1;
  double next_t = 0;
  for (const LegPlan& piece : plan.legs) {
    if (piece.leg != leg) {
      leg = piece.leg;
      offset = 0;
    }
    const double dur = piece.time_min * 60.0;
    const double gs = piece.gs_kt * kKt;
    while (next_t <= t0 + dur) {
      const double tau = next_t - t0;
      Fix f;
      f.t_s = next_t;
      double az2 = 0;
      const LatLon p = direct(route[static_cast<std::size_t>(leg)].pos, leg_course[static_cast<std::size_t>(leg)],
                              offset + gs * tau, &az2);
      // Noise: uniform direction, triangular radius up to 2 x noise_m.
      const double dir = rng.uniform() * 360.0, r = (rng.uniform() + rng.uniform()) * noise_m;
      f.pos = r > 0 ? direct(p, dir, r) : p;
      f.alt_ft = piece.alt_start_ft + (piece.alt_end_ft - piece.alt_start_ft) * (dur > 0 ? tau / dur : 0);
      f.gs_kt = piece.gs_kt;
      f.track_deg = az2;
      out.push_back(f);
      next_t += interval_s;
    }
    t0 += dur;
    offset += piece.dist_nm * kNm;
  }
  return out;
}

std::uint64_t track_hash(const std::vector<Fix>& fixes) {
  std::uint64_t h = 1469598103934665603ull;
  auto mix = [&](std::int64_t v) {
    for (int k = 0; k < 8; ++k) {
      h ^= static_cast<std::uint64_t>(v >> (8 * k)) & 0xff;
      h *= 1099511628211ull;
    }
  };
  for (const Fix& f : fixes) {
    mix(std::llround(f.t_s * 1000));
    mix(std::llround(f.pos.lat * 1e7));
    mix(std::llround(f.pos.lon * 1e7));
    mix(std::llround(f.alt_ft * 10));
  }
  return h;
}

}  // namespace skybag
