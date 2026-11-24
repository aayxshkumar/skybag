#include "skybag/terrain.hpp"

#include <algorithm>
#include <cmath>

#include "skybag/units.hpp"
#include "terra/terrain.hpp"

namespace skybag {

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kFlatInner = 4000.0, kFlatOuter = 20000.0;

terra::PlanetParams params(std::uint32_t seed) {
  terra::PlanetParams p;
  p.seed = seed;
  p.octaves = 12;  // ~2 km finest wavelength is plenty for route clearance
  return p;
}
}  // namespace

Terrain::Terrain(const NavDb* db, std::uint32_t seed) : db_(db), seed_(seed) {}

double Terrain::raw_height_m(LatLon p) const {
  return std::max(0.0, terra::planet_height(params(seed_), p.lat * kPi / 180.0, p.lon * kPi / 180.0));
}

double Terrain::height_m(LatLon p) const {
  const double raw = raw_height_m(p);
  if (!db_) return raw;
  // Nearest airport inside the blend radius decides the flattening.
  const Airport* a = db_->nearest(p, kFlatOuter);
  if (!a) return raw;
  const double d = inverse(p, a->pos).dist_m;
  const double field = (a->has_elevation ? a->elevation_ft : 0.0) * kFt;
  if (d <= kFlatInner) return field;
  const double x = (d - kFlatInner) / (kFlatOuter - kFlatInner);
  const double w = x * x * (3 - 2 * x);  // smoothstep
  return field + (raw - field) * w;
}

double msa_ft(double terrain_m) {
  const double ft = terrain_m / kFt;
  const double margin = ft > 5000 ? 2000.0 : 1000.0;
  return std::ceil((ft + margin) / 100.0) * 100.0;
}

Clearance route_clearance(const Terrain& t, const std::vector<LatLon>& route, double /*half_width_m*/) {
  Clearance c;
  c.max_terrain_m = -1;
  for (std::size_t k = 0; k < route.size(); ++k) {
    const double h = t.height_m(route[k]);
    if (h > c.max_terrain_m) {
      c.max_terrain_m = h;
      c.where = route[k];
      c.leg = static_cast<int>(k == 0 ? 0 : k - 1);
    }
  }
  c.max_terrain_m = std::max(0.0, c.max_terrain_m);
  c.msa_ft = msa_ft(c.max_terrain_m);
  return c;
}

Clearance brute_force_clearance(const Terrain& t, const std::vector<LatLon>& route, double half_width_m,
                                double along_step_m, int across_samples) {
  Clearance c;
  c.max_terrain_m = -1;
  for (std::size_t leg = 0; leg + 1 < route.size(); ++leg) {
    const std::vector<LatLon> pts = densify(route[leg], route[leg + 1], along_step_m);
    for (std::size_t k = 0; k < pts.size(); ++k) {
      // Course at this point, perpendicular offsets on both sides.
      const LatLon& p = pts[k];
      const LatLon& q = k + 1 < pts.size() ? pts[k + 1] : pts[k - 1];
      double crs = inverse(p, q).az1_deg;
      if (k + 1 >= pts.size()) crs = wrap360(crs + 180.0);
      for (int j = -across_samples; j <= across_samples; ++j) {
        const double off = half_width_m * j / std::max(1, across_samples);
        const LatLon s = off == 0 ? p : direct(p, wrap360(crs + (off > 0 ? 90.0 : -90.0)), std::abs(off));
        const double h = t.height_m(s);
        if (h > c.max_terrain_m) {
          c.max_terrain_m = h;
          c.where = s;
          c.leg = static_cast<int>(leg);
        }
      }
    }
  }
  c.max_terrain_m = std::max(0.0, c.max_terrain_m);
  c.msa_ft = msa_ft(c.max_terrain_m);
  return c;
}

}  // namespace skybag
