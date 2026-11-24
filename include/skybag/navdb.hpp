// Airport and runway database loaded from the OurAirports CSV dumps
// (public domain, data/ourairports/). Heliports, balloonports, seaplane bases
// and closed airports are skipped. Lookups by ident are exact; geographic
// queries go through a 1 degree grid index.
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

#include "skybag/geo.hpp"

namespace skybag {

// RFC 4180-style CSV: quoted fields, doubled quotes, newlines inside quotes.
std::vector<std::vector<std::string>> parse_csv(std::string_view text);

struct RunwayEnd {
  std::string ident;          // "28L"
  LatLon threshold;           // valid only if has_position
  bool has_position = false;
  double heading_true = 0;    // degrees; from the data, else from the geometry, else from the ident (magnetic!)
  double elevation_ft = 0;
  double displaced_ft = 0;
};

struct Runway {
  RunwayEnd le, he;           // low and high ends
  double length_ft = 0, width_ft = 0;
  std::string surface;
  bool lighted = false, closed = false;
  bool hard_surface() const;  // asphalt or concrete
};

struct Airport {
  std::string ident, type, name, country, municipality, iata;
  LatLon pos;
  double elevation_ft = 0;
  bool has_elevation = false;
  bool scheduled = false;
  std::vector<Runway> runways;
};

// Declared distances for taking off or landing on one runway end. OurAirports
// has no stopway or clearway data, so TODA = ASDA = TORA = runway length and
// LDA = length - displaced threshold.
struct RunwayDistances {
  double tora_m = 0, toda_m = 0, asda_m = 0, lda_m = 0;
  double heading_true = 0;
  double elevation_ft = 0;
  double slope_pct = 0;  // positive = uphill in the take-off direction
};

class NavDb {
 public:
  // dir holds airports.csv and runways.csv.
  static NavDb load(const std::string& dir, std::string* error = nullptr);

  const Airport* find(std::string_view ident) const;
  // Airports within radius_m of c (geodesic distance), nearest first.
  std::vector<const Airport*> within(LatLon c, double radius_m) const;
  const Airport* nearest(LatLon c, double max_m) const;
  // Case-insensitive prefix match on ident / IATA, then substring on name.
  std::vector<const Airport*> search(std::string_view q, std::size_t limit) const;

  const std::vector<Airport>& airports() const { return airports_; }
  std::size_t runway_count() const;

  // Find a runway end ("28L") at an airport. Returns false if unknown.
  static bool distances(const Airport& apt, std::string_view end, RunwayDistances* out, const Runway** rwy = nullptr);

 private:
  void build_index();
  static int cell_of(double lat, double lon);

  std::vector<Airport> airports_;
  std::unordered_map<std::string, std::size_t> by_ident_;
  std::unordered_map<int, std::vector<std::uint32_t>> cells_;
};

}  // namespace skybag
