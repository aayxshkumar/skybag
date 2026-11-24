#include "skybag/navdb.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace skybag {

std::vector<std::vector<std::string>> parse_csv(std::string_view text) {
  std::vector<std::vector<std::string>> rows;
  std::vector<std::string> row;
  std::string field;
  bool quoted = false, field_started = false;
  for (std::size_t i = 0; i < text.size(); ++i) {
    const char c = text[i];
    if (quoted) {
      if (c == '"') {
        if (i + 1 < text.size() && text[i + 1] == '"') {
          field.push_back('"');
          ++i;
        } else {
          quoted = false;
        }
      } else {
        field.push_back(c);
      }
      continue;
    }
    if (c == '"' && !field_started) {
      quoted = true;
      field_started = true;
    } else if (c == ',') {
      row.push_back(std::move(field));
      field.clear();
      field_started = false;
    } else if (c == '\n' || c == '\r') {
      if (c == '\r' && i + 1 < text.size() && text[i + 1] == '\n') ++i;
      row.push_back(std::move(field));
      field.clear();
      field_started = false;
      rows.push_back(std::move(row));
      row.clear();
    } else {
      field.push_back(c);
      field_started = true;
    }
  }
  if (field_started || !row.empty()) {
    row.push_back(std::move(field));
    rows.push_back(std::move(row));
  }
  return rows;
}

namespace {

bool read_file(const std::string& path, std::string* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

struct Columns {
  std::unordered_map<std::string, std::size_t> idx;
  explicit Columns(const std::vector<std::string>& header) {
    for (std::size_t k = 0; k < header.size(); ++k) idx[header[k]] = k;
  }
  const std::string& get(const std::vector<std::string>& row, const char* name) const {
    static const std::string empty;
    const auto it = idx.find(name);
    if (it == idx.end() || it->second >= row.size()) return empty;
    return row[it->second];
  }
  bool has(const char* name) const { return idx.count(name) != 0; }
};

bool to_double(const std::string& s, double* out) {
  if (s.empty()) return false;
  char* end = nullptr;
  const double v = std::strtod(s.c_str(), &end);
  if (end == s.c_str()) return false;
  *out = v;
  return true;
}

std::string upper(std::string_view s) {
  std::string r(s);
  for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return r;
}

bool skipped_type(const std::string& t) {
  return t == "heliport" || t == "balloonport" || t == "seaplane_base" || t == "closed";
}

// Heading implied by a runway designator: "28L" -> 280 (magnetic, so only a
// fallback when the data has neither a true heading nor end coordinates).
bool ident_heading(const std::string& ident, double* out) {
  int n = 0, digits = 0;
  for (char c : ident) {
    if (std::isdigit(static_cast<unsigned char>(c)) && digits < 2) {
      n = n * 10 + (c - '0');
      ++digits;
    } else {
      break;
    }
  }
  if (digits == 0 || n < 1 || n > 36) return false;
  *out = n * 10.0;
  return true;
}

constexpr double kPi = 3.14159265358979323846;

}  // namespace

bool Runway::hard_surface() const {
  const std::string s = upper(surface);
  return s.find("ASP") != std::string::npos || s.find("CON") != std::string::npos || s.find("PEM") != std::string::npos;
}

NavDb NavDb::load(const std::string& dir, std::string* error) {
  NavDb db;
  std::string text;
  if (!read_file(dir + "/airports.csv", &text)) {
    if (error) *error = "cannot read " + dir + "/airports.csv";
    return db;
  }
  {
    const auto rows = parse_csv(text);
    if (rows.empty()) {
      if (error) *error = "airports.csv is empty";
      return db;
    }
    const Columns col(rows[0]);
    db.airports_.reserve(rows.size());
    for (std::size_t r = 1; r < rows.size(); ++r) {
      const auto& row = rows[r];
      const std::string& type = col.get(row, "type");
      if (skipped_type(type)) continue;
      Airport a;
      a.ident = col.get(row, "ident");
      a.type = type;
      a.name = col.get(row, "name");
      a.country = col.get(row, "iso_country");
      a.municipality = col.get(row, "municipality");
      a.iata = col.get(row, "iata_code");
      a.scheduled = col.get(row, "scheduled_service") == "yes";
      if (!to_double(col.get(row, "latitude_deg"), &a.pos.lat) || !to_double(col.get(row, "longitude_deg"), &a.pos.lon))
        continue;
      if (std::abs(a.pos.lat) > 90 || std::abs(a.pos.lon) > 180) continue;
      a.pos.lon = wrap180(a.pos.lon);
      a.has_elevation = to_double(col.get(row, "elevation_ft"), &a.elevation_ft);
      if (a.ident.empty()) continue;
      db.by_ident_[a.ident] = db.airports_.size();
      db.airports_.push_back(std::move(a));
    }
  }
  if (!read_file(dir + "/runways.csv", &text)) {
    if (error) *error = "cannot read " + dir + "/runways.csv";
    return db;
  }
  {
    const auto rows = parse_csv(text);
    if (rows.empty()) return db;
    const Columns col(rows[0]);
    for (std::size_t r = 1; r < rows.size(); ++r) {
      const auto& row = rows[r];
      const auto it = db.by_ident_.find(col.get(row, "airport_ident"));
      if (it == db.by_ident_.end()) continue;
      Airport& apt = db.airports_[it->second];
      Runway rw;
      to_double(col.get(row, "length_ft"), &rw.length_ft);
      to_double(col.get(row, "width_ft"), &rw.width_ft);
      rw.surface = col.get(row, "surface");
      rw.lighted = col.get(row, "lighted") == "1";
      rw.closed = col.get(row, "closed") == "1";
      auto fill = [&](RunwayEnd& e, const char* p) {
        const std::string pre(p);
        e.ident = col.get(row, (pre + "_ident").c_str());
        double lat = 0, lon = 0;
        e.has_position = to_double(col.get(row, (pre + "_latitude_deg").c_str()), &lat) &&
                         to_double(col.get(row, (pre + "_longitude_deg").c_str()), &lon);
        e.threshold = {lat, wrap180(lon)};
        if (!to_double(col.get(row, (pre + "_elevation_ft").c_str()), &e.elevation_ft)) e.elevation_ft = apt.elevation_ft;
        to_double(col.get(row, (pre + "_displaced_threshold_ft").c_str()), &e.displaced_ft);
        double h = 0;
        if (to_double(col.get(row, (pre + "_heading_degT").c_str()), &h)) e.heading_true = wrap360(h);
        else e.heading_true = -1;
      };
      fill(rw.le, "le");
      fill(rw.he, "he");
      if (rw.le.ident.empty() && rw.he.ident.empty()) continue;
      if (rw.le.has_position && rw.he.has_position) {
        const Inverse g = inverse(rw.le.threshold, rw.he.threshold);
        if (rw.le.heading_true < 0) rw.le.heading_true = g.az1_deg;
        if (rw.he.heading_true < 0) rw.he.heading_true = wrap360(g.az2_deg + 180.0);
      }
      double h = 0;
      if (rw.le.heading_true < 0) rw.le.heading_true = ident_heading(rw.le.ident, &h) ? h : 0;
      if (rw.he.heading_true < 0)
        rw.he.heading_true = ident_heading(rw.he.ident, &h) ? h : wrap360(rw.le.heading_true + 180.0);
      apt.runways.push_back(std::move(rw));
    }
  }
  db.build_index();
  return db;
}

int NavDb::cell_of(double lat, double lon) {
  const int i = std::clamp(static_cast<int>(std::floor(lat)) + 90, 0, 179);
  const int j = ((static_cast<int>(std::floor(lon)) + 180) % 360 + 360) % 360;
  return i * 360 + j;
}

void NavDb::build_index() {
  cells_.clear();
  for (std::size_t k = 0; k < airports_.size(); ++k)
    cells_[cell_of(airports_[k].pos.lat, airports_[k].pos.lon)].push_back(static_cast<std::uint32_t>(k));
}

const Airport* NavDb::find(std::string_view ident) const {
  const auto it = by_ident_.find(upper(ident));
  return it == by_ident_.end() ? nullptr : &airports_[it->second];
}

std::vector<const Airport*> NavDb::within(LatLon c, double radius_m) const {
  // 1 degree of latitude is at least 110 574 m anywhere on WGS84.
  const double dlat = radius_m / 110574.0 + 0.01;
  const double lat_lo = std::max(-90.0, c.lat - dlat), lat_hi = std::min(90.0, c.lat + dlat);
  const double phi_max = std::max(std::abs(lat_lo), std::abs(lat_hi));
  double dlon = 180.0;
  if (phi_max < 89.5) {
    const double delta = radius_m / 6356752.0;  // angle on the smallest sphere that fits
    const double s = std::sin(std::min(delta, kPi / 2)) / std::cos(phi_max * kPi / 180.0);
    if (delta < kPi / 2 && s < 1) dlon = std::asin(s) * 180.0 / kPi * 1.01 + 0.01;
  }
  std::vector<std::pair<double, const Airport*>> hits;
  const int i_lo = std::clamp(static_cast<int>(std::floor(lat_lo)) + 90, 0, 179);
  const int i_hi = std::clamp(static_cast<int>(std::floor(lat_hi)) + 90, 0, 179);
  const int j_lo = static_cast<int>(std::floor(c.lon - dlon)), j_hi = static_cast<int>(std::floor(c.lon + dlon));
  const int span = std::min(j_hi - j_lo, 359);
  for (int i = i_lo; i <= i_hi; ++i) {
    for (int dj = 0; dj <= span; ++dj) {
      const int j = (((j_lo + dj) + 180) % 360 + 360) % 360;
      const auto it = cells_.find(i * 360 + j);
      if (it == cells_.end()) continue;
      for (std::uint32_t k : it->second) {
        const LatLon& q = airports_[k].pos;
        if (std::abs(q.lat - c.lat) > dlat || std::abs(wrap180(q.lon - c.lon)) > dlon) continue;  // cheap reject
        const Inverse g = inverse(c, q);
        if (g.dist_m <= radius_m) hits.push_back({g.dist_m, &airports_[k]});
      }
    }
  }
  std::sort(hits.begin(), hits.end(), [](const auto& a, const auto& b) {
    return a.first != b.first ? a.first < b.first : a.second->ident < b.second->ident;
  });
  std::vector<const Airport*> out;
  out.reserve(hits.size());
  for (const auto& h : hits) out.push_back(h.second);
  return out;
}

const Airport* NavDb::nearest(LatLon c, double max_m) const {
  const auto v = within(c, max_m);
  return v.empty() ? nullptr : v.front();
}

std::vector<const Airport*> NavDb::search(std::string_view q, std::size_t limit) const {
  const std::string uq = upper(q);
  std::vector<const Airport*> exact, prefix, name;
  if (uq.empty()) return {};
  for (const Airport& a : airports_) {
    if (a.ident == uq || a.iata == uq) exact.push_back(&a);
    else if (a.ident.rfind(uq, 0) == 0) prefix.push_back(&a);
    else if (uq.size() >= 3 && upper(a.name).find(uq) != std::string::npos) name.push_back(&a);
  }
  auto by_size = [](const Airport* x, const Airport* y) {
    auto rank = [](const Airport* a) {
      return a->type == "large_airport" ? 0 : a->type == "medium_airport" ? 1 : 2;
    };
    return rank(x) != rank(y) ? rank(x) < rank(y) : x->ident < y->ident;
  };
  std::sort(prefix.begin(), prefix.end(), by_size);
  std::sort(name.begin(), name.end(), by_size);
  std::vector<const Airport*> out = exact;
  for (auto* v : {&prefix, &name})
    for (const Airport* a : *v) {
      if (out.size() >= limit) return out;
      out.push_back(a);
    }
  if (out.size() > limit) out.resize(limit);
  return out;
}

std::size_t NavDb::runway_count() const {
  std::size_t n = 0;
  for (const Airport& a : airports_) n += a.runways.size();
  return n;
}

bool NavDb::distances(const Airport& apt, std::string_view end, RunwayDistances* out, const Runway** rwy) {
  const std::string want = upper(end);
  for (const Runway& r : apt.runways) {
    const RunwayEnd* e = nullptr;
    const RunwayEnd* other = nullptr;
    if (upper(r.le.ident) == want) e = &r.le, other = &r.he;
    else if (upper(r.he.ident) == want) e = &r.he, other = &r.le;
    if (!e) continue;
    const double len_m = r.length_ft * 0.3048;
    out->tora_m = out->toda_m = out->asda_m = len_m;
    out->lda_m = std::max(0.0, len_m - e->displaced_ft * 0.3048);
    out->heading_true = e->heading_true;
    out->elevation_ft = e->elevation_ft;
    out->slope_pct = len_m > 0 ? (other->elevation_ft - e->elevation_ft) * 0.3048 / len_m * 100.0 : 0.0;
    if (rwy) *rwy = &r;
    return true;
  }
  return false;
}

}  // namespace skybag
