// The local JSON API behind the web UI, and the static file server for web/.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "skybag/http.hpp"
#include "skybag/navdb.hpp"
#include "skybag/notam.hpp"
#include "skybag/planner.hpp"
#include "skybag/terrain.hpp"
#include "skybag/wind.hpp"

namespace skybag {

struct App {
  NavDb db;
  std::unique_ptr<Terrain> terrain;
  std::vector<Notam> notams;
  std::string web_dir;
  std::int64_t default_time_min = 0;  // "now" for NOTAM checks when the UI doesn't send a time
};

// Loads data_dir/ourairports, data_dir/notams/*.txt. Empty string on success.
std::string load_app(App* app, const std::string& data_dir, const std::string& web_dir);

// "KSFO KLAX", "KSFO,40.5/-120.25,KLAX": idents or lat/lon pairs.
bool parse_route(const NavDb& db, const std::string& text, std::vector<Waypoint>* out, std::string* error);

Response handle(const App& app, const Request& req);

}  // namespace skybag
