#include "skybag/api.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>

#include "skybag/json.hpp"
#include "skybag/perf.hpp"
#include "skybag/track.hpp"
#include "skybag/units.hpp"
#include "skybag/wb.hpp"
#include "terra/camera.hpp"
#include "terra/render.hpp"

namespace skybag {

namespace {

constexpr double kPi = 3.14159265358979323846;

Response json(const JsonWriter& w) { return {200, "application/json", w.str()}; }

Response error(int status, const std::string& msg) {
  JsonWriter w;
  w.begin_object().kv("error", msg).end_object();
  return {status, "application/json", w.str()};
}

bool read_file(const std::string& path, std::string* out) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::ostringstream ss;
  ss << f.rdbuf();
  *out = ss.str();
  return true;
}

std::string content_type(const std::string& path) {
  auto ends = [&](const char* ext) {
    const std::string e(ext);
    return path.size() >= e.size() && path.compare(path.size() - e.size(), e.size(), e) == 0;
  };
  if (ends(".html")) return "text/html; charset=utf-8";
  if (ends(".js")) return "text/javascript; charset=utf-8";
  if (ends(".css")) return "text/css; charset=utf-8";
  if (ends(".svg")) return "image/svg+xml";
  if (ends(".png")) return "image/png";
  return "application/octet-stream";
}

std::int64_t request_time(const App& app, const Request& req) {
  const std::string t = req.get("time");
  if (t.empty()) return app.default_time_min;
  const std::int64_t v = parse_notam_time(t);
  return v < 0 ? app.default_time_min : v;
}

void write_airport(JsonWriter& w, const Airport& a, bool runways) {
  w.begin_object()
      .kv("ident", a.ident)
      .kv("name", a.name)
      .kv("type", a.type)
      .kv("iata", a.iata)
      .kv("country", a.country)
      .kv("municipality", a.municipality)
      .kv("lat", a.pos.lat)
      .kv("lon", a.pos.lon)
      .kv("elevation_ft", a.elevation_ft);
  if (runways) {
    w.key("runways").begin_array();
    for (const Runway& r : a.runways) {
      w.begin_object()
          .kv("length_ft", r.length_ft)
          .kv("width_ft", r.width_ft)
          .kv("surface", r.surface)
          .kv("closed", r.closed);
      for (const RunwayEnd* e : {&r.le, &r.he}) {
        w.key(e == &r.le ? "le" : "he").begin_object().kv("ident", e->ident).kv("heading_true", e->heading_true);
        w.kv("elevation_ft", e->elevation_ft).kv("displaced_ft", e->displaced_ft).kv("has_position", e->has_position);
        if (e->has_position) w.kv("lat", e->threshold.lat).kv("lon", e->threshold.lon);
        w.end_object();
      }
      w.end_object();
    }
    w.end_array();
  }
  w.end_object();
}

PlanInput plan_input(const Request& req, std::vector<Waypoint> route) {
  PlanInput in;
  in.route = std::move(route);
  in.cruise_ft = req.num("fl", 350) * 100;
  in.mach = req.num("mach", 0.78);
  in.zfw_kg = req.num("zfw", 58000);
  in.alternate_kg = req.num("alt_fuel", 0);
  in.contingency_pct = req.num("cont", 5);
  return in;
}

WindField wind_for(const Request& req) {
  return WindField(static_cast<std::uint32_t>(req.num("wind_seed", 1)), req.num("wind_scale", 1.0));
}

void write_plan(JsonWriter& w, const Plan& p) {
  w.begin_object()
      .kv("ok", p.ok)
      .kv("error", p.error)
      .kv("cruise_ft", p.cruise_ft)
      .kv("dist_nm", p.dist_nm)
      .kv("time_min", p.time_min)
      .kv("toc_nm", p.toc_nm)
      .kv("tod_nm", p.tod_nm)
      .kv("trip_kg", p.trip_kg)
      .kv("contingency_kg", p.contingency_kg)
      .kv("reserve_kg", p.reserve_kg)
      .kv("alternate_kg", p.alternate_kg)
      .kv("taxi_kg", p.taxi_kg)
      .kv("block_kg", p.block_kg)
      .kv("tow_kg", p.tow_kg)
      .kv("ldw_kg", p.ldw_kg);
  w.key("warnings").begin_array();
  for (const auto& s : p.warnings) w.value(s);
  w.end_array();
  w.key("legs").begin_array();
  for (const LegPlan& l : p.legs) {
    w.begin_object()
        .kv("from", l.from)
        .kv("to", l.to)
        .kv("leg", l.leg)
        .kv("phase", l.phase)
        .kv("dist_nm", l.dist_nm)
        .kv("course", l.course_deg)
        .kv("heading", l.heading_deg)
        .kv("wind_from", l.wind_from_deg)
        .kv("wind_kt", l.wind_kt)
        .kv("tas_kt", l.tas_kt)
        .kv("gs_kt", l.gs_kt)
        .kv("alt_start_ft", l.alt_start_ft)
        .kv("alt_end_ft", l.alt_end_ft)
        .kv("time_min", l.time_min)
        .kv("fuel_kg", l.fuel_kg)
        .end_object();
  }
  w.end_array();
  w.end_object();
}

Response api_plan(const App& app, const Request& req) {
  std::vector<Waypoint> route;
  std::string err;
  if (!parse_route(app.db, req.get("route"), &route, &err)) return error(400, err);
  const PlanInput in = plan_input(req, route);
  const Plan p = plan(sb20(), wind_for(req), in);
  JsonWriter w;
  w.begin_object().key("plan");
  write_plan(w, p);
  // Route geometry for the map: the geodesic, about every 20 nm.
  w.key("route").begin_array();
  for (const Waypoint& wp : route) w.begin_object().kv("name", wp.name).kv("lat", wp.pos.lat).kv("lon", wp.pos.lon).end_object();
  w.end_array();
  w.key("path").begin_array();
  std::vector<LatLon> path;
  for (std::size_t k = 0; k + 1 < route.size(); ++k) {
    auto seg = densify(route[k].pos, route[k + 1].pos, 20 * kNm);
    if (!path.empty()) seg.erase(seg.begin());
    path.insert(path.end(), seg.begin(), seg.end());
  }
  for (const LatLon& q : path) w.begin_array().value(q.lat).value(q.lon).end_array();
  w.end_array();
  // Terrain clearance as the planner computes it (waypoints only).
  const double half_width = req.num("corridor_nm", 10) * kNm;
  std::vector<LatLon> pts;
  for (const Waypoint& wp : route) pts.push_back(wp.pos);
  const Clearance c = route_clearance(*app.terrain, pts, half_width);
  w.key("clearance").begin_object().kv("max_terrain_ft", c.max_terrain_m / kFt).kv("msa_ft", c.msa_ft).kv("leg", c.leg);
  w.kv("lat", c.where.lat).kv("lon", c.where.lon).kv("method", "waypoints only").end_object();
  // Vertical profile: terrain under the track and the planned altitude.
  w.key("profile").begin_array();
  if (p.ok) {
    const int n = 240;
    double x0 = 0;
    std::size_t leg = 0;
    std::vector<double> leg_len;
    for (std::size_t k = 0; k + 1 < route.size(); ++k) leg_len.push_back(inverse(route[k].pos, route[k + 1].pos).dist_m / kNm);
    for (int i = 0; i <= n; ++i) {
      const double x = p.dist_nm * i / n;
      while (leg + 1 < leg_len.size() && x > x0 + leg_len[leg]) x0 += leg_len[leg++];
      const Inverse g = inverse(route[leg].pos, route[leg + 1].pos);
      const LatLon q = direct(route[leg].pos, g.az1_deg, std::min(x - x0, leg_len[leg]) * kNm);
      double alt = p.cruise_ft;
      if (x < p.toc_nm) alt = p.cruise_ft * x / std::max(1e-9, p.toc_nm);
      if (x > p.tod_nm) alt = p.cruise_ft * (p.dist_nm - x) / std::max(1e-9, p.dist_nm - p.tod_nm);
      w.begin_array().value(x).value(app.terrain->height_m(q) / kFt).value(alt).end_array();
    }
  }
  w.end_array();
  w.end_object();
  return json(w);
}

Response api_track(const App& app, const Request& req) {
  std::vector<Waypoint> route;
  std::string err;
  if (!parse_route(app.db, req.get("route"), &route, &err)) return error(400, err);
  const Plan p = plan(sb20(), wind_for(req), plan_input(req, route));
  if (!p.ok) return error(400, p.error);
  double interval = req.num("interval", 10);
  interval = std::max(interval, p.time_min * 60.0 / 4000.0);  // at most ~4000 fixes
  const auto fixes = simulate_track(p, route, interval, static_cast<std::uint64_t>(req.num("seed", 1)));
  JsonWriter w;
  w.begin_object().kv("interval_s", interval).kv("hash", track_hash(fixes)).key("fixes").begin_array();
  for (const Fix& f : fixes)
    w.begin_array().value(f.t_s).value(f.pos.lat).value(f.pos.lon).value(f.alt_ft).value(f.gs_kt).value(f.track_deg).end_array();
  w.end_array().end_object();
  return json(w);
}

bool runway_conditions(const App& app, const Request& req, const Airport** apt_out, RunwayDistances* d,
                       RunwayConditions* rw, std::string* err, double* crosswind) {
  const Airport* apt = app.db.find(req.get("apt"));
  if (!apt) return *err = "unknown airport " + req.get("apt"), false;
  if (!NavDb::distances(*apt, req.get("rwy"), d)) return *err = "unknown runway " + req.get("rwy"), false;
  const Notam* by = nullptr;
  if (runway_closed(app.notams, apt->ident, req.get("rwy"), request_time(app, req), &by))
    return *err = "runway " + req.get("rwy") + " closed by NOTAM " + by->id + ": " + by->text, false;
  rw->elevation_ft = d->elevation_ft;
  rw->qnh_hpa = req.num("qnh", 1013.25);
  rw->oat_c = req.num("oat", 15);
  rw->slope_pct = d->slope_pct;
  rw->wet = req.get("wet") == "1" || req.get("wet") == "true";
  double head = 0, cross = 0;
  runway_components(d->heading_true, req.num("wind_dir", 0), req.num("wind_kt", 0), &head, &cross);
  // Regulations credit only half the headwind and require 150% of tailwind.
  rw->headwind_kt = head >= 0 ? 0.5 * head : 1.5 * head;
  *crosswind = cross;
  *apt_out = apt;
  return true;
}

Response api_takeoff(const App& app, const Request& req) {
  const Airport* apt = nullptr;
  RunwayDistances d;
  RunwayConditions rw;
  std::string err;
  double cross = 0;
  if (!runway_conditions(app, req, &apt, &d, &rw, &err, &cross)) return error(400, err);
  TakeoffInput in;
  in.mass_kg = req.num("mass", 70000);
  in.rw = rw;
  in.tora_m = d.tora_m;
  in.toda_m = d.toda_m;
  in.asda_m = d.asda_m;
  const TakeoffResult r = takeoff(sb20(), in);
  JsonWriter w;
  w.begin_object()
      .kv("airport", apt->ident)
      .kv("runway", req.get("rwy"))
      .kv("tora_m", d.tora_m)
      .kv("slope_pct", d.slope_pct)
      .kv("heading_true", d.heading_true)
      .kv("factored_headwind_kt", rw.headwind_kt)
      .kv("crosswind_kt", cross)
      .kv("v1_kt", r.v1_kt)
      .kv("vr_kt", r.speeds.vr_kt)
      .kv("v2_kt", r.speeds.v2_kt)
      .kv("vs_kt", r.speeds.vs_kt)
      .kv("tod_all_engines_m", r.tod_all_engines_m)
      .kv("tod_oei_m", r.tod_oei_m)
      .kv("asd_m", r.asd_m)
      .kv("required_m", r.required_m)
      .kv("balanced", r.balanced)
      .kv("ok", r.ok)
      .kv("limit", r.limit)
      .end_object();
  return json(w);
}

Response api_landing(const App& app, const Request& req) {
  const Airport* apt = nullptr;
  RunwayDistances d;
  RunwayConditions rw;
  std::string err;
  double cross = 0;
  if (!runway_conditions(app, req, &apt, &d, &rw, &err, &cross)) return error(400, err);
  LandingInput in;
  in.mass_kg = req.num("mass", 60000);
  in.rw = rw;
  in.lda_m = d.lda_m;
  const LandingResult r = landing(sb20(), in);
  JsonWriter w;
  w.begin_object()
      .kv("airport", apt->ident)
      .kv("runway", req.get("rwy"))
      .kv("lda_m", d.lda_m)
      .kv("crosswind_kt", cross)
      .kv("vref_kt", r.vref_kt)
      .kv("air_m", r.air_m)
      .kv("ground_m", r.ground_m)
      .kv("actual_m", r.actual_m)
      .kv("required_m", r.required_m)
      .kv("ok", r.ok)
      .kv("limit", r.limit)
      .end_object();
  return json(w);
}

Response api_runways(const App& app, const Request& req) {
  const Airport* apt = app.db.find(req.get("apt"));
  if (!apt) return error(404, "unknown airport " + req.get("apt"));
  const std::int64_t t = request_time(app, req);
  JsonWriter w;
  w.begin_object().kv("airport", apt->ident).key("ends").begin_array();
  for (const Runway& r : apt->runways) {
    for (const RunwayEnd* e : {&r.le, &r.he}) {
      if (e->ident.empty()) continue;
      RunwayDistances d;
      NavDb::distances(*apt, e->ident, &d);
      const Notam* by = nullptr;
      const bool closed = r.closed || runway_closed(app.notams, apt->ident, e->ident, t, &by);
      w.begin_object()
          .kv("ident", e->ident)
          .kv("heading_true", e->heading_true)
          .kv("tora_m", d.tora_m)
          .kv("lda_m", d.lda_m)
          .kv("surface", r.surface)
          .kv("closed", closed)
          .kv("notam", by ? by->id : "")
          .end_object();
    }
  }
  w.end_array().end_object();
  return json(w);
}

Response api_wb(const Request& req) {
  std::vector<LoadItem> load = {
      {"crew", req.num("crew_kg", 400)},
      {"pax-forward", req.num("pax_fwd", 40) * 84.0},
      {"pax-mid", req.num("pax_mid", 45) * 84.0},
      {"pax-aft", req.num("pax_aft", 40) * 84.0},
      {"cargo-forward", req.num("cargo_fwd", 1500)},
      {"cargo-aft", req.num("cargo_aft", 1800)},
  };
  const WbResult r =
      weight_and_balance(sb20(), load, req.num("block", 9000), req.num("taxi", 200), req.num("trip", 6000));
  JsonWriter w;
  w.begin_object().kv("ok", r.ok).key("points").begin_array();
  for (const WbPoint& p : r.points)
    w.begin_object().kv("name", p.name).kv("mass_kg", p.mass_kg).kv("cg_mac", p.cg_mac).kv("inside", p.inside).end_object();
  w.end_array().key("problems").begin_array();
  for (const auto& s : r.problems) w.value(s);
  w.end_array().key("envelope").begin_array();
  for (const auto& [m, c] : sb20_envelope()) w.begin_array().value(m).value(c).end_array();
  w.end_array().end_object();
  return json(w);
}

Response api_notams(const App& app, const Request& req) {
  const std::string apt = req.get("apt");
  const std::int64_t t = request_time(app, req);
  JsonWriter w;
  w.begin_object().kv("time", format_notam_time(t)).key("notams").begin_array();
  for (const Notam& n : app.notams) {
    if (!apt.empty() && n.location != apt) continue;
    w.begin_object()
        .kv("id", n.id)
        .kv("location", n.location)
        .kv("from", format_notam_time(n.from_min))
        .kv("to", format_notam_time(n.to_min))
        .kv("active", active(n, t))
        .kv("text", n.text);
    w.key("closed_runways").begin_array();
    for (const auto& r : n.closed_runways) w.value(r);
    w.end_array().end_object();
  }
  w.end_array().end_object();
  return json(w);
}

Response api_view(const App& app, const Request& req) {
  const int width = std::clamp(static_cast<int>(req.num("w", 480)), 64, 1280);
  const int height = std::clamp(static_cast<int>(req.num("h", 270)), 64, 720);
  const terra::Ellipsoid e = terra::Ellipsoid::wgs84();
  const double lat = req.num("lat", 37.6), lon = req.num("lon", -122.4);
  const LatLon here{lat, lon};
  const double ground = app.terrain->height_m(here);
  const double alt = std::max(req.num("alt_ft", 5000) * kFt, ground + 30.0);
  const terra::Camera cam = terra::Camera::from_geodetic(e, {lat * kPi / 180, lon * kPi / 180, alt},
                                                         req.num("hdg", 0), req.num("pitch", -5), 60, width, height);
  const Terrain* t = app.terrain.get();
  const terra::HeightFn fn = [t](double la, double lo) { return t->height_m({la * 180 / kPi, lo * 180 / kPi}); };
  terra::RenderOptions opt;
  const terra::Image img = terra::render(e, fn, cam, opt);
  const std::vector<std::uint8_t> png = terra::encode_png(img);
  return {200, "image/png", std::string(png.begin(), png.end())};
}

Response api_search(const App& app, const Request& req) {
  JsonWriter w;
  w.begin_object().key("airports").begin_array();
  for (const Airport* a : app.db.search(req.get("q"), 20)) write_airport(w, *a, false);
  w.end_array().end_object();
  return json(w);
}

Response api_near(const App& app, const Request& req) {
  const LatLon c{req.num("lat", 0), req.num("lon", 0)};
  const double r = std::min(req.num("radius_nm", 50), 500.0) * kNm;
  const std::size_t limit = static_cast<std::size_t>(std::clamp(req.num("limit", 300), 1.0, 2000.0));
  const bool major = req.get("major") == "1";
  JsonWriter w;
  w.begin_object().key("airports").begin_array();
  std::size_t n = 0;
  for (const Airport* a : app.db.within(c, r)) {
    if (major && a->type != "large_airport" && a->type != "medium_airport") continue;
    if (n++ >= limit) break;
    write_airport(w, *a, false);
  }
  w.end_array().end_object();
  return json(w);
}

}  // namespace

std::string load_app(App* app, const std::string& data_dir, const std::string& web_dir) {
  std::string err;
  app->db = NavDb::load(data_dir + "/ourairports", &err);
  if (!err.empty()) return err;
  app->terrain = std::make_unique<Terrain>(&app->db);
  std::string text;
  if (read_file(data_dir + "/notams/notams.txt", &text)) {
    std::vector<std::string> errors;
    app->notams = parse_notams(text, &errors);
    if (!errors.empty()) return "notams.txt: " + errors.front();
  }
  app->web_dir = web_dir;
  app->default_time_min = parse_notam_time("2611241200");  // the demo clock: the synthetic feed is for this day
  return "";
}

bool parse_route(const NavDb& db, const std::string& text, std::vector<Waypoint>* out, std::string* error) {
  out->clear();
  std::vector<std::string> toks;
  std::string cur;
  for (char c : text) {
    if (c == ' ' || c == ',' || c == '\t') {
      if (!cur.empty()) toks.push_back(cur), cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) toks.push_back(cur);
  for (const std::string& t : toks) {
    const std::size_t slash = t.find('/');
    if (slash != std::string::npos) {
      char* e1 = nullptr;
      char* e2 = nullptr;
      const std::string a = t.substr(0, slash), b = t.substr(slash + 1);
      const double lat = std::strtod(a.c_str(), &e1), lon = std::strtod(b.c_str(), &e2);
      if (a.empty() || b.empty() || *e1 || *e2 || std::abs(lat) > 90 || std::abs(lon) > 180) {
        *error = "bad coordinate " + t;
        return false;
      }
      char name[64];
      std::snprintf(name, sizeof name, "%c%05.2f%c%06.2f", lat >= 0 ? 'N' : 'S', std::abs(lat), lon >= 0 ? 'E' : 'W',
                    std::abs(lon));
      out->push_back({name, {lat, wrap180(lon)}});
    } else {
      const Airport* a = db.find(t);
      if (!a) {
        *error = "unknown airport " + t;
        return false;
      }
      out->push_back({a->ident, a->pos});
    }
  }
  if (out->size() < 2) {
    *error = "a route needs at least two points";
    return false;
  }
  return true;
}

Response handle(const App& app, const Request& req) {
  const std::string& p = req.path;
  if (p == "/api/plan") return api_plan(app, req);
  if (p == "/api/track") return api_track(app, req);
  if (p == "/api/takeoff") return api_takeoff(app, req);
  if (p == "/api/landing") return api_landing(app, req);
  if (p == "/api/runways") return api_runways(app, req);
  if (p == "/api/wb") return api_wb(req);
  if (p == "/api/notams") return api_notams(app, req);
  if (p == "/api/view") return api_view(app, req);
  if (p == "/api/search") return api_search(app, req);
  if (p == "/api/near") return api_near(app, req);
  if (p == "/api/airport") {
    const Airport* a = app.db.find(req.get("ident"));
    if (!a) return error(404, "unknown airport " + req.get("ident"));
    JsonWriter w;
    write_airport(w, *a, true);
    return json(w);
  }
  if (p.rfind("/api/", 0) == 0) return error(404, "no such endpoint " + p);
  // Static files from web/; no "..", no hidden files.
  std::string rel = p == "/" ? "/index.html" : p;
  if (rel.find("..") != std::string::npos || rel.find("/.") != std::string::npos) return {404, "text/plain", "not found\n"};
  std::string body;
  if (!read_file(app.web_dir + rel, &body)) return {404, "text/plain", "not found\n"};
  return {200, content_type(rel), body};
}

}  // namespace skybag
