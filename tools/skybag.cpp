// skybag: the flight bag's command line. Runs the local web UI, plans and
// performance from the terminal, and the checks that measure the engine
// against closed forms, references and brute force.
//
//   skybag serve [--port 8080] [--host 127.0.0.1]
//   skybag plan KSFO KLAX [--fl 350] [--zfw 58000] [--mach 0.78] [--wind-seed 1]
//   skybag takeoff KSFO 28R [--mass 70000] [--oat 15] [--qnh 1013] [--wind 280/10] [--wet]
//   skybag landing KSFO 28R [--mass 60000] ...
//   skybag --check
//   skybag --bench
//   skybag geodesic-dump [--n 2000] [--seed 1]   random inverse problems, for tools/geodesic_check.py
//
// Data comes from <repo>/data unless --data DIR or $SKYBAG_DATA says otherwise.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "skybag/api.hpp"
#include "skybag/geo.hpp"
#include "skybag/navdb.hpp"
#include "skybag/notam.hpp"
#include "skybag/perf.hpp"
#include "skybag/planner.hpp"
#include "skybag/terrain.hpp"
#include "skybag/track.hpp"
#include "skybag/units.hpp"
#include "skybag/wb.hpp"
#include "skybag/wind.hpp"

using namespace skybag;

namespace {

struct Args {
  std::vector<std::string> pos;
  std::vector<std::pair<std::string, std::string>> opts;
  std::string get(const std::string& k, const std::string& d = "") const {
    for (const auto& [a, b] : opts)
      if (a == k) return b;
    return d;
  }
  bool has(const std::string& k) const {
    for (const auto& [a, b] : opts)
      if (a == k) return true;
    return false;
  }
  double num(const std::string& k, double d) const {
    const std::string v = get(k);
    return v.empty() ? d : std::atof(v.c_str());
  }
};

Args parse_args(int argc, char** argv, int from) {
  Args a;
  for (int i = from; i < argc; ++i) {
    std::string s = argv[i];
    if (s.rfind("--", 0) == 0) {
      const bool flag = i + 1 >= argc || std::strncmp(argv[i + 1], "--", 2) == 0;
      a.opts.push_back({s.substr(2), flag ? "1" : argv[++i]});
    } else {
      a.pos.push_back(s);
    }
  }
  return a;
}

std::string data_dir(const Args& a) {
  if (a.has("data")) return a.get("data");
  if (const char* e = std::getenv("SKYBAG_DATA")) return e;
  return std::string(SKYBAG_SOURCE_DIR) + "/data";
}

// ---------------------------------------------------------------- checks

struct Row {
  std::string name;
  enum { Pass, Fail, Gap } result;
  double value, threshold;
  std::string measures;
};

Row judge(const std::string& name, double value, double threshold, const std::string& measures, bool gap = false) {
  const bool ok = value <= threshold;
  return {name, ok ? Row::Pass : (gap ? Row::Gap : Row::Fail), value, threshold, measures};
}

// xorshift64*, for check inputs that must be identical on every platform.
struct Rng {
  std::uint64_t s;
  explicit Rng(std::uint64_t seed) : s(seed) {}
  double u() {
    s ^= s >> 12;
    s ^= s << 25;
    s ^= s >> 27;
    return static_cast<double>((s * 0x2545f4914f6cdd1dull) >> 11) * (1.0 / 9007199254740992.0);
  }
};

// Independent point-in-polygon reference: winding number with exact edge test.
bool winding_inside(const std::vector<std::pair<double, double>>& poly, double x, double y) {
  int wn = 0;
  const std::size_t n = poly.size();
  for (std::size_t i = 0; i < n; ++i) {
    const auto [x0, y0] = poly[i];
    const auto [x1, y1] = poly[(i + 1) % n];
    const double cross = (x1 - x0) * (y - y0) - (y1 - y0) * (x - x0);
    const double scale = std::max({std::abs(x1 - x0), std::abs(y1 - y0), 1.0});
    if (std::abs(cross) <= 1e-9 * scale * scale && x >= std::min(x0, x1) && x <= std::max(x0, x1) &&
        y >= std::min(y0, y1) && y <= std::max(y0, y1))
      return true;
    if (y0 <= y) {
      if (y1 > y && cross > 0) ++wn;
    } else if (y1 <= y && cross < 0) {
      --wn;
    }
  }
  return wn != 0;
}

std::vector<Waypoint> split_route(const std::vector<Waypoint>& r, double max_leg_nm) {
  std::vector<Waypoint> out{r.front()};
  for (std::size_t k = 0; k + 1 < r.size(); ++k) {
    const Inverse g = inverse(r[k].pos, r[k + 1].pos);
    const int n = std::max(1, static_cast<int>(std::ceil(g.dist_m / kNm / max_leg_nm)));
    for (int j = 1; j < n; ++j) out.push_back({"P" + std::to_string(out.size()), direct(r[k].pos, g.az1_deg, g.dist_m * j / n)});
    out.push_back(r[k + 1]);
  }
  return out;
}

std::vector<Row> run_checks(const App& app) {
  std::vector<Row> rows;
  const NavDb& db = app.db;
  Rng rng(20261124);

  // ISA against the ICAO table.
  {
    struct C { double h, T, p, rho; } cs[] = {
        {0, 288.15, 101325.0, 1.225}, {5000, 255.65, 54019.9, 0.736116}, {11000, 216.65, 22632.1, 0.363918},
        {20000, 216.65, 5474.89, 0.0880349}};
    double worst = 0;
    for (const C& c : cs) {
      const Atmosphere at = isa(c.h);
      worst = std::max({worst, std::abs(at.T - c.T) / c.T, std::abs(at.p - c.p) / c.p, std::abs(at.rho - c.rho) / c.rho});
    }
    rows.push_back(judge("isa-table", worst, 1e-4, "max relative error vs the ICAO standard atmosphere table"));
  }
  // Vincenty direct then inverse.
  {
    double worst = 0;
    for (int k = 0; k < 100000; ++k) {
      const LatLon a{(rng.u() * 2 - 1) * 85, (rng.u() * 2 - 1) * 180};
      const double az = rng.u() * 360, s = rng.u() * 1.5e7;
      const LatLon b = direct(a, az, s);
      const Inverse g = inverse(a, b);
      if (g.ok) worst = std::max(worst, std::abs(g.dist_m - s));
    }
    rows.push_back(judge("geodesic-roundtrip", worst, 1e-3, "max |inverse(direct(s)) - s| over 100k lines up to 15 000 km (m)"));
  }
  // Nearly antipodal points: Vincenty's inverse fails to converge.
  {
    int failed = 0, n = 2000;
    for (int k = 0; k < n; ++k) {
      const LatLon a{(rng.u() * 2 - 1) * 60, (rng.u() * 2 - 1) * 180};
      const LatLon b{-a.lat + (rng.u() * 2 - 1) * 0.5, wrap180(a.lon + 180 + (rng.u() * 2 - 1) * 0.5)};
      if (!inverse(a, b).ok) ++failed;
    }
    rows.push_back(judge("geodesic-antipodal", static_cast<double>(failed) / n, 0.0,
                         "fraction of near-antipodal pairs where the inverse doesn't converge", true));
  }
  // Grid index vs brute force.
  {
    int mismatches = 0;
    const auto& all = db.airports();
    for (int k = 0; k < 60; ++k) {
      LatLon c;
      if (k < 50) {
        const Airport& a = all[static_cast<std::size_t>(rng.u() * static_cast<double>(all.size())) % all.size()];
        c = {std::clamp(a.pos.lat + (rng.u() * 2 - 1), -90.0, 90.0), wrap180(a.pos.lon + (rng.u() * 2 - 1))};
      } else {
        c = k < 55 ? LatLon{(rng.u() * 2 - 1) * 60, 179.7 + rng.u() * 0.6 - (rng.u() < 0.5 ? 360 : 0)}
                   : LatLon{80 + rng.u() * 9.9, (rng.u() * 2 - 1) * 180};
        c.lon = wrap180(c.lon);
      }
      const double r = (5 + rng.u() * 295) * kNm;
      const auto got = db.within(c, r);
      std::size_t want = 0;
      for (const Airport& a : all)
        if (inverse(c, a.pos).dist_m <= r) ++want;
      if (got.size() != want) ++mismatches;
    }
    rows.push_back(judge("navdb-index", mismatches, 0, "radius queries where the grid index != brute force, of 60"));
  }
  // Wind triangle on closed-form cases.
  {
    double worst = 0;
    auto wind = [](double from, double kt) {
      Wind w;
      const double r = from * 3.14159265358979323846 / 180;
      w.u_kt = -kt * std::sin(r);
      w.v_kt = -kt * std::cos(r);
      w.from_deg = from;
      w.speed_kt = kt;
      return w;
    };
    const WindTriangle a = wind_triangle(0, 400, wind(270, 100));
    worst = std::max({worst, std::abs(a.heading_deg - (360 - std::asin(0.25) * 180 / 3.14159265358979323846)),
                      std::abs(a.groundspeed_kt - std::sqrt(400.0 * 400 - 100 * 100))});
    const WindTriangle b = wind_triangle(90, 450, wind(90, 80));
    worst = std::max({worst, std::abs(b.groundspeed_kt - 370), std::abs(b.heading_deg - 90)});
    const WindTriangle c = wind_triangle(180, 300, wind(0, 50));
    worst = std::max({worst, std::abs(c.groundspeed_kt - 350), std::abs(c.heading_deg - 180)});
    rows.push_back(judge("wind-triangle", worst, 1e-9, "max error on closed-form heading / ground speed cases (deg, kt)"));
  }
  // Take-off: balanced V1, step-size convergence, monotonicity.
  {
    double worst_balance = 0, worst_dt = 0;
    int violations = 0, balanced = 0, total = 0;
    const double masses[] = {55000, 62000, 68000, 74000};
    const double elevs[] = {0, 5000};
    const double oats[] = {0, 15, 35};
    const double winds[] = {-10, 0, 20};
    for (double e : elevs)
      for (double o : oats)
        for (double w : winds) {
          double prev = 0;
          for (double m : masses) {
            TakeoffInput in;
            in.mass_kg = m;
            in.rw.elevation_ft = e;
            in.rw.oat_c = o;
            in.rw.headwind_kt = w;
            const TakeoffResult r = takeoff(sb20(), in);
            ++total;
            if (r.balanced) {
              ++balanced;
              worst_balance = std::max(worst_balance, std::abs(r.tod_oei_m - r.asd_m) / r.required_m);
            }
            if (r.required_m < prev * (1 - 1e-9)) ++violations;
            prev = r.required_m;
          }
        }
    for (double m : {60000.0, 74000.0})
      for (double o : {15.0, 35.0}) {
        TakeoffInput in;
        in.mass_kg = m;
        in.rw.oat_c = o;
        const double coarse = takeoff(sb20(), in).required_m;
        in.dt = 0.002;
        const double fine = takeoff(sb20(), in).required_m;
        worst_dt = std::max(worst_dt, std::abs(coarse - fine) / fine);
      }
    for (double m : {60000.0, 70000.0}) {
      double prev = 0;
      for (double o : {-10.0, 10.0, 25.0, 40.0}) {
        TakeoffInput in;
        in.mass_kg = m;
        in.rw.oat_c = o;
        const double d = takeoff(sb20(), in).required_m;
        if (d < prev * (1 - 1e-9)) ++violations;
        prev = d;
      }
    }
    rows.push_back(judge("takeoff-balanced", worst_balance, 1e-3,
                         "|OEI distance - ASD| / required at the balanced V1 (" + std::to_string(balanced) + " of " +
                             std::to_string(total) + " cases balanced)"));
    rows.push_back(judge("takeoff-step-size", worst_dt, 2e-3, "required distance, dt 0.05 s vs 0.002 s (relative)"));
    rows.push_back(judge("takeoff-monotonic", violations, 0, "cases where more mass or a hotter day needs less runway"));
  }
  {
    int violations = 0;
    for (double w : {-10.0, 0.0, 20.0}) {
      double prev = 0;
      for (double m : {50000.0, 56000.0, 60000.0, 64500.0}) {
        LandingInput in;
        in.mass_kg = m;
        in.rw.headwind_kt = w;
        const double d = landing(sb20(), in).required_m;
        if (d < prev * (1 - 1e-9)) ++violations;
        prev = d;
      }
    }
    rows.push_back(judge("landing-monotonic", violations, 0, "cases where more mass needs less landing distance"));
  }
  // Weight and balance envelope vs an independent winding-number test.
  {
    const auto& env = sb20_envelope();
    int mismatches = 0;
    for (int k = 0; k < 20000; ++k) {
      double x = 38000 + rng.u() * 38000, y = 12 + rng.u() * 30;
      if (k % 10 == 0) {  // on an edge
        const auto& p = env[static_cast<std::size_t>(k / 10) % env.size()];
        const auto& q = env[(static_cast<std::size_t>(k / 10) + 1) % env.size()];
        const double t = rng.u();
        x = p.first + (q.first - p.first) * t;
        y = p.second + (q.second - p.second) * t;
      }
      if (point_in_polygon(env, x, y) != winding_inside(env, x, y)) ++mismatches;
    }
    for (const auto& v : env)
      if (!point_in_polygon(env, v.first, v.second)) ++mismatches;
    rows.push_back(judge("wb-envelope", mismatches, 0, "CG envelope point tests that disagree with a winding-number reference"));
  }
  // NOTAM parsing fixtures.
  {
    struct Case {
      const char* raw;
      bool parses;
      const char* closed;  // comma-joined runways
      const char* from;
      const char* to;
    };
    const Case cases[] = {
        {"A0412/26 NOTAMN\nQ) KZOA/QMRLC/IV/NBO/A/000/999/3737N12223W005\nA) KSFO B) 2611240600 C) 2611251800\nE) RWY 10L/28R CLSD",
         true, "10L,28R", "2611240600", "2611251800"},
        {"A0001/26 NOTAMN A) EGLL B) 2611010000 C) PERM E) TWY B CLSD", true, "", "2611010000", "PERM"},
        {"A0002/26 NOTAMN A) KJFK B) 2611240000 C) 2611242359EST E) RWY 04L/22R CLSD DUE WIP", true, "04L,22R",
         "2611240000", "2611242359"},
        {"A0003/26 NOTAMN A) KJFK B) 2613011200 C) 2611251800 E) RWY 13R CLSD", false, "", "", ""},
        {"A0004/26 NOTAMN A) KJFK B) 2611251800 C) 2611241800 E) RWY 13R CLSD", false, "", "", ""},
        {"A0005/26 NOTAMN A) KDEN B) 2602281200 C) 2602291200 E) RWY 16L CLSD", false, "", "", ""},
        {"A0006/24 NOTAMN A) KDEN B) 2402281200 C) 2402291200 E) RWY 16L/34R AND RWY 07/25 CLSD", true,
         "16L,34R,07,25", "2402281200", "2402291200"},
        {"A0007/26 NOTAMN A) KLAX B) 2611240600 E) RWY 37 CLSD", true, "", "2611240600", "PERM"},
    };
    int bad = 0;
    for (const Case& c : cases) {
      const auto n = parse_notam(c.raw);
      if (n.has_value() != c.parses) {
        ++bad;
        continue;
      }
      if (!n) continue;
      std::string joined;
      for (const auto& r : n->closed_runways) joined += (joined.empty() ? "" : ",") + r;
      if (joined != c.closed || format_notam_time(n->from_min) != c.from || format_notam_time(n->to_min) != c.to) ++bad;
    }
    // Validity window is half-open: active at B, not at C.
    const auto n = parse_notam(cases[0].raw);
    if (!n || !active(*n, n->from_min) || active(*n, n->to_min) || !active(*n, n->to_min - 1)) ++bad;
    rows.push_back(judge("notam-fixtures", bad, 0, "NOTAM fixture cases parsed wrong (ids, closures, windows), of 9"));
  }
  // Terrain flattened to field elevation at airports.
  {
    double worst = 0;
    const auto& all = db.airports();
    for (int k = 0; k < 300; ++k) {
      const Airport& a = all[static_cast<std::size_t>(rng.u() * static_cast<double>(all.size())) % all.size()];
      if (!a.has_elevation) continue;
      worst = std::max(worst, std::abs(app.terrain->height_m(a.pos) - a.elevation_ft * kFt));
    }
    rows.push_back(judge("terrain-at-field", worst, 1e-6, "|terrain - field elevation| at 300 random airports (m)"));
  }
  // Terrain clearance: waypoints only vs a dense corridor search.
  {
    const char* routes[][2] = {{"KDEN", "KSLC"}, {"LSZH", "LIMC"}, {"RJAA", "PANC"}, {"KSEA", "CYVR"}};
    double worst = 0;
    for (const auto& r : routes) {
      const Airport* a = db.find(r[0]);
      const Airport* b = db.find(r[1]);
      if (!a || !b) continue;
      const std::vector<LatLon> pts{a->pos, b->pos};
      const Clearance fast = route_clearance(*app.terrain, pts, 10 * kNm);
      const Clearance ref = brute_force_clearance(*app.terrain, pts, 10 * kNm, 1000, 8);
      worst = std::max(worst, ref.max_terrain_m - fast.max_terrain_m);
    }
    rows.push_back(judge("terrain-corridor", worst, 1.0,
                         "highest terrain within 10 nm of the route missed by the planner, 4 routes (m)", true));
  }
  // Fuel: plan vs step-by-step reference.
  {
    const Airport* a = db.find("KJFK");
    const Airport* b = db.find("EGLL");
    PlanInput in;
    in.route = {{"KJFK", a->pos}, {"EGLL", b->pos}};
    in.zfw_kg = 56000;
    const WindField wind(1);
    const Plan direct_plan = plan(sb20(), wind, in);
    const double ref = reference_trip_fuel(sb20(), wind, in, direct_plan.cruise_ft, direct_plan.tow_kg, 1.0);
    PlanInput split = in;
    split.route = split_route(in.route, 50);
    const Plan split_plan = plan(sb20(), wind, split);
    const double ref2 = reference_trip_fuel(sb20(), wind, split, split_plan.cruise_ft, split_plan.tow_kg, 1.0);
    rows.push_back(judge("fuel-short-legs", std::abs(split_plan.trip_kg - ref2) / ref2, 0.005,
                         "KJFK-EGLL in 50 nm legs: |planned trip fuel - 1 nm reference| / reference"));
    rows.push_back(judge("fuel-along-leg", std::abs(direct_plan.trip_kg - ref) / ref, 0.005,
                         "KJFK-EGLL as one direct leg: |planned trip fuel - 1 nm reference| / reference", true));
    const Plan& p = direct_plan;
    const double block_err = std::abs(p.block_kg - (p.trip_kg + p.contingency_kg + p.reserve_kg + p.alternate_kg + p.taxi_kg));
    const double tow_err = std::abs(p.tow_kg - (in.zfw_kg + p.block_kg - p.taxi_kg));
    rows.push_back(judge("fuel-bookkeeping", std::max(block_err, tow_err), 0.05,
                         "block = trip + contingency + reserve + alternate + taxi, TOW = ZFW + block - taxi (kg)"));
    // Replay determinism.
    const auto t1 = simulate_track(split_plan, split.route, 5, 42);
    const auto t2 = simulate_track(split_plan, split.route, 5, 42);
    const auto t3 = simulate_track(split_plan, split.route, 5, 43);
    const double bad = (track_hash(t1) != track_hash(t2)) + (track_hash(t1) == track_hash(t3)) + (t1.empty() ? 1 : 0);
    rows.push_back(judge("replay-determinism", bad, 0, "same seed different hash, or new seed same hash"));
  }
  return rows;
}

int check(const App& app) {
  const auto t0 = std::chrono::steady_clock::now();
  const auto rows = run_checks(app);
  const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  std::printf("%-20s %-10s %14s %12s  %s\n", "check", "result", "value", "threshold", "measures");
  int failed = 0, gaps = 0;
  for (const Row& r : rows) {
    const char* res = r.result == Row::Pass ? "PASS" : r.result == Row::Gap ? "KNOWN GAP" : "FAIL";
    if (r.result == Row::Fail) ++failed;
    if (r.result == Row::Gap) ++gaps;
    std::printf("%-20s %-10s %14.6g %12.6g  %s\n", r.name.c_str(), res, r.value, r.threshold, r.measures.c_str());
  }
  std::printf("\n%d failed, %d known gaps (%.1f s)\n", failed, gaps, secs);
  return failed ? 1 : 0;
}

// ---------------------------------------------------------------- commands

bool load(App* app, const Args& a) {
  const std::string err = load_app(app, data_dir(a), std::string(SKYBAG_SOURCE_DIR) + "/web");
  if (!err.empty()) {
    std::fprintf(stderr, "skybag: %s\n", err.c_str());
    return false;
  }
  return true;
}

int cmd_plan(const App& app, const Args& a) {
  std::string route_text;
  for (const auto& p : a.pos) route_text += p + " ";
  std::vector<Waypoint> route;
  std::string err;
  if (!parse_route(app.db, route_text, &route, &err)) {
    std::fprintf(stderr, "skybag plan: %s\n", err.c_str());
    return 2;
  }
  PlanInput in;
  in.route = route;
  in.cruise_ft = a.num("fl", 350) * 100;
  in.zfw_kg = a.num("zfw", 58000);
  in.mach = a.num("mach", 0.78);
  in.alternate_kg = a.num("alt-fuel", 0);
  const Plan p = plan(sb20(), WindField(static_cast<std::uint32_t>(a.num("wind-seed", 1))), in);
  if (!p.ok) {
    std::fprintf(stderr, "skybag plan: %s\n", p.error.c_str());
    return 1;
  }
  std::printf("%-10s %-10s %-8s %8s %6s %6s %9s %5s %5s %7s %7s\n", "from", "to", "phase", "dist nm", "crs", "hdg",
              "wind", "tas", "gs", "min", "fuel kg");
  for (const LegPlan& l : p.legs)
    std::printf("%-10s %-10s %-8s %8.1f %6.1f %6.1f %03.0f/%-4.0f %5.0f %5.0f %7.1f %7.0f\n", l.from.c_str(), l.to.c_str(),
                l.phase.c_str(), l.dist_nm, l.course_deg, l.heading_deg, l.wind_from_deg, l.wind_kt, l.tas_kt, l.gs_kt,
                l.time_min, l.fuel_kg);
  std::printf("\nFL%.0f  %.0f nm  %.0f min  TOC %.0f nm  TOD %.0f nm\n", p.cruise_ft / 100, p.dist_nm, p.time_min, p.toc_nm,
              p.tod_nm);
  std::printf("trip %.0f  contingency %.0f  reserve %.0f  alternate %.0f  taxi %.0f  block %.0f kg\n", p.trip_kg,
              p.contingency_kg, p.reserve_kg, p.alternate_kg, p.taxi_kg, p.block_kg);
  std::printf("TOW %.0f kg  LDW %.0f kg\n", p.tow_kg, p.ldw_kg);
  for (const auto& w : p.warnings) std::printf("WARNING: %s\n", w.c_str());
  return 0;
}

int cmd_perf(const App& app, const Args& a, bool is_takeoff) {
  if (a.pos.size() < 2) {
    std::fprintf(stderr, "usage: skybag %s APT RWY [--mass KG] [--oat C] [--qnh HPA] [--wind DDD/KK] [--wet]\n",
                 is_takeoff ? "takeoff" : "landing");
    return 2;
  }
  Request req;
  req.path = is_takeoff ? "/api/takeoff" : "/api/landing";
  req.query["apt"] = a.pos[0];
  req.query["rwy"] = a.pos[1];
  for (const char* k : {"mass", "oat", "qnh", "time"})
    if (a.has(k)) req.query[k] = a.get(k);
  if (a.has("wet")) req.query["wet"] = "1";
  const std::string wind = a.get("wind");
  if (const std::size_t s = wind.find('/'); s != std::string::npos) {
    req.query["wind_dir"] = wind.substr(0, s);
    req.query["wind_kt"] = wind.substr(s + 1);
  }
  const Response r = handle(app, req);
  std::printf("%s\n", r.body.c_str());
  return r.status == 200 ? 0 : 1;
}

int cmd_serve(const App& app, const Args& a) {
  HttpServer server;
  std::string err;
  if (!server.listen(a.get("host", "127.0.0.1"), static_cast<int>(a.num("port", 8080)), &err)) {
    std::fprintf(stderr, "skybag serve: %s\n", err.c_str());
    return 1;
  }
  std::printf("skybag: %zu airports, %zu runways, %zu NOTAMs\n", app.db.airports().size(), app.db.runway_count(),
              app.notams.size());
  std::printf("skybag: open http://%s:%d/\n", a.get("host", "127.0.0.1").c_str(), server.port());
  std::fflush(stdout);
  server.serve([&](const Request& req) { return handle(app, req); });
  return 0;
}

int cmd_bench(const App& app) {
  auto time = [](auto&& fn, int reps) {
    const auto t0 = std::chrono::steady_clock::now();
    for (int k = 0; k < reps; ++k) fn();
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() / reps;
  };
  const Airport* a = app.db.find("KJFK");
  const Airport* b = app.db.find("EGLL");
  PlanInput in;
  in.route = {{"KJFK", a->pos}, {"EGLL", b->pos}};
  const WindField wind(1);
  std::printf("plan KJFK-EGLL          %8.3f ms\n", time([&] { (void)plan(sb20(), wind, in); }, 20));
  std::printf("reference fuel (1 nm)   %8.3f ms\n",
              time([&] { (void)reference_trip_fuel(sb20(), wind, in, 35000, 70000, 1.0); }, 5));
  TakeoffInput ti;
  std::printf("takeoff (balanced V1)   %8.3f ms\n", time([&] { (void)takeoff(sb20(), ti); }, 20));
  const std::vector<LatLon> pts{app.db.find("KDEN")->pos, app.db.find("KSLC")->pos};
  std::printf("corridor brute force    %8.3f ms  (KDEN-KSLC, 1 km x 17 across)\n",
              time([&] { (void)brute_force_clearance(*app.terrain, pts, 10 * kNm, 1000, 8); }, 3));
  std::printf("radius query 100 nm     %8.3f ms\n", time([&] { (void)app.db.within(a->pos, 100 * kNm); }, 200));
  Request view;
  view.path = "/api/view";
  view.query = {{"lat", "42.0"}, {"lon", "-71.5"}, {"alt_ft", "16000"}, {"hdg", "52"}, {"pitch", "-4"}, {"w", "400"}, {"h", "225"}};
  std::printf("synthetic view 400x225  %8.1f ms  (FL160 over New England)\n", time([&] { (void)handle(app, view); }, 1));
  return 0;
}

int cmd_geodesic_dump(const Args& a) {
  const int n = static_cast<int>(a.num("n", 2000));
  Rng rng(static_cast<std::uint64_t>(a.num("seed", 1)));
  for (int k = 0; k < n; ++k) {
    const LatLon p{(rng.u() * 2 - 1) * 89.9, (rng.u() * 2 - 1) * 180};
    const LatLon q{(rng.u() * 2 - 1) * 89.9, (rng.u() * 2 - 1) * 180};
    const Inverse g = inverse(p, q);
    std::printf("%.12f %.12f %.12f %.12f %.6f %.12f %d\n", p.lat, p.lon, q.lat, q.lon, g.dist_m, g.az1_deg, g.ok ? 1 : 0);
  }
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string cmd = argc > 1 ? argv[1] : "";
  const Args a = parse_args(argc, argv, 2);
  if (cmd.empty() || cmd == "--help" || cmd == "-h") {
    std::printf("usage: skybag serve|plan|takeoff|landing|--check|--bench (see the comment at the top of tools/skybag.cpp)\n");
    return cmd.empty() ? 2 : 0;
  }
  if (cmd == "geodesic-dump") return cmd_geodesic_dump(a);
  const auto t0 = std::chrono::steady_clock::now();
  App app;
  if (!load(&app, a)) return 1;
  if (cmd == "--bench")
    std::printf("load navdb + notams       %8.1f ms\n",
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
  if (cmd == "--check") return check(app);
  if (cmd == "--bench") return cmd_bench(app);
  if (cmd == "serve") return cmd_serve(app, a);
  if (cmd == "plan") return cmd_plan(app, a);
  if (cmd == "takeoff") return cmd_perf(app, a, true);
  if (cmd == "landing") return cmd_perf(app, a, false);
  std::fprintf(stderr, "skybag: unknown command %s\n", cmd.c_str());
  return 2;
}
