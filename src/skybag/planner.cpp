#include "skybag/planner.hpp"

#include <algorithm>
#include <cmath>

#include "skybag/units.hpp"

namespace skybag {

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Leg {
  LatLon start;
  double dist_nm = 0, course = 0;
  double x0 = 0;  // along-track distance at the start of the leg
};

enum Phase { kClimb, kCruise, kDescent };

double phase_alt(Phase ph, double cruise_ft) { return ph == kCruise ? cruise_ft : cruise_ft / 2; }

double cruise_tas_kt(double mach, double alt_ft, double isa_dev) {
  return mach * isa(alt_ft * kFt, isa_dev).a / kKt;
}

double tas_for(const Aircraft& ac, Phase ph, double mach, double cruise_ft, const Wind& w) {
  if (ph == kClimb) return ac.climb_tas_kt;
  if (ph == kDescent) return ac.descent_tas_kt;
  return cruise_tas_kt(mach, cruise_ft, w.isa_dev_c);
}

double descent_air_nm(const Aircraft& ac, double alt_ft) {
  return alt_ft * kFt / std::tan(ac.descent_angle_deg * kPi / 180.0) / kNm;
}

}  // namespace

double cruise_fuel_flow(const Aircraft& ac, double mass_kg, double alt_ft, double mach, double isa_dev) {
  const Atmosphere at = isa(alt_ft * kFt, isa_dev);
  const double v = mach * at.a;
  const double q = 0.5 * at.rho * v * v * ac.wing_area;
  const double cl = mass_kg * kG0 / q;
  const double drag = q * (ac.cd0_clean + ac.k_clean * cl * cl);
  return ac.tsfc * drag * 3600.0;
}

double holding_fuel_flow(const Aircraft& ac, double mass_kg) {
  // Minimum-drag speed (L/D max), clean; 15% extra for the low-altitude,
  // low-speed engine being less efficient than in cruise.
  const double cl = std::sqrt(ac.cd0_clean / ac.k_clean);
  const double drag = mass_kg * kG0 * (ac.cd0_clean + ac.k_clean * cl * cl) / cl;
  return ac.tsfc * 1.15 * drag * 3600.0;
}

Plan plan(const Aircraft& ac, const WindField& wind, const PlanInput& in) {
  Plan p;
  if (in.route.size() < 2) {
    p.error = "a route needs at least two points";
    return p;
  }
  std::vector<Leg> legs;
  double x = 0;
  for (std::size_t k = 0; k + 1 < in.route.size(); ++k) {
    const Inverse g = inverse(in.route[k].pos, in.route[k + 1].pos);
    if (!g.ok) {
      p.error = "geodesic did not converge between " + in.route[k].name + " and " + in.route[k + 1].name;
      return p;
    }
    legs.push_back({in.route[k].pos, g.dist_m / kNm, g.az1_deg, x});
    x += g.dist_m / kNm;
  }
  p.dist_nm = x;

  // Per-leg wind, TAS and ground speed for one phase.
  auto leg_wind = [&](const Leg&, LatLon at, Phase ph, double cruise_ft) {
    return wind.at(at, phase_alt(ph, cruise_ft));
  };
  auto leg_gs = [&](const Leg& l, LatLon at, Phase ph, double cruise_ft) {
    const Wind w = leg_wind(l, at, ph, cruise_ft);
    const WindTriangle tri = wind_triangle(l.course, tas_for(ac, ph, in.mach, cruise_ft, w), w);
    return tri.ok ? tri.groundspeed_kt : 1.0;
  };

  // Find TOC and TOD; lower the cruise level until they fit.
  double cruise = in.cruise_ft;
  for (;; cruise -= 1000) {
    if (cruise < 10000) {
      p.error = "route too short for a cruise segment";
      return p;
    }
    double climb_left_h = cruise / ac.climb_rate_fpm / 60.0;
    double toc = p.dist_nm;
    for (const Leg& l : legs) {
      const double gs = leg_gs(l, l.start, kClimb, cruise);
      const double t = l.dist_nm / gs;
      if (t >= climb_left_h) {
        toc = l.x0 + gs * climb_left_h;
        break;
      }
      climb_left_h -= t;
    }
    double descent_left_h = descent_air_nm(ac, cruise) / ac.descent_tas_kt;
    double tod = 0;
    for (auto it = legs.rbegin(); it != legs.rend(); ++it) {
      const double gs = leg_gs(*it, it->start, kDescent, cruise);
      const double t = it->dist_nm / gs;
      if (t >= descent_left_h) {
        tod = it->x0 + it->dist_nm - gs * descent_left_h;
        break;
      }
      descent_left_h -= t;
    }
    if (toc < tod) {
      p.toc_nm = toc;
      p.tod_nm = tod;
      break;
    }
  }
  p.cruise_ft = cruise;

  // Trip fuel for a take-off mass, filling the leg pieces.
  auto trip = [&](double tow, std::vector<LegPlan>* out) {
    double mass = tow, fuel = 0, time_h = 0;
    for (std::size_t k = 0; k < legs.size(); ++k) {
      const Leg& l = legs[k];
      const double cuts[4] = {l.x0, std::clamp(p.toc_nm, l.x0, l.x0 + l.dist_nm),
                              std::clamp(p.tod_nm, l.x0, l.x0 + l.dist_nm), l.x0 + l.dist_nm};
      for (int ph = 0; ph < 3; ++ph) {
        const double a = cuts[ph], b = cuts[ph + 1];
        if (b - a < 1e-9) continue;
        const Phase phase = static_cast<Phase>(ph);
        const LatLon start = a == l.x0 ? l.start : direct(l.start, l.course, (a - l.x0) * kNm);
        const Wind w = leg_wind(l, start, phase, cruise);
        const double tas = tas_for(ac, phase, in.mach, cruise, w);
        const WindTriangle tri = wind_triangle(l.course, tas, w);
        const double gs = tri.ok ? tri.groundspeed_kt : 1.0;
        const double hours = (b - a) / gs;
        double ff;
        if (phase == kClimb) ff = ac.climb_ff;
        else if (phase == kDescent) ff = ac.descent_ff;
        else ff = cruise_fuel_flow(ac, mass, cruise, in.mach, w.isa_dev_c);
        const double burn = ff * hours;
        if (out) {
          LegPlan lp;
          lp.from = in.route[k].name;
          lp.to = in.route[k + 1].name;
          lp.leg = static_cast<int>(k);
          lp.phase = phase == kClimb ? "climb" : phase == kCruise ? "cruise" : "descent";
          lp.dist_nm = b - a;
          lp.course_deg = l.course;
          lp.heading_deg = tri.heading_deg;
          lp.wind_from_deg = w.from_deg;
          lp.wind_kt = w.speed_kt;
          lp.tas_kt = tas;
          lp.gs_kt = gs;
          const double climb_ft_per_h = ac.climb_rate_fpm * 60.0;
          auto alt_at = [&](double xx, double t_h) {
            if (xx <= p.toc_nm) return std::min(cruise, t_h * climb_ft_per_h);
            if (xx >= p.tod_nm) return cruise * std::max(0.0, (p.dist_nm - xx) / (p.dist_nm - p.tod_nm));
            return cruise;
          };
          lp.alt_start_ft = alt_at(a, time_h);
          lp.alt_end_ft = alt_at(b, time_h + hours);
          lp.time_min = hours * 60;
          lp.fuel_kg = burn;
          out->push_back(lp);
        }
        fuel += burn;
        mass -= burn;
        time_h += hours;
      }
    }
    return std::pair<double, double>{fuel, time_h * 60};
  };

  // Fixed point: TOW depends on the fuel, the fuel on the TOW.
  double block = 0.25 * ac.max_fuel;
  for (int it = 0; it < 30; ++it) {
    const double tow = in.zfw_kg + block - in.taxi_kg;
    const auto [trip_kg, minutes] = trip(tow, nullptr);
    const double ldw = tow - trip_kg;
    const double reserve = holding_fuel_flow(ac, ldw) * in.final_reserve_min / 60.0;
    const double next = trip_kg * (1 + in.contingency_pct / 100.0) + reserve + in.alternate_kg + in.taxi_kg;
    p.trip_kg = trip_kg;
    p.time_min = minutes;
    p.contingency_kg = trip_kg * in.contingency_pct / 100.0;
    p.reserve_kg = reserve;
    const bool done = std::abs(next - block) < 0.01;
    block = next;
    if (done) break;
  }
  p.alternate_kg = in.alternate_kg;
  p.taxi_kg = in.taxi_kg;
  p.block_kg = block;
  p.tow_kg = in.zfw_kg + block - in.taxi_kg;
  p.ldw_kg = p.tow_kg - p.trip_kg;
  p.legs.clear();
  trip(p.tow_kg, &p.legs);

  if (p.tow_kg > ac.mtow) p.warnings.push_back("take-off mass above MTOW");
  if (p.ldw_kg > ac.mlw) p.warnings.push_back("landing mass above MLW");
  if (block > ac.max_fuel) p.warnings.push_back("block fuel above tank capacity");
  if (in.zfw_kg > ac.mzfw) p.warnings.push_back("zero-fuel mass above MZFW");
  p.ok = true;
  return p;
}

double reference_trip_fuel(const Aircraft& ac, const WindField& wind, const PlanInput& in, double cruise_ft,
                           double tow_kg, double step_nm, double* time_min) {
  std::vector<LatLon> pts;
  for (std::size_t k = 0; k + 1 < in.route.size(); ++k) {
    std::vector<LatLon> seg = densify(in.route[k].pos, in.route[k + 1].pos, step_nm * kNm);
    if (!pts.empty()) seg.erase(seg.begin());
    pts.insert(pts.end(), seg.begin(), seg.end());
  }
  std::vector<double> ds(pts.size(), 0.0), crs(pts.size(), 0.0);
  double total = 0;
  for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
    const Inverse g = inverse(pts[k], pts[k + 1]);
    ds[k] = g.dist_m / kNm;
    crs[k] = g.az1_deg;
    total += ds[k];
  }
  double alt = 0, mass = tow_kg, fuel = 0, hours = 0, x = 0;
  bool descending = false;
  const double tan_g = std::tan(ac.descent_angle_deg * kPi / 180.0);
  for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
    if (!descending) {
      const Wind wd = wind.at(pts[k], alt / 2);
      const WindTriangle td = wind_triangle(crs[k], ac.descent_tas_kt, wd);
      const double gs_d = td.ok ? td.groundspeed_kt : 1.0;
      const double need = alt * kFt / tan_g / kNm * gs_d / ac.descent_tas_kt;
      if (total - x <= need && alt >= cruise_ft - 1e-6) descending = true;
    }
    const Wind w = wind.at(pts[k], alt);
    double tas, ff;
    if (descending) {
      tas = ac.descent_tas_kt;
      ff = ac.descent_ff;
    } else if (alt < cruise_ft) {
      tas = ac.climb_tas_kt;
      ff = ac.climb_ff;
    } else {
      tas = cruise_tas_kt(in.mach, cruise_ft, w.isa_dev_c);
      ff = cruise_fuel_flow(ac, mass, cruise_ft, in.mach, w.isa_dev_c);
    }
    const WindTriangle tri = wind_triangle(crs[k], tas, w);
    const double gs = tri.ok ? tri.groundspeed_kt : 1.0;
    const double h = ds[k] / gs;
    if (descending) alt = std::max(0.0, alt - tas * h * kNm * tan_g / kFt);
    else if (alt < cruise_ft) alt = std::min(cruise_ft, alt + ac.climb_rate_fpm * 60.0 * h);
    fuel += ff * h;
    mass -= ff * h;
    hours += h;
    x += ds[k];
  }
  if (time_min) *time_min = hours * 60;
  return fuel;
}

}  // namespace skybag
