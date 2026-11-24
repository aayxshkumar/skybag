#include "skybag/perf.hpp"

#include <algorithm>
#include <cmath>

#include "skybag/units.hpp"

namespace skybag {

namespace {

constexpr double kScreenTakeoff = 35 * kFt;  // 10.67 m
constexpr double kScreenLanding = 50 * kFt;
constexpr double kNever = 1e9;               // "cannot be done" distance
constexpr double kPi = 3.14159265358979323846;

struct Field {
  Atmosphere at;
  double delta_t = 0;
  double vw = 0;     // headwind, m/s
  double sin_slope = 0, cos_slope = 1;
  double tas_per_cas = 1;
};

Field field(const RunwayConditions& rw) {
  Field f;
  const double hp = pressure_altitude_m(rw.elevation_ft * kFt, rw.qnh_hpa);
  f.delta_t = rw.oat_c + 273.15 - isa(hp).T;
  f.at = isa(hp, f.delta_t);
  f.vw = rw.headwind_kt * kKt;
  const double th = std::atan(rw.slope_pct / 100.0);
  f.sin_slope = std::sin(th);
  f.cos_slope = std::cos(th);
  // Low-speed CAS ~ EAS: TAS = CAS * sqrt(rho0 / rho). Compressibility is
  // below 1% at take-off speeds and is ignored.
  f.tas_per_cas = std::sqrt(kIsaRho0 / f.at.rho);
  return f;
}

// One ground-roll configuration: thrust from `operating` engines (or idle),
// a lift and drag coefficient, and a friction coefficient.
struct Config {
  int operating = 2;
  bool idle = false;
  double cl = 0, cd = 0, mu = 0;
};

struct Roll {
  const Aircraft& ac;
  const Field& f;
  double m;
  double dt;
  double s = 0;   // distance along the runway, m
  double vg = 0;  // ground speed, m/s
  double t = 0;

  double va() const { return vg + f.vw; }

  double accel(const Config& c) const {
    const double v = va();
    const double q = 0.5 * f.at.rho * v * std::abs(v) * ac.wing_area;
    double thrust = c.idle ? ac.static_thrust * ac.idle_thrust * c.operating
                           : ac.thrust(std::abs(v), f.at.rho, f.at.a, f.delta_t, c.operating);
    const double lift = q * c.cl;
    const double drag = q * c.cd;
    const double weight = m * kG0;
    const double normal = std::max(0.0, weight * f.cos_slope - lift);
    return (thrust - drag - c.mu * normal - weight * f.sin_slope) / m;
  }

  // Advance with constant acceleration over h seconds.
  void step(double a, double h) {
    s += vg * h + 0.5 * a * h * h;
    vg += a * h;
    t += h;
  }

  // Accelerate until the airspeed reaches target (TAS). False if it can't.
  bool to_speed(const Config& c, double target) {
    for (int guard = 0; guard < 2000000 && va() < target; ++guard) {
      const double a = accel(c);
      if (a <= 1e-6) return false;
      const double need = (target - va()) / a;
      if (need <= dt) {
        step(a, need);
        vg = target - f.vw;
        return true;
      }
      step(a, dt);
    }
    return va() >= target;
  }

  void for_time(const Config& c, double seconds) {
    double left = seconds;
    while (left > 1e-12) {
      const double h = std::min(dt, left);
      step(accel(c), h);
      left -= h;
    }
  }

  // Decelerate to a stop (ground speed 0).
  bool to_stop(const Config& c) {
    for (int guard = 0; guard < 2000000 && vg > 0; ++guard) {
      const double a = accel(c);
      if (a >= -1e-6) return false;
      const double need = -vg / a;
      if (need <= dt) {
        step(a, need);
        vg = 0;
        return true;
      }
      step(a, dt);
    }
    return vg <= 0;
  }
};

// Ground distance to climb from lift-off to the screen height at V2 with
// `operating` engines: constant flight-path angle from the excess thrust.
double air_distance(const Aircraft& ac, const Field& f, double m, double v2_tas, int operating, double screen) {
  const double q = 0.5 * f.at.rho * v2_tas * v2_tas * ac.wing_area;
  const double cl = m * kG0 / q;
  const double drag = q * (ac.cd0_takeoff + ac.k_takeoff * cl * cl);
  const double thrust = ac.thrust(v2_tas, f.at.rho, f.at.a, f.delta_t, operating);
  const double gamma = (thrust - drag) / (m * kG0);
  if (gamma <= 1e-3) return kNever;
  const double ground_over_air = std::max(0.0, (v2_tas - f.vw) / v2_tas);
  return screen / gamma * ground_over_air;
}

}  // namespace

TakeoffSpeeds takeoff_speeds(const Aircraft& ac, double mass_kg, const RunwayConditions&) {
  TakeoffSpeeds sp;
  // In CAS the stall speed does not depend on altitude.
  const double vs = std::sqrt(2 * mass_kg * kG0 / (kIsaRho0 * ac.wing_area * ac.clmax_takeoff));
  sp.vs_kt = vs / kKt;
  sp.vr_kt = std::max(1.10 * sp.vs_kt, 1.05 * ac.vmcg_kt);
  sp.v2_kt = std::max(1.18 * sp.vs_kt, sp.vr_kt + 8.0);
  return sp;
}

double continued_distance(const Aircraft& ac, const TakeoffInput& in, double vef_kt) {
  const Field f = field(in.rw);
  const TakeoffSpeeds sp = takeoff_speeds(ac, in.mass_kg, in.rw);
  const double vr = sp.vr_kt * kKt * f.tas_per_cas, v2 = sp.v2_kt * kKt * f.tas_per_cas;
  Roll r{ac, f, in.mass_kg, in.dt};
  const Config all{ac.engines, false, ac.cl_ground, ac.cd0_takeoff + ac.k_takeoff * ac.cl_ground * ac.cl_ground,
                   ac.mu_roll};
  Config oei = all;
  oei.operating = ac.engines - 1;
  const bool failure = vef_kt >= 0;
  if (failure) {
    const double vef = std::min(vef_kt * kKt * f.tas_per_cas, vr);
    if (!r.to_speed(all, vef)) return kNever;
    if (!r.to_speed(oei, vr)) return kNever;
  } else if (!r.to_speed(all, vr)) {
    return kNever;
  }
  const Config& c = failure ? oei : all;
  r.for_time(c, ac.rotation_s);
  const double air = air_distance(ac, f, in.mass_kg, std::max(v2, r.va()), c.operating, kScreenTakeoff);
  if (air >= kNever) return kNever;
  return r.s + air;
}

double accelerate_stop_distance(const Aircraft& ac, const TakeoffInput& in, double vef_kt) {
  const Field f = field(in.rw);
  const TakeoffSpeeds sp = takeoff_speeds(ac, in.mass_kg, in.rw);
  const double vr = sp.vr_kt * kKt * f.tas_per_cas;
  const double vef = std::min(vef_kt * kKt * f.tas_per_cas, vr);
  Roll r{ac, f, in.mass_kg, in.dt};
  const Config all{ac.engines, false, ac.cl_ground, ac.cd0_takeoff + ac.k_takeoff * ac.cl_ground * ac.cl_ground,
                   ac.mu_roll};
  if (!r.to_speed(all, vef)) return kNever;
  Config oei = all;
  oei.operating = ac.engines - 1;
  r.for_time(oei, ac.reaction_s);
  // Idle on the live engine, spoilers (no lift, more drag), full braking.
  const Config stop{ac.engines - 1, true, 0.0, ac.cd0_landing, in.rw.wet ? ac.mu_brake_wet : ac.mu_brake_dry};
  if (!r.to_stop(stop)) return kNever;
  return r.s;
}

TakeoffResult takeoff(const Aircraft& ac, const TakeoffInput& in) {
  TakeoffResult res;
  res.speeds = takeoff_speeds(ac, in.mass_kg, in.rw);
  res.tod_all_engines_m = continued_distance(ac, in, -1);

  const double lo = std::min(ac.vmcg_kt, res.speeds.vr_kt), hi = res.speeds.vr_kt;
  auto gap = [&](double v) { return continued_distance(ac, in, v) - accelerate_stop_distance(ac, in, v); };
  double v1;
  if (gap(lo) <= 0) {
    v1 = lo;  // stopping is the problem even at VMCG
  } else if (gap(hi) >= 0) {
    v1 = hi;  // continuing is the problem even at VR
  } else {
    double a = lo, b = hi;
    for (int it = 0; it < 60; ++it) {
      const double mid = 0.5 * (a + b);
      (gap(mid) > 0 ? a : b) = mid;
    }
    v1 = 0.5 * (a + b);
    res.balanced = true;
  }
  res.v1_kt = v1;
  res.tod_oei_m = continued_distance(ac, in, v1);
  res.asd_m = accelerate_stop_distance(ac, in, v1);
  const double tod_req = std::max(1.15 * res.tod_all_engines_m, res.tod_oei_m);
  res.required_m = std::max(tod_req, res.asd_m);
  res.ok = true;
  if (in.mass_kg > ac.mtow) res.ok = false, res.limit = "mass above MTOW";
  else if (tod_req > in.toda_m) res.ok = false, res.limit = "take-off distance";
  else if (res.asd_m > in.asda_m) res.ok = false, res.limit = "accelerate-stop distance";
  else if (res.tod_all_engines_m > in.tora_m) res.ok = false, res.limit = "take-off run";
  return res;
}

LandingResult landing(const Aircraft& ac, const LandingInput& in) {
  LandingResult res;
  const Field f = field(in.rw);
  const double vs = std::sqrt(2 * in.mass_kg * kG0 / (kIsaRho0 * ac.wing_area * ac.clmax_landing));
  res.vref_kt = 1.23 * vs / kKt;
  const double vref = res.vref_kt * kKt * f.tas_per_cas;
  const double ground_over_air = std::max(0.0, (vref - f.vw) / vref);
  const double gamma = ac.descent_angle_deg * kPi / 180.0;
  // 3 degree path from 50 ft, then a 2 s flare bleeding off 3% of the speed.
  const double vtd = 0.97 * vref;
  res.air_m = (kScreenLanding / std::tan(gamma)) * ground_over_air + 0.5 * (vref + vtd) * 2.0 - f.vw * 2.0;
  Roll r{ac, f, in.mass_kg, in.dt};
  r.vg = vtd - f.vw;
  const Config roll{ac.engines, true, 0.3, ac.cd0_landing, ac.mu_roll};
  r.for_time(roll, 1.0);  // derotation, spoilers deploy
  const Config brake{ac.engines, true, 0.0, ac.cd0_landing, in.rw.wet ? ac.mu_brake_wet : ac.mu_brake_dry};
  res.ground_m = r.to_stop(brake) ? r.s : kNever;
  res.actual_m = res.air_m + res.ground_m;
  res.required_m = res.actual_m / 0.6 * (in.rw.wet ? 1.15 : 1.0);
  res.ok = true;
  if (in.mass_kg > ac.mlw) res.ok = false, res.limit = "mass above MLW";
  else if (res.required_m > in.lda_m) res.ok = false, res.limit = "landing distance";
  return res;
}

}  // namespace skybag
