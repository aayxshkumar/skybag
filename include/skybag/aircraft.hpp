// The SB-20, a made-up 150-seat twin jet. Real performance manuals are
// proprietary, so every number here is invented, sized to be in the right
// family (narrow-body, 74 t MTOW) and documented where it is used. Nothing in
// skybag is for real-world navigation or dispatch.
#pragma once

#include <string>
#include <vector>

namespace skybag {

struct Aircraft {
  std::string name = "SB-20";

  // Masses, kg.
  double oew = 42000, mzfw = 61000, mtow = 74000, mlw = 64500, max_fuel = 18700;

  // Aerodynamics.
  double wing_area = 122.6;     // m^2
  double cd0_clean = 0.0215, k_clean = 0.0410;
  double cd0_takeoff = 0.0420, k_takeoff = 0.0440;   // flaps 2, gear down on the ground
  double cl_ground = 0.55;      // lift coefficient during the ground roll, before rotation
  double clmax_takeoff = 2.20;  // flaps 2
  double clmax_landing = 2.75;  // flaps full
  double cd0_landing = 0.0800, k_landing = 0.0460;   // flaps full, gear down, spoilers out after touchdown: see perf

  // Engines (two). Static sea-level thrust, flat rated to ISA + 15 C.
  int engines = 2;
  double static_thrust = 120000.0;  // N per engine
  double flat_rating_dt = 15.0;     // K above ISA before thrust starts to drop
  double hot_thrust_loss = 0.010;   // fraction per K above the flat-rating temperature
  double idle_thrust = 0.06;        // fraction of static thrust at idle
  double tsfc = 1.65e-5;            // kg/(N s) at cruise (about 0.58 lb/lbf/h)

  // Ground.
  double mu_roll = 0.020;
  double mu_brake_dry = 0.40, mu_brake_wet = 0.20;
  double vmcg_kt = 108.0;           // minimum control speed on the ground (CAS)
  double rotation_s = 3.0;          // rotation time from VR to lift-off
  double reaction_s = 2.0;          // engine failure -> brakes, idle, spoilers (recognition + action)

  // Flight-phase simplifications used by the planner.
  double climb_rate_fpm = 2000, climb_tas_kt = 380, climb_ff = 3600;    // kg/h, both engines
  double descent_angle_deg = 3.0, descent_tas_kt = 380, descent_ff = 650;

  // Weight and balance.
  double lemac_m = 17.80, mac_m = 4.19;       // leading edge of MAC from the datum, MAC length
  double oew_arm_m = 19.20;                   // OEW centre of gravity
  double fuel_arm_m = 18.90;                  // wing tanks (one arm, so fuel burn moves CG along a straight line)

  // Thrust of all operating engines at true airspeed v (m/s) in an
  // atmosphere with density rho and temperature deviation dt.
  double thrust(double v, double rho, double a, double delta_t, int operating) const;
};

const Aircraft& sb20();

struct Station {
  std::string name;
  double arm_m = 0;
  double max_kg = 0;
};

// Payload stations of the SB-20 (passenger zones at 84 kg each, cargo holds).
const std::vector<Station>& sb20_stations();

// Certified CG envelope: (mass kg, %MAC) polygon (closed implicitly, clockwise in this plane).
const std::vector<std::pair<double, double>>& sb20_envelope();

}  // namespace skybag
