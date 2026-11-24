// Unit conversions and the ICAO standard atmosphere (Doc 7488, up to 20 km).
// Everything inside skybag is SI; feet, knots and nautical miles only appear
// at the edges (inputs, outputs, the web UI).
#pragma once

namespace skybag {

constexpr double kFt = 0.3048;            // m per ft
constexpr double kNm = 1852.0;            // m per nautical mile
constexpr double kKt = 1852.0 / 3600.0;   // m/s per knot
constexpr double kG0 = 9.80665;           // m/s^2
constexpr double kR = 287.05287;          // J/(kg K), dry air
constexpr double kGamma = 1.4;

constexpr double kIsaT0 = 288.15;     // K
constexpr double kIsaP0 = 101325.0;   // Pa
constexpr double kIsaRho0 = 1.225;    // kg/m^3
constexpr double kIsaLapse = 0.0065;  // K/m, troposphere
constexpr double kTropopause = 11000.0;

struct Atmosphere {
  double T = 0;    // K
  double p = 0;    // Pa
  double rho = 0;  // kg/m^3
  double a = 0;    // speed of sound, m/s
};

// Geopotential pressure altitude in metres; delta_t is the deviation from
// ISA temperature in kelvin (a hot day is +20). Pressure follows ISA for the
// pressure altitude, density uses the actual temperature.
Atmosphere isa(double pressure_alt_m, double delta_t = 0.0);

// Pressure altitude of a field from its elevation and QNH (hPa), using the
// standard 27 ft per hPa near sea level, refined with the ISA relation.
double pressure_altitude_m(double elevation_m, double qnh_hpa);

}  // namespace skybag
