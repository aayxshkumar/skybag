#include <gtest/gtest.h>

#include <cmath>

#include "skybag/geo.hpp"
#include "skybag/units.hpp"
#include "skybag/wind.hpp"

using namespace skybag;

TEST(Geo, WrapAngles) {
  EXPECT_DOUBLE_EQ(wrap180(190), -170);
  EXPECT_DOUBLE_EQ(wrap180(-180), -180);
  EXPECT_DOUBLE_EQ(wrap360(-10), 350);
  EXPECT_DOUBLE_EQ(wrap360(720), 0);
}

// Vincenty's published example, Flinders Peak -> Buninyong.
TEST(Geo, VincentyExample) {
  const LatLon a{-(37 + 57 / 60.0 + 3.72030 / 3600), 144 + 25 / 60.0 + 29.52440 / 3600};
  const LatLon b{-(37 + 39 / 60.0 + 10.15610 / 3600), 143 + 55 / 60.0 + 35.38390 / 3600};
  const Inverse g = inverse(a, b);
  ASSERT_TRUE(g.ok);
  EXPECT_NEAR(g.dist_m, 54972.271, 1e-3);
  const LatLon c = direct(a, g.az1_deg, g.dist_m);
  EXPECT_NEAR(c.lat, b.lat, 1e-9);
  EXPECT_NEAR(c.lon, b.lon, 1e-9);
}

TEST(Geo, DirectCrossesTheAntimeridian) {
  const LatLon p = direct({0, 179.9}, 90, 50000);
  EXPECT_LT(p.lon, -179.0);
  EXPECT_GE(p.lon, -180.0);
}

TEST(Geo, DensifyKeepsEndsAndSpacing) {
  const LatLon a{40.64, -73.78}, b{51.47, -0.45};
  const auto pts = densify(a, b, 50 * kNm);
  ASSERT_GE(pts.size(), 3u);
  EXPECT_EQ(pts.front().lat, a.lat);
  EXPECT_EQ(pts.back().lon, b.lon);
  const double step = inverse(pts[0], pts[1]).dist_m;
  for (std::size_t k = 1; k + 1 < pts.size(); ++k) EXPECT_NEAR(inverse(pts[k], pts[k + 1]).dist_m, step, 1e-3);
}

TEST(Units, PressureAltitudeFromQnh) {
  EXPECT_NEAR(pressure_altitude_m(0, 1013.25), 0, 1e-9);
  // About 27-28 ft per hPa near sea level.
  EXPECT_NEAR(pressure_altitude_m(0, 1003.25) / kFt, 273, 5);
}

TEST(Wind, RunwayComponents) {
  double head = 0, cross = 0;
  runway_components(280, 280, 20, &head, &cross);
  EXPECT_NEAR(head, 20, 1e-12);
  EXPECT_NEAR(cross, 0, 1e-12);
  runway_components(280, 310, 20, &head, &cross);
  EXPECT_NEAR(head, 20 * std::cos(30 * 3.14159265358979323846 / 180), 1e-12);
  EXPECT_NEAR(cross, 10, 1e-12);  // from the right
}

TEST(Wind, FieldIsDeterministicAndSeeded) {
  const WindField a(1), b(1), c(2);
  const Wind wa = a.at({45, -30}, 35000), wb = b.at({45, -30}, 35000), wc = c.at({45, -30}, 35000);
  EXPECT_EQ(wa.speed_kt, wb.speed_kt);
  EXPECT_NE(wa.speed_kt, wc.speed_kt);
  // The jet is westerly in mid-latitudes at FL340.
  EXPECT_GT(wa.u_kt, 40);
}
