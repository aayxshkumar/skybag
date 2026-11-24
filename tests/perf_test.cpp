#include <gtest/gtest.h>

#include "skybag/perf.hpp"

using namespace skybag;

TEST(Takeoff, SpeedsAreOrdered) {
  const TakeoffSpeeds s = takeoff_speeds(sb20(), 70000, {});
  EXPECT_LT(s.vs_kt, s.vr_kt);
  EXPECT_LT(s.vr_kt, s.v2_kt);
  EXPECT_GE(s.vr_kt, 1.05 * sb20().vmcg_kt);
}

TEST(Takeoff, BalancedAtMtowSeaLevel) {
  TakeoffInput in;
  in.mass_kg = 74000;
  const TakeoffResult r = takeoff(sb20(), in);
  EXPECT_TRUE(r.balanced);
  EXPECT_GE(r.v1_kt, sb20().vmcg_kt);
  EXPECT_LE(r.v1_kt, r.speeds.vr_kt);
  EXPECT_NEAR(r.tod_oei_m, r.asd_m, 1e-3 * r.required_m);
  EXPECT_GT(r.required_m, 1200);
  EXPECT_LT(r.required_m, 3000);
  EXPECT_TRUE(r.ok);
}

TEST(Takeoff, ShortRunwayIsLimited) {
  TakeoffInput in;
  in.mass_kg = 74000;
  in.tora_m = in.toda_m = in.asda_m = 1200;
  const TakeoffResult r = takeoff(sb20(), in);
  EXPECT_FALSE(r.ok);
  EXPECT_FALSE(r.limit.empty());
}

TEST(Takeoff, WetStopsLonger) {
  TakeoffInput dry;
  dry.mass_kg = 70000;
  TakeoffInput wet = dry;
  wet.rw.wet = true;
  EXPECT_GT(accelerate_stop_distance(sb20(), wet, 130), accelerate_stop_distance(sb20(), dry, 130));
}

TEST(Takeoff, HeadwindHelps) {
  TakeoffInput calm;
  calm.mass_kg = 70000;
  TakeoffInput head = calm;
  head.rw.headwind_kt = 15;
  EXPECT_LT(takeoff(sb20(), head).required_m, takeoff(sb20(), calm).required_m);
}

TEST(Landing, WetNeedsMore) {
  LandingInput in;
  in.mass_kg = 60000;
  const double dry = landing(sb20(), in).required_m;
  in.rw.wet = true;
  // Wet braking is weaker and the regulatory factor adds 15% on top.
  EXPECT_GT(landing(sb20(), in).required_m, 1.15 * dry);
}
