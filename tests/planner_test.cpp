#include <gtest/gtest.h>

#include "skybag/planner.hpp"
#include "skybag/track.hpp"
#include "skybag/units.hpp"

using namespace skybag;

namespace {
PlanInput sfo_lax() {
  PlanInput in;
  in.route = {{"KSFO", {37.618999, -122.375}}, {"KLAX", {33.942501, -118.407997}}};
  return in;
}
}  // namespace

TEST(Planner, FuelAddsUp) {
  const Plan p = plan(sb20(), WindField(1), sfo_lax());
  ASSERT_TRUE(p.ok) << p.error;
  EXPECT_NEAR(p.block_kg, p.trip_kg + p.contingency_kg + p.reserve_kg + p.alternate_kg + p.taxi_kg, 0.05);
  EXPECT_NEAR(p.contingency_kg, 0.05 * p.trip_kg, 1e-6);
  double legs = 0;
  for (const LegPlan& l : p.legs) legs += l.fuel_kg;
  EXPECT_NEAR(legs, p.trip_kg, 1e-6);
}

TEST(Planner, FinalReserveIsThirtyMinutesHolding) {
  const Plan p = plan(sb20(), WindField(1), sfo_lax());
  EXPECT_NEAR(p.reserve_kg, holding_fuel_flow(sb20(), p.ldw_kg) * 0.5, 0.01);
}

TEST(Planner, ShortRouteLowersCruise) {
  PlanInput in;
  in.route = {{"A", {40.0, -100.0}}, {"B", {40.0, -98.0}}};
  const Plan p = plan(sb20(), WindField(1), in);
  ASSERT_TRUE(p.ok) << p.error;
  EXPECT_LT(p.cruise_ft, 35000);
  EXPECT_LT(p.toc_nm, p.tod_nm);
}

TEST(Planner, NoWindMeansGroundSpeedIsAirspeed) {
  const Plan p = plan(sb20(), WindField(1, 0.0), sfo_lax());
  for (const LegPlan& l : p.legs) EXPECT_NEAR(l.gs_kt, l.tas_kt, 1e-9);
}

TEST(Planner, RejectsOneWaypoint) {
  PlanInput in;
  in.route = {{"A", {0, 0}}};
  EXPECT_FALSE(plan(sb20(), WindField(1), in).ok);
}

TEST(Track, EndsNearTheDestination) {
  const PlanInput in = sfo_lax();
  const Plan p = plan(sb20(), WindField(1), in);
  const auto fixes = simulate_track(p, in.route, 1.0, 7, 0.0);
  ASSERT_FALSE(fixes.empty());
  EXPECT_LT(inverse(fixes.back().pos, in.route.back().pos).dist_m, 2 * kNm);
  EXPECT_NEAR(fixes.back().t_s, p.time_min * 60, 1.0);
}
