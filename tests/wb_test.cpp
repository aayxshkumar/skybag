#include <gtest/gtest.h>

#include "skybag/wb.hpp"

using namespace skybag;

TEST(PointInPolygon, SquareInsideOutsideEdges) {
  const std::vector<std::pair<double, double>> sq = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
  EXPECT_TRUE(point_in_polygon(sq, 5, 5));
  EXPECT_FALSE(point_in_polygon(sq, 11, 5));
  EXPECT_FALSE(point_in_polygon(sq, -0.001, 5));
  EXPECT_TRUE(point_in_polygon(sq, 0, 5));    // edge
  EXPECT_TRUE(point_in_polygon(sq, 10, 10));  // vertex
}

TEST(WeightAndBalance, TypicalLoadIsInside) {
  const std::vector<LoadItem> load = {{"crew", 400},           {"pax-forward", 40 * 84.0}, {"pax-mid", 45 * 84.0},
                                      {"pax-aft", 40 * 84.0},  {"cargo-forward", 1500},    {"cargo-aft", 1800}};
  const WbResult r = weight_and_balance(sb20(), load, 9000, 200, 6000);
  EXPECT_TRUE(r.ok) << (r.problems.empty() ? "" : r.problems.front());
  ASSERT_EQ(r.points.size(), 3u);
  EXPECT_GT(r.points[1].mass_kg, r.points[0].mass_kg);
}

TEST(WeightAndBalance, AllCargoAftIsOutside) {
  const std::vector<LoadItem> load = {{"pax-aft", 50 * 84.0}, {"cargo-aft", 4500}};
  const WbResult r = weight_and_balance(sb20(), load, 6000, 200, 3000);
  EXPECT_FALSE(r.ok);
}

TEST(WeightAndBalance, StationLimit) {
  const WbResult r = weight_and_balance(sb20(), {{"cargo-forward", 9999}}, 5000, 200, 2000);
  EXPECT_FALSE(r.ok);
}
