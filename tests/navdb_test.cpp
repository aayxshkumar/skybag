#include <gtest/gtest.h>

#include "skybag/navdb.hpp"
#include "skybag/units.hpp"

using namespace skybag;

namespace {
const NavDb& db() {
  static const NavDb d = NavDb::load(std::string(SKYBAG_SOURCE_DIR) + "/data/ourairports");
  return d;
}
}  // namespace

TEST(Csv, QuotesCommasAndNewlines) {
  const auto rows = parse_csv("a,\"b,c\",\"say \"\"hi\"\"\"\n1,,\"two\nlines\"\r\n");
  ASSERT_EQ(rows.size(), 2u);
  ASSERT_EQ(rows[0].size(), 3u);
  EXPECT_EQ(rows[0][1], "b,c");
  EXPECT_EQ(rows[0][2], "say \"hi\"");
  EXPECT_EQ(rows[1][1], "");
  EXPECT_EQ(rows[1][2], "two\nlines");
}

TEST(Csv, LastLineWithoutNewline) {
  const auto rows = parse_csv("x,y\n1,2");
  ASSERT_EQ(rows.size(), 2u);
  EXPECT_EQ(rows[1][1], "2");
}

TEST(NavDb, LoadsAndSkipsHeliports) {
  EXPECT_GT(db().airports().size(), 30000u);
  EXPECT_EQ(db().find("00A"), nullptr);  // a heliport in the dump
  for (const Airport& a : db().airports()) ASSERT_NE(a.type, "heliport");
}

TEST(NavDb, KsfoRunways) {
  const Airport* a = db().find("ksfo");
  ASSERT_NE(a, nullptr);
  EXPECT_EQ(a->runways.size(), 4u);
  RunwayDistances d;
  ASSERT_TRUE(NavDb::distances(*a, "28L", &d));
  EXPECT_GT(d.tora_m, 3000);
  EXPECT_NEAR(d.heading_true, 298, 2);
  EXPECT_FALSE(NavDb::distances(*a, "36", &d));
}

TEST(NavDb, WithinIsSortedAndBounded) {
  const Airport* a = db().find("EGLL");
  ASSERT_NE(a, nullptr);
  const auto near = db().within(a->pos, 30 * kNm);
  ASSERT_FALSE(near.empty());
  EXPECT_EQ(near.front()->ident, "EGLL");
  double prev = 0;
  for (const Airport* b : near) {
    const double d = inverse(a->pos, b->pos).dist_m;
    EXPECT_LE(d, 30 * kNm);
    EXPECT_GE(d, prev);
    prev = d;
  }
}

TEST(NavDb, SearchFindsByIataAndName) {
  const auto r = db().search("LHR", 5);
  ASSERT_FALSE(r.empty());
  EXPECT_EQ(r.front()->ident, "EGLL");
  const auto n = db().search("heathrow", 5);
  ASSERT_FALSE(n.empty());
}
