#include <gtest/gtest.h>

#include <fstream>
#include <sstream>

#include "skybag/notam.hpp"

using namespace skybag;

TEST(NotamTime, Rejects) {
  EXPECT_EQ(parse_notam_time("abcdefghij"), -1);
}

TEST(NotamTime, RoundTrips) {
  for (const char* t : {"2611240600", "2402290000", "2612312359", "2601010000"})
    EXPECT_EQ(format_notam_time(parse_notam_time(t)), t);
  EXPECT_EQ(parse_notam_time("2602290000"), -1);  // 2026 is not a leap year
  EXPECT_EQ(parse_notam_time("2613010000"), -1);
  EXPECT_EQ(parse_notam_time("261124060"), -1);
  EXPECT_EQ(parse_notam_time("2611242400"), -1);
  EXPECT_EQ(parse_notam_time("2611250000") - parse_notam_time("2611240000"), 1440);
}

TEST(Notam, RunwayClosure) {
  const auto n = parse_notam("A0412/26 NOTAMN A) KSFO B) 2611240600 C) 2611251800 E) RWY 10L/28R CLSD");
  ASSERT_TRUE(n.has_value());
  EXPECT_EQ(n->id, "A0412/26");
  EXPECT_EQ(n->location, "KSFO");
  ASSERT_EQ(n->closed_runways.size(), 2u);
  const std::vector<Notam> all{*n};
  EXPECT_TRUE(runway_closed(all, "KSFO", "28R", parse_notam_time("2611241200")));
  EXPECT_FALSE(runway_closed(all, "KSFO", "28L", parse_notam_time("2611241200")));
  EXPECT_FALSE(runway_closed(all, "KSFO", "28R", parse_notam_time("2611251800")));  // C) is exclusive
  EXPECT_TRUE(runway_closed(all, "KSFO", "28R", parse_notam_time("2611240600")));
}

TEST(Notam, MissingFieldsAreErrors) {
  std::string err;
  EXPECT_FALSE(parse_notam("A1/26 NOTAMN B) 2611240600 E) RWY 01 CLSD", &err));
  EXPECT_NE(err.find("A)"), std::string::npos);
  EXPECT_FALSE(parse_notam("NOTAMN A) KSFO B) 2611240600 E) X", &err));
}

TEST(Notam, DemoFeedParsesCleanly) {
  std::ifstream f(std::string(SKYBAG_SOURCE_DIR) + "/data/notams/notams.txt");
  std::stringstream ss;
  ss << f.rdbuf();
  std::vector<std::string> errors;
  const auto all = parse_notams(ss.str(), &errors);
  EXPECT_TRUE(errors.empty()) << errors.front();
  EXPECT_GE(all.size(), 10u);
}
