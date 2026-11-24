#include <gtest/gtest.h>

#include "skybag/api.hpp"
#include "skybag/http.hpp"
#include "skybag/json.hpp"

using namespace skybag;

TEST(Http, RequestLineAndQuery) {
  Request r;
  ASSERT_TRUE(parse_request_line("GET /api/plan?route=KSFO+KLAX&fl=350&x=%2F HTTP/1.1", &r));
  EXPECT_EQ(r.method, "GET");
  EXPECT_EQ(r.path, "/api/plan");
  EXPECT_EQ(r.get("route"), "KSFO KLAX");
  EXPECT_EQ(r.get("x"), "/");
  EXPECT_DOUBLE_EQ(r.num("fl", 0), 350);
  EXPECT_DOUBLE_EQ(r.num("missing", 7), 7);
  EXPECT_FALSE(parse_request_line("GET nope HTTP/1.1", &r));
  EXPECT_FALSE(parse_request_line("GET /", &r));
}

TEST(Json, EscapesAndNulls) {
  JsonWriter w;
  w.begin_object().kv("s", "a\"b\n").kv("n", 1.5).key("arr").begin_array().value(1).value(2).end_array();
  w.kv("inf", 1.0 / 0.0).end_object();
  EXPECT_EQ(w.str(), "{\"s\":\"a\\\"b\\n\",\"n\":1.5,\"arr\":[1,2],\"inf\":null}");
}

TEST(Api, RouteParsing) {
  App app;
  ASSERT_EQ(load_app(&app, std::string(SKYBAG_SOURCE_DIR) + "/data", std::string(SKYBAG_SOURCE_DIR) + "/web"), "");
  std::vector<Waypoint> r;
  std::string err;
  ASSERT_TRUE(parse_route(app.db, "KSFO, 36.5/-120.25 KLAX", &r, &err)) << err;
  ASSERT_EQ(r.size(), 3u);
  EXPECT_EQ(r[1].name, "N36.50W120.25");
  EXPECT_FALSE(parse_route(app.db, "KSFO NOWHERE", &r, &err));
  EXPECT_FALSE(parse_route(app.db, "KSFO 95/0", &r, &err));

  Request req;
  req.path = "/../CMakeLists.txt";
  EXPECT_EQ(handle(app, req).status, 404);
  req.path = "/api/takeoff";
  req.query = {{"apt", "KSFO"}, {"rwy", "28R"}};  // closed by the demo feed
  EXPECT_EQ(handle(app, req).status, 400);
  req.query = {{"apt", "KSFO"}, {"rwy", "28L"}, {"mass", "70000"}};
  EXPECT_EQ(handle(app, req).status, 200);
}
