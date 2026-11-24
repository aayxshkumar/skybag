// NOTAMs in the ICAO format (Annex 15 / Doc 8126 layout):
//
//   A0412/26 NOTAMN
//   Q) KZOA/QMRLC/IV/NBO/A/000/999/3737N12223W005
//   A) KSFO B) 2611240600 C) 2611251800
//   E) RWY 10L/28R CLSD
//
// Only the fields skybag uses are parsed: the id, the location (A), the
// validity window (B, C; "PERM" for permanent) and the free text (E), from
// which runway closures are recognised. Times are UTC, minutes since 1970.
// The feed in data/notams/ is synthetic.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace skybag {

constexpr std::int64_t kPermanent = INT64_MAX;

struct Notam {
  std::string id, location, text, raw;
  std::int64_t from_min = 0, to_min = 0;  // [from, to)
  std::vector<std::string> closed_runways;  // runway ends closed by this NOTAM ("10L", "28R")
};

// "2611240600" -> minutes since 1970-01-01 00:00 UTC. -1 on a malformed time.
std::int64_t parse_notam_time(std::string_view yymmddhhmm);
std::string format_notam_time(std::int64_t minutes);

std::optional<Notam> parse_notam(std::string_view raw, std::string* error = nullptr);
// NOTAMs separated by blank lines; malformed ones are reported in errors.
std::vector<Notam> parse_notams(std::string_view text, std::vector<std::string>* errors = nullptr);

bool active(const Notam& n, std::int64_t t_min);
// Is this runway end closed at t by any of the NOTAMs for the airport?
bool runway_closed(const std::vector<Notam>& notams, std::string_view airport, std::string_view runway_end,
                   std::int64_t t_min, const Notam** by = nullptr);

}  // namespace skybag
