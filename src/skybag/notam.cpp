#include "skybag/notam.hpp"

#include <cctype>
#include <cstdio>

namespace skybag {

namespace {

// Days from 1970-01-01 to y-m-d (proleptic Gregorian), H. Hinnant's algorithm.
std::int64_t days_from_civil(int y, int m, int d) {
  y -= m <= 2;
  const int era = (y >= 0 ? y : y - 399) / 400;
  const int yoe = y - era * 400;
  const int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
  const int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
  return static_cast<std::int64_t>(era) * 146097 + doe - 719468;
}

int days_in_month(int y, int m) {
  static const int dm[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  const bool leap = (y % 4 == 0 && y % 100 != 0) || y % 400 == 0;
  return m == 2 && leap ? 29 : dm[m - 1];
}

std::string trim(std::string_view s) {
  std::size_t a = 0, b = s.size();
  while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
  while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
  return std::string(s.substr(a, b - a));
}

std::string upper(std::string_view s) {
  std::string r(s);
  for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
  return r;
}

bool runway_designator(const std::string& s) {
  if (s.size() < 2 || s.size() > 3) return false;
  if (!std::isdigit(static_cast<unsigned char>(s[0])) || !std::isdigit(static_cast<unsigned char>(s[1]))) return false;
  const int n = (s[0] - '0') * 10 + (s[1] - '0');
  if (n < 1 || n > 36) return false;
  return s.size() == 2 || s[2] == 'L' || s[2] == 'R' || s[2] == 'C';
}

// Position of an item label like "B)" at the start of a token, from pos.
std::size_t find_label(const std::string& t, char letter, std::size_t from = 0) {
  for (std::size_t i = from; i + 1 < t.size(); ++i) {
    if (t[i] == letter && t[i + 1] == ')' && (i == 0 || std::isspace(static_cast<unsigned char>(t[i - 1]))))
      return i;
  }
  return std::string::npos;
}

}  // namespace

std::int64_t parse_notam_time(std::string_view s) {
  if (s.size() != 10) return -1;
  for (char c : s)
    if (!std::isdigit(static_cast<unsigned char>(c))) return -1;
  auto num = [&](int at) { return (s[at] - '0') * 10 + (s[at + 1] - '0'); };
  const int y = 2000 + num(0), mo = num(2), d = num(4), h = num(6), mi = num(8);
  if (mo < 1 || mo > 12 || d < 1 || d > days_in_month(y, mo) || h > 23 || mi > 59) return -1;
  return days_from_civil(y, mo, d) * 1440 + h * 60 + mi;
}

std::string format_notam_time(std::int64_t minutes) {
  if (minutes == kPermanent) return "PERM";
  std::int64_t days = minutes / 1440;
  int rem = static_cast<int>(minutes % 1440);
  // Inverse of days_from_civil.
  days += 719468;
  const std::int64_t era = (days >= 0 ? days : days - 146096) / 146097;
  const int doe = static_cast<int>(days - era * 146097);
  const int yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
  int y = static_cast<int>(yoe + era * 400);
  const int doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
  const int mp = (5 * doy + 2) / 153;
  const int d = doy - (153 * mp + 2) / 5 + 1;
  const int m = mp + (mp < 10 ? 3 : -9);
  y += m <= 2;
  char buf[16];
  std::snprintf(buf, sizeof buf, "%02d%02d%02d%02d%02d", y % 100, m, d, rem / 60, rem % 60);
  return buf;
}

std::optional<Notam> parse_notam(std::string_view raw, std::string* error) {
  auto fail = [&](const std::string& why) -> std::optional<Notam> {
    if (error) *error = why;
    return std::nullopt;
  };
  Notam n;
  n.raw = trim(raw);
  const std::string t = n.raw;
  {
    std::size_t e = 0;
    while (e < t.size() && !std::isspace(static_cast<unsigned char>(t[e]))) ++e;
    n.id = t.substr(0, e);
    if (n.id.find('/') == std::string::npos) return fail("missing NOTAM id");
  }
  const char order[] = {'Q', 'A', 'B', 'C', 'D', 'E', 'F', 'G'};
  auto field = [&](char letter) -> std::optional<std::string> {
    const std::size_t at = find_label(t, letter);
    if (at == std::string::npos) return std::nullopt;
    std::size_t end = t.size();
    for (char other : order) {
      if (other == letter) continue;
      const std::size_t o = find_label(t, other, at + 2);
      if (o != std::string::npos && o < end) end = o;
    }
    return trim(std::string_view(t).substr(at + 2, end - at - 2));
  };
  const auto a = field('A'), b = field('B'), c = field('C'), e = field('E');
  if (!a || a->empty()) return fail(n.id + ": no A) location");
  if (!b) return fail(n.id + ": no B) start time");
  if (!e) return fail(n.id + ": no E) text");
  n.location = upper(*a);
  n.from_min = parse_notam_time(*b);
  if (n.from_min < 0) return fail(n.id + ": bad B) time '" + *b + "'");
  if (!c || upper(*c) == "PERM") {
    n.to_min = kPermanent;
  } else {
    std::string cv = upper(*c);
    if (cv.size() > 10 && cv.substr(10) == "EST") cv = cv.substr(0, 10);
    n.to_min = parse_notam_time(cv);
    if (n.to_min < 0) return fail(n.id + ": bad C) time '" + *c + "'");
    if (n.to_min <= n.from_min) return fail(n.id + ": C) is not after B)");
  }
  n.text = *e;

  // Runway closures: "RWY 10L/28R CLSD", possibly several RWY groups.
  const std::string ut = upper(n.text);
  if (ut.find("CLSD") != std::string::npos) {
    std::size_t pos = 0;
    while ((pos = ut.find("RWY", pos)) != std::string::npos) {
      pos += 3;
      while (pos < ut.size() && ut[pos] == ' ') ++pos;
      std::size_t end = pos;
      while (end < ut.size() && !std::isspace(static_cast<unsigned char>(ut[end]))) ++end;
      const std::string group = ut.substr(pos, end - pos);
      std::size_t s = 0;
      while (s <= group.size()) {
        std::size_t slash = group.find('/', s);
        if (slash == std::string::npos) slash = group.size();
        const std::string d = group.substr(s, slash - s);
        if (runway_designator(d)) n.closed_runways.push_back(d);
        s = slash + 1;
      }
      pos = end;
    }
  }
  return n;
}

std::vector<Notam> parse_notams(std::string_view text, std::vector<std::string>* errors) {
  std::vector<Notam> out;
  std::string block;
  auto flush = [&] {
    if (trim(block).empty()) {
      block.clear();
      return;
    }
    std::string err;
    if (auto n = parse_notam(block, &err)) out.push_back(std::move(*n));
    else if (errors) errors->push_back(err);
    block.clear();
  };
  std::size_t start = 0;
  while (start <= text.size()) {
    std::size_t nl = text.find('\n', start);
    if (nl == std::string_view::npos) nl = text.size();
    const std::string line = trim(text.substr(start, nl - start));
    if (line.empty()) flush();
    else if (line[0] != '#') block += line + "\n";
    start = nl + 1;
  }
  flush();
  return out;
}

bool active(const Notam& n, std::int64_t t) { return n.from_min <= t && t < n.to_min; }

bool runway_closed(const std::vector<Notam>& notams, std::string_view airport, std::string_view runway_end,
                   std::int64_t t, const Notam** by) {
  const std::string apt = upper(airport), rwy = upper(runway_end);
  for (const Notam& n : notams) {
    if (n.location != apt || !active(n, t)) continue;
    for (const std::string& r : n.closed_runways)
      if (r == rwy) {
        if (by) *by = &n;
        return true;
      }
  }
  return false;
}

}  // namespace skybag
