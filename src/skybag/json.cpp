#include "skybag/json.hpp"

#include <cmath>
#include <cstdio>

namespace skybag {

std::string json_escape(std::string_view s) {
  std::string r;
  r.reserve(s.size() + 2);
  for (unsigned char c : s) {
    switch (c) {
      case '"': r += "\\\""; break;
      case '\\': r += "\\\\"; break;
      case '\n': r += "\\n"; break;
      case '\r': r += "\\r"; break;
      case '\t': r += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof buf, "\\u%04x", c);
          r += buf;
        } else {
          r.push_back(static_cast<char>(c));
        }
    }
  }
  return r;
}

void JsonWriter::sep() {
  if (after_key_) {
    after_key_ = false;
    return;
  }
  if (!first_.empty()) {
    if (!first_.back()) out_ += ',';
    first_.back() = false;
  }
}

JsonWriter& JsonWriter::begin_object() {
  sep();
  out_ += '{';
  first_.push_back(true);
  return *this;
}

JsonWriter& JsonWriter::end_object() {
  out_ += '}';
  first_.pop_back();
  return *this;
}

JsonWriter& JsonWriter::begin_array() {
  sep();
  out_ += '[';
  first_.push_back(true);
  return *this;
}

JsonWriter& JsonWriter::end_array() {
  out_ += ']';
  first_.pop_back();
  return *this;
}

JsonWriter& JsonWriter::key(std::string_view k) {
  sep();
  out_ += '"';
  out_ += json_escape(k);
  out_ += "\":";
  after_key_ = true;
  return *this;
}

JsonWriter& JsonWriter::value(double v) {
  sep();
  if (!std::isfinite(v)) {
    out_ += "null";
    return *this;
  }
  char buf[32];
  std::snprintf(buf, sizeof buf, "%.10g", v);
  out_ += buf;
  return *this;
}

JsonWriter& JsonWriter::value(std::int64_t v) {
  sep();
  out_ += std::to_string(v);
  return *this;
}

JsonWriter& JsonWriter::value(std::uint64_t v) {
  sep();
  out_ += std::to_string(v);
  return *this;
}

JsonWriter& JsonWriter::value(bool v) {
  sep();
  out_ += v ? "true" : "false";
  return *this;
}

JsonWriter& JsonWriter::value(std::string_view v) {
  sep();
  out_ += '"';
  out_ += json_escape(v);
  out_ += '"';
  return *this;
}

JsonWriter& JsonWriter::null() {
  sep();
  out_ += "null";
  return *this;
}

}  // namespace skybag
