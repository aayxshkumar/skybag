// Minimal streaming JSON writer for the local API. Non-finite numbers are
// written as null.
#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace skybag {

std::string json_escape(std::string_view s);

class JsonWriter {
 public:
  JsonWriter& begin_object();
  JsonWriter& end_object();
  JsonWriter& begin_array();
  JsonWriter& end_array();
  JsonWriter& key(std::string_view k);
  JsonWriter& value(double v);
  JsonWriter& value(int v) { return value(static_cast<std::int64_t>(v)); }
  JsonWriter& value(std::int64_t v);
  JsonWriter& value(std::uint64_t v);
  JsonWriter& value(bool v);
  JsonWriter& value(std::string_view v);
  JsonWriter& value(const char* v) { return value(std::string_view(v)); }
  JsonWriter& null();

  // key + value in one call.
  template <class T>
  JsonWriter& kv(std::string_view k, const T& v) {
    key(k);
    return value(v);
  }

  const std::string& str() const { return out_; }

 private:
  void sep();
  std::string out_;
  std::vector<bool> first_;
  bool after_key_ = false;
};

}  // namespace skybag
