// A small HTTP/1.1 server on POSIX sockets for the local web UI: GET only,
// one request per connection, one connection at a time. Binds to 127.0.0.1
// by default. Not meant to face a network.
#pragma once

#include <atomic>
#include <functional>
#include <map>
#include <string>
#include <string_view>

namespace skybag {

struct Request {
  std::string method, path;
  std::map<std::string, std::string> query;

  std::string get(const std::string& k, const std::string& fallback = "") const;
  double num(const std::string& k, double fallback) const;  // fallback if missing or not a number
};

struct Response {
  int status = 200;
  std::string content_type = "application/json";
  std::string body;
};

std::string url_decode(std::string_view s);
std::map<std::string, std::string> parse_query(std::string_view q);
// Parses "GET /path?x=1 HTTP/1.1"; false if malformed.
bool parse_request_line(std::string_view line, Request* out);
std::string status_text(int status);

class HttpServer {
 public:
  using Handler = std::function<Response(const Request&)>;
  ~HttpServer();
  bool listen(const std::string& host, int port, std::string* error);
  int port() const { return port_; }
  // Serves until *stop becomes true (checked between connections, 200 ms poll).
  void serve(const Handler& handler, const std::atomic<bool>* stop = nullptr);

 private:
  int fd_ = -1;
  int port_ = 0;
};

}  // namespace skybag
