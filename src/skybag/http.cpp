#include "skybag/http.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <signal.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cctype>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

namespace skybag {

std::string Request::get(const std::string& k, const std::string& fallback) const {
  const auto it = query.find(k);
  return it == query.end() ? fallback : it->second;
}

double Request::num(const std::string& k, double fallback) const {
  const auto it = query.find(k);
  if (it == query.end() || it->second.empty()) return fallback;
  char* end = nullptr;
  const double v = std::strtod(it->second.c_str(), &end);
  return end && *end == '\0' ? v : fallback;
}

std::string url_decode(std::string_view s) {
  std::string r;
  r.reserve(s.size());
  for (std::size_t i = 0; i < s.size(); ++i) {
    if (s[i] == '+') {
      r.push_back(' ');
    } else if (s[i] == '%' && i + 2 < s.size() && std::isxdigit(static_cast<unsigned char>(s[i + 1])) &&
               std::isxdigit(static_cast<unsigned char>(s[i + 2]))) {
      r.push_back(static_cast<char>(std::strtol(std::string(s.substr(i + 1, 2)).c_str(), nullptr, 16)));
      i += 2;
    } else {
      r.push_back(s[i]);
    }
  }
  return r;
}

std::map<std::string, std::string> parse_query(std::string_view q) {
  std::map<std::string, std::string> out;
  std::size_t start = 0;
  while (start <= q.size()) {
    std::size_t amp = q.find('&', start);
    if (amp == std::string_view::npos) amp = q.size();
    const std::string_view part = q.substr(start, amp - start);
    if (!part.empty()) {
      const std::size_t eq = part.find('=');
      if (eq == std::string_view::npos) out[url_decode(part)] = "";
      else out[url_decode(part.substr(0, eq))] = url_decode(part.substr(eq + 1));
    }
    start = amp + 1;
  }
  return out;
}

bool parse_request_line(std::string_view line, Request* out) {
  const std::size_t a = line.find(' ');
  if (a == std::string_view::npos) return false;
  const std::size_t b = line.find(' ', a + 1);
  if (b == std::string_view::npos) return false;
  if (line.substr(b + 1).rfind("HTTP/1.", 0) != 0) return false;
  out->method = std::string(line.substr(0, a));
  const std::string_view target = line.substr(a + 1, b - a - 1);
  if (target.empty() || target[0] != '/') return false;
  const std::size_t qm = target.find('?');
  out->path = url_decode(target.substr(0, qm));
  out->query = qm == std::string_view::npos ? std::map<std::string, std::string>{} : parse_query(target.substr(qm + 1));
  return true;
}

std::string status_text(int status) {
  switch (status) {
    case 200: return "OK";
    case 400: return "Bad Request";
    case 404: return "Not Found";
    case 405: return "Method Not Allowed";
    case 413: return "Payload Too Large";
    default: return "Internal Server Error";
  }
}

HttpServer::~HttpServer() {
  if (fd_ >= 0) ::close(fd_);
}

bool HttpServer::listen(const std::string& host, int port, std::string* error) {
  fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd_ < 0) {
    if (error) *error = std::strerror(errno);
    return false;
  }
  int one = 1;
  ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<std::uint16_t>(port));
  if (::inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
    if (error) *error = "bad host " + host;
    return false;
  }
  if (::bind(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof addr) != 0 || ::listen(fd_, 16) != 0) {
    if (error) *error = std::strerror(errno);
    return false;
  }
  socklen_t len = sizeof addr;
  ::getsockname(fd_, reinterpret_cast<sockaddr*>(&addr), &len);
  port_ = ntohs(addr.sin_port);
  return true;
}

void HttpServer::serve(const Handler& handler, const std::atomic<bool>* stop) {
  ::signal(SIGPIPE, SIG_IGN);
  while (!stop || !stop->load()) {
    pollfd p{fd_, POLLIN, 0};
    if (::poll(&p, 1, 200) <= 0) continue;
    const int c = ::accept(fd_, nullptr, nullptr);
    if (c < 0) continue;
    std::string head;
    char buf[4096];
    bool too_big = false;
    while (head.find("\r\n\r\n") == std::string::npos) {
      pollfd q{c, POLLIN, 0};
      if (::poll(&q, 1, 2000) <= 0) break;
      const ssize_t n = ::recv(c, buf, sizeof buf, 0);
      if (n <= 0) break;
      head.append(buf, static_cast<std::size_t>(n));
      if (head.size() > 16384) {
        too_big = true;
        break;
      }
    }
    Response res;
    Request req;
    const std::size_t eol = head.find("\r\n");
    if (too_big) {
      res = {413, "text/plain", "request too large\n"};
    } else if (eol == std::string::npos || !parse_request_line(std::string_view(head).substr(0, eol), &req)) {
      res = {400, "text/plain", "bad request\n"};
    } else if (req.method != "GET") {
      res = {405, "text/plain", "GET only\n"};
    } else {
      try {
        res = handler(req);
      } catch (const std::exception& e) {
        res = {500, "text/plain", std::string("error: ") + e.what() + "\n"};
      }
    }
    std::string out = "HTTP/1.1 " + std::to_string(res.status) + " " + status_text(res.status) + "\r\n";
    out += "Content-Type: " + res.content_type + "\r\n";
    out += "Content-Length: " + std::to_string(res.body.size()) + "\r\n";
    out += "Cache-Control: no-store\r\nConnection: close\r\n\r\n";
    out += res.body;
    std::size_t sent = 0;
    while (sent < out.size()) {
      const ssize_t n = ::send(c, out.data() + sent, out.size() - sent, 0);
      if (n <= 0) break;
      sent += static_cast<std::size_t>(n);
    }
    ::close(c);
  }
}

}  // namespace skybag
