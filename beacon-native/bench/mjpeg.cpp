#include "mjpeg.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstring>

namespace beacon {

int64_t monotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

// ---------------------------------------------------------------------------------- client

MjpegClient::~MjpegClient() { close(); }

void MjpegClient::close() {
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
    buf_.clear();
}

static bool parseUrl(const std::string& url, std::string& host, std::string& port, std::string& path) {
    const std::string pre = "http://";
    if (url.compare(0, pre.size(), pre) != 0) return false;
    std::string rest = url.substr(pre.size());
    size_t slash = rest.find('/');
    std::string hp = slash == std::string::npos ? rest : rest.substr(0, slash);
    path = slash == std::string::npos ? "/" : rest.substr(slash);
    size_t colon = hp.find(':');
    host = colon == std::string::npos ? hp : hp.substr(0, colon);
    port = colon == std::string::npos ? "80" : hp.substr(colon + 1);
    return !host.empty();
}

bool MjpegClient::open(const std::string& url, int timeoutMs, std::string& err) {
    close();
    timeoutMs_ = timeoutMs;
    std::string host, port, path;
    if (!parseUrl(url, host, port, path)) {
        err = "bad URL (expected http://host:port/path): " + url;
        return false;
    }
    addrinfo hints{}, *res = nullptr;
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
    if (rc != 0) {
        err = "cannot resolve " + host + ": " + gai_strerror(rc) +
              " (is the Limelight plugged in? try its IP address instead of limelight.local)";
        return false;
    }
    for (addrinfo* a = res; a != nullptr; a = a->ai_next) {
        int fd = socket(a->ai_family, a->ai_socktype, a->ai_protocol);
        if (fd < 0) continue;
        int flags = fcntl(fd, F_GETFL, 0);
        fcntl(fd, F_SETFL, flags | O_NONBLOCK);
        rc = connect(fd, a->ai_addr, a->ai_addrlen);
        if (rc != 0 && errno == EINPROGRESS) {
            pollfd p{fd, POLLOUT, 0};
            if (poll(&p, 1, timeoutMs) == 1) {
                int so = 0;
                socklen_t len = sizeof so;
                getsockopt(fd, SOL_SOCKET, SO_ERROR, &so, &len);
                rc = so == 0 ? 0 : -1;
            } else {
                rc = -1;
            }
        }
        if (rc == 0) {
            fcntl(fd, F_SETFL, flags);
            fd_ = fd;
            break;
        }
        ::close(fd);
    }
    freeaddrinfo(res);
    if (fd_ < 0) {
        err = "cannot connect to " + host + ":" + port + " within " + std::to_string(timeoutMs) + " ms";
        return false;
    }
    int one = 1;
    setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
#ifdef SO_NOSIGPIPE
    setsockopt(fd_, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
    std::string req = "GET " + path + " HTTP/1.1\r\nHost: " + host + "\r\nConnection: keep-alive\r\n\r\n";
    if (send(fd_, req.data(), req.size(), 0) != (ssize_t)req.size()) {
        err = "cannot send HTTP request";
        close();
        return false;
    }
    while (buf_.find("\r\n\r\n") == std::string::npos) {
        if (!fill(err)) return false;
    }
    size_t end = buf_.find("\r\n\r\n");
    std::string head = buf_.substr(0, end);
    buf_.erase(0, end + 4);
    if (head.find(" 200") == std::string::npos) {
        err = "HTTP error: " + head.substr(0, head.find("\r\n"));
        close();
        return false;
    }
    size_t b = head.find("boundary=");
    if (b != std::string::npos) {
        size_t e = head.find_first_of("\r\n;", b);
        boundary_ = head.substr(b + 9, e == std::string::npos ? std::string::npos : e - b - 9);
    }
    return true;
}

bool MjpegClient::fill(std::string& err) {
    if (fd_ < 0) {
        err = "not connected";
        return false;
    }
    pollfd p{fd_, POLLIN, 0};
    int rc = poll(&p, 1, timeoutMs_);
    if (rc <= 0) {
        err = "stream timed out (no data for " + std::to_string(timeoutMs_) + " ms)";
        return false;
    }
    char tmp[65536];
    ssize_t n = recv(fd_, tmp, sizeof tmp, 0);
    if (n <= 0) {
        err = "stream closed by the server";
        return false;
    }
    buf_.append(tmp, (size_t)n);
    return true;
}

bool MjpegClient::next(std::vector<uint8_t>& jpeg, int64_t& recvNs, std::string& err) {
    // JPEG start (FFD8) and end (FFD9) markers; FF bytes inside entropy-coded data are stuffed,
    // so FFD9 marks the end of a baseline MJPEG frame. Content-Length is not required.
    for (;;) {
        size_t soi = buf_.find("\xFF\xD8");
        if (soi != std::string::npos) {
            size_t eoi = buf_.find("\xFF\xD9", soi + 2);
            if (eoi != std::string::npos) {
                jpeg.assign(buf_.begin() + (long)soi, buf_.begin() + (long)eoi + 2);
                buf_.erase(0, eoi + 2);
                recvNs = monotonicNs();
                return true;
            }
            if (soi > 0) buf_.erase(0, soi);
        } else if (buf_.size() > 1) {
            buf_.erase(0, buf_.size() - 1);
        }
        if (buf_.size() > (64u << 20)) {
            err = "frame larger than 64 MB";
            return false;
        }
        if (!fill(err)) return false;
    }
}

// ---------------------------------------------------------------------------------- server

MjpegServer::~MjpegServer() { stop(); }

int MjpegServer::start(const std::vector<std::vector<uint8_t>>& jpegs, double fps, int port, bool loop) {
    jpegs_ = jpegs;
    fps_ = fps;
    loop_ = loop;
    listenFd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listenFd_ < 0) return -1;
    int one = 1;
    setsockopt(listenFd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);
    if (bind(listenFd_, (sockaddr*)&addr, sizeof addr) != 0 || listen(listenFd_, 4) != 0) {
        ::close(listenFd_);
        listenFd_ = -1;
        return -1;
    }
    socklen_t len = sizeof addr;
    getsockname(listenFd_, (sockaddr*)&addr, &len);
    stop_ = false;
    thread_ = std::thread([this] { run(); });
    return ntohs(addr.sin_port);
}

void MjpegServer::stop() {
    stop_ = true;
    if (listenFd_ >= 0) {
        shutdown(listenFd_, SHUT_RDWR);
        ::close(listenFd_);
        listenFd_ = -1;
    }
    if (thread_.joinable()) thread_.join();
}

void MjpegServer::run() {
    while (!stop_) {
        pollfd p{listenFd_, POLLIN, 0};
        if (poll(&p, 1, 100) <= 0) continue;
        int c = accept(listenFd_, nullptr, nullptr);
        if (c < 0) continue;
        int one = 1;
#ifdef SO_NOSIGPIPE
        setsockopt(c, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof one);
#endif
        (void)one;
        char req[2048];
        recv(c, req, sizeof req, 0);
        std::string head =
            "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=beaconframe\r\n\r\n";
        bool ok = send(c, head.data(), head.size(), 0) > 0;
        auto period = std::chrono::nanoseconds((int64_t)(1e9 / fps_));
        auto next = std::chrono::steady_clock::now();
        size_t i = 0;
        while (ok && !stop_ && !jpegs_.empty()) {
            if (i == jpegs_.size()) {
                if (!loop_) break;
                i = 0;
            }
            const auto& j = jpegs_[i++];
            std::string part = "--beaconframe\r\nContent-Type: image/jpeg\r\nContent-Length: " +
                               std::to_string(j.size()) + "\r\n\r\n";
            ok = send(c, part.data(), part.size(), 0) > 0 && send(c, j.data(), j.size(), 0) > 0 &&
                 send(c, "\r\n", 2, 0) > 0;
            next += period;
            std::this_thread::sleep_until(next);
        }
        ::close(c);
    }
}

}  // namespace beacon
