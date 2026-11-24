// Minimal MJPEG-over-HTTP client (Limelight stream, http://limelight.local:5800) and a tiny
// local MJPEG server used by tests and for replaying recordings as a fake Limelight. Bench only.
#pragma once

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

namespace beacon {

class MjpegClient {
public:
    ~MjpegClient();
    /** Connects and reads the HTTP response headers. False with a message on failure. */
    bool open(const std::string& url, int timeoutMs, std::string& err);
    /** Next JPEG image. recvNs: steady-clock time when its last byte arrived. */
    bool next(std::vector<uint8_t>& jpeg, int64_t& recvNs, std::string& err);
    void close();

private:
    bool fill(std::string& err);  // reads more bytes into buf_
    int fd_ = -1;
    int timeoutMs_ = 2000;
    std::string buf_;
    std::string boundary_;
};

/** Serves a list of JPEG images as multipart/x-mixed-replace on 127.0.0.1:port, looping. */
class MjpegServer {
public:
    ~MjpegServer();
    /** port 0 = any free port; returns the bound port or -1. */
    int start(const std::vector<std::vector<uint8_t>>& jpegs, double fps, int port = 0, bool loop = true);
    void stop();

private:
    void run();
    std::vector<std::vector<uint8_t>> jpegs_;
    double fps_ = 30;
    bool loop_ = true;
    int listenFd_ = -1;
    std::atomic<bool> stop_{false};
    std::thread thread_;
};

/** Steady (monotonic) clock in nanoseconds, the bench's stand-in for elapsedRealtimeNanos. */
int64_t monotonicNs();

}  // namespace beacon
