// beacon_live against a local fake Limelight MJPEG stream (no hardware here).
#include <gtest/gtest.h>

#include <cstdio>
#include <opencv2/imgcodecs.hpp>

#include "mjpeg.h"
#include "render.h"

using namespace beacon;

namespace {

std::string run(const std::string& cmd, int& code) {
    std::string out;
    FILE* p = popen((cmd + " 2>&1").c_str(), "r");
    char buf[512];
    while (p && fgets(buf, sizeof buf, p)) out += buf;
    int st = p ? pclose(p) : -1;
    code = WIFEXITED(st) ? WEXITSTATUS(st) : -1;
    return out;
}

}  // namespace

TEST(Live, mjpeg_client_reads_fake_stream) {
    SynthOptions o;
    o.traj.durationS = 2;
    o.fps = 10;
    Dataset d;
    std::vector<cv::Mat> imgs;
    std::string err;
    ASSERT_TRUE(makeSynthRun(o, perimeterFieldJson(), d, &imgs, "", err)) << err;
    std::vector<std::vector<uint8_t>> jpegs;
    for (const cv::Mat& m : imgs) {
        std::vector<uint8_t> j;
        cv::imencode(".jpg", m, j);
        jpegs.push_back(j);
    }
    MjpegServer server;
    int port = server.start(jpegs, 30);
    ASSERT_GT(port, 0);
    MjpegClient c;
    ASSERT_TRUE(c.open("http://127.0.0.1:" + std::to_string(port) + "/", 2000, err)) << err;
    for (int i = 0; i < 10; ++i) {
        std::vector<uint8_t> j;
        int64_t t;
        ASSERT_TRUE(c.next(j, t, err)) << err;
        EXPECT_EQ(j, jpegs[i % jpegs.size()]);
    }
}

TEST(Live, beacon_live_runs_on_fake_stream_and_reports_unreachable) {
    SynthOptions o;
    o.traj.durationS = 4;
    o.fps = 15;
    Dataset d;
    std::vector<cv::Mat> imgs;
    std::string err;
    ASSERT_TRUE(makeSynthRun(o, perimeterFieldJson(), d, &imgs, "", err)) << err;
    std::vector<std::vector<uint8_t>> jpegs;
    for (const cv::Mat& m : imgs) {
        std::vector<uint8_t> j;
        cv::imencode(".jpg", m, j, {cv::IMWRITE_JPEG_QUALITY, 90});
        jpegs.push_back(j);
    }
    MjpegServer server;
    int port = server.start(jpegs, 15);
    ASSERT_GT(port, 0);
    std::string exe = std::string(BEACON_BINARY_DIR) + "/beacon_live";
    int code = 0;
    std::string out = run(exe + " --url http://127.0.0.1:" + std::to_string(port) + "/ --no-view --frames 40", code);
    std::printf("%s", out.c_str());
    EXPECT_EQ(code, 0);
    EXPECT_NE(out.find("done: 40 frames received"), std::string::npos);
    // Field mode with the bench field, no odometry (bench-only constant-velocity mode).
    out = run(exe + " --url http://127.0.0.1:" + std::to_string(port) + "/ --no-view --frames 30 --mode field --start " +
                  std::to_string(d.start[0].x) + "," + std::to_string(d.start[0].y) + "," +
                  std::to_string(d.start[0].heading * 180 / M_PI),
              code);
    std::printf("%s", out.c_str());
    EXPECT_EQ(code, 0);
    server.stop();
    out = run(exe + " --url http://127.0.0.1:" + std::to_string(port) + "/ --no-view --frames 5", code);
    EXPECT_EQ(code, 2) << out;
    EXPECT_NE(out.find("not reachable"), std::string::npos) << out;
}
