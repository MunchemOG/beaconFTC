// C1: the C API contract (I3).
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <thread>
#include <vector>

#include "beacon/beacon.h"
#include "engine.h"
#include "render.h"

using namespace beacon;

namespace {

beacon_camera cam() { return benchCamera(640, 480, 54.5); }
beacon_mount mount() { return beacon_mount{0.15, 0.0, 0.35, 0.0, 8 * M_PI / 180, 0.0}; }

beacon_ctx* makeCtx(const char* params = nullptr) {
    beacon_camera c = cam();
    beacon_mount m = mount();
    char err[256] = {0};
    std::string field = perimeterFieldJson();
    beacon_ctx* ctx = beacon_create(field.c_str(), &c, &m, params, err, sizeof err);
    EXPECT_NE(ctx, nullptr) << err;
    return ctx;
}

}  // namespace

TEST(Api, api_rejects_bad_field_json) {
    beacon_camera c = cam();
    beacon_mount m = mount();
    char err[256] = {0};
    EXPECT_EQ(beacon_create("{not json", &c, &m, nullptr, err, sizeof err), nullptr);
    EXPECT_NE(std::string(err).find("field model"), std::string::npos) << err;
    err[0] = 0;
    EXPECT_EQ(beacon_create("{\"schema\": 1, \"floor\": {\"halfSizeM\": 1.8288, \"tilePitchM\": 0.6096}, \"walls\": []}",
                            &c, &m, nullptr, err, sizeof err),
              nullptr);
    EXPECT_NE(std::string(err).find("walls"), std::string::npos) << err;
    err[0] = 0;
    EXPECT_EQ(beacon_create(nullptr, &c, &m, nullptr, err, sizeof err), nullptr);  // field mode needs a field
    EXPECT_NE(std::string(err).find("field"), std::string::npos) << err;
}

TEST(Api, rejects_bad_params_and_camera) {
    beacon_camera c = cam();
    beacon_mount m = mount();
    std::string field = perimeterFieldJson();
    char err[256] = {0};
    EXPECT_EQ(beacon_create(field.c_str(), &c, &m, "{\"noSuchKey\": 1}", err, sizeof err), nullptr);
    EXPECT_NE(std::string(err).find("noSuchKey"), std::string::npos) << err;
    c.fx = NAN;
    EXPECT_EQ(beacon_create(field.c_str(), &c, &m, nullptr, err, sizeof err), nullptr);
    c = cam();
    m.pitch = INFINITY;
    EXPECT_EQ(beacon_create(field.c_str(), &c, &m, nullptr, err, sizeof err), nullptr);
    // Tiny error buffers and NULL are fine.
    EXPECT_EQ(beacon_create("x", &c, &m, nullptr, err, 1), nullptr);
    EXPECT_EQ(err[0], '\0');
    EXPECT_EQ(beacon_create("x", &c, &m, nullptr, nullptr, 0), nullptr);
}

TEST(Api, null_context_is_harmless) {
    beacon_pose2 p{0, 0, 0, 0};
    beacon_set_start_pose(nullptr, &p);
    beacon_push_odom(nullptr, &p);
    uint8_t px = 0;
    EXPECT_EQ(beacon_push_frame(nullptr, 0, &px, 1, 1, 1), BEACON_ERR);
    beacon_fix f;
    EXPECT_EQ(beacon_latest_fix(nullptr, &f), 0);
    EXPECT_EQ(beacon_status(nullptr), BEACON_STATUS_INVALID);
    char buf[64];
    beacon_debug_line(nullptr, buf, sizeof buf);
    EXPECT_STREQ(buf, "beacon: no context");
    beacon_destroy(nullptr);
}

TEST(Api, no_fix_without_frames) {
    beacon_ctx* ctx = makeCtx();
    beacon_pose2 s{1000, 0.5, 0.5, 0.0};
    beacon_set_start_pose(ctx, &s);
    for (int i = 0; i < 200; ++i) {
        beacon_pose2 o{1000 + i * 5000000LL, 0.001 * i, 0, 0};
        beacon_push_odom(ctx, &o);
    }
    beacon_fix f;
    EXPECT_EQ(beacon_latest_fix(ctx, &f), 0);
    EXPECT_EQ(beacon_status(ctx), BEACON_STATUS_INITIALIZING);
    char buf[256];
    beacon_debug_line(ctx, buf, sizeof buf);
    EXPECT_GT(std::strlen(buf), 0u);
    beacon_destroy(ctx);
}

TEST(Api, nonfinite_odom_ignored) {
    beacon_ctx* ctx = makeCtx("{\"synchronous\": true}");
    std::vector<VizFrame> seen;
    beacon_set_observer(ctx, [&](const VizFrame& v) { seen.push_back(v); });
    beacon_pose2 s{0, 0.0, 0.0, 0.0};
    beacon_set_start_pose(ctx, &s);
    for (int i = 0; i <= 100; ++i) {
        beacon_pose2 o{i * 10000000LL, 0.01 * i, 0.0, 0.0};  // 1 m/s along x
        beacon_push_odom(ctx, &o);
    }
    const double bad[] = {NAN, INFINITY, -INFINITY};
    for (double b : bad) {
        beacon_pose2 o1{1005000000LL, b, 0.0, 0.0}, o2{1005000000LL, 0.0, b, 0.0}, o3{1005000000LL, 0.0, 0.0, b};
        beacon_push_odom(ctx, &o1);
        beacon_push_odom(ctx, &o2);
        beacon_push_odom(ctx, &o3);
    }
    // Start pose with NaN is ignored too.
    beacon_pose2 badStart{0, NAN, 0, 0};
    beacon_set_start_pose(ctx, &badStart);
    // A frame at t = 0.5 s: the predicted pose must be exactly the clean odometry (x = 0.5).
    std::vector<uint8_t> img(640 * 480, 128);
    ASSERT_EQ(beacon_push_frame(ctx, 500000000LL, img.data(), 640, 480, 640), BEACON_OK);
    ASSERT_EQ(seen.size(), 1u);
    ASSERT_TRUE(seen[0].poseValid);
    SE3 robot = seen[0].tWorldCam * seen[0].tCamRobot;
    EXPECT_NEAR(robot.translation().x(), 0.5, 1e-9);
    EXPECT_NEAR(robot.translation().y(), 0.0, 1e-9);
    EXPECT_TRUE(robot.translation().allFinite());
    beacon_destroy(ctx);
}

TEST(Api, push_frame_rejects_bad_sizes) {
    beacon_ctx* ctx = makeCtx();
    std::vector<uint8_t> img(640 * 480, 0);
    EXPECT_EQ(beacon_push_frame(ctx, 1, img.data(), 320, 240, 320), BEACON_ERR);
    EXPECT_EQ(beacon_push_frame(ctx, 1, img.data(), 640, 480, 100), BEACON_ERR);
    EXPECT_EQ(beacon_push_frame(ctx, 1, nullptr, 640, 480, 640), BEACON_ERR);
    beacon_destroy(ctx);
}

TEST(Api, push_frame_never_blocks) {
    beacon_ctx* ctx = makeCtx();
    // Make the core busy: the observer runs on the worker thread and sleeps.
    std::atomic<int> started{0};
    beacon_set_observer(ctx, [&](const VizFrame&) {
        started++;
        std::this_thread::sleep_for(std::chrono::milliseconds(800));
    });
    beacon_pose2 s{0, 0, 0, 0};
    beacon_set_start_pose(ctx, &s);
    beacon_pose2 o0{0, 0, 0, 0}, o1{2000000000LL, 0, 0, 0};
    beacon_push_odom(ctx, &o0);
    beacon_push_odom(ctx, &o1);
    std::vector<uint8_t> img(640 * 480, 100);
    ASSERT_EQ(beacon_push_frame(ctx, 1000000, img.data(), 640, 480, 640), BEACON_OK);
    while (started.load() == 0) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_EQ(beacon_push_frame(ctx, 2000000, img.data(), 640, 480, 640), BEACON_OK);  // fills the slot
    double worstMs = 0;
    int dropped = 0;
    for (int i = 0; i < 1000; ++i) {
        auto t0 = std::chrono::steady_clock::now();
        int rc = beacon_push_frame(ctx, 3000000 + i, img.data(), 640, 480, 640);
        double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
        worstMs = std::max(worstMs, ms);
        if (rc == BEACON_DROPPED) ++dropped;
    }
    EXPECT_EQ(dropped, 1000);
    EXPECT_LT(worstMs, 1.0);
    RecordProperty("worst_push_ms", std::to_string(worstMs));
    std::printf("push_frame_never_blocks: 1000 pushes, %d dropped, worst %.3f ms\n", dropped, worstMs);
    beacon_destroy(ctx);  // joins the worker after its sleeps
}

TEST(Api, explore_mode_needs_no_field_json) {
    beacon_camera c = cam();
    beacon_mount m = mount();
    char err[256] = {0};
    beacon_ctx* ctx = beacon_create(nullptr, &c, &m, "{\"mode\": \"explore\", \"synchronous\": true}", err, sizeof err);
    ASSERT_NE(ctx, nullptr) << err;
    std::vector<uint8_t> img(640 * 480, 90);
    for (int i = 0; i < 5; ++i) EXPECT_EQ(beacon_push_frame(ctx, i * 50000000LL, img.data(), 640, 480, 640), BEACON_OK);
    EXPECT_NE(beacon_status(ctx), BEACON_STATUS_INVALID);
    beacon_destroy(ctx);
}
