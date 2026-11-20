// C2: odometry prediction and model-edge tracking in field mode.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include "predict.h"
#include "render.h"
#include "replay.h"

using namespace beacon;

TEST(Predict, predict_interpolates_at_capture_time) {
    OdomBuffer b(64);
    // 1 m/s along x, turning 1 rad/s, sampled at 200 Hz; heading crosses +pi.
    for (int i = 0; i <= 100; ++i) {
        double t = i * 0.005;
        b.push((int64_t)(t * 1e9), SE2(t, 0.0, wrapAngle(3.0 + t)));
    }
    SE2 p;
    ASSERT_TRUE(b.at(252500000, 0, p));  // between samples 50 and 51
    EXPECT_NEAR(p.x, 0.2525, 1e-9);
    EXPECT_NEAR(wrapAngle(p.theta - (3.0 + 0.2525)), 0.0, 1e-9);  // no long-way-round jump at +-pi
    ASSERT_TRUE(b.at(510000000, 20000000, p));                      // 10 ms extrapolation allowed
    EXPECT_NEAR(p.x, 0.51, 1e-9);
    EXPECT_FALSE(b.at(600000000, 20000000, p));                     // too far ahead
    EXPECT_FALSE(b.at(-1, 0, p));                                   // before the buffer
    double v, w;
    ASSERT_TRUE(b.velocity(250000000, v, w));
    EXPECT_NEAR(v, 1.0, 1e-6);
    EXPECT_NEAR(w, 1.0, 1e-6);
    EXPECT_FALSE(b.push(100, SE2(0, 0, 0)));                         // out of order
    EXPECT_FALSE(b.push(600000000, SE2(NAN, 0, 0)));                 // non-finite
}

TEST(Predict, alignment_update_pulls_toward_fix) {
    Alignment a;
    a.set(SE2(0, 0, 0), 0.1, 0.05);
    Mat3 r = Mat3::Zero();
    r(0, 0) = r(1, 1) = 0.01 * 0.01;
    r(2, 2) = 0.005 * 0.005;
    SE2 odom(1.0, 0.5, 0.3);
    SE2 fix(1.05, 0.48, 0.32);
    for (int i = 0; i < 20; ++i) a.update(fix, r, odom);
    SE2 p = a.value() * odom;
    EXPECT_NEAR(p.x, 1.05, 2e-3);
    EXPECT_NEAR(p.y, 0.48, 2e-3);
    EXPECT_NEAR(p.theta, 0.32, 1e-3);
}

TEST(Field, edge_fix_recovers_offset) {
    // Bench perimeter field, match-like driving, typed start pose 10 cm / 3 deg off.
    SynthOptions o;
    o.traj.kind = TrajKind::Match;
    o.traj.durationS = 20;
    o.traj.seed = 2;
    o.traj.maxSpeed = 1.2;
    o.traj.maxOmega = 2.0;
    o.fps = 15;
    o.startErrorM = 0.10;
    o.startErrorDeg = 3.0;
    o.odom.scaleSigma = 0.02;           // harder than a Pinpoint
    o.odom.headingWalkDegPerMin = 3.0;
    Dataset d;
    std::vector<cv::Mat> images;
    std::string err;
    ASSERT_TRUE(makeSynthRun(o, perimeterFieldJson(), d, &images, "", err)) << err;
    ReplayOptions opt;
    ReplayResult r;
    ASSERT_TRUE(runReplay(d, &images, opt, r, err)) << err;
    std::vector<TimedPose> truth;
    for (const PoseRow& p : d.truth) truth.push_back({p.tNs, SE2(p.x, p.y, p.heading)});
    std::vector<double> late;
    double first = -1;
    for (const beacon_fix& f : r.fixes) {
        SE2 t = sampleTrajectory(truth, f.t_ns);
        double e = std::hypot(f.x - t.x, f.y - t.y);
        if (first < 0) first = e;
        if (f.t_ns - d.frames[0].tNs > 3000000000LL) late.push_back(e);
    }
    ASSERT_GT(late.size(), 20u);
    std::sort(late.begin(), late.end());
    double median = late[late.size() / 2], p95 = late[late.size() * 95 / 100];
    std::printf("edge fixes: %zu total, first error %.1f cm, after 3 s: median %.2f cm, p95 %.2f cm, max %.2f cm\n",
                r.fixes.size(), 100 * first, 100 * median, 100 * p95, 100 * late.back());
    EXPECT_LT(median, 0.02);
    EXPECT_LT(p95, 0.03);
}

TEST(FieldModel, biobuzz_hive_parts_have_two_tilt_states) {
    std::string text = readFile(std::string(BEACON_REPO_DIR) + "/beacon-field/biobuzz-2026.json");
    if (text.empty()) GTEST_SKIP() << "beacon-field/biobuzz-2026.json not present";
    FieldModel m;
    std::string err;
    ASSERT_TRUE(FieldModel::parse(text, m, err)) << err;
    ASSERT_EQ(m.parts.size(), 2u);
    int perState[2][2] = {{0, 0}, {0, 0}}, statics = 0;
    for (const ModelEdge& e : m.edges) {
        if (e.movingPart >= 0) perState[e.movingPart][e.state]++;
        else statics++;
    }
    for (int p = 0; p < 2; ++p) {
        EXPECT_GT(perState[p][0], 0);
        EXPECT_EQ(perState[p][0], perState[p][1]);
        const MovingPart& mp = m.parts[p];
        Vec3 q(0.1, 0.2, 1.3);
        EXPECT_LT((mp.moved(q, mp.startState) - q).norm(), 1e-12);         // start state = as drawn
        EXPECT_GT((mp.moved(q, 1 - mp.startState) - q).norm(), 0.05);     // the other tilt moves it
        EXPECT_LT(std::abs((mp.moved(q, 1 - mp.startState) - mp.point).norm() - (q - mp.point).norm()), 1e-9);
    }
    EXPECT_GT(statics, 20);
}
