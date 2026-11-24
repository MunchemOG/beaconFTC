// Optimizer building blocks on exact synthetic data.
#include <gtest/gtest.h>

#include <random>

#include "map.h"
#include "matcher.h"
#include "optimizer.h"

using namespace beacon;

namespace {

SE3 pose(double rx, double ry, double rz, double tx, double ty, double tz) {
    SE3 T = SE3::Identity();
    T.linear() = (Eigen::AngleAxisd(rz, Vec3::UnitZ()) * Eigen::AngleAxisd(ry, Vec3::UnitY()) *
                  Eigen::AngleAxisd(rx, Vec3::UnitX()))
                     .toRotationMatrix();
    T.translation() = Vec3(tx, ty, tz);
    return T;
}

}  // namespace

TEST(Optimizer, pose_only_converges_from_offset) {
    Pinhole cam{450, 450, 320, 240, 640, 480};
    Map map;
    Frame F;
    SE3 truth = pose(0.1, -0.2, 0.05, 0.3, -0.1, 0.2);
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> ux(-2, 2), uz(2, 6);
    F.f.scaleFactors = {1.0f};
    F.f.invLevelSigma2 = {1.0f};
    while (F.f.kps.size() < 200) {
        Vec3 pw(ux(rng), ux(rng), uz(rng));
        Vec3 pc = truth * pw;
        float u, v;
        if (!cam.project(pc, u, v) || u < 0 || v < 0 || u > 640 || v > 480) continue;
        MapPoint* p = map.newMapPoint(pw, 0);
        cv::KeyPoint k(u, v, 31);
        k.octave = 0;
        F.f.kps.push_back(k);
        F.f.un.push_back(cv::Point2f(u, v));
        F.mps.push_back(p->id);
        F.outlier.push_back(0);
    }
    // 5 degrees and 10 cm off.
    F.tcw = pose(0.05, 0.05, -0.05, 0.1, 0.1, -0.05) * truth;
    int inl = optimizePose(F, map, cam);
    EXPECT_GE(inl, 195);
    EXPECT_LT((F.tcw.translation() - truth.translation()).norm(), 1e-6);
    EXPECT_LT(Eigen::AngleAxisd(F.tcw.linear() * truth.linear().transpose()).angle(), 1e-6);
}

TEST(Optimizer, essential_graph_spreads_loop_correction) {
    // 40 keyframes on a circle (radius 2, camera looking outward). The estimate drifts in yaw
    // and position; the loop tells the last keyframe its true pose. After the pose graph, every
    // keyframe must be closer to the truth than before, and the loop end exact.
    Map map;
    const int n = 40;
    std::vector<SE3> truth(n), drifted(n);
    for (int i = 0; i < n; ++i) {
        double a = 2 * M_PI * i / n;
        SE3 twc = pose(0, 0, a, 2 * std::cos(a), 2 * std::sin(a), 0);  // camera in world
        truth[i] = twc.inverse();
        // drift grows with i: rotate and shift the whole camera pose about the origin
        SE3 d = pose(0, 0, 0.004 * i, 0.01 * i, -0.005 * i, 0.002 * i);
        drifted[i] = (d * twc).inverse();
    }
    for (int i = 0; i < n; ++i) {
        KeyFrame* k = map.newKeyFrame();
        k->tcw = drifted[i];
        if (i > 0) {
            k->parent = i - 1;
            map.kf(i - 1)->children.insert(i);
        }
    }
    KeyFrame* cur = map.kf(n - 1);
    KeyFrame* loopKf = map.kf(0);
    std::map<int, Sim3> corrected, nonCorrected;
    corrected[cur->id] = Sim3(truth[n - 1]);
    nonCorrected[cur->id] = Sim3(drifted[n - 1]);
    cur->tcw = truth[n - 1];
    std::map<int, std::set<int>> links;
    links[cur->id].insert(loopKf->id);
    double before = 0;
    for (int i = 0; i < n; ++i) before += (drifted[i].inverse().translation() - truth[i].inverse().translation()).norm();
    optimizeEssentialGraph(map, loopKf, cur, corrected, nonCorrected, links, 100, false, SE3::Identity(), OdomEdgeParams());
    double after = 0, worst = 0;
    for (int i = 0; i < n; ++i) {
        double e = (map.kf(i)->tcw.inverse().translation() - truth[i].inverse().translation()).norm();
        after += e;
        worst = std::max(worst, e);
    }
    std::printf("pose graph: mean centre error %.4f -> %.4f m, worst %.4f m, loop end %.5f m\n", before / n, after / n,
                worst, (map.kf(n - 1)->centre() - truth[n - 1].inverse().translation()).norm());
    EXPECT_LT(after, 0.5 * before);
    EXPECT_LT((map.kf(n - 1)->centre() - truth[n - 1].inverse().translation()).norm(), 0.02);
}
