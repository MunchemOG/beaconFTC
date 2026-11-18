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
