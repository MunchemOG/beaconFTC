// Optimization for the SLAM back end: motion-only pose refinement (Gauss-Newton, robust),
// local bundle adjustment and Sim(3) pose-graph / loop optimization (Ceres).
// Measurements are undistorted pixel positions; each is weighted by its pyramid level.
#pragma once

#include <vector>

#include "map.h"
#include "matcher.h"

namespace beacon {

/** Similarity transform x' = s * R * x + t. */
struct Sim3 {
    double s = 1;
    Mat3 R = Mat3::Identity();
    Vec3 t = Vec3::Zero();
    Sim3() = default;
    Sim3(double s_, const Mat3& R_, const Vec3& t_) : s(s_), R(R_), t(t_) {}
    explicit Sim3(const SE3& T) : s(1), R(T.linear()), t(T.translation()) {}
    Sim3 operator*(const Sim3& b) const { return Sim3(s * b.s, R * b.R, s * (R * b.t) + t); }
    Sim3 inverse() const { return Sim3(1.0 / s, R.transpose(), -(1.0 / s) * (R.transpose() * t)); }
    Vec3 map(const Vec3& x) const { return s * (R * x) + t; }
    SE3 toSE3() const {  // drops the scale: rigid camera pose with the same centre
        SE3 T = SE3::Identity();
        T.linear() = R;
        T.translation() = t / s;
        return T;
    }
};

/** Odometry prior for the motion-only refinement: predicted Tcw and its 1-sigma. */
struct PosePrior {
    bool use = false;
    SE3 tcw = SE3::Identity();
    double sigmaT = 0.05, sigmaR = 0.05;
};

/** Refines F.tcw from its matched map points; marks outliers. Returns the inlier count. */
int optimizePose(Frame& F, const Map& map, const Pinhole& cam, const PosePrior& prior = PosePrior());

struct OdomEdgeParams {
    double sigmaTBase = 0.001, sigmaTPerM = 0.004;    // m (Pinpoint: about 0.5 % of distance)
    double sigmaRBase = 0.002, sigmaRPerRad = 0.005;  // rad
};

/**
 * Local BA around kf: its covisible keyframes (up to window) and their points are optimized,
 * other keyframes that see those points are held fixed. tRobotCam is used for odometry factors
 * between keyframes that carry Pinpoint poses. Returns the number of observations removed.
 */
int localBundleAdjustment(KeyFrame* kf, Map& map, const Pinhole& cam, int window, bool useOdom,
                          const SE3& tRobotCam, const OdomEdgeParams& odomParams, long baId);

/** Relative camera motion measured by odometry between two keyframes (camera 1 <- camera 2). */
SE3 odomRelative(const KeyFrame* a, const KeyFrame* b, const SE3& tRobotCam);

/**
 * Essential-graph optimization after a loop: spanning tree, strong covisibility (>= minW),
 * loop edges, odometry edges. corrected: keyframes whose pose was replaced by the loop
 * correction (measurements use those poses). nonCorrected: their poses before correction.
 * fixedScale: odometry present (metric), scale held at 1.
 */
void optimizeEssentialGraph(Map& map, KeyFrame* loopKf, KeyFrame* curKf, const std::map<int, Sim3>& corrected,
                            const std::map<int, Sim3>& nonCorrected, const std::map<int, std::set<int>>& newLinks,
                            int minW, bool fixedScale, const SE3& tRobotCam, const OdomEdgeParams& odomParams);

/** Refines S12 (k1 <- k2) from matched point pairs; marks outliers (matches[i] = -1). */
int optimizeSim3(KeyFrame* k1, KeyFrame* k2, std::vector<int>& matches12, Sim3& s12, const Pinhole& cam,
                 const Map& map, double th2, bool fixScale);

/** Full BA over all keyframes and points (after loop closing). */
void globalBundleAdjustment(Map& map, const Pinhole& cam, int iterations, bool useOdom, const SE3& tRobotCam,
                            const OdomEdgeParams& odomParams);

}  // namespace beacon
