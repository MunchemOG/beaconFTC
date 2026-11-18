// Map initialization from two views.
//  - Monocular (no odometry): essential matrix with RANSAC, cheirality, triangulation; scale is
//    arbitrary (set later so the median scene depth is 1).
//  - Known relative pose (Pinpoint odometry): triangulation only; the map is metric.
#pragma once

#include <vector>

#include "features.h"
#include "matcher.h"

namespace beacon {

struct TwoViewResult {
    bool ok = false;
    SE3 t21 = SE3::Identity();       // camera 2 <- camera 1
    std::vector<Vec3> points;        // per feature of view 1, in camera-1 coordinates
    std::vector<char> good;          // per feature of view 1
    int nGood = 0;
    double medianParallaxDeg = 0;
    std::string why;
};

/** Monocular two-view reconstruction from matches m12 (feature of 1 -> feature of 2). */
TwoViewResult initializeMonocular(const Features& f1, const Features& f2, const std::vector<int>& m12,
                                  const Pinhole& cam, double minParallaxDeg, int minPoints);

/** Triangulation with a known relative pose t21. */
TwoViewResult triangulateKnown(const Features& f1, const Features& f2, const std::vector<int>& m12,
                               const SE3& t21, const Pinhole& cam, double minParallaxDeg);

/** Linear (DLT) triangulation of normalized coordinates with camera poses T1, T2 (world->cam). */
bool triangulate(const Vec2& n1, const Vec2& n2, const SE3& T1, const SE3& T2, Vec3& X);

}  // namespace beacon
