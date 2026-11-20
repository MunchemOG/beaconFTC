// Descriptor matching for the SLAM back end: by projection (tracking, fusion, loop closing), by
// bag-of-words node (relocalization, loops), for initialization and for triangulation.
// All image positions are UNDISTORTED pixels (pinhole fx, fy, cx, cy).
#pragma once

#include <vector>

#include "orb_features.h"
#include "map.h"

namespace beacon {

/** The frame being tracked. */
struct Frame {
    long id = 0;
    int64_t tNs = 0;
    Features f;
    std::vector<int> mps;        // per feature: map point id or -1
    std::vector<char> outlier;
    SE3 tcw = SE3::Identity();
    bool hasPose = false;
    BowVector bow;
    FeatureVector fv;
    bool hasOdom = false;
    SE2 odom;
    uint64_t odomEpoch = 0;
    int refKf = -1;
    void reset(size_t n) {
        mps.assign(n, -1);
        outlier.assign(n, 0);
    }
};

/** A map point prepared for projection into the current frame. */
struct Projected {
    MapPoint* p;
    float u, v;
    int level;
    float viewCos;
};

struct Pinhole {
    double fx, fy, cx, cy;
    int width, height;
    bool project(const Vec3& pc, float& u, float& v) const {
        if (pc.z() <= 1e-6) return false;
        u = (float)(fx * pc.x() / pc.z() + cx);
        v = (float)(fy * pc.y() / pc.z() + cy);
        return true;
    }
};

class Matcher {
public:
    static constexpr int kHigh = 100, kLow = 50;
    Matcher(const Map& map, const Pinhole& cam) : map_(map), cam_(cam) {}

    /** Visible check + prediction of scale level; fills `out` when the point should be searched. */
    bool inFrustum(MapPoint* p, const SE3& tcw, const Features& f, float viewCosLimit, Projected& out) const;

    /** Local map tracking: match projected points into F. th scales the search radius. */
    int searchByProjection(Frame& F, const std::vector<Projected>& pts, float th) const;
    /** Motion model: project last frame's points with F.tcw. */
    int searchByProjection(Frame& F, const Frame& last, float th) const;
    /** Relocalization helper: project a keyframe's points, skipping those already found. */
    int searchByProjection(Frame& F, KeyFrame* kf, const std::vector<int>& alreadyFound, float th, int orbDist) const;
    /** Matches by vocabulary node; out[i] = map point id for F's feature i or -1. */
    int searchByBoW(KeyFrame* kf, Frame& F, std::vector<int>& out) const;
    /** Keyframe-keyframe by vocabulary node; out[i1] = map point id of k2 or -1. */
    int searchByBoW(KeyFrame* k1, KeyFrame* k2, std::vector<int>& out) const;
    /** Keyframe-keyframe, all pairs of features that both carry map points (loop verification:
     *  higher recall than by-node matching, affordable for a few candidates). */
    int searchBruteForce(KeyFrame* k1, KeyFrame* k2, std::vector<int>& out) const;
    /** Two frames, window search around previous match positions (monocular init). */
    int searchForInitialization(const Features& f1, const Features& f2, std::vector<cv::Point2f>& prev,
                                std::vector<int>& matches12, int window) const;
    /** Unmatched features of k1 and k2 consistent with the epipolar geometry. */
    int searchForTriangulation(KeyFrame* k1, KeyFrame* k2, std::vector<std::pair<int, int>>& pairs) const;
    /** Fuses points into kf (replace duplicates / add observations). Returns fused count. */
    int fuse(KeyFrame* kf, const std::vector<MapPoint*>& pts, float th, Map& map) const;
    /** Projects points with a corrected world->camera similarity (scw: scale*R, t). */
    int searchByProjectionSim(KeyFrame* kf, const Mat3& sR, const Vec3& t, double s,
                              const std::vector<MapPoint*>& pts, std::vector<int>& matched, int th) const;

private:
    const Map& map_;
    Pinhole cam_;
};

/** Keeps the three most populated orientation-difference bins (rejects inconsistent matches). */
void rotationConsistency(const std::vector<float>& rotDiffs, const std::vector<int>& idx, std::vector<int>& reject);

}  // namespace beacon
