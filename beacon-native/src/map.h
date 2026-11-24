// Sparse map: keyframes, map points, the covisibility graph and its spanning tree.
// Objects are owned by Map and addressed by integer id; nothing is freed during a run (bad
// objects are flagged), which keeps ids stable for the viewer and the loop closer.
#pragma once

#include <map>
#include <memory>
#include <set>
#include <unordered_map>
#include <vector>

#include "orb_features.h"
#include "geometry.h"
#include "vocab.h"

namespace beacon {

struct MapPoint {
    int id = -1;
    Vec3 pos = Vec3::Zero();
    cv::Mat desc;                 // 1 x 32, most distinctive observation
    std::map<int, int> obs;       // keyframe id -> feature index
    Vec3 normal = Vec3::Zero();   // mean viewing direction
    float minDist = 0, maxDist = 0;
    int refKf = -1, firstKf = -1;
    int visible = 1, found = 1;   // tracking statistics (found / visible ratio)
    bool bad = false;
    int replacedBy = -1;
    // Per-frame scratch marks (avoid double work in one pass).
    long lastSeenFrame = -1, trackRefFrame = -1, localBaFor = -1, fuseFor = -1, loopFor = -1, correctedBy = -1;
    int correctedRef = -1;        // keyframe whose loop correction moved this point
    bool inBackend = false;       // refined by the last local BA
    bool onSurface = false;       // field mode: created on and consistent with a field surface
    double foundRatio() const { return visible > 0 ? (double)found / visible : 0.0; }
};

struct KeyFrame {
    int id = -1;
    long frameId = -1;
    int64_t tNs = 0;
    SE3 tcw = SE3::Identity();    // world -> camera
    Features f;
    BowVector bow;
    FeatureVector fv;
    std::vector<int> mps;         // per feature: map point id or -1
    std::map<int, int> conn;      // covisibility weights
    std::vector<int> ordered;     // connected keyframes, strongest first
    int parent = -1;
    std::set<int> children, loopEdges;
    bool bad = false;
    bool hasOdom = false;
    SE2 odom;                     // Pinpoint pose (odometry world) at tNs
    uint64_t odomEpoch = 0;       // odometry world segment (bumped by beacon_reset_odom_world)
    long localBaFor = -1, fixedBaFor = -1, loopQueryFor = -1, relocQueryFor = -1;
    SE3 twc() const { return tcw.inverse(); }
    Vec3 centre() const { return -(tcw.linear().transpose() * tcw.translation()); }
    int trackedPoints(int minObs, const class Map& m) const;
};

class Map {
public:
    KeyFrame* newKeyFrame();
    MapPoint* newMapPoint(const Vec3& pos, int refKf);
    KeyFrame* kf(int id) const;
    MapPoint* mp(int id) const;
    /** Good keyframes / points (sorted by id). */
    std::vector<KeyFrame*> keyFrames() const;
    std::vector<MapPoint*> mapPoints() const;
    size_t numKeyFrames() const;

    void addObservation(MapPoint* p, KeyFrame* k, int idx);
    void eraseObservation(MapPoint* p, KeyFrame* k);
    /** Replaces p by q everywhere (q keeps p's observations where it has none). */
    void replace(MapPoint* p, MapPoint* q);
    void setBad(MapPoint* p);
    void setBad(KeyFrame* k);
    void updateConnections(KeyFrame* k);
    void computeDescriptor(MapPoint* p);
    void updateNormalAndDepth(MapPoint* p);
    std::vector<KeyFrame*> bestCovisibles(KeyFrame* k, int n) const;
    std::vector<KeyFrame*> covisiblesByWeight(KeyFrame* k, int minWeight) const;
    std::vector<MapPoint*> pointsOf(KeyFrame* k) const;

    KeyFrameDatabase db;
    int origin = -1;  // first keyframe id
    std::vector<std::pair<int, int>> loopLinks;  // closed loops (keyframe ids)

private:
    std::unordered_map<int, std::unique_ptr<KeyFrame>> kfs_;
    std::unordered_map<int, std::unique_ptr<MapPoint>> mps_;
    int nextKf_ = 0, nextMp_ = 0;
};

}  // namespace beacon
