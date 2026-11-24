// Loop closing: bag-of-words place recognition over keyframes with a temporal consistency
// check, Sim(3) (or SE(3) with odometry) estimation between the query and the loop keyframe,
// map-point fusion, and essential-graph optimization.
#pragma once

#include <set>
#include <string>
#include <vector>

#include "map.h"
#include "matcher.h"
#include "optimizer.h"
#include "vocab.h"

namespace beacon {

class LoopCloser {
public:
    LoopCloser(Map& map, const Vocabulary& voc, const Pinhole& cam, bool fixedScale, const SE3& tRobotCam,
               const OdomEdgeParams& odom, bool globalBA)
        : map_(map), voc_(voc), cam_(cam), fixedScale_(fixedScale), tRobotCam_(tRobotCam), odom_(odom),
          globalBA_(globalBA) {}

    /** Call once per new keyframe (after local mapping). True when a loop was closed. */
    bool process(KeyFrame* kf);
    void setFixedScale(bool f) { fixedScale_ = f; }
    std::string lastMessage;
    std::string debug;  // last detection attempt, for tuning

private:
    bool detect(KeyFrame* kf, std::vector<KeyFrame*>& candidates);
    bool computeSim3(KeyFrame* kf, const std::vector<KeyFrame*>& candidates);
    void correct(KeyFrame* kf);

    struct Group {
        std::set<int> kfs;
        int consistency = 0;
    };
    Map& map_;
    const Vocabulary& voc_;
    Pinhole cam_;
    bool fixedScale_;
    SE3 tRobotCam_;
    OdomEdgeParams odom_;
    bool globalBA_;
    std::vector<Group> groups_;
    KeyFrame* matched_ = nullptr;
    Sim3 scw_;
    std::vector<int> curMatched_;            // per query feature: loop map point id or -1
    std::vector<MapPoint*> loopPoints_;
    int lastLoopKf_ = -100;
    double maxLogScale_ = 0.2;        // |log scale| of a loop Sim(3): within ~20 % (global BA keeps drift small)
};

}  // namespace beacon
