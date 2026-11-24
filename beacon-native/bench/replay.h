// Replays a data set through the C API in time order (odometry and frames merged, each frame
// delivered `latency` after capture, as on the robot). Used by beacon_replay and the tests.
#pragma once

#include <functional>
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

#include "dataset.h"
#include "engine.h"

namespace beacon {

class RerunViewer;

struct ReplayOptions {
    std::string paramsJson;       // merged with {"synchronous": true} unless async
    bool async = false;           // use the worker thread (frames may be dropped)
    double latencyS = 0.04;       // frame reaches the core this long after capture
    bool useOdom = true;          // false: bench-only mode without odometry
    int maxFrames = -1;
    bool realtime = false;        // pace to the recorded timestamps (for watching)
    RerunViewer* viewer = nullptr;
    std::function<void(const VizFrame&)> onFrame;  // extra observer (tests)
};

struct ReplayResult {
    std::vector<beacon_fix> fixes;
    std::vector<std::pair<int64_t, SE3>> cameraPoses;  // per processed frame with a valid pose
    int frames = 0;
    int framesTracking = 0;
    int loopClosures = 0;
    std::vector<SE3> finalKeyframes;
    std::vector<Vec3> finalMap;
    int statusAtEnd = 0;
    std::vector<double> frameMs;  // synchronous mode: time spent in beacon_push_frame per frame
    std::string lastDebug;
};

/** images: optional in-memory frames (same order as d.frames); otherwise PNGs are read. */
bool runReplay(const Dataset& d, const std::vector<cv::Mat>* images, const ReplayOptions& opt,
               ReplayResult& out, std::string& err);

bool writeFixesCsv(const std::string& path, const std::vector<beacon_fix>& fixes, std::string& err);

/** Merges two flat JSON objects (b's keys win). */
std::string mergeJson(const std::string& a, const std::string& b);

}  // namespace beacon
