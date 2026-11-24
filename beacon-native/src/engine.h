// The engine interface shared by field mode and explore mode, plus the visualization snapshot
// handed to desktop tools (the viewer lives in bench/, never in the robot library).
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <opencv2/core.hpp>
#include <string>
#include <utility>
#include <vector>

#include "beacon/beacon.h"
#include "field_model.h"
#include "geometry.h"
#include "params.h"
#include "predict.h"

namespace beacon {

/** What one processed frame looked like, for viewers and tests. Filled only when observed. */
struct VizFrame {
    int64_t tNs = 0;
    cv::Mat image;                          // the gray frame (read-only)
    std::vector<cv::Point2f> tracked;       // keypoints matched to map points (green)
    std::vector<cv::Point2f> fresh;         // new / candidate keypoints (blue)
    std::vector<std::pair<cv::Point2f, cv::Point2f>> projectedEdges;  // field model overlay
    bool poseValid = false;
    SE3 tWorldCam = SE3::Identity();        // camera in the world (field, odometry or map frame)
    int status = BEACON_STATUS_INITIALIZING;
    bool hasFix = false;
    beacon_fix fix{};
    bool odomValid = false;                 // odometry ("pose_imu") camera pose in the world
    SE3 tWorldCamOdom = SE3::Identity();
    SE3 tCamRobot = SE3::Identity();        // for drawing the robot frame (tf)
    std::vector<Vec3> trackedPoints;        // 3-D map points matched in this frame ("points")
    // Heavy parts, refreshed only when mapChanged is true.
    bool mapChanged = false;
    std::vector<SE3> keyframes;             // camera poses ("kfs")
    std::vector<Vec3> mapPoints;            // all map points ("lc_database_map")
    std::vector<Vec3> localPoints;          // frontend local map ("frontend_map")
    std::vector<Vec3> backendPoints;        // points refined by the last local BA
    std::vector<std::pair<Vec3, Vec3>> loopLinks;   // keyframe centre pairs ("loop_closures")
    std::vector<std::pair<Vec3, Vec3>> graphEdges;  // pose-graph edges ("pose_graph")
    std::vector<Vec3> trajectory;           // optimized camera path ("backend_pose_imu_viz")
    std::vector<int64_t> trajectoryT;       // capture time of each trajectory sample
    int keyframeCount = 0, mapPointCount = 0, loopCount = 0;
    std::vector<std::pair<Vec3, Vec3>> modelEdges;  // field model, world frame
    std::string text;                       // one-line summary
};

using VizObserver = std::function<void(const VizFrame&)>;

/** State shared between the API threads and the worker. */
struct Shared {
    explicit Shared(const Params& p) : params(p) {}
    Params params;
    Camera camera;
    SE3 tRobotCam = SE3::Identity();
    OdomBuffer odom;

    std::mutex mu;  // guards everything below
    bool hasStart = false;
    beacon_pose2 start{};
    uint64_t startSeq = 0;
    uint64_t odomEpoch = 0;      // bumped by beacon_reset_odom_world
    int64_t odomResetT = 0;
    bool hasFix = false, fixNew = false;
    beacon_fix fix{};
    int status = BEACON_STATUS_INITIALIZING;
    std::string debug = "beacon: starting";
    uint64_t framesIn = 0, framesDropped = 0, framesDone = 0, fixes = 0;
    double lastProcessMs = 0;
};

class Engine {
public:
    virtual ~Engine() = default;
    /** Processes one frame (worker thread). May publish a fix through `shared`. */
    virtual void process(int64_t tNs, const cv::Mat& gray, Shared& shared) = 0;
    void setObserver(VizObserver obs) { observer_ = std::move(obs); }

protected:
    VizObserver observer_;
};

std::unique_ptr<Engine> makeFieldEngine(const FieldModel& field, Shared& shared);
std::unique_ptr<Engine> makeExploreEngine(const FieldModel* fieldOrNull, Shared& shared);

/** Publishes a fix and status from the worker. */
void publishFix(Shared& s, const beacon_fix& fix);

}  // namespace beacon

/** Desktop-tool hook (C++ only, not part of I3): observe every processed frame. */
void beacon_set_observer(beacon_ctx* ctx, beacon::VizObserver observer);
