// Rerun viewer for the desktop bench (never built for the robot). Logs every processed frame
// to the entities of plan Task C6 (1:1 with the reference videos' RViz displays):
//   beacon/world/grid, beacon/world/tf, beacon/points, beacon/backend_pose_imu_viz,
//   beacon/image/cam0 (+ /tracked green, /new blue), beacon/loop_closures, beacon/pose_imu,
//   beacon/kfs, beacon/frontend_map, beacon/pose_graph, beacon/lc_database_map,
//   beacon/backend_points
// plus beacon/world/camera (current camera), beacon/world/field (field model, field mode) and
// beacon/status (text).
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "engine.h"

namespace rerun {
class RecordingStream;
}

namespace beacon {

struct ViewerOptions {
    bool spawn = false;           // start the Rerun viewer and stream to it live
    std::string saveRrd;          // also/only write a recording file (.rrd)
    std::string blueprint;        // .rbl applied to the spawned viewer (layout, visibility)
    std::string rerunExecutable;  // default: third_party/src/rerun-cli-0.41.0, then PATH
    int port = 9876;
    bool worldZUp = true;         // false: explore mode without odometry (map = first camera, y down)
    int imageEvery = 1;           // log every Nth image (keeps .rrd files small)
    int jpegQuality = 80;
    bool onlyLast = false;        // log just the final state at flush() (screenshots, small files)
};

/** The entity paths of the C6 table, for tests. */
const std::vector<std::string>& viewerEntityPaths();

class RerunViewer {
public:
    RerunViewer();
    ~RerunViewer();
    bool open(const ViewerOptions& opt, std::string& err);
    void log(const VizFrame& v);
    void flush();
    int framesLogged() const { return frames_; }

private:
    void logStatic(const VizFrame& v);
    void logNow(const VizFrame& v);
    VizFrame last_;
    bool hasLast_ = false;
    std::unique_ptr<rerun::RecordingStream> rec_;
    ViewerOptions opt_;
    int frames_ = 0;
    bool staticDone_ = false;
    int viewerPid_ = -1;
};

}  // namespace beacon
