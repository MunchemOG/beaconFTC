// Minimal synthetic bench data (until the Field engineer's tools/sim lands): a ray-cast
// renderer of the field model (textured foam tiles with seams, clear perimeter panels with
// opaque rails and posts, a random venue backdrop seen through the panels, optional hive faces
// and clutter boxes), trajectories, and a Pinpoint-like odometry simulator. Bench only.
#pragma once

#include <cstdint>
#include <functional>
#include <opencv2/core.hpp>
#include <random>
#include <string>
#include <vector>

#include "beacon/beacon.h"
#include "dataset.h"
#include "field_model.h"
#include "geometry.h"

namespace beacon {

struct Box {  // axis-aligned clutter box (another robot, a game piece)
    Vec3 lo, hi;
    uint32_t seed = 1;
};

struct RenderOptions {
    uint32_t seed = 1;          // venue backdrop and floor texture
    double noiseSigma = 1.5;    // gray levels
    int supersample = 2;        // per axis
    double bottomBandM = 0.025; // opaque strip at the bottom of each wall
    double postHalfWidthM = 0.004;  // 8 mm dark joint, as in beacon-field and tools/sim
    std::vector<Box> clutter;
};

/** What a pixel ray hits first, for ground-truth checks in tests. */
enum class HitKind { None, Floor, Wall, Post, Hive, Clutter, Venue };

class Renderer {
public:
    Renderer(const FieldModel& field, const Camera& cam, const RenderOptions& opt);

    /**
     * Renders an 8-bit gray image. poseAtRow(row) gives T_field_cam for that image row (rolling
     * shutter); for a global shutter return the same pose for every row.
     */
    cv::Mat render(const std::function<SE3(int row)>& poseAtRow, uint32_t frameSeed) const;

    /** First hit of the ray through pixel (u, v) from camera pose T_field_cam. */
    HitKind trueHit(const SE3& tFieldCam, double u, double v, Vec3& point) const;

private:
    double shade(const Vec3& o, const Vec3& d, HitKind* kind, Vec3* point) const;
    double venue(const Vec3& p) const;
    double floorTexture(const Vec2& p) const;

    const FieldModel& field_;
    Camera cam_;
    RenderOptions opt_;
    std::vector<Vec3> rays_;  // camera-frame unit rays, supersampled, row-major
};

// ------------------------------------------------------------------------- trajectories

struct TimedPose {
    int64_t tNs;
    SE2 pose;
};

enum class TrajKind { Match, Spin, Still, Blind, Clutter, Square };

struct TrajOptions {
    TrajKind kind = TrajKind::Match;
    double durationS = 30;
    uint32_t seed = 1;
    double maxSpeed = 1.5;     // m/s
    double maxOmega = 3.0;     // rad/s
    double marginM = 0.45;     // keep the robot centre this far from the walls
    double rateHz = 200;       // sample rate of the returned truth
};

/** Truth trajectory in the field frame, sampled at rateHz, starting at t = t0Ns. */
std::vector<TimedPose> makeTrajectory(const TrajOptions& opt, double halfSizeM, int64_t t0Ns);

/** Truth pose at t (linear interpolation, heading unwrapped). */
SE2 sampleTrajectory(const std::vector<TimedPose>& traj, int64_t tNs);

struct OdomOptions {
    uint32_t seed = 1;
    double scaleSigma = 0.005;          // per-axis scale error (1 sigma)
    double headingWalkDegPerMin = 0.5;  // heading random walk
    double slipProbPerS = 0.0;          // collision slip bursts
};

/** Pinpoint-like odometry: starts at (0,0,0), integrates truth increments with errors. */
std::vector<TimedPose> simulateOdometry(const std::vector<TimedPose>& truth, const OdomOptions& opt);

// --------------------------------------------------------------------- whole data sets

struct SynthOptions {
    TrajOptions traj;
    OdomOptions odom;
    RenderOptions render;
    double fps = 15;
    double camHeightM = 0.35;
    double camPitchDeg = 8;
    double camForwardM = 0.15;
    double rsReadoutS = 0;
    double startErrorM = 0;        // error added to the typed start pose (along +x of field)
    double startErrorDeg = 0;
    bool withOdom = true;
    int width = 640, height = 480;
    double hfovDeg = 54.5;
};

/** The standard bench camera (640x480, HFOV 54.5 deg, no distortion). */
beacon_camera benchCamera(int width = 640, int height = 480, double hfovDeg = 54.5);

/** Perimeter-only test field (bench/testdata/perimeter-field.json contents). */
std::string perimeterFieldJson();

/**
 * Builds a synthetic run. If outDir is non-empty, writes an I2 directory (frames as PNG).
 * Otherwise only fills `out` and `images`.
 */
bool makeSynthRun(const SynthOptions& opt, const std::string& fieldJson, Dataset& out,
                  std::vector<cv::Mat>* images, const std::string& outDir, std::string& err);

}  // namespace beacon
