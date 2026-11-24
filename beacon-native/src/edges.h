// Model-edge tracking (field mode): project the field model's straight edges (wall-floor line,
// wall top, rail, posts, hive edges) with the predicted pose, search the image along each
// projected edge's normal for the matching intensity edge, and solve the planar robot pose
// (x, y, heading) with robust Gauss-Newton.
#pragma once

#include <opencv2/core.hpp>
#include <utility>
#include <vector>

#include "field_model.h"
#include "geometry.h"
#include "params.h"

namespace beacon {

struct EdgeResult {
    bool ok = false;          // a pose was solved
    SE2 pose;                 // robot pose in the field frame
    Mat3 cov = Mat3::Identity() * 1e6;  // covariance of (x, y, theta)
    int samples = 0;          // edge samples projected inside the image
    int matched = 0;          // samples with an image edge found
    int inliers = 0;          // matched samples within the final inlier band
    double rmsPx = 0;
    SE2 alignment;            // window solves: odometry world -> field
    bool ambiguous = false;   // wide search: a clearly different pose explains the edges as well
    double worstPartFrac = 1; // newest frame: lowest inlier fraction of a model part (a wall, the hive frame)
    int worstPartSamples = 0; //   ... with at least partMinSamples samples, and its sample count
    std::vector<std::pair<cv::Point2f, cv::Point2f>> overlay;  // projected model edges (final pose)
    // Surface points (C3) in the solve, newest frame: matched = mature points with a descriptor
    // match, inliers = within the inlier band at the final pose.
    int pointsMatched = 0, pointInliers = 0;
    double pointRmsPx = 0;
    std::vector<char> pointInlier;  // per newest-frame point observation
};

/**
 * One surface-point observation for the window solve: a field-surface map point (field frame,
 * position fixed) and up to three candidate image positions (UNDISTORTED pixels) whose
 * descriptors matched it. The residual uses the candidate nearest to the projection, like the
 * edge samples use the nearest image edge.
 */
struct PointObs {
    Vec3 world;
    Vec2 cand[3];
    int nCand = 0;
    double scale = 1;  // pyramid scale of the best match (pixel noise grows with it)
    int point = -1;    // map point index
    int feature[3] = {-1, -1, -1};
};

/** One frame prepared for edge tracking: gradients plus the odometry pose at capture time. */
struct EdgeFrame {
    cv::Mat gx, gy;  // Sobel gradients (CV_32F, 8-bit intensity units per pixel)
    cv::Mat blur;    // blurred intensity (CV_32F), for thin dark lines (tile seams)
    SE2 odom;        // robot pose in the odometry world at capture time
    int64_t tNs = 0;
    std::vector<PointObs> points;  // surface map points matched in this frame (may be empty)
};

class EdgeTracker {
public:
    EdgeTracker(const FieldModel& field, const Camera& cam, const Params& params);

    /**
     * Aligns the field model to `gray`. prior/priorCov: predicted robot field pose and its
     * covariance (sets the search range and a weak prior). tRobotCam: camera mount.
     */
    EdgeResult solve(const cv::Mat& gray, int64_t tNs, const SE2& prior, const Mat3& priorCov,
                     const SE3& tRobotCam, bool wantOverlay);

    /**
     * Wide search for when the prediction is poor (start pose typed by hand, after a long blind
     * stretch): solves from a grid of heading and position hypotheses spanning about 2 sigma of
     * priorCov and keeps the solution with the most inliers (ties: lower RMS).
     */
    EdgeResult solveWide(const cv::Mat& gray, int64_t tNs, const SE2& prior, const Mat3& priorCov,
                         const SE3& tRobotCam, bool wantOverlay);

    EdgeFrame prepare(const cv::Mat& gray, const SE2& odom, int64_t tNs) const;

    /**
     * Sliding-window solve: one odometry-world -> field alignment A shared by several recent
     * frames (Pinpoint is accurate over a second or two), pose_f = A * odom_f. Motion between
     * the frames gives parallax that a single frame lacks. Returns the newest frame's pose.
     */
    EdgeResult solveWindow(const std::vector<const EdgeFrame*>& frames, const SE2& a0, const Mat3& covA,
                           const SE3& tRobotCam, bool wantOverlay);
    EdgeResult solveWindowWide(const std::vector<const EdgeFrame*>& frames, const SE2& a0, const Mat3& covA,
                               const SE3& tRobotCam, bool wantOverlay);

    /** The 45 alignment hypotheses of a wide search (about 2 sigma of covA, pivoting at the newest
     *  robot position) and the local prior each one is solved with. */
    std::vector<SE2> hypotheses(const SE2& a0, const Mat3& covA, const SE2& newestOdom, Mat3& localPrior) const;

    /** Current tilt state per tipping hive part (-1 = unknown: its edges are not used). */
    void setPartStates(const std::vector<int>& states) { partState_ = states; }

    /**
     * Fraction of the part's edge samples (drawn in tilt state `state` at `pose`) that have an
     * image edge within 2 px, and how many samples were checked.
     */
    double partSupport(const EdgeFrame& f, const SE2& pose, const SE3& tRobotCam, int part, int state, int& samples);

private:
    struct Sample {
        Vec3 world;       // model point, field frame
        Vec2 normal;      // unit image normal at the prediction
        Vec2 px;          // projection at the pose being refined
        std::vector<double> candidates;  // signed offsets along the normal of image edges
        int edge = 0;
        int polarity = 0;  // expected sign of the intensity gradient along `normal` (0 = any)
        bool ridge = false;  // thin dark line (tile seam, panel joint): search intensity minima
        double ridgeOffsetPx = 2;  // compare against the sides this far away (about half the width + 1)
    };

    void makeSamples(const SE2& pose, const SE3& tRobotCam, std::vector<Sample>& out, int onlyPart = -1,
                     int onlyState = -1) const;
    void search(Sample& s, double rangePx, const EdgeFrame& f) const;
    bool project(const SE2& pose, const SE3& tRobotCam, const Vec3& pw, Vec2& px) const;
    /** Field point -> undistorted pixel (pinhole), for surface points. */
    bool projectUndist(const SE2& pose, const SE3& tRobotCam, const Vec3& pw, Vec2& px) const;

    const FieldModel& field_;
    const Camera& cam_;
    const Params& p_;
    std::vector<int> partState_;
};

}  // namespace beacon
