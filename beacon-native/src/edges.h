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
    std::vector<std::pair<cv::Point2f, cv::Point2f>> overlay;  // projected model edges (final pose)
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

private:
    struct Sample {
        Vec3 world;       // model point, field frame
        Vec2 normal;      // unit image normal at the prediction
        Vec2 px;          // projection at the pose being refined
        std::vector<double> candidates;  // signed offsets along the normal of image edges
        int edge = 0;
    };

    void makeSamples(const SE2& pose, const SE3& tRobotCam, std::vector<Sample>& out) const;
    void search(Sample& s, double rangePx) const;
    bool project(const SE2& pose, const SE3& tRobotCam, const Vec3& pw, Vec2& px) const;

    const FieldModel& field_;
    const Camera& cam_;
    const Params& p_;
    cv::Mat gx_, gy_;  // Sobel gradients (CV_32F) of the current frame
};

}  // namespace beacon
