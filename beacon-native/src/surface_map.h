// Field mode, C3: ORB map points that live only on known field surfaces (design §5.4 item 2,
// §6.4 "new map points"). A point is created from a single view by intersecting its ORB
// keypoint's ray with the floor (inside the perimeter) or a static hive frame face at the
// current fixed pose; it stays a CANDIDATE (not used to solve poses) until several keyframes
// seen from different places agree that it sits on that surface. Mature points are frozen
// landmarks that the window solve uses next to the model edges (they fix the direction along a
// wall when only parallel rails are visible) and that expose wrong edge locks.
//
// Matching: the keypoint's surface is known, so its reference patch is warped through the
// plane into the predicted view and found by normalized cross-correlation around the
// prediction. (BRIEF descriptors on the foam tiles: about 20 % of floor keypoints matched one
// frame later, about 3 % six frames later; the warped patch: over 90 % half a second later.)
//
// Clear wall panels never get points (the venue shows through them): a ray that meets a wall
// before a surface is rejected, as is one that meets a tipping hive part or any other hive face.
#pragma once

#include <array>
#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

#include "edges.h"
#include "field_model.h"
#include "geometry.h"
#include "orb_features.h"
#include "params.h"

namespace beacon {

struct SurfacePoint {
    struct Obs {
        SE2 odom;      // robot pose in the odometry world at the keyframe
        Vec3 bearing;  // unit ray in the camera frame (undistorted)
    };
    static constexpr int kRefHalf = 15;               // reference patch 31 x 31 px
    static constexpr int kRefSize = 2 * kRefHalf + 1;
    static constexpr int kMaxObs = 8;

    Vec3 pos = Vec3::Zero();  // field frame, on its surface
    int surface = -1;         // -1 = floor, else index into FieldModel::hiveFaces (static frame face)
    int side = 1;             // hive face points: side of the face the camera saw it from
    std::array<uint8_t, kRefSize * kRefSize> ref{};  // reference patch (gray)
    std::array<uint8_t, kRefSize * kRefSize> ref1{}; // same, half resolution, twice the extent
    Vec2 refOrigin = Vec2::Zero();                   // reference image pixel of ref[0]
    Vec2 refOrigin1 = Vec2::Zero();                  // reference image pixel of ref1[0]'s first source pixel
    SE3 refCam = SE3::Identity();                    // T_field_cam of the reference view
    bool mature = false, bad = false;
    int visible = 0, found = 0, strikes = 0;
    int64_t createdT = 0, lastSeenT = 0;
    double consistPx = 0;     // RMS reprojection (px) of the surface-constrained fit
    std::vector<Obs> obs;     // keyframe observations, oldest first (at most kMaxObs)
};

/** Matches of one frame against the map: mature points (solve) and candidates (refinement). */
struct PointMatches {
    std::vector<PointObs> mature, candidates;
    int visibleMature = 0, visibleCandidates = 0;  // projected inside the image
};

class SurfaceMap {
public:
    SurfaceMap(const FieldModel& field, const Camera& cam, const Params& p);

    /** Tilt state per tipping hive part (-1 unknown): their faces occlude in that state only,
     *  in both states while unknown. */
    void setPartStates(const std::vector<int>& s) { partState_ = s; }

    /**
     * Image regions (CV_8U, image size, 255 = allowed) where a field surface that may carry
     * points is seen from `pose`, within the point range, shrunk by about one block so a pose
     * error of a few pixels cannot pull a wall, the venue or a tipping hive part into it.
     */
    void buildMask(const SE2& pose, const SE3& tRobotCam, cv::Mat& mask) const;

    /**
     * Finds the points in `gray` around their projection at the predicted pose (search radius
     * from the prediction covariance) by correlating their plane-warped reference patches.
     */
    void match(const cv::Mat& gray, const SE2& pred, const Mat3& predCov, const SE3& tRobotCam, int64_t tNs,
               PointMatches& out);

    /** True when the robot has moved enough since the last keyframe to add observations. */
    bool keyframeDue(const SE2& odom, int64_t tNs) const;

    /**
     * After each frame with a prediction. pose = align * odom is the accepted fix or, without
     * one, the prediction. On keyframes candidates get one more observation and are matured or
     * culled by the surface test; with an accepted fix that most points agree with, mature
     * points that disagree collect strikes; with an accepted fix better than
     * pointCreateMaxSigmaXY, new candidates are created from ORB keypoints on surfaces.
     */
    void update(int64_t tNs, const cv::Mat& gray, const PointMatches& m, const std::vector<char>& matureInlier,
                bool fixAccepted, bool fixAgreesWithPoints, const SE2& align, const SE2& odom, double fixSigmaXY,
                const SE3& tRobotCam);

    /** Odometry restarted (Pinpoint reboot): candidates' observations are in the old world. */
    void dropCandidates();

    /**
     * First surface hit of a field-frame ray (o, unit dir): returns -1 floor, >= 0 hive frame
     * face index, -2 nothing allowed (wall/panel first, tipping part or other hive face first,
     * outside the perimeter margin, out of range). t = distance along dir.
     */
    int surfaceHit(const Vec3& o, const Vec3& dir, double& t, bool rangeCheck = true) const;

    const std::vector<SurfacePoint>& points() const { return pts_; }
    int matureCount() const;
    /** Keypoints detected at the last creation keyframe (diagnostics). */
    size_t lastDetected() const { return lastDetected_; }

private:
    struct Plane {
        Vec3 o, e1, e2, n;
    };
    Plane plane(int surface) const;
    bool fit(SurfacePoint& p, const SE2& align, const SE3& tRobotCam) const;
    /** Distorted pixel in a view (T_field_cam) -> distorted pixel in the point's reference
     *  view, through the point's plane. */
    bool toRef(const SurfacePoint& sp, const Plane& pl, const SE3& tFieldCam, const Vec2& px, Vec2& refPx) const;
    void setReference(SurfacePoint& sp, const cv::Mat& gray, const Vec2& px, const SE3& tFieldCam) const;
    Vec2 undistort(const Vec2& px) const;  // distorted -> undistorted pixel
    Vec3 rayField(const SE3& tFieldCam, double u, double v) const;  // undistorted pixel -> field ray

    const FieldModel& field_;
    const Camera& cam_;
    const Params& p_;
    Params orbParams_;
    OrbExtractor orb_;
    std::vector<int> partState_;
    std::vector<std::pair<Vec3, double>> faceSphere_;  // bounding sphere per hive face
    Vec3 hiveCentre_ = Vec3::Zero();
    double hiveRadius_ = 0;
    std::vector<SurfacePoint> pts_;
    bool haveKf_ = false;
    SE2 lastKfOdom_;
    int64_t lastKfT_ = 0, lastCreateT_ = 0;
    size_t lastDetected_ = 0;
};

}  // namespace beacon
