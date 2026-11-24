// Small geometry helpers shared by both modes: planar poses (SE2), the camera model with
// OpenCV radial-tangential distortion, and the robot->camera mount.
//
// Naming: T_a_b is the pose of frame b expressed in frame a (maps b coordinates into a).
// Frames: field (F), odometry world (O), robot (R), camera optical (C: x right, y down,
// z forward). Metres, radians.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <cstdint>

namespace beacon {

using Vec2 = Eigen::Vector2d;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;
using SE3 = Eigen::Isometry3d;

/** Wraps an angle into (-pi, pi]. */
inline double wrapAngle(double a) {
    a = std::fmod(a + M_PI, 2.0 * M_PI);
    if (a <= 0) a += 2.0 * M_PI;
    return a - M_PI;
}

/** Planar rigid transform: rotate by theta, then translate by (x, y). */
struct SE2 {
    double x = 0, y = 0, theta = 0;

    SE2() = default;
    SE2(double x_, double y_, double th) : x(x_), y(y_), theta(th) {}

    SE2 operator*(const SE2& b) const {
        double c = std::cos(theta), s = std::sin(theta);
        return SE2(x + c * b.x - s * b.y, y + s * b.x + c * b.y, wrapAngle(theta + b.theta));
    }
    SE2 inverse() const {
        double c = std::cos(theta), s = std::sin(theta);
        return SE2(-c * x - s * y, s * x - c * y, wrapAngle(-theta));
    }
    Vec2 apply(const Vec2& p) const {
        double c = std::cos(theta), s = std::sin(theta);
        return Vec2(x + c * p.x() - s * p.y(), y + s * p.x() + c * p.y());
    }
    /** As a 3-D transform about +z (z unchanged). */
    SE3 toSE3() const {
        SE3 t = SE3::Identity();
        t.linear() = Eigen::AngleAxisd(theta, Vec3::UnitZ()).toRotationMatrix();
        t.translation() = Vec3(x, y, 0);
        return t;
    }
    bool finite() const { return std::isfinite(x) && std::isfinite(y) && std::isfinite(theta); }
};

/**
 * Pinhole camera with OpenCV distortion (k1, k2, p1, p2, k3). project() takes a point in the
 * camera optical frame and returns distorted pixel coordinates.
 */
struct Camera {
    int width = 0, height = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double k1 = 0, k2 = 0, p1 = 0, p2 = 0, k3 = 0;
    double rsReadoutS = 0;

    bool hasDistortion() const { return k1 != 0 || k2 != 0 || p1 != 0 || p2 != 0 || k3 != 0; }

    /** Normalized (undistorted) image coordinates -> distorted normalized coordinates. */
    Vec2 distort(const Vec2& n) const {
        double x = n.x(), y = n.y();
        double r2 = x * x + y * y;
        double radial = 1 + r2 * (k1 + r2 * (k2 + r2 * k3));
        return Vec2(x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                    y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);
    }
    /** Pixel -> undistorted normalized coordinates (fixed-point iteration, as OpenCV does). */
    Vec2 undistortPixel(double u, double v) const {
        Vec2 d((u - cx) / fx, (v - cy) / fy);
        if (!hasDistortion()) return d;
        Vec2 n = d;
        for (int i = 0; i < 20; ++i) {
            Vec2 e = distort(n) - d;
            n -= e;
            if (e.squaredNorm() < 1e-20) break;
        }
        return n;
    }
    /** Camera-frame point -> distorted pixel. Caller checks z > 0. */
    Vec2 project(const Vec3& pc) const {
        Vec2 n(pc.x() / pc.z(), pc.y() / pc.z());
        Vec2 d = distort(n);
        return Vec2(fx * d.x() + cx, fy * d.y() + cy);
    }
    /** Normalized undistorted coords -> pixel without distortion (for the undistorted plane). */
    Vec2 toPixelNoDistortion(const Vec2& n) const { return Vec2(fx * n.x() + cx, fy * n.y() + cy); }
    bool inImage(const Vec2& px, double margin = 0) const {
        return px.x() >= margin && px.y() >= margin && px.x() < width - margin &&
               px.y() < height - margin;
    }
};

/** Builds T_robot_cam from the BeaconConfig mount convention (see beacon.h). */
SE3 mountToTransform(double x, double y, double z, double roll, double pitch, double yaw);

/** Camera pose in the field for a planar robot pose: T_field_cam = T_field_robot * T_robot_cam. */
inline SE3 cameraInField(const SE2& robot, const SE3& tRobotCam) { return robot.toSE3() * tRobotCam; }

}  // namespace beacon
