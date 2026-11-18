// Pose prediction from Pinpoint odometry: a time-indexed buffer of odometry samples with
// interpolation at a frame's capture time, and the odometry-world -> field alignment.
#pragma once

#include <cstdint>
#include <mutex>
#include <vector>

#include "geometry.h"

namespace beacon {

struct OdomSample {
    int64_t tNs = 0;
    SE2 pose;  // robot in the odometry world; theta unwrapped (continuous) inside the buffer
};

/**
 * Ring buffer of odometry samples. push() is O(1) and takes a mutex for a few copies only, so
 * it is safe from any thread. Samples must arrive in time order; others are ignored.
 */
class OdomBuffer {
public:
    explicit OdomBuffer(size_t capacity = 8192);

    /** Returns false (and stores nothing) for non-finite or out-of-order samples. */
    bool push(int64_t tNs, const SE2& pose);

    /**
     * Pose at time t: linear interpolation between the bracketing samples, or constant-velocity
     * extrapolation up to maxExtrapNs past the newest sample. False when t is outside that.
     */
    bool at(int64_t tNs, int64_t maxExtrapNs, SE2& out) const;

    /** Planar speed (m/s) and turn rate (rad/s) around t, from samples about +/- 25 ms apart. */
    bool velocity(int64_t tNs, double& speed, double& omega) const;

    bool empty() const;
    int64_t newestT() const;
    int64_t oldestT() const;
    size_t size() const;
    void dropOlderThan(int64_t tNs);

private:
    // Caller holds mu_. Index i counts from the oldest sample.
    const OdomSample& get(size_t i) const { return buf_[(head_ + i) % buf_.size()]; }
    bool atLocked(int64_t tNs, int64_t maxExtrapNs, SE2& out) const;

    mutable std::mutex mu_;
    std::vector<OdomSample> buf_;
    size_t head_ = 0, count_ = 0;
};

/**
 * Odometry-world -> field alignment A (field = A * odom) with a 3x3 covariance, updated like a
 * Kalman filter: grows with distance travelled and turned, shrinks with each accepted fix.
 * Covariance is on (x, y, theta) of A expressed in the field frame.
 */
class Alignment {
public:
    bool initialized() const { return init_; }
    void set(const SE2& a, double sigmaXY, double sigmaTheta);
    const SE2& value() const { return a_; }
    const Mat3& cov() const { return cov_; }

    /** Drift growth between two odometry poses dt seconds apart. */
    void addDrift(const SE2& odomFrom, const SE2& odomTo, double dtS, double perMetre,
                  double headingPerRad, double headingPerS);

    /**
     * Measurement: robot field pose `fix` with covariance fixCov (x, y, theta in field) at an
     * instant when the odometry read odomAtFix. Returns the Mahalanobis^2 of the innovation.
     */
    double update(const SE2& fix, const Mat3& fixCov, const SE2& odomAtFix);

    /** Covariance of the predicted robot field pose for odometry pose `odom` (field frame). */
    Mat3 predictedCov(const SE2& odom) const;

    /** Mahalanobis^2 of a fix against the prediction, without updating. */
    double chi2(const SE2& fix, const Mat3& fixCov, const SE2& odomAtFix) const;

private:
    bool init_ = false;
    SE2 a_;
    Mat3 cov_ = Mat3::Identity();
};

}  // namespace beacon
