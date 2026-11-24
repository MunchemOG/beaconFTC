#include "predict.h"

#include <algorithm>
#include <cmath>

namespace beacon {

// ------------------------------------------------------------------------------ OdomBuffer

OdomBuffer::OdomBuffer(size_t capacity) : buf_(std::max<size_t>(capacity, 4)) {}

bool OdomBuffer::push(int64_t tNs, const SE2& pose) {
    if (!pose.finite()) return false;
    std::lock_guard<std::mutex> lock(mu_);
    SE2 p = pose;
    if (count_ > 0) {
        const OdomSample& last = get(count_ - 1);
        if (tNs <= last.tNs) return false;
        // Keep heading continuous so interpolation never goes the long way round.
        p.theta = last.pose.theta + wrapAngle(pose.theta - last.pose.theta);
    }
    if (count_ == buf_.size()) {  // full: overwrite the oldest
        head_ = (head_ + 1) % buf_.size();
        --count_;
    }
    buf_[(head_ + count_) % buf_.size()] = OdomSample{tNs, p};
    ++count_;
    return true;
}

static SE2 lerp(const OdomSample& a, const OdomSample& b, int64_t t) {
    double u = (double)(t - a.tNs) / (double)(b.tNs - a.tNs);
    return SE2(a.pose.x + u * (b.pose.x - a.pose.x), a.pose.y + u * (b.pose.y - a.pose.y),
               a.pose.theta + u * (b.pose.theta - a.pose.theta));
}

bool OdomBuffer::atLocked(int64_t t, int64_t maxExtrapNs, SE2& out) const {
    if (count_ == 0) return false;
    const OdomSample& first = get(0);
    const OdomSample& last = get(count_ - 1);
    if (t < first.tNs) return false;
    if (t >= last.tNs) {
        if (t - last.tNs > maxExtrapNs) return false;
        if (count_ < 2) {
            out = last.pose;
        } else {
            out = lerp(get(count_ - 2), last, t);  // u > 1: constant-velocity extrapolation
        }
        out.theta = wrapAngle(out.theta);
        return true;
    }
    // Binary search for the last sample with tNs <= t.
    size_t lo = 0, hi = count_ - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (get(mid).tNs <= t) lo = mid;
        else hi = mid;
    }
    out = lerp(get(lo), get(hi), t);
    out.theta = wrapAngle(out.theta);
    return true;
}

bool OdomBuffer::at(int64_t t, int64_t maxExtrapNs, SE2& out) const {
    std::lock_guard<std::mutex> lock(mu_);
    return atLocked(t, maxExtrapNs, out);
}

bool OdomBuffer::velocity(int64_t t, double& speed, double& omega) const {
    std::lock_guard<std::mutex> lock(mu_);
    const int64_t h = 25000000;  // 25 ms
    SE2 a, b;
    int64_t t0 = t - h, t1 = t + h;
    if (count_ == 0) return false;
    t0 = std::max(t0, get(0).tNs);
    t1 = std::min(t1, get(count_ - 1).tNs);
    if (t1 - t0 < 2000000) return false;
    if (!atLocked(t0, 0, a) || !atLocked(t1, 0, b)) return false;
    double dt = (t1 - t0) * 1e-9;
    speed = std::hypot(b.x - a.x, b.y - a.y) / dt;
    omega = wrapAngle(b.theta - a.theta) / dt;
    return true;
}

bool OdomBuffer::empty() const {
    std::lock_guard<std::mutex> lock(mu_);
    return count_ == 0;
}
int64_t OdomBuffer::newestT() const {
    std::lock_guard<std::mutex> lock(mu_);
    return count_ ? get(count_ - 1).tNs : 0;
}
int64_t OdomBuffer::oldestT() const {
    std::lock_guard<std::mutex> lock(mu_);
    return count_ ? get(0).tNs : 0;
}
size_t OdomBuffer::size() const {
    std::lock_guard<std::mutex> lock(mu_);
    return count_;
}
void OdomBuffer::clear() {
    std::lock_guard<std::mutex> lock(mu_);
    head_ = 0;
    count_ = 0;
}

void OdomBuffer::dropOlderThan(int64_t t) {
    std::lock_guard<std::mutex> lock(mu_);
    while (count_ > 2 && get(1).tNs < t) {
        head_ = (head_ + 1) % buf_.size();
        --count_;
    }
}

// ------------------------------------------------------------------------------- Alignment

void Alignment::set(const SE2& a, double sxy, double sth) {
    a_ = a;
    cov_ = Mat3::Zero();
    cov_(0, 0) = cov_(1, 1) = sxy * sxy;
    cov_(2, 2) = sth * sth;
    init_ = true;
}

// d(robot field pose)/d(alignment params ax, ay, atheta) at odometry pose o.
static Mat3 jacobian(const SE2& a, const SE2& o) {
    SE2 p = a * o;
    Mat3 j = Mat3::Identity();
    j(0, 2) = -(p.y - a.y);
    j(1, 2) = p.x - a.x;
    return j;
}

void Alignment::addDrift(const SE2& from, const SE2& to, double dt, double perM, double perRad,
                         double perS) {
    double dist = std::hypot(to.x - from.x, to.y - from.y);
    double turn = std::abs(wrapAngle(to.theta - from.theta));
    double sxy = perM * dist;
    double sth = perRad * turn + perS * std::max(0.0, dt);
    Mat3 q = Mat3::Zero();
    q(0, 0) = q(1, 1) = sxy * sxy;
    q(2, 2) = sth * sth;
    Mat3 jinv = jacobian(a_, to).inverse();
    cov_ += jinv * q * jinv.transpose();
}

Mat3 Alignment::predictedCov(const SE2& o) const {
    Mat3 j = jacobian(a_, o);
    return j * cov_ * j.transpose();
}

static Eigen::Vector3d innovation(const SE2& fix, const SE2& pred) {
    return Eigen::Vector3d(fix.x - pred.x, fix.y - pred.y, wrapAngle(fix.theta - pred.theta));
}

double Alignment::chi2(const SE2& fix, const Mat3& fixCov, const SE2& o) const {
    Mat3 s = predictedCov(o) + fixCov;
    Eigen::Vector3d r = innovation(fix, a_ * o);
    return r.dot(s.ldlt().solve(r));
}

double Alignment::update(const SE2& fix, const Mat3& fixCov, const SE2& o) {
    Mat3 j = jacobian(a_, o);
    Mat3 s = j * cov_ * j.transpose() + fixCov;
    Eigen::Vector3d r = innovation(fix, a_ * o);
    double m2 = r.dot(s.ldlt().solve(r));
    Eigen::Matrix3d k = cov_ * j.transpose() * s.inverse();
    Eigen::Vector3d da = k * r;
    a_.x += da.x();
    a_.y += da.y();
    a_.theta = wrapAngle(a_.theta + da.z());
    cov_ = (Mat3::Identity() - k * j) * cov_;
    cov_ = 0.5 * (cov_ + cov_.transpose());
    return m2;
}

}  // namespace beacon
