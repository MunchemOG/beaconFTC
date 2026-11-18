// Field mode: Pinpoint odometry is the motion prior; each frame is matched against the field
// model (edges) and field-surface map points to produce an absolute fix in the field frame.
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "edges.h"
#include "engine.h"
#include "gate.h"

namespace beacon {

namespace {

class FieldEngine : public Engine {
public:
    FieldEngine(const FieldModel& field, Shared& s) : field_(field), edges_(field, s.camera, s.params) {}

    void process(int64_t t, const cv::Mat& gray, Shared& s) override {
        const Params& p = s.params;
        Prediction pred;
        std::string why;
        bool ok = predict(t, s, pred, why);
        EdgeResult er;
        bool accepted = false;
        beacon_fix fix{};
        if (ok) {
            er = edges_.solve(gray, t, pred.pose, pred.cov, s.tRobotCam, observer_ != nullptr);
            GateInput g;
            g.result = &er;
            g.predCov = pred.cov;
            g.predPose = pred.pose;
            g.speed = pred.speed;
            g.omega = pred.omega;
            g.hasVelocity = pred.hasVelocity;
            g.chi2 = er.ok ? chi2(er, pred) : 0;
            GateDecision d = gate(g, p);
            why = d.reason;
            if (d.accept) {
                accepted = true;
                fix.t_ns = t;
                fix.x = er.pose.x;
                fix.y = er.pose.y;
                fix.heading = er.pose.theta;
                fix.sigma_xy = std::sqrt(std::max(er.cov(0, 0), er.cov(1, 1)));
                fix.sigma_heading = std::sqrt(er.cov(2, 2));
                fix.inliers = er.inliers;
                fix.kind = BEACON_FIX_EDGES | (noOdom_ ? BEACON_FIX_NO_ODOM : 0);
                applyFix(t, er, s);
                publishFix(s, fix);
                lastFixT_ = t;
            }
        }
        updateStatus(t, s, ok, accepted, why, er);
        if (observer_) {
            VizFrame v;
            v.tNs = t;
            v.image = gray;
            v.projectedEdges = er.overlay;
            v.poseValid = ok;
            v.tWorldCam = cameraInField(accepted ? er.pose : pred.pose, s.tRobotCam);
            v.tCamRobot = s.tRobotCam.inverse();
            {
                std::lock_guard<std::mutex> l(s.mu);
                v.status = s.status;
                v.text = s.debug;
            }
            v.hasFix = accepted;
            v.fix = fix;
            if (ok && !noOdom_) {
                v.odomValid = true;
                v.tWorldCamOdom = cameraInField(pred.pose, s.tRobotCam);
            }
            if (firstViz_) {
                for (const ModelEdge& e : field_.edges) v.modelEdges.push_back({e.a, e.b});
                firstViz_ = false;
            }
            trajectory_.push_back(v.tWorldCam.translation());
            v.trajectory = trajectory_;
            v.mapChanged = true;
            observer_(v);
        }
    }

private:
    struct Prediction {
        SE2 pose, odom;
        Mat3 cov = Mat3::Identity();
        double speed = 0, omega = 0;
        bool hasVelocity = false;
    };

    // Predicts the robot field pose at capture time t from odometry and the alignment, or (bench
    // only, no odometry at all) from the last fixes with constant velocity.
    bool predict(int64_t t, Shared& s, Prediction& out, std::string& why) {
        const Params& p = s.params;
        beacon_pose2 start{};
        bool newStart = false;
        {
            std::lock_guard<std::mutex> l(s.mu);
            if (s.hasStart && s.startSeq != startSeq_) {
                start = s.start;
                startSeq_ = s.startSeq;
                newStart = true;
            }
        }
        if (newStart) {
            pendingStart_ = true;
            startPose_ = start;
        }
        noOdom_ = s.odom.empty();
        if (noOdom_) {
            // Bench-only: the "odometry" is the core's own fix history in the field frame.
            if (pendingStart_ && !align_.initialized()) {
                align_.set(SE2(), p.noOdomSigmaXY, p.noOdomSigmaHeading);
                selfOdom_.push(startPose_.t_ns, SE2(startPose_.x, startPose_.y, startPose_.heading));
                pendingStart_ = false;
            }
            if (!align_.initialized()) {
                why = "waiting for start pose";
                return false;
            }
            SE2 o;
            if (!selfOdom_.at(t, (int64_t)(5e9), o)) {
                // Constant position when only one sample exists or t precedes it.
                if (selfOdom_.empty()) return false;
                selfOdom_.at(selfOdom_.newestT(), 0, o);
            }
            out.pose = o;
            out.odom = o;
            out.cov = Mat3::Zero();
            double dt = (t - selfOdom_.newestT()) * 1e-9;
            double grow = 1.0 + 4.0 * std::max(0.0, dt);
            out.cov(0, 0) = out.cov(1, 1) = std::pow(p.noOdomSigmaXY * grow, 2);
            out.cov(2, 2) = std::pow(p.noOdomSigmaHeading * grow, 2);
            out.hasVelocity = false;
            return true;
        }
        int64_t maxEx = (int64_t)(p.odomMaxExtrapolateS * 1e9);
        if (pendingStart_) {
            SE2 o;
            // Align at the start pose's own time; if odometry does not reach back that far, use
            // the oldest sample (the robot is still in INIT, G304 H).
            if (!s.odom.at(startPose_.t_ns, maxEx, o)) s.odom.at(s.odom.oldestT(), 0, o);
            SE2 f(startPose_.x, startPose_.y, startPose_.heading);
            align_.set(f * o.inverse(), p.startSigmaXY, p.startSigmaHeading);
            lastOdom_ = o;
            lastOdomT_ = startPose_.t_ns;
            pendingStart_ = false;
        }
        if (!align_.initialized()) {
            why = "waiting for start pose";
            return false;
        }
        SE2 o;
        if (!s.odom.at(t, maxEx, o)) {
            why = "no odometry at frame time";
            return false;
        }
        align_.addDrift(lastOdom_, o, (t - lastOdomT_) * 1e-9, p.driftPerMetre, p.driftHeadingPerRad,
                        p.driftHeadingPerS);
        lastOdom_ = o;
        lastOdomT_ = t;
        out.odom = o;
        out.pose = align_.value() * o;
        out.cov = align_.predictedCov(o);
        out.hasVelocity = s.odom.velocity(t, out.speed, out.omega);
        s.odom.dropOlderThan(t - (int64_t)(p.odomHistoryS * 1e9));
        return true;
    }

    double chi2(const EdgeResult& er, const Prediction& pred) const {
        Mat3 sum = pred.cov + er.cov;
        Eigen::Vector3d r(er.pose.x - pred.pose.x, er.pose.y - pred.pose.y,
                          wrapAngle(er.pose.theta - pred.pose.theta));
        return r.dot(sum.ldlt().solve(r));
    }

    void applyFix(int64_t t, const EdgeResult& er, Shared& s) {
        if (noOdom_) {
            selfOdom_.push(t, er.pose);
            return;
        }
        align_.update(er.pose, er.cov, lastOdom_);
        (void)s;
    }

    void updateStatus(int64_t t, Shared& s, bool predicted, bool accepted, const std::string& why,
                      const EdgeResult& er) {
        char line[200];
        int status;
        if (lastFixT_ == 0) status = BEACON_STATUS_INITIALIZING;
        else if ((t - lastFixT_) * 1e-9 > s.params.lostAfterS) status = BEACON_STATUS_LOST;
        else status = BEACON_STATUS_TRACKING;
        std::snprintf(line, sizeof line, "vision %s: %s%s (edges %d/%d)",
                      status == BEACON_STATUS_TRACKING ? "tracking"
                      : status == BEACON_STATUS_LOST   ? "degraded"
                                                       : "initializing",
                      accepted ? "fix" : (predicted ? "no fix, " : ""), accepted ? "" : why.c_str(),
                      er.inliers, er.samples);
        std::lock_guard<std::mutex> l(s.mu);
        s.status = status;
        s.debug = line;
    }

    const FieldModel& field_;
    EdgeTracker edges_;
    Alignment align_;
    OdomBuffer selfOdom_{256};
    bool noOdom_ = false;
    bool pendingStart_ = false;
    beacon_pose2 startPose_{};
    uint64_t startSeq_ = 0;
    SE2 lastOdom_;
    int64_t lastOdomT_ = 0;
    int64_t lastFixT_ = 0;
    bool firstViz_ = true;
    std::vector<Vec3> trajectory_;
};

}  // namespace

std::unique_ptr<Engine> makeFieldEngine(const FieldModel& field, Shared& shared) {
    return std::make_unique<FieldEngine>(field, shared);
}

}  // namespace beacon
