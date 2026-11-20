// Field mode: Pinpoint odometry is the motion prior; each frame is matched against the field
// model (edges) and field-surface map points to produce an absolute fix in the field frame.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <deque>

#include "edges.h"
#include "engine.h"
#include "gate.h"

namespace beacon {

namespace {

class FieldEngine : public Engine {
public:
    FieldEngine(const FieldModel& field, Shared& s) : field_(field), edges_(field, s.camera, s.params) {
        // Hive parts start in their rule-book start tilt (Event Field Setup Guide).
        for (const MovingPart& mp : field_.parts) {
            partEvidence_.push_back(mp.startState == 1 ? s.params.hiveStartEvidence : -s.params.hiveStartEvidence);
            partState_.push_back(mp.startState);
        }
        edges_.setPartStates(partState_);
    }

    void process(int64_t t, const cv::Mat& gray, Shared& s) override {
        if (firstT_ == 0) firstT_ = t;
        const Params& p = s.params;
        Prediction pred;
        std::string why;
        bool ok = predict(t, s, pred, why);
        EdgeResult er;
        bool stillRelock = false;
        bool accepted = false;
        beacon_fix fix{};
        if (ok) {
            // Poor prediction (typed start pose, long blind stretch): search wide first.
            bool wide = std::sqrt(std::max(pred.cov(0, 0), pred.cov(1, 1))) > p.wideSearchSigmaXY ||
                        std::sqrt(pred.cov(2, 2)) > p.wideSearchSigmaHeading;
            bool fastNow = pred.hasVelocity && (std::abs(pred.omega) > p.gateMaxOmegaRadS || pred.speed > p.gateMaxSpeedMps);
            if (noOdom_) {
                er = wide ? edges_.solveWide(gray, t, pred.pose, pred.cov, s.tRobotCam, observer_ != nullptr)
                          : edges_.solve(gray, t, pred.pose, pred.cov, s.tRobotCam, observer_ != nullptr);
            } else if (!fastNow) {
                // Sliding window of recent frames tied together by odometry.
                EdgeFrame f = edges_.prepare(gray, pred.odom, t);
                if (!window_.empty() && (t - window_.back().tNs) * 1e-9 < p.edgeWindowMinSpacingS)
                    window_.back() = std::move(f);  // keep frames spread out; newest always last
                else
                    window_.push_back(std::move(f));
                // The long window (for the wide search, which needs motion) and the short one used
                // to track: odometry scale error (up to 1.5 %) makes frames more than a few tenths
                // of a second apart disagree by centimetres, so tracking ties only the newest few.
                while ((int)window_.size() > std::max({1, p.edgeWindowFrames, p.edgeWideWindowFrames})) window_.pop_front();
                std::vector<const EdgeFrame*> all, fr;
                for (const EdgeFrame& w : window_) all.push_back(&w);
                for (size_t k = window_.size() - std::min(window_.size(), (size_t)std::max(1, p.edgeWindowFrames));
                     k < window_.size(); ++k)
                    fr.push_back(&window_[k]);
                // A single view of the field can be ambiguous (two poses explain the edges equally
                // well); a wide search therefore waits until the window holds some motion.
                double spread = 0, turn = 0;
                for (const EdgeFrame& w : window_) {
                    spread = std::max(spread, std::hypot(w.odom.x - window_.back().odom.x, w.odom.y - window_.back().odom.y));
                    turn = std::max(turn, std::abs(wrapAngle(w.odom.theta - window_.back().odom.theta)));
                }
                bool diverse = spread >= p.wideMinSpreadM || turn >= p.wideMinTurnRad;
                // Wide searches (no lock yet) and lock re-checks run incrementally: at most
                // searchSolvesPerFrame hypothesis solves per frame, so no frame stalls (Hub budget).
                // A single still view can be ambiguous, so searches start only with motion.
                if (!search_.active && diverse) {
                    bool verifyDue = !wide && p.relockCheckS > 0 && (t - lastVerifyT_) * 1e-9 >= p.relockCheckS;
                    if (wide || verifyDue) {
                        Mat3 cov = align_.cov();
                        if (!wide) {
                            cov = Mat3::Zero();
                            cov(0, 0) = cov(1, 1) = std::pow(p.relockGridXY / 0.75, 2);
                            cov(2, 2) = std::pow(p.relockGridHeading / 0.75, 2);
                            lastVerifyT_ = t;
                        }
                        search_.init = edges_.hypotheses(align_.value(), cov, window_.back().odom, search_.local);
                        search_.verify = !wide;
                        search_.next = 0;
                        search_.found.clear();
                        search_.active = true;
                    }
                }
                if (!wide) {
                    er = edges_.solveWindow(fr, align_.value(), align_.cov(), s.tRobotCam, observer_ != nullptr);
                } else if (!search_.active && !diverse && lastFixT_ != 0) {
                    // Lost precision while standing still after tracking: a local solve from the
                    // prediction (wide search range) is safer than a grid search from one view;
                    // it must pass a stricter quality bar (below).
                    er = edges_.solveWindow(fr, align_.value(), align_.cov(), s.tRobotCam, observer_ != nullptr);
                    stillRelock = true;
                } else {
                    why = search_.active ? "searching for the start pose" : "waiting for motion to resolve the start pose";
                }
                if (search_.active) stepSearch(fr, er, wide, s);
            }
            // Calibrated uncertainty (fit against truth on the hard runs; see DECISIONS.md):
            // error grows with the residual RMS and falls with the inlier ratio,
            // q = rms_px / ratio^2; linear in q with a quadratic tail above the knee. It replaces
            // the edge covariance (about 10x too small) everywhere: the gate, the chi-square
            // test, the core's own alignment update and the published sigma.
            double sigXY = 0, sigH = 0;
            if (er.ok) {
                double ratio = er.samples > 0 ? (double)er.inliers / er.samples : 0.0;
                double q = er.rmsPx / std::pow(std::max(0.3, ratio), 2);
                double qs = q + p.fixSigmaQuadratic / p.fixSigmaPerQXY * std::pow(std::max(0.0, q - p.fixSigmaKneeQ), 2);
                sigXY = std::max({p.fixSigmaFloorXY, p.fixSigmaPerQXY * qs, std::sqrt(std::max(er.cov(0, 0), er.cov(1, 1)))});
                sigH = std::max({p.fixSigmaFloorHeading, p.fixSigmaPerQHeading * qs, std::sqrt(er.cov(2, 2))});
                er.cov = Mat3::Zero();
                er.cov(0, 0) = er.cov(1, 1) = sigXY * sigXY;
                er.cov(2, 2) = sigH * sigH;
            }
            GateInput g;
            g.result = &er;
            g.predCov = pred.cov;
            g.predPose = pred.pose;
            g.speed = pred.speed;
            g.omega = pred.omega;
            g.hasVelocity = pred.hasVelocity;
            g.chi2 = er.ok ? chi2(er, pred) : 0;
            GateDecision d = gate(g, p);
            if (d.accept && p.partMinInlierFrac > 0 && er.worstPartSamples >= p.partMinSamples &&
                er.worstPartFrac < p.partMinInlierFrac) {
                d = GateDecision{false, "jump: a model part disagrees"};  // counts toward re-locking
                lastVerifyT_ = 0;  // re-check the lock as soon as the window has motion
            }
            if (er.ambiguous && d.accept) d = GateDecision{false, "ambiguous view"};
            if (stillRelock && d.accept && (er.inliers < p.stillRelockMinInliers || sigXY > p.stillRelockMaxSigmaXY))
                d = GateDecision{false, "still: weak re-lock"};

            why = d.reason;
            // Repeated rejections only for disagreeing with the prediction mean the prediction
            // is what is wrong (odometry slip or heading jump in a collision): widen it so the
            // next frames can re-lock (wide search once the sigma is large).
            // Escalation: a mild widening first (a slip of a few cm re-locks locally); if the
            // rejections go on, a full widening that starts a wide search.
            if (!d.accept && d.reason.rfind("jump", 0) == 0) {
                ++jumpRejects_;
                if (jumpRejects_ % p.relockAfterRejects == 0) {
                    bool full = jumpRejects_ >= 3 * p.relockAfterRejects;
                    align_.inflate(full ? p.relockWideSigmaXY : p.relockSigmaXY,
                                   full ? p.relockWideSigmaHeading : p.relockSigmaHeading);
                    if (full) jumpRejects_ = 0;
                }
            } else if (d.accept) {
                jumpRejects_ = 0;
            }
            if (d.accept) {
                accepted = true;
                fix.t_ns = t;
                fix.x = er.pose.x;
                fix.y = er.pose.y;
                fix.heading = er.pose.theta;
                fix.sigma_xy = sigXY;
                fix.sigma_heading = sigH;
                fix.inliers = er.inliers;
                fix.kind = BEACON_FIX_EDGES | (noOdom_ ? BEACON_FIX_NO_ODOM : 0);
                if (const char* dump = std::getenv("BEACON_FIX_DUMP")) {  // bench: sigma calibration data
                    static FILE* df = nullptr;
                    if (!df) {
                        df = std::fopen(dump, "w");
                        if (df) std::fprintf(df, "t_ns,sig_cov_xy,sig_cov_h,inliers,matched,samples,rms_px,window,speed,omega,chi2\n");
                    }
                    if (df)
                        std::fprintf(df, "%lld,%.5f,%.5f,%d,%d,%d,%.3f,%zu,%.3f,%.3f,%.2f\n", (long long)t,
                                     std::sqrt(std::max(er.cov(0, 0), er.cov(1, 1))), std::sqrt(er.cov(2, 2)), er.inliers,
                                     er.matched, er.samples, er.rmsPx, window_.size(), pred.speed, pred.omega, g.chi2);
                }
                applyFix(t, er, s);
                publishFix(s, fix);
                lastFixT_ = t;
                if (!window_.empty()) updateHiveStates(window_.back(), er.pose, s);
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
            if (s.odomEpoch != odomEpoch_) {
                // Pinpoint restarted: re-anchor the new odometry world at our last field pose.
                odomEpoch_ = s.odomEpoch;
                window_.clear();
                search_.active = false;
                if (align_.initialized() && hasLastPose_) reanchor_ = true;
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
            rawLast_ = o;
            corr_ = o;
            lastOdom_ = o;
            lastOdomT_ = startPose_.t_ns;
            pendingStart_ = false;
        }
        if (reanchor_ && !s.odom.empty()) {
            SE2 o0;
            s.odom.at(s.odom.oldestT(), 0, o0);
            align_.set(lastPose_ * o0.inverse(), p.startSigmaXY, p.startSigmaHeading);
            rawLast_ = o0;
            corr_ = o0;
            resetScalePair();
            lastOdom_ = o0;
            lastOdomT_ = s.odom.oldestT();
            reanchor_ = false;
        }
        if (!align_.initialized()) {
            why = "waiting for start pose";
            return false;
        }
        if (reanchor_) {
            why = "odometry restarted, waiting for samples";
            return false;
        }
        SE2 o;
        if (!s.odom.at(t, maxEx, o)) {
            why = "no odometry at frame time";
            return false;
        }
        // Scale-corrected odometry: Pinpoint increments with the estimated per-axis scale.
        {
            SE2 d = rawLast_.inverse() * o;
            double phi = align_.value().theta + corr_.theta;  // robot heading in the field
            double c = std::cos(phi), sn = std::sin(phi);
            scaleU_ += Vec2(c * d.x, sn * d.x);
            scaleV_ += Vec2(-sn * d.y, c * d.y);
            scalePath_ += std::hypot(d.x, d.y);
            corr_ = corr_ * SE2(scaleX_ * d.x, scaleY_ * d.y, d.theta);
            corr_.theta = wrapAngle(corr_.theta);
            rawLast_ = o;
            o = corr_;
        }
        align_.addDrift(lastOdom_, o, (t - lastOdomT_) * 1e-9, p.driftPerMetre, p.driftHeadingPerRad,
                        p.driftHeadingPerS);
        lastOdom_ = o;
        lastOdomT_ = t;
        out.odom = o;
        out.pose = align_.value() * o;
        out.cov = align_.predictedCov(o);
        lastPose_ = out.pose;
        hasLastPose_ = true;
        out.hasVelocity = s.odom.velocity(t, out.speed, out.omega);
        s.odom.dropOlderThan(t - (int64_t)(p.odomHistoryS * 1e9));
        return true;
    }

    // Two-state tilt estimate per hive part: compare how well each tilt's edges are seen from a
    // pose we trust (an accepted fix). Evidence decays, so a tip during the match is followed;
    // while a hive moves both states score low and its edges are dropped (state -1).
    void updateHiveStates(const EdgeFrame& f, const SE2& pose, Shared& s) {
        const Params& p = s.params;
        bool changed = false;
        for (size_t i = 0; i < field_.parts.size(); ++i) {
            int n0 = 0, n1 = 0;
            double a = edges_.partSupport(f, pose, s.tRobotCam, (int)i, 0, n0);
            double b = edges_.partSupport(f, pose, s.tRobotCam, (int)i, 1, n1);
            if (std::getenv("BEACON_HIVE_DEBUG") && (n0 > 0 || n1 > 0))
                std::fprintf(stderr, "hivesup %s t=%lld n %d %d support %.2f %.2f ev %.2f\n", field_.parts[i].name.c_str(),
                             (long long)f.tNs, n0, n1, a, b, partEvidence_[i]);
            // Usually only one tilt's edges are in view: a well supported state is evidence for
            // it, an unsupported one (in view, edges absent) evidence against it.
            bool in0 = n0 >= p.hiveMinSamples, in1 = n1 >= p.hiveMinSamples;
            if (!in0 && !in1) continue;
            double e = (in1 ? b - p.hiveSupportNeutral : 0.0) - (in0 ? a - p.hiveSupportNeutral : 0.0);
            partEvidence_[i] = p.hiveEvidenceDecay * partEvidence_[i] + p.hiveEvidenceGain * e;
            int st = partEvidence_[i] > p.hiveDecide ? 1 : partEvidence_[i] < -p.hiveDecide ? 0 : -1;
            if (st != partState_[i]) {
                partState_[i] = st;
                changed = true;
                if (std::getenv("BEACON_HIVE_DEBUG"))
                    std::fprintf(stderr, "hive %s -> state %d at t=%lld (support %.2f / %.2f)\n", field_.parts[i].name.c_str(),
                                 st, (long long)f.tNs, a, b);
            }
        }
        if (changed) edges_.setPartStates(partState_);
    }

    // One slice of the incremental hypothesis search. When all hypotheses are done, the distinct
    // converged candidates are re-scored on the current window and the best one is taken (wide)
    // or compared with the current lock (re-check).
    void stepSearch(const std::vector<const EdgeFrame*>& fr, EdgeResult& er, bool wide, Shared& s) {
        const Params& p = s.params;
        // Time budget: at least one hypothesis per frame, then stop once searchBudgetMs is spent.
        auto t0 = std::chrono::steady_clock::now();
        auto spent = [&] { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count(); };
        for (int k = 0; k < p.searchSolvesPerFrame && search_.next < search_.init.size(); ++k, ++search_.next) {
            // Wall-clock budget on the robot; replays (synchronous) use the solve count only, so
            // results do not depend on machine load.
            if (k > 0 && !p.synchronous && spent() > p.searchBudgetMs) break;
            EdgeResult r = edges_.solveWindow(fr, search_.init[search_.next], search_.local, s.tRobotCam, false);
            if (r.ok) search_.found.push_back({r.inliers, r.alignment});
        }
        if (search_.next < search_.init.size()) return;
        if (!p.synchronous && spent() > 0.5 * p.searchBudgetMs) return;  // re-score on the next frame
        search_.active = false;
        // Distinct candidates (3 cm / 1.5 deg apart at the newest frame), best first.
        std::sort(search_.found.begin(), search_.found.end(),
                  [](const std::pair<int, SE2>& a, const std::pair<int, SE2>& b) { return a.first > b.first; });
        const SE2& o = fr.back()->odom;
        std::vector<SE2> cands;
        for (const auto& c : search_.found) {
            SE2 pc = c.second * o;
            bool dup = false;
            for (const SE2& d : cands) {
                SE2 pd = d * o;
                if (std::hypot(pc.x - pd.x, pc.y - pd.y) < 0.03 && std::abs(wrapAngle(pc.theta - pd.theta)) < 0.026) dup = true;
            }
            if (!dup) cands.push_back(c.second);
            if ((int)cands.size() >= p.searchRescoreTop) break;
        }
        if (cands.empty()) return;
        std::vector<EdgeResult> rs;
        for (const SE2& c : cands) rs.push_back(edges_.solveWindow(fr, c, search_.local, s.tRobotCam, observer_ != nullptr));
        size_t bi = 0;
        for (size_t i = 1; i < rs.size(); ++i)
            if (rs[i].ok && rs[i].inliers > rs[bi].inliers) bi = i;
        EdgeResult best = rs[bi];
        double margin = std::max(p.wideAmbiguityMinInliers, p.wideAmbiguityFraction * best.inliers);
        for (size_t i = 0; i < rs.size(); ++i) {
            if (i == bi || !rs[i].ok) continue;
            double dpos = std::hypot(rs[i].pose.x - best.pose.x, rs[i].pose.y - best.pose.y);
            double dth = std::abs(wrapAngle(rs[i].pose.theta - best.pose.theta));
            if ((dpos > 0.03 || dth > 0.026) && rs[i].inliers >= best.inliers - margin) best.ambiguous = true;
        }
        if (!search_.verify) {
            if (wide) er = best;  // gated below (ambiguous answers are not published)
            return;
        }
        if (!er.ok || best.ambiguous) return;
        double dpos = std::hypot(best.pose.x - er.pose.x, best.pose.y - er.pose.y);
        double dth = std::abs(wrapAngle(best.pose.theta - er.pose.theta));
        if ((dpos > 0.03 || dth > 0.026) && best.inliers > er.inliers * (1.0 + p.relockMargin) + 10) {
            if (std::getenv("BEACON_RELOCK_DEBUG"))
                std::fprintf(stderr, "relock: %d -> %d inliers, moved %.1f cm %.2f deg\n", er.inliers, best.inliers,
                             100 * dpos, dth * 180 / M_PI);
            er = best;
            align_.set(er.alignment, p.relockSigmaXY, p.relockSigmaHeading);
            relocks_++;
        }
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
        lastPose_ = er.pose;
        updateScale(t, er.pose, s.params);
    }

    void resetScalePair() {
        scaleU_ = scaleV_ = Vec2::Zero();
        scalePath_ = 0;
        scaleHasFix_ = false;
    }

    // Online per-axis odometry scale: between two accepted fixes the field displacement is
    // sx * U + sy * V, where U and V sum the raw forward and sideways increments rotated into
    // the field. Least squares with forgetting; corrected odometry then lets the window span
    // longer without the scale error (hard profile: up to 1.5 %) biasing it.
    void updateScale(int64_t t, const SE2& fixPose, const Params& p) {
        if (!p.odomScaleEstimate) return;
        if (scaleHasFix_ && scalePath_ >= p.odomScaleMinPathM && (t - scaleFixT_) * 1e-9 <= p.odomScaleMaxPairS) {
            Vec2 d(fixPose.x - scaleFixPos_.x(), fixPose.y - scaleFixPos_.y());
            Eigen::Matrix<double, 2, 2> A;
            A.col(0) = scaleU_;
            A.col(1) = scaleV_;
            scaleN_ = p.odomScaleForget * scaleN_ + A.transpose() * A;
            scaleB_ = p.odomScaleForget * scaleB_ + A.transpose() * d;
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es(scaleN_);
            if (es.eigenvalues().minCoeff() > p.odomScaleMinInfo) {
                Vec2 sc = scaleN_.ldlt().solve(scaleB_);
                scaleX_ = std::min(1.05, std::max(0.95, sc.x()));
                scaleY_ = std::min(1.05, std::max(0.95, sc.y()));
            }
        }
        if (!scaleHasFix_ || scalePath_ >= p.odomScaleMinPathM || (t - scaleFixT_) * 1e-9 > p.odomScaleMaxPairS) {
            scaleU_ = scaleV_ = Vec2::Zero();
            scalePath_ = 0;
            scaleFixPos_ = Vec2(fixPose.x, fixPose.y);
            scaleFixT_ = t;
            scaleHasFix_ = true;
        }
    }

    void updateStatus(int64_t t, Shared& s, bool predicted, bool accepted, const std::string& why,
                      const EdgeResult& er) {
        char line[200];
        int status;
        if (lastFixT_ == 0) status = BEACON_STATUS_INITIALIZING;
        else if ((t - lastFixT_) * 1e-9 > s.params.lostAfterS) status = BEACON_STATUS_LOST;
        else status = BEACON_STATUS_TRACKING;
        std::snprintf(line, sizeof line, "vision %s: %s%s (edges %d/%d, odom scale %.3f %.3f)",
                      status == BEACON_STATUS_TRACKING ? "tracking"
                      : status == BEACON_STATUS_LOST   ? "degraded"
                                                       : "initializing",
                      accepted ? "fix" : (predicted ? "no fix, " : ""), accepted ? "" : why.c_str(),
                      er.inliers, er.samples, scaleX_, scaleY_);
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
    uint64_t odomEpoch_ = 0;
    bool reanchor_ = false, hasLastPose_ = false;
    SE2 lastPose_;
    SE2 lastOdom_;
    int64_t lastOdomT_ = 0;
    int64_t lastFixT_ = 0;
    bool firstViz_ = true;
    std::vector<Vec3> trajectory_;
    std::deque<EdgeFrame> window_;
    int jumpRejects_ = 0;
    int64_t lastVerifyT_ = 0, firstT_ = 0;
    int relocks_ = 0;
    SE2 rawLast_, corr_;
    double scaleX_ = 1.0, scaleY_ = 1.0;
    Vec2 scaleU_ = Vec2::Zero(), scaleV_ = Vec2::Zero(), scaleFixPos_ = Vec2::Zero();
    double scalePath_ = 0;
    int64_t scaleFixT_ = 0;
    bool scaleHasFix_ = false;
    Eigen::Matrix2d scaleN_ = Eigen::Matrix2d::Zero();
    Eigen::Vector2d scaleB_ = Eigen::Vector2d::Zero();
    struct Search {
        bool active = false, verify = false;
        std::vector<SE2> init;
        size_t next = 0;
        std::vector<std::pair<int, SE2>> found;  // (inliers, converged alignment)
        Mat3 local = Mat3::Identity();
    } search_;
    std::vector<double> partEvidence_;
    std::vector<int> partState_;
};

}  // namespace

std::unique_ptr<Engine> makeFieldEngine(const FieldModel& field, Shared& shared) {
    return std::make_unique<FieldEngine>(field, shared);
}

}  // namespace beacon
