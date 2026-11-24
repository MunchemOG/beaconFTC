// Explore mode: general monocular ORB-SLAM-style SLAM (features anywhere, keyframes, local BA,
// bag-of-words loop closing with essential-graph optimization), field-free. Without odometry
// the map starts from a two-view monocular initialization and has arbitrary scale. With
// Pinpoint odometry (beacon_push_odom) the map is metric from the first keyframe: no mono
// init, odometry factors in local BA and the pose graph, and the odometry carries the pose
// through frames that cannot be tracked.
//
// Local mapping and loop closing run synchronously after each new keyframe (simple and
// deterministic); in the threaded API a long keyframe step makes the next frames drop.
#include <cstdio>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <opencv2/calib3d.hpp>

#include "engine.h"
#include "orb_features.h"
#include "init_mono.h"
#include "loop.h"
#include "map.h"
#include "matcher.h"
#include "optimizer.h"
#include "vocab.h"

namespace beacon {

namespace {

enum class State { NotInit, Ok, Coasting, Lost };

double yawOf(const Mat3& R) { return std::atan2(R(1, 0), R(0, 0)); }

class ExploreEngine : public Engine {
public:
    static Params doubled(const Params& p) {
        Params q = p;
        q.orbFeatures *= 2;
        return q;
    }

    ExploreEngine(Shared& s)
        : s_(s), p_(s.params), initParams_(doubled(s.params)), orb_(s.params, s.camera), initOrb_(initParams_, s.camera),
          cam_{s.camera.fx, s.camera.fy, s.camera.cx, s.camera.cy, s.camera.width, s.camera.height} {
        if (!p_.vocabularyPath.empty()) vocOk_ = voc_.load(p_.vocabularyPath);
        loop_ = std::make_unique<LoopCloser>(map_, voc_, cam_, false, s.tRobotCam, odomParams_, p_.exploreGlobalBA);
    }

    void process(int64_t t, const cv::Mat& gray, Shared& s) override {
        Frame& F = cur_;
        F = Frame();
        F.id = frameCount_++;
        F.tNs = t;
        // Before the map exists, extract twice as many features (more matches for the two-view
        // initialization), as ORB-SLAM does.
        if (state_ == State::NotInit && !useOdom_ && s.odom.empty()) initOrb_.extract(gray, F.f);
        else orb_.extract(gray, F.f);
        F.reset(F.f.size());
        if (!s.odom.empty()) useOdom_ = true;
        {
            std::lock_guard<std::mutex> l(s.mu);
            F.odomEpoch = s.odomEpoch;
        }
        if (useOdom_) F.hasOdom = s.odom.at(t, (int64_t)(p_.odomMaxExtrapolateS * 1e9), F.odom);
        loop_->setFixedScale(useOdom_);
        newPoints_.clear();
        bool ok = false;
        if (!atlas_.empty() && vocOk_ && F.id % std::max(1, p_.exploreAtlasEvery) == 0 &&
            (state_ == State::NotInit || state_ == State::Lost || map_.numKeyFrames() < 40) && atlasRelocalize()) {
            ok = true;
        } else if (state_ == State::NotInit) {
            ok = useOdom_ ? initWithOdometry() : initMonocular();
        } else {
            ok = track();
        }
        if (ok) {
            trajectory_.push_back({F.tNs, {refKf_ ? refKf_->id : -1, refKf_ ? F.tcw * refKf_->twc() : F.tcw}});
        } else if (state_ == State::Coasting && F.hasPose && refKf_) {
            trajectory_.push_back({F.tNs, {refKf_->id, F.tcw * refKf_->twc()}});
        }
        publish(t, s, ok);
        if (observer_) visualize(gray, s, ok);
        if (state_ != State::NotInit) last_ = F;
    }

private:
    // ------------------------------------------------------------------ initialization

    void resetMap() {
        map_ = Map();
        loop_ = std::make_unique<LoopCloser>(map_, voc_, cam_, useOdom_, s_.tRobotCam, odomParams_, p_.exploreGlobalBA);
        refKf_ = lastKf_ = nullptr;
        hasInitRef_ = false;
        state_ = State::NotInit;
        hasVelocity_ = false;
        trajectory_.clear();
        recentPoints_.clear();
        lostFrames_ = 0;
    }

    /** Tracking is lost: keep the old map aside (bounded) and start a new one, as ORB-SLAM3's Atlas does. */
    void stashMapAndRestart() {
        atlas_.push_back(std::move(map_));
        while ((int)atlas_.size() > p_.exploreAtlasMaps) {
            size_t worst = 0;
            for (size_t i = 1; i < atlas_.size(); ++i)
                if (atlas_[i].numKeyFrames() < atlas_[worst].numKeyFrames()) worst = i;
            atlas_.erase(atlas_.begin() + worst);
        }
        localPoints_.clear();
        resetMap();
        status_ = "lost: new map";
    }

    /** Looks for the current frame in the stored maps. On success that map becomes the active one
     *  (the young map is stored in its place). Costs one BoW query + a few PnP tries per stored map. */
    bool atlasRelocalize() {
        int saved = relocMaxCands_;
        relocMaxCands_ = std::max(1, p_.exploreAtlasCands);
        bool found = false;
        for (size_t i = 0; i < atlas_.size() && !found; ++i) {
            if (atlas_[i].numKeyFrames() == 0) continue;
            std::swap(map_, atlas_[i]);
            if (relocalize()) {
                loop_ = std::make_unique<LoopCloser>(map_, voc_, cam_, useOdom_, s_.tRobotCam, odomParams_,
                                                     p_.exploreGlobalBA);
                lastKf_ = refKf_;
                lastKfFrame_ = cur_.id;
                hasInitRef_ = false;
                hasVelocity_ = false;
                localPoints_.clear();
                state_ = State::Ok;
                lostFrames_ = 0;
                trajectory_.clear();
                trajectory_.push_back({cur_.tNs, {refKf_->id, cur_.tcw * refKf_->twc()}});
                mapChanged_ = true;
                if (atlas_[i].numKeyFrames() < 3) atlas_.erase(atlas_.begin() + i);
                found = true;
            } else {
                std::swap(map_, atlas_[i]);
            }
        }
        relocMaxCands_ = saved;
        return found;
    }

    bool initMonocular() {
        Frame& F = cur_;
        if (!hasInitRef_ || initRef_.f.size() <= 100) {
            if (F.f.size() > 100) {
                initRef_ = F;
                prevMatched_ = F.f.un;
                hasInitRef_ = true;
            }
            status_ = "initializing: need texture";
            return false;
        }
        std::vector<int> m12;
        int nm = matcher().searchForInitialization(initRef_.f, F.f, prevMatched_, m12, p_.exploreInitWindow);
        if (nm < p_.exploreInitMinMatches) {
            hasInitRef_ = false;
            status_ = "initializing: move the camera sideways";
            return false;
        }
        TwoViewResult r = initializeMonocular(initRef_.f, F.f, m12, cam_, p_.exploreInitMinParallaxDeg, 50);
        if (!r.ok) {
            status_ = "initializing: " + r.why;
            return false;
        }
        KeyFrame* k0 = makeKeyFrame(initRef_);
        k0->tcw = SE3::Identity();
        F.tcw = r.t21;
        KeyFrame* k1 = makeKeyFrame(F);
        for (size_t i = 0; i < m12.size(); ++i) {
            if (!r.good[i]) continue;
            MapPoint* mp = map_.newMapPoint(r.points[i], k0->id);
            map_.addObservation(mp, k0, (int)i);
            map_.addObservation(mp, k1, m12[i]);
            map_.computeDescriptor(mp);
            map_.updateNormalAndDepth(mp);
        }
        map_.updateConnections(k0);
        map_.updateConnections(k1);
        globalBundleAdjustment(map_, cam_, 20, false, s_.tRobotCam, odomParams_);
        // Scale: median scene depth of the first keyframe = 1.
        std::vector<double> depths;
        for (MapPoint* p : map_.pointsOf(k0)) depths.push_back((k0->tcw * p->pos).z());
        if (depths.size() < 50) {
            resetMap();
            return false;
        }
        std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
        double med = depths[depths.size() / 2];
        if (med <= 0 || k1->trackedPoints(1, map_) < 50) {
            resetMap();
            status_ = "initializing: bad initial map";
            return false;
        }
        double inv = 1.0 / med;
        k1->tcw.translation() *= inv;
        for (MapPoint* p : map_.mapPoints()) {
            p->pos *= inv;
            map_.updateNormalAndDepth(p);
        }
        finishInit(k0, k1);
        return true;
    }

    bool initWithOdometry() {
        Frame& F = cur_;
        if (!F.hasOdom) {
            status_ = "initializing: waiting for odometry";
            return false;
        }
        SE3 twc = F.odom.toSE3() * s_.tRobotCam;
        F.tcw = twc.inverse();
        if (!hasInitRef_ || initRef_.f.size() <= 100) {
            if (F.f.size() > 100) {
                initRef_ = F;
                prevMatched_ = F.f.un;
                hasInitRef_ = true;
            }
            status_ = "initializing (odometry): need texture";
            return false;
        }
        std::vector<int> m12;
        int nm = matcher().searchForInitialization(initRef_.f, F.f, prevMatched_, m12, 120);
        SE3 t21 = F.tcw * initRef_.tcw.inverse();
        if (nm < 60) {
            initRef_ = F;  // lost the matches: re-anchor here
            prevMatched_ = F.f.un;
            status_ = "initializing (odometry): re-anchoring";
            return false;
        }
        if (t21.translation().norm() < 0.05) {
            status_ = "initializing (odometry): drive a little";
            return false;
        }
        TwoViewResult r = triangulateKnown(initRef_.f, F.f, m12, t21, cam_, p_.exploreInitMinParallaxDeg);
        if (r.nGood < 60) {
            status_ = "initializing (odometry): not enough parallax";
            return false;
        }
        KeyFrame* k0 = makeKeyFrame(initRef_);
        KeyFrame* k1 = makeKeyFrame(F);
        SE3 twc0 = k0->twc();
        for (size_t i = 0; i < m12.size(); ++i) {
            if (!r.good[i]) continue;
            MapPoint* mp = map_.newMapPoint(twc0 * r.points[i], k0->id);
            map_.addObservation(mp, k0, (int)i);
            map_.addObservation(mp, k1, m12[i]);
            map_.computeDescriptor(mp);
            map_.updateNormalAndDepth(mp);
        }
        map_.updateConnections(k0);
        map_.updateConnections(k1);
        globalBundleAdjustment(map_, cam_, 20, true, s_.tRobotCam, odomParams_);
        finishInit(k0, k1);
        return true;
    }

    void finishInit(KeyFrame* k0, KeyFrame* k1) {
        Frame& F = cur_;
        F.tcw = k1->tcw;
        F.hasPose = true;
        F.reset(F.f.size());
        for (size_t i = 0; i < k1->mps.size(); ++i) F.mps[i] = k1->mps[i];
        loop_->process(k0);
        loop_->process(k1);
        refKf_ = lastKf_ = k1;
        lastKfFrame_ = F.id;
        state_ = State::Ok;
        hasVelocity_ = false;
        trajectory_.clear();
        trajectory_.push_back({initRef_.tNs, {k0->id, SE3::Identity()}});
        status_ = "initialized";
        mapChanged_ = true;
    }

    KeyFrame* makeKeyFrame(const Frame& F) {
        KeyFrame* k = map_.newKeyFrame();
        k->frameId = F.id;
        k->tNs = F.tNs;
        k->tcw = F.tcw;
        k->f = F.f;
        k->mps.assign(F.f.size(), -1);
        k->hasOdom = F.hasOdom;
        k->odom = F.odom;
        k->odomEpoch = F.odomEpoch;
        voc_.transform(k->f.desc, k->bow, k->fv, 2);
        return k;
    }

    Matcher matcher() const { return Matcher(map_, cam_); }

    // ------------------------------------------------------------------------ tracking

    bool track() {
        Frame& F = cur_;
        PosePrior prior;
        // Prediction: odometry when available, else constant velocity.
        if (useOdom_ && F.hasOdom && last_.hasOdom && last_.hasPose && last_.odomEpoch == F.odomEpoch) {
            SE3 rel = s_.tRobotCam.inverse() * (last_.odom.inverse() * F.odom).toSE3() * s_.tRobotCam;
            F.tcw = (last_.tcw.inverse() * rel).inverse();
            double d = rel.translation().norm();
            prior.use = true;
            prior.tcw = F.tcw;
            prior.sigmaT = 0.003 + 0.02 * d;
            prior.sigmaR = 0.003 + 0.02 * Eigen::AngleAxisd(rel.linear()).angle();
        } else if (hasVelocity_ && last_.hasPose) {
            F.tcw = velocity_ * last_.tcw;
        } else {
            F.tcw = last_.tcw;
        }
        F.hasPose = true;
        bool ok = false;
        bool recentReloc = F.id < lastRelocFrame_ + 20;
        if (state_ == State::Ok || state_ == State::Coasting) {
            float th = state_ == State::Coasting ? 25.f : 15.f;
            // Two motion hypotheses: the prediction (odometry or constant velocity) and "no
            // motion" (a shaking hand-held camera reverses often). Keep the better supported.
            std::vector<SE3> hyps = {F.tcw};
            if (!prior.use && hasVelocity_ && last_.hasPose) hyps.push_back(last_.tcw);
            int bestIn = -1;
            Frame best;
            for (const SE3& h : hyps) {
                F.reset(F.f.size());
                F.tcw = h;
                Matcher m = matcher();
                int nm = m.searchByProjection(F, last_, th);
                if (nm < 20) {
                    F.reset(F.f.size());
                    nm = m.searchByProjection(F, last_, 2 * th);
                }
                int in1 = nm >= 20 ? optimizeAndClean(prior) : 0;
                if (in1 < 0.4 * nm) in1 = std::min(in1, 9);  // mostly outliers: not trusted
                if (in1 > bestIn) {
                    bestIn = in1;
                    best = F;
                }
            }
            F = best;
            ok = bestIn >= 15;
            if (!ok && refKf_) ok = trackReferenceKeyFrame(prior);
            dbg_ = "mm " + std::to_string(bestIn) + (ok ? " ok " : " fail ");
            if (!ok && state_ == State::Coasting && useOdom_) ok = trackLocalMapFrom(prior, 6.f);
        }
        if (!ok && (state_ == State::Lost || (state_ == State::Coasting && F.id % 5 == 0))) ok = relocalize();
        if (ok) {
            int inl = trackLocalMap(prior, recentReloc ? 3.f : (state_ == State::Coasting ? 3.f : 1.f));
            ok = inl >= (recentReloc ? 50 : 30);
            dbg_ += "local " + std::to_string(inl) + "/" + std::to_string(localPoints_.size());
        }
        if (ok) {
            state_ = State::Ok;
            lostFrames_ = 0;
            if (last_.hasPose) {
                velocity_ = F.tcw * last_.tcw.inverse();
                hasVelocity_ = true;
            }
            for (size_t i = 0; i < F.mps.size(); ++i)
                if (F.mps[i] >= 0 && F.outlier[i]) F.mps[i] = -1;
            if (needNewKeyFrame()) createKeyFrame();
            status_ = "tracking";
            return true;
        }
        if (useOdom_ && F.hasOdom && last_.hasPose) {
            // Pinpoint carries the pose until the map is found again.
            F.tcw = prior.use ? prior.tcw : F.tcw;
            F.hasPose = true;
            F.reset(F.f.size());
            state_ = State::Coasting;
            status_ = "coasting on odometry";
            // Keep mapping from the odometry pose so the map grows into the new view and
            // tracking can resume (points are triangulated against the previous keyframes).
            bool moved = false;
            if (lastKf_ && lastKf_->hasOdom && lastKf_->odomEpoch == F.odomEpoch) {
                SE2 d = lastKf_->odom.inverse() * F.odom;
                moved = std::hypot(d.x, d.y) > 0.08 || std::abs(d.theta) > 0.12;
            }
            if (moved || F.id >= lastKfFrame_ + 10) createKeyFrame();
            return false;
        }
        F.hasPose = false;
        hasVelocity_ = false;
        if (map_.numKeyFrames() <= 5) {
            resetMap();
            status_ = "lost early: re-initializing";
            return false;
        }
        state_ = State::Lost;
        status_ = "lost: relocalizing";
        if (++lostFrames_ >= p_.exploreLostFramesNewMap && p_.exploreAtlasMaps > 0 && !useOdom_ && vocOk_ &&
            map_.numKeyFrames() > 10)
            stashMapAndRestart();
        return false;
    }

    int optimizeAndClean(const PosePrior& prior) {
        Frame& F = cur_;
        int inl = optimizePose(F, map_, cam_, prior);
        for (size_t i = 0; i < F.mps.size(); ++i)
            if (F.mps[i] >= 0 && F.outlier[i]) {
                F.mps[i] = -1;
                F.outlier[i] = 0;
            }
        return inl;
    }

    bool trackReferenceKeyFrame(const PosePrior& prior) {
        Frame& F = cur_;
        voc_.transform(F.f.desc, F.bow, F.fv, 2);
        std::vector<int> m;
        int nm = matcher().searchByBoW(refKf_, F, m);
        if (nm < 15) return false;
        F.reset(F.f.size());
        F.mps = m;
        F.tcw = last_.hasPose ? last_.tcw : refKf_->tcw;
        if (prior.use) F.tcw = prior.tcw;
        return optimizeAndClean(prior) >= 10;
    }

    bool trackLocalMapFrom(const PosePrior& prior, float th) {
        Frame& F = cur_;
        F.reset(F.f.size());
        F.tcw = prior.tcw;
        updateLocalMap();
        std::vector<Projected> pts;
        for (MapPoint* p : localPoints_) {
            Projected pr;
            if (matcher().inFrustum(p, F.tcw, F.f, 0.5f, pr)) pts.push_back(pr);
        }
        matcher().searchByProjection(F, pts, th);
        return optimizeAndClean(prior) >= 15;
    }

    void updateLocalMap() {
        Frame& F = cur_;
        std::map<int, int> counter;
        for (int id : F.mps) {
            MapPoint* p = map_.mp(id);
            if (!p || p->bad) continue;
            for (const auto& o : p->obs) counter[o.first]++;
        }
        localKfs_.clear();
        std::set<int> inLocal;
        int best = 0;
        KeyFrame* bestK = nullptr;
        for (const auto& c : counter) {
            KeyFrame* k = map_.kf(c.first);
            if (!k || k->bad) continue;
            localKfs_.push_back(k);
            inLocal.insert(k->id);
            if (c.second > best) {
                best = c.second;
                bestK = k;
            }
        }
        if (localKfs_.empty() && refKf_) {
            localKfs_.push_back(refKf_);
            inLocal.insert(refKf_->id);
        }
        size_t n0 = localKfs_.size();
        for (size_t i = 0; i < n0 && localKfs_.size() < 80; ++i)
            for (KeyFrame* k : map_.bestCovisibles(localKfs_[i], 10))
                if (inLocal.insert(k->id).second) localKfs_.push_back(k);
        if (bestK) {
            refKf_ = bestK;
            F.refKf = bestK->id;
        }
        localPoints_.clear();
        std::set<int> seen;
        for (KeyFrame* k : localKfs_)
            for (MapPoint* p : map_.pointsOf(k))
                if (seen.insert(p->id).second) localPoints_.push_back(p);
    }

    int trackLocalMap(const PosePrior& prior, float th) {
        Frame& F = cur_;
        updateLocalMap();
        for (int id : F.mps) {
            MapPoint* p = map_.mp(id);
            if (!p || p->bad) continue;
            p->visible++;
            p->lastSeenFrame = F.id;
        }
        std::vector<Projected> pts;
        Matcher m = matcher();
        for (MapPoint* p : localPoints_) {
            if (p->lastSeenFrame == F.id) continue;
            Projected pr;
            if (m.inFrustum(p, F.tcw, F.f, 0.5f, pr)) {
                p->visible++;
                pts.push_back(pr);
            }
        }
        m.searchByProjection(F, pts, th);
        int inl = optimizePose(F, map_, cam_, prior);
        int good = 0;
        for (size_t i = 0; i < F.mps.size(); ++i) {
            MapPoint* p = map_.mp(F.mps[i]);
            if (!p) continue;
            if (F.outlier[i]) continue;
            p->found++;
            ++good;
        }
        (void)inl;
        return good;
    }

    bool relocalize() {
        Frame& F = cur_;
        if (voc_.empty()) return false;
        voc_.transform(F.f.desc, F.bow, F.fv, 2);
        std::map<int, const BowVector*> bows;
        for (KeyFrame* k : map_.keyFrames()) bows[k->id] = &k->bow;
        auto cands = map_.db.query(F.bow, {}, 0.0, bows, p_.relocMinCommonFrac);
        static const bool dbg = std::getenv("BEACON_ATLAS_DEBUG") != nullptr;
        if (dbg) std::fprintf(stderr, "[reloc] f%ld kfs %zu cands %zu top %.3f\n", F.id, map_.numKeyFrames(), cands.size(), cands.empty() ? 0.0 : cands[0].first);
        Matcher m = matcher();
        cv::Mat K = (cv::Mat_<double>(3, 3) << cam_.fx, 0, cam_.cx, 0, cam_.fy, cam_.cy, 0, 0, 1);
        for (size_t c = 0; c < cands.size() && (int)c < relocMaxCands_; ++c) {
            KeyFrame* k = map_.kf(cands[c].second);
            std::vector<int> mm;
            int nb = m.searchByBoW(k, F, mm);
            if (dbg) std::fprintf(stderr, "[reloc]   cand kf%d score %.3f bow-matches %d\n", k->id, cands[c].first, nb);
            if (nb < 15) continue;
            std::vector<cv::Point3f> obj;
            std::vector<cv::Point2f> img;
            std::vector<int> idx;
            for (size_t i = 0; i < mm.size(); ++i) {
                MapPoint* p = map_.mp(mm[i]);
                if (!p || p->bad) continue;
                obj.push_back(cv::Point3f((float)p->pos.x(), (float)p->pos.y(), (float)p->pos.z()));
                img.push_back(F.f.un[i]);
                idx.push_back((int)i);
            }
            if (obj.size() < 15) continue;
            cv::Mat rvec, tvec;
            std::vector<int> inl;
            if (!cv::solvePnPRansac(obj, img, K, cv::noArray(), rvec, tvec, false, 200, 4.0f, 0.99, inl,
                                    cv::SOLVEPNP_EPNP) ||
                inl.size() < 10)
                continue;
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            SE3 T = SE3::Identity();
            for (int r = 0; r < 3; ++r) {
                for (int cc = 0; cc < 3; ++cc) T.linear()(r, cc) = R.at<double>(r, cc);
                T.translation()(r) = tvec.at<double>(r);
            }
            F.reset(F.f.size());
            for (int i : inl) F.mps[idx[i]] = mm[idx[i]];
            F.tcw = T;
            int good = optimizeAndClean(PosePrior());
            if (dbg) std::fprintf(stderr, "[reloc]   pnp inl %zu opt %d\n", inl.size(), good);
            if (good < 10) continue;
            if (good < p_.relocMinInliers) {
                std::vector<int> found;
                for (int id : F.mps)
                    if (id >= 0) found.push_back(id);
                m.searchByProjection(F, k, found, 10, 100);
                good = optimizeAndClean(PosePrior());
            }
            if (good >= p_.relocMinInliers) {
                lastRelocFrame_ = F.id;
                refKf_ = k;
                status_ = "relocalized";
                return true;
            }
        }
        return false;
    }

    bool needNewKeyFrame() {
        const Frame& F = cur_;
        size_t nKfs = map_.numKeyFrames();
        int maxFrames = p_.exploreMaxFramesBetweenKf;
        if (F.id < lastRelocFrame_ + maxFrames && (int)nKfs > maxFrames) return false;
        int tracked = 0;
        for (size_t i = 0; i < F.mps.size(); ++i)
            if (F.mps[i] >= 0 && !F.outlier[i]) ++tracked;
        int refMatches = refKf_ ? refKf_->trackedPoints(nKfs <= 2 ? 2 : 3, map_) : 0;
        bool c1 = F.id >= lastKfFrame_ + p_.exploreMinFramesBetweenKf || tracked < 60;
        bool c2 = (tracked < refMatches * 0.9 || F.id >= lastKfFrame_ + maxFrames) && tracked > 15;
        bool moved = false;
        if (useOdom_ && lastKf_ && lastKf_->hasOdom && F.hasOdom && lastKf_->odomEpoch == F.odomEpoch) {
            SE2 d = lastKf_->odom.inverse() * F.odom;
            moved = std::hypot(d.x, d.y) > 0.25 || std::abs(d.theta) > 0.26;
        }
        return (c1 && c2) || (moved && tracked > 15);
    }

    // --------------------------------------------------------------------- local mapping

    void createKeyFrame() {
        Frame& F = cur_;
        KeyFrame* k = makeKeyFrame(F);
        for (size_t i = 0; i < F.mps.size(); ++i) {
            MapPoint* p = map_.mp(F.mps[i]);
            if (!p || p->bad || F.outlier[i]) continue;
            map_.addObservation(p, k, (int)i);
            map_.updateNormalAndDepth(p);
            map_.computeDescriptor(p);
        }
        map_.updateConnections(k);
        lastKf_ = refKf_ = k;
        lastKfFrame_ = F.id;
        localMapping(k);
        if (!useOdom_) F.tcw = k->tcw;  // the keyframe is this frame: take its BA-refined pose
        if (vocOk_ && p_.exploreLoopClosing) {
            bool closed = loop_->process(k);
            if (!loop_->debug.empty() && std::getenv("BEACON_LOOP_DEBUG")) std::fprintf(stderr, "loopdbg %s\n", loop_->debug.c_str());
            if (closed) {
                // The current frame moved with its keyframe.
                F.tcw = k->tcw;
                last_.hasPose = false;
                hasVelocity_ = false;
                loops_++;
                status_ = loop_->lastMessage;
                if (std::getenv("BEACON_LOOP_DEBUG")) std::fprintf(stderr, "LOOP %s\n", status_.c_str());
            }
        } else {
            map_.db.add(k->id, k->bow);
        }
        checkReobservationLoop(k);
        mapChanged_ = true;
    }

    // A loop closed by tracking itself: the new keyframe re-observes points of a keyframe made
    // long ago (the place-recognition loop closer never fires then, because that keyframe is
    // already connected and excluded from its candidates). The local BA held the old part of
    // the map fixed, so the remaining inconsistency is spread with a global BA (the "snap" in
    // the reference videos). Logged as a loop closure.
    void checkReobservationLoop(KeyFrame* k) {
        if (k->id < lastReobsLoopKf_ + p_.exploreReobsMinGapKf / 2) return;
        KeyFrame* oldest = nullptr;
        for (int id : k->ordered) {
            KeyFrame* o = map_.kf(id);
            if (!o || o->bad || k->conn[id] < 20) continue;
            if (k->id - o->id < p_.exploreReobsMinGapKf || (k->tNs - o->tNs) * 1e-9 < 20.0) continue;
            if (!oldest || o->id < oldest->id) oldest = o;
        }
        if (!oldest || k->loopEdges.count(oldest->id)) return;
        k->loopEdges.insert(oldest->id);
        oldest->loopEdges.insert(k->id);
        map_.loopLinks.push_back({oldest->id, k->id});
        lastReobsLoopKf_ = k->id;
        loops_++;
        if (p_.exploreGlobalBA) globalBundleAdjustment(map_, cam_, p_.exploreGlobalBAIterations, useOdom_, s_.tRobotCam, odomParams_);
        cur_.tcw = k->tcw;
        last_.hasPose = false;
        hasVelocity_ = false;
        char b[128];
        std::snprintf(b, sizeof b, "loop (re-observation): keyframe %d <-> %d", k->id, oldest->id);
        status_ = b;
        if (std::getenv("BEACON_LOOP_DEBUG")) std::fprintf(stderr, "LOOP %s\n", b);
    }

    void localMapping(KeyFrame* k) {
        Matcher m = matcher();
        // Recent point culling.
        std::vector<MapPoint*> keep;
        for (MapPoint* p : recentPoints_) {
            if (p->bad) continue;
            if (p->foundRatio() < 0.25 || (k->id - p->firstKf >= 2 && p->obs.size() <= 2)) {
                map_.setBad(p);
                continue;
            }
            if (k->id - p->firstKf < 3) keep.push_back(p);
        }
        recentPoints_ = keep;
        // New points by triangulation with the best neighbours.
        std::vector<KeyFrame*> nbs = map_.bestCovisibles(k, 20);
        if (useOdom_)
            for (int id = k->id - 1; id >= 0 && id >= k->id - 3; --id)
                if (map_.kf(id) && !map_.kf(id)->bad && std::find(nbs.begin(), nbs.end(), map_.kf(id)) == nbs.end())
                    nbs.push_back(map_.kf(id));
        float sf = orb_.scaleFactor();
        for (KeyFrame* n : nbs) {
            double baseline = (k->centre() - n->centre()).norm();
            std::vector<double> depths;
            for (MapPoint* p : map_.pointsOf(n)) depths.push_back((n->tcw * p->pos).z());
            if (depths.empty()) continue;
            std::nth_element(depths.begin(), depths.begin() + depths.size() / 2, depths.end());
            if (baseline / depths[depths.size() / 2] < 0.01) continue;
            std::vector<std::pair<int, int>> pairs;
            m.searchForTriangulation(k, n, pairs);
            for (const auto& pr : pairs) {
                const cv::Point2f& a = k->f.un[pr.first];
                const cv::Point2f& b = n->f.un[pr.second];
                Vec2 n1((a.x - cam_.cx) / cam_.fx, (a.y - cam_.cy) / cam_.fy);
                Vec2 n2((b.x - cam_.cx) / cam_.fx, (b.y - cam_.cy) / cam_.fy);
                Vec3 r1 = k->tcw.linear().transpose() * Vec3(n1.x(), n1.y(), 1);
                Vec3 r2 = n->tcw.linear().transpose() * Vec3(n2.x(), n2.y(), 1);
                double cosPar = r1.normalized().dot(r2.normalized());
                if (cosPar > 0.9998 || cosPar < 0) continue;
                Vec3 X;
                if (!triangulate(n1, n2, k->tcw, n->tcw, X)) continue;
                Vec3 x1 = k->tcw * X, x2 = n->tcw * X;
                if (x1.z() <= 0 || x2.z() <= 0) continue;
                int o1 = k->f.kps[pr.first].octave, o2 = n->f.kps[pr.second].octave;
                auto chi = [&](const Vec3& x, const cv::Point2f& u, int o, const Features& f) {
                    double du = cam_.fx * x.x() / x.z() + cam_.cx - u.x, dv = cam_.fy * x.y() / x.z() + cam_.cy - u.y;
                    return (du * du + dv * dv) * f.invLevelSigma2[o];
                };
                if (chi(x1, a, o1, k->f) > 5.991 || chi(x2, b, o2, n->f) > 5.991) continue;
                double d1 = (X - k->centre()).norm(), d2 = (X - n->centre()).norm();
                double ratioDist = d2 / d1;
                double ratioOct = k->f.scaleFactors[o1] / n->f.scaleFactors[o2];
                if (ratioDist * 1.5 * sf < ratioOct || ratioDist > ratioOct * 1.5 * sf) continue;
                MapPoint* p = map_.newMapPoint(X, k->id);
                p->firstKf = k->id;
                map_.addObservation(p, k, pr.first);
                map_.addObservation(p, n, pr.second);
                map_.computeDescriptor(p);
                map_.updateNormalAndDepth(p);
                recentPoints_.push_back(p);
                newPoints_.push_back(p->id);
            }
        }
        // Fuse duplicates with the neighbourhood.
        std::vector<KeyFrame*> targets;
        std::set<int> tset;
        for (KeyFrame* n : map_.bestCovisibles(k, 20)) {
            if (tset.insert(n->id).second) targets.push_back(n);
            for (KeyFrame* n2 : map_.bestCovisibles(n, 5))
                if (n2 != k && tset.insert(n2->id).second) targets.push_back(n2);
        }
        std::vector<MapPoint*> mine = map_.pointsOf(k);
        for (KeyFrame* t : targets) m.fuse(t, mine, 3.0f, map_);
        std::vector<MapPoint*> theirs;
        std::set<int> seen;
        for (KeyFrame* t : targets)
            for (MapPoint* p : map_.pointsOf(t))
                if (seen.insert(p->id).second) theirs.push_back(p);
        m.fuse(k, theirs, 3.0f, map_);
        for (MapPoint* p : map_.pointsOf(k)) {
            map_.computeDescriptor(p);
            map_.updateNormalAndDepth(p);
        }
        map_.updateConnections(k);
        // Local BA.
        for (int id : backend_)
            if (MapPoint* p = map_.mp(id)) p->inBackend = false;
        if (map_.numKeyFrames() > 2 && !std::getenv("BEACON_NO_LBA"))
            localBundleAdjustment(k, map_, cam_, p_.exploreLocalWindow, useOdom_, s_.tRobotCam, odomParams_, ++baId_);
        backend_.clear();
        for (MapPoint* p : map_.mapPoints())
            if (p->inBackend && p->localBaFor == baId_) backend_.push_back(p->id);
        cullKeyFrames(k);
    }

    void cullKeyFrames(KeyFrame* k) {
        for (KeyFrame* c : map_.bestCovisibles(k, 1000)) {
            if (c->id == map_.origin || c == k || c->id >= k->id - 1) continue;
            int n = 0, redundant = 0;
            for (size_t i = 0; i < c->mps.size(); ++i) {
                MapPoint* p = map_.mp(c->mps[i]);
                if (!p || p->bad) continue;
                ++n;
                if (p->obs.size() <= 3) continue;
                int level = c->f.kps[i].octave, seenBy = 0;
                for (const auto& o : p->obs) {
                    if (o.first == c->id) continue;
                    KeyFrame* ok = map_.kf(o.first);
                    if (ok && ok->f.kps[o.second].octave <= level + 1) ++seenBy;
                    if (seenBy >= 3) break;
                }
                if (seenBy >= 3) ++redundant;
            }
            if (n > 0 && redundant > 0.9 * n) map_.setBad(c);
        }
    }

    // ------------------------------------------------------------------ output / viewer

    void publish(int64_t t, Shared& s, bool ok) {
        int status = state_ == State::NotInit ? BEACON_STATUS_INITIALIZING
                     : ok                     ? BEACON_STATUS_TRACKING
                                              : BEACON_STATUS_LOST;
        if (ok) {
            // Robot pose in the map frame (explore mode: map frame, not the field frame).
            SE3 twr = cur_.tcw.inverse() * s.tRobotCam.inverse();
            beacon_fix fx{};
            fx.t_ns = t;
            fx.x = twr.translation().x();
            fx.y = twr.translation().y();
            fx.heading = yawOf(twr.linear());
            fx.sigma_xy = NAN;
            fx.sigma_heading = NAN;
            int inl = 0;
            for (size_t i = 0; i < cur_.mps.size(); ++i)
                if (cur_.mps[i] >= 0 && !cur_.outlier[i]) ++inl;
            fx.inliers = inl;
            fx.kind = BEACON_FIX_POINTS | (useOdom_ ? 0 : BEACON_FIX_NO_ODOM);
            publishFix(s, fx);
        }
        char line[200];
        std::snprintf(line, sizeof line, "explore%s: %s | kf %zu, points %zu, loops %d | %s", useOdom_ ? "+odom" : "",
                      status_.c_str(), map_.numKeyFrames(), map_.mapPoints().size(), loops_, dbg_.c_str());
        std::lock_guard<std::mutex> l(s.mu);
        s.status = status;
        s.debug = line;
    }

    void visualize(const cv::Mat& gray, Shared& s, bool ok) {
        VizFrame v;
        v.tNs = cur_.tNs;
        v.image = gray;
        v.poseValid = cur_.hasPose && state_ != State::NotInit && state_ != State::Lost;
        v.tWorldCam = cur_.tcw.inverse();
        v.tCamRobot = s.tRobotCam.inverse();
        v.status = state_ == State::NotInit ? BEACON_STATUS_INITIALIZING : ok ? BEACON_STATUS_TRACKING : BEACON_STATUS_LOST;
        if (cur_.hasOdom && useOdom_) {
            v.odomValid = true;
            v.tWorldCamOdom = cur_.odom.toSE3() * s.tRobotCam;
        }
        std::set<int> fresh(newPoints_.begin(), newPoints_.end());
        for (size_t i = 0; i < cur_.mps.size(); ++i) {
            MapPoint* p = map_.mp(cur_.mps[i]);
            if (!p || p->bad || cur_.outlier[i]) continue;
            if (p->obs.size() <= 2 || fresh.count(p->id)) v.fresh.push_back(cur_.f.kps[i].pt);
            else v.tracked.push_back(cur_.f.kps[i].pt);
            v.trackedPoints.push_back(p->pos);
        }
        v.keyframeCount = (int)map_.numKeyFrames();
        v.loopCount = loops_;
        if (mapChanged_ || cur_.id % 10 == 0) {
            v.mapChanged = true;
            for (KeyFrame* k : map_.keyFrames()) {
                v.keyframes.push_back(k->twc());
                if (k->parent >= 0) {
                    KeyFrame* pk = map_.kf(k->parent);
                    if (pk && !pk->bad) v.graphEdges.push_back({k->centre(), pk->centre()});
                }
                for (KeyFrame* c : map_.covisiblesByWeight(k, 100))
                    if (c->id < k->id) v.graphEdges.push_back({k->centre(), c->centre()});
            }
            for (const auto& l : map_.loopLinks) {
                KeyFrame* a = map_.kf(l.first);
                KeyFrame* b = map_.kf(l.second);
                if (a && b) {
                    v.loopLinks.push_back({a->centre(), b->centre()});
                    v.graphEdges.push_back({a->centre(), b->centre()});
                }
            }
            for (MapPoint* p : map_.mapPoints()) v.mapPoints.push_back(p->pos);
            for (MapPoint* p : localPoints_)
                if (!p->bad) v.localPoints.push_back(p->pos);
            for (int id : backend_)
                if (MapPoint* p = map_.mp(id))
                    if (!p->bad) v.backendPoints.push_back(p->pos);
            for (const auto& e : trajectory_) {
                KeyFrame* r = map_.kf(e.second.first);
                SE3 tcw = e.second.second;
                if (r) {
                    // Follow the reference through culled keyframes to a good ancestor.
                    while (r->bad && r->parent >= 0) r = map_.kf(r->parent);
                    tcw = e.second.second * r->tcw;
                    if (r->id != e.second.first) continue;  // reference culled: skip sample
                }
                v.trajectory.push_back(tcw.inverse().translation());
                v.trajectoryT.push_back(e.first);
            }
            mapChanged_ = false;
        }
        v.mapPointCount = (int)map_.mapPoints().size();
        v.text = s.debug;
        observer_(v);
    }

    Shared& s_;
    const Params& p_;
    Params initParams_;
    OrbExtractor orb_, initOrb_;
    Pinhole cam_;
    Vocabulary voc_;
    bool vocOk_ = false;
    Map map_;
    std::vector<Map> atlas_;   // stored maps (tracking was lost), oldest first
    int lostFrames_ = 0, relocMaxCands_ = 10;
    OdomEdgeParams odomParams_;
    std::unique_ptr<LoopCloser> loop_;
    State state_ = State::NotInit;
    bool useOdom_ = false;
    Frame cur_, last_, initRef_;
    bool hasInitRef_ = false;
    std::vector<cv::Point2f> prevMatched_;
    long frameCount_ = 0, lastKfFrame_ = 0, lastRelocFrame_ = -1000, baId_ = 0;
    KeyFrame* refKf_ = nullptr;
    KeyFrame* lastKf_ = nullptr;
    SE3 velocity_ = SE3::Identity();
    bool hasVelocity_ = false;
    std::vector<KeyFrame*> localKfs_;
    std::vector<MapPoint*> localPoints_;
    std::vector<MapPoint*> recentPoints_;
    std::vector<int> newPoints_, backend_;
    // Per frame: capture time, (reference keyframe id, T_cam_ref) so loops move the path too.
    std::vector<std::pair<int64_t, std::pair<int, SE3>>> trajectory_;
    std::string status_ = "starting";
    std::string dbg_;
    bool mapChanged_ = false;
    int loops_ = 0;
    int lastReobsLoopKf_ = -1000000;
};

}  // namespace

std::unique_ptr<Engine> makeExploreEngine(const FieldModel*, Shared& shared) {
    return std::make_unique<ExploreEngine>(shared);
}

}  // namespace beacon
