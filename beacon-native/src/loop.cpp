#include "loop.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <random>

namespace beacon {

namespace {

// Closed-form similarity dst ~ s R src + t (Umeyama). fixScale: s = 1.
Sim3 umeyamaSim3(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, bool fixScale) {
    size_t n = src.size();
    Vec3 ms = Vec3::Zero(), md = Vec3::Zero();
    for (size_t i = 0; i < n; ++i) {
        ms += src[i];
        md += dst[i];
    }
    ms /= (double)n;
    md /= (double)n;
    Mat3 C = Mat3::Zero();
    double vs = 0;
    for (size_t i = 0; i < n; ++i) {
        C += (dst[i] - md) * (src[i] - ms).transpose();
        vs += (src[i] - ms).squaredNorm();
    }
    C /= (double)n;
    vs /= (double)n;
    Eigen::JacobiSVD<Mat3> svd(C, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 S = Mat3::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0) S(2, 2) = -1;
    Mat3 R = svd.matrixU() * S * svd.matrixV().transpose();
    double s = fixScale || vs <= 0 ? 1.0 : (svd.singularValues().asDiagonal() * S).trace() / vs;
    return Sim3(s, R, md - s * R * ms);
}

double reprojChi2(const Vec3& x, const cv::Point2f& u, float inv2, const Pinhole& cam) {
    if (x.z() <= 0) return 1e9;
    double du = u.x - (cam.fx * x.x() / x.z() + cam.cx), dv = u.y - (cam.fy * x.y() / x.z() + cam.cy);
    return (du * du + dv * dv) * inv2;
}

}  // namespace

bool LoopCloser::process(KeyFrame* kf) {
    lastMessage.clear();
    std::vector<KeyFrame*> cands;
    bool found = detect(kf, cands);
    map_.db.add(kf->id, kf->bow);
    if (!found) return false;
    if (!computeSim3(kf, cands)) return false;
    correct(kf);
    return true;
}

bool LoopCloser::detect(KeyFrame* kf, std::vector<KeyFrame*>& out) {
    if (voc_.empty() || map_.numKeyFrames() < 12 || kf->id < lastLoopKf_ + 10) return false;
    if (const char* m = std::getenv("BEACON_MAX_LOOPS"))
        if ((int)map_.loopLinks.size() >= std::atoi(m)) return false;
    std::vector<int> exclude = {kf->id};
    double minScore = 1.0;
    for (int id : kf->ordered) {
        KeyFrame* c = map_.kf(id);
        if (!c || c->bad) continue;
        exclude.push_back(id);
        minScore = std::min(minScore, Vocabulary::score(kf->bow, c->bow));
    }
    if (kf->ordered.empty()) return false;
    // Also exclude the most recent keyframes (temporal neighbours are not loops).
    for (int id = kf->id - 1; id >= 0 && id >= kf->id - 15; --id) exclude.push_back(id);
    std::map<int, const BowVector*> bows;
    for (KeyFrame* k : map_.keyFrames()) bows[k->id] = &k->bow;
    auto scored = map_.db.query(kf->bow, exclude, std::max(minScore, 0.01), bows);
    debug = "kf " + std::to_string(kf->id) + " minScore " + std::to_string(minScore).substr(0, 5) + " cands " +
            std::to_string(scored.size());
    if (std::getenv("BEACON_LOOP_DEBUG") && !kf->ordered.empty()) {
        Matcher mm(map_, cam_);
        std::vector<int> m12;
        KeyFrame* nb = map_.kf(kf->ordered[0]);
        int pts = 0;
        for (int id : kf->mps) pts += id >= 0;
        debug += " [covis kf" + std::to_string(nb->id) + " w" + std::to_string(kf->conn[nb->id]) + " bow " +
                 std::to_string(mm.searchByBoW(kf, nb, m12)) + " pts " + std::to_string(pts) + " words " +
                 std::to_string(kf->bow.size()) + " nodes " + std::to_string(kf->fv.size()) + " minConnId " +
                 std::to_string(*std::min_element(kf->ordered.begin(), kf->ordered.end())) + "]";
        int oldest = *std::min_element(kf->ordered.begin(), kf->ordered.end());
        KeyFrame* ok = map_.kf(oldest);
        if (ok && kf->id - oldest > 100) {
            std::vector<int> mo;
            debug += " [oldest kf" + std::to_string(oldest) + " w" + std::to_string(kf->conn[oldest]) + " bow " +
                     std::to_string(mm.searchByBoW(kf, ok, mo)) + " score " +
                     std::to_string(Vocabulary::score(kf->bow, ok->bow)).substr(0, 5) + "]";
        }
    }
    // Group each candidate with its covisible keyframes; keep the best of each strong group.
    std::vector<std::pair<double, KeyFrame*>> groupsBest;
    double bestAcc = 0;
    std::map<int, double> scoreOf;
    for (const auto& s : scored) scoreOf[s.second] = s.first;
    for (const auto& s : scored) {
        KeyFrame* c = map_.kf(s.second);
        double acc = s.first, best = s.first;
        KeyFrame* bestK = c;
        for (KeyFrame* n : map_.bestCovisibles(c, 10)) {
            auto it = scoreOf.find(n->id);
            if (it == scoreOf.end()) continue;
            acc += it->second;
            if (it->second > best) {
                best = it->second;
                bestK = n;
            }
        }
        groupsBest.push_back({acc, bestK});
        bestAcc = std::max(bestAcc, acc);
    }
    std::set<int> chosen;
    std::vector<KeyFrame*> cands;
    for (const auto& g : groupsBest)
        if (g.first >= 0.75 * bestAcc && !chosen.count(g.second->id)) {
            chosen.insert(g.second->id);
            cands.push_back(g.second);
        }
    // Temporal consistency: a candidate group must be seen in 3 consecutive keyframes.
    std::vector<Group> next;
    out.clear();
    for (KeyFrame* c : cands) {
        Group g;
        g.kfs.insert(c->id);
        for (int id : c->ordered) g.kfs.insert(id);
        int best = 0;
        for (const Group& prev : groups_) {
            bool shared = false;
            for (int id : g.kfs)
                if (prev.kfs.count(id)) {
                    shared = true;
                    break;
                }
            if (shared) best = std::max(best, prev.consistency + 1);
        }
        g.consistency = best;
        if (best >= 2) out.push_back(c);
        next.push_back(g);
    }
    groups_ = next;
    debug += " consistent " + std::to_string(out.size());
    return !out.empty();
}

bool LoopCloser::computeSim3(KeyFrame* kf, const std::vector<KeyFrame*>& cands) {
    Matcher matcher(map_, cam_);
    std::mt19937 rng(1234 + kf->id);
    for (KeyFrame* c : cands) {
        if (c->bad) continue;
        std::vector<int> m12;
        int nm = matcher.searchBruteForce(kf, c, m12);
        debug += " | c" + std::to_string(c->id) + " bf " + std::to_string(nm);
        if (nm < 20) continue;
        // 3-D points in each camera.
        std::vector<int> idx1;
        std::vector<Vec3> P1, P2;
        std::vector<int> idx2;
        for (size_t i = 0; i < m12.size(); ++i) {
            if (m12[i] < 0) continue;
            MapPoint* a = map_.mp(kf->mps[i]);
            MapPoint* b = map_.mp(m12[i]);
            if (!a || !b || a->bad || b->bad || !b->obs.count(c->id)) continue;
            idx1.push_back((int)i);
            idx2.push_back(b->obs.at(c->id));
            P1.push_back(kf->tcw * a->pos);
            P2.push_back(c->tcw * b->pos);
        }
        if (P1.size() < 20) continue;
        // RANSAC over minimal sets of 3; inliers by reprojection in both images.
        Sim3 best;
        int bestIn = 0;
        std::uniform_int_distribution<size_t> pick(0, P1.size() - 1);
        for (int it = 0; it < 300; ++it) {
            size_t a = pick(rng), b = pick(rng), d = pick(rng);
            if (a == b || b == d || a == d) continue;
            Sim3 S = umeyamaSim3({P2[a], P2[b], P2[d]}, {P1[a], P1[b], P1[d]}, fixedScale_);
            // Monocular scale drift between a keyframe and its loop partner is far below 2x;
            // tiny or huge scales are degenerate fits (points collapsed onto one spot).
            if (!std::isfinite(S.s) || std::abs(std::log(S.s)) > maxLogScale_) continue;
            Sim3 Si = S.inverse();
            int in = 0;
            for (size_t k = 0; k < P1.size(); ++k) {
                float i1 = kf->f.invLevelSigma2[kf->f.kps[idx1[k]].octave];
                float i2 = c->f.invLevelSigma2[c->f.kps[idx2[k]].octave];
                if (reprojChi2(S.map(P2[k]), kf->f.un[idx1[k]], i1, cam_) < 9.21 &&
                    reprojChi2(Si.map(P1[k]), c->f.un[idx2[k]], i2, cam_) < 9.21)
                    ++in;
            }
            if (in > bestIn) {
                bestIn = in;
                best = S;
            }
        }
        debug += " ransac " + std::to_string(bestIn) + " (s " + std::to_string(best.s).substr(0, 5) + ")";
        if (bestIn < 20) continue;
        int in = optimizeSim3(kf, c, m12, best, cam_, map_, 10.0, fixedScale_);
        if (std::abs(std::log(best.s)) > maxLogScale_) continue;
        debug += " opt " + std::to_string(in) + " (s " + std::to_string(best.s).substr(0, 5) + ")";
        if (in < 20) continue;
        // Accept: S_cw = S_cm * S_mw.
        matched_ = c;
        scw_ = best * Sim3(c->tcw);
        // Gather the loop neighbourhood's points and match more by projection.
        loopPoints_.clear();
        std::set<int> seen;
        std::vector<KeyFrame*> loopKfs = map_.bestCovisibles(c, 1000);
        loopKfs.push_back(c);
        for (KeyFrame* k : loopKfs)
            for (MapPoint* p : map_.pointsOf(k))
                if (seen.insert(p->id).second) loopPoints_.push_back(p);
        curMatched_.assign(kf->f.size(), -1);
        for (size_t i = 0; i < m12.size(); ++i) curMatched_[i] = m12[i];
        matcher.searchByProjectionSim(kf, scw_.s * scw_.R, scw_.t, scw_.s, loopPoints_, curMatched_, 10);
        int total = 0;
        for (int m : curMatched_)
            if (m >= 0) ++total;
        debug += " total " + std::to_string(total);
        // Plausibility: a loop corrects accumulated drift, so the correction must be small
        // compared with the path travelled since the loop keyframe (monocular drift is a few %).
        double path = 0;
        KeyFrame* prev = nullptr;
        for (KeyFrame* k : map_.keyFrames()) {
            if (k->id < c->id || k->id > kf->id) continue;
            if (prev) path += (k->centre() - prev->centre()).norm();
            prev = k;
        }
        Vec3 cOld = kf->centre(), cNew = scw_.inverse().map(Vec3::Zero());
        double shift = (cNew - cOld).norm();
        bool plausible = std::abs(std::log(scw_.s)) <= maxLogScale_;
        debug += " total " + std::to_string(total) + " shift " + std::to_string(shift).substr(0, 6) + "/path " +
                 std::to_string(path).substr(0, 6) + (plausible ? " ok" : " implausible");
        if (total >= 40 && plausible) {
            char buf[256];
            std::snprintf(buf, sizeof buf, "loop: keyframe %d (t %lld) <-> %d (t %lld), %d matches, scale %.3f, shift %.3f",
                          kf->id, (long long)kf->tNs, c->id, (long long)c->tNs, total, scw_.s, (cNew - cOld).norm());
            lastMessage = buf;
            return true;
        }
    }
    return false;
}

void LoopCloser::correct(KeyFrame* kf) {
    Matcher matcher(map_, cam_);
    std::vector<KeyFrame*> connected = map_.bestCovisibles(kf, 1000);
    connected.push_back(kf);
    std::map<int, Sim3> corrected, nonCorrected;
    SE3 twc = kf->twc();
    corrected[kf->id] = scw_;
    nonCorrected[kf->id] = Sim3(kf->tcw);
    for (KeyFrame* k : connected) {
        if (k == kf) continue;
        Sim3 sic(k->tcw * twc);  // rigid relative pose (camera k <- current)
        corrected[k->id] = sic * scw_;
        nonCorrected[k->id] = Sim3(k->tcw);
    }
    // Move the points seen by the neighbourhood with their keyframe's correction.
    for (KeyFrame* k : connected) {
        const Sim3& c = corrected[k->id];
        const Sim3& nc = nonCorrected[k->id];
        Sim3 ci = c.inverse();
        for (MapPoint* p : map_.pointsOf(k)) {
            if (p->correctedBy == kf->id) continue;
            p->pos = ci.map(nc.map(p->pos));
            p->correctedBy = kf->id;
            p->correctedRef = k->id;
            map_.updateNormalAndDepth(p);
        }
    }
    for (KeyFrame* k : connected) k->tcw = corrected[k->id].toSE3();
    // Fuse the loop's points into the query keyframe (and its neighbourhood).
    for (size_t i = 0; i < curMatched_.size(); ++i) {
        MapPoint* m = map_.mp(curMatched_[i]);
        if (!m || m->bad) continue;
        MapPoint* q = map_.mp(kf->mps[i]);
        if (q && !q->bad) {
            if (q != m) map_.replace(q, m);
        } else {
            map_.addObservation(m, kf, (int)i);
            map_.computeDescriptor(m);
        }
    }
    std::map<int, std::set<int>> before;
    for (KeyFrame* k : connected) before[k->id] = std::set<int>(k->ordered.begin(), k->ordered.end());
    for (KeyFrame* k : connected) matcher.fuse(k, loopPoints_, 4.0f, map_);
    std::set<int> connIds;
    for (KeyFrame* k : connected) connIds.insert(k->id);
    std::map<int, std::set<int>> newLinks;
    for (KeyFrame* k : connected) {
        map_.updateConnections(k);
        for (int id : k->ordered)
            if (!before[k->id].count(id) && !connIds.count(id)) newLinks[k->id].insert(id);
    }
    optimizeEssentialGraph(map_, matched_, kf, corrected, nonCorrected, newLinks, 100, fixedScale_, tRobotCam_, odom_);
    matched_->loopEdges.insert(kf->id);
    kf->loopEdges.insert(matched_->id);
    map_.loopLinks.push_back({matched_->id, kf->id});
    if (globalBA_) globalBundleAdjustment(map_, cam_, 10, fixedScale_, tRobotCam_, odom_);
    lastLoopKf_ = kf->id;
}

}  // namespace beacon
