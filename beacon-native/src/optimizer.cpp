#include "optimizer.h"

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <set>

namespace beacon {

namespace {

constexpr double kChi2Mono = 5.991;  // 95 %, 2 dof

Mat3 skew(const Vec3& v) {
    Mat3 m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

Mat3 expSO3(const Vec3& w) {
    double th = w.norm();
    if (th < 1e-12) return Mat3::Identity() + skew(w);
    return Eigen::AngleAxisd(th, w / th).toRotationMatrix();
}

Vec3 logSO3(const Mat3& R) {
    Eigen::AngleAxisd aa(R);
    return aa.angle() * aa.axis();
}

// ---------------------------------------------------------------------- Ceres cost functors

struct ReprojError {
    ReprojError(double u, double v, double w, const Pinhole& c) : u(u), v(v), w(w), fx(c.fx), fy(c.fy), cx(c.cx), cy(c.cy) {}
    template <typename T>
    bool operator()(const T* q, const T* t, const T* X, T* r) const {
        Eigen::Map<const Eigen::Quaternion<T>> qq(q);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> tt(t), XX(X);
        Eigen::Matrix<T, 3, 1> pc = qq * XX + tt;
        if (pc.z() < T(1e-6)) pc.z() = T(1e-6);
        r[0] = T(w) * (T(u) - (T(fx) * pc.x() / pc.z() + T(cx)));
        r[1] = T(w) * (T(v) - (T(fy) * pc.y() / pc.z() + T(cy)));
        return true;
    }
    double u, v, w, fx, fy, cx, cy;
};

// Relative camera motion T_c1c2 = T1 * T2^-1 (T = world->camera) against a measurement.
struct RelPoseError {
    RelPoseError(const SE3& m, double sT, double sR) : qm(m.linear()), tm(m.translation()), sT(sT), sR(sR) {}
    template <typename T>
    bool operator()(const T* q1, const T* t1, const T* q2, const T* t2, T* r) const {
        Eigen::Map<const Eigen::Quaternion<T>> Q1(q1), Q2(q2);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> T1(t1), T2(t2);
        Eigen::Quaternion<T> q12 = Q1 * Q2.conjugate();
        Eigen::Matrix<T, 3, 1> t12 = T1 - q12 * T2;
        Eigen::Quaternion<T> e = qm.cast<T>().conjugate() * q12;
        if (e.w() < T(0)) e.coeffs() = -e.coeffs();
        r[0] = T(2.0 / sR) * e.x();
        r[1] = T(2.0 / sR) * e.y();
        r[2] = T(2.0 / sR) * e.z();
        Eigen::Matrix<T, 3, 1> dt = t12 - tm.cast<T>();
        r[3] = dt.x() / T(sT);
        r[4] = dt.y() / T(sT);
        r[5] = dt.z() / T(sT);
        return true;
    }
    Eigen::Quaterniond qm;
    Vec3 tm;
    double sT, sR;
};

// Sim3 pose-graph edge: S_ij (measured) * S_jw * S_iw^-1 = identity.
struct Sim3Edge {
    Sim3Edge(const Sim3& m, double w) : qm(m.R), tm(m.t), sm(m.s), w(w) {}
    template <typename T>
    bool operator()(const T* qi, const T* ti, const T* si, const T* qj, const T* tj, const T* sj, T* r) const {
        Eigen::Map<const Eigen::Quaternion<T>> Qi(qi), Qj(qj);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> Ti(ti), Tj(tj);
        T scI = exp(si[0]), scJ = exp(sj[0]);
        // A = S_jw * S_iw^-1
        Eigen::Quaternion<T> qA = Qj * Qi.conjugate();
        T sA = scJ / scI;
        Eigen::Matrix<T, 3, 1> tA = Tj - sA * (qA * Ti);
        // E = S_m * A
        Eigen::Quaternion<T> qE = qm.cast<T>() * qA;
        if (qE.w() < T(0)) qE.coeffs() = -qE.coeffs();
        Eigen::Matrix<T, 3, 1> tE = T(sm) * (qm.cast<T>() * tA) + tm.cast<T>();
        T sE = T(sm) * sA;
        r[0] = T(2 * w) * qE.x();
        r[1] = T(2 * w) * qE.y();
        r[2] = T(2 * w) * qE.z();
        r[3] = T(w) * tE.x();
        r[4] = T(w) * tE.y();
        r[5] = T(w) * tE.z();
        r[6] = T(w) * log(sE);
        return true;
    }
    Eigen::Quaterniond qm;
    Vec3 tm;
    double sm, w;
};

// Sim3 between two keyframes from point pairs: obs1 ~ proj(S12 * P2), obs2 ~ proj(S12^-1 * P1).
struct Sim3ProjError {
    Sim3ProjError(const Vec3& P, double u, double v, double w, bool inverse, const Pinhole& c)
        : P(P), u(u), v(v), w(w), inv(inverse), fx(c.fx), fy(c.fy), cx(c.cx), cy(c.cy) {}
    template <typename T>
    bool operator()(const T* q, const T* t, const T* ls, T* r) const {
        Eigen::Map<const Eigen::Quaternion<T>> Q(q);
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> TT(t);
        T s = exp(ls[0]);
        Eigen::Matrix<T, 3, 1> x;
        if (!inv) {
            x = s * (Q * P.cast<T>()) + TT;
        } else {
            x = (Q.conjugate() * (P.cast<T>() - TT)) / s;
        }
        if (x.z() < T(1e-6)) x.z() = T(1e-6);
        r[0] = T(w) * (T(u) - (T(fx) * x.x() / x.z() + T(cx)));
        r[1] = T(w) * (T(v) - (T(fy) * x.y() / x.z() + T(cy)));
        return true;
    }
    Vec3 P;
    double u, v, w;
    bool inv;
    double fx, fy, cx, cy;
};

struct PoseBlock {
    double q[4];  // x y z w (Eigen layout)
    double t[3];
    void set(const SE3& T) {
        Eigen::Quaterniond qq(T.linear());
        qq.normalize();
        q[0] = qq.x(); q[1] = qq.y(); q[2] = qq.z(); q[3] = qq.w();
        t[0] = T.translation().x(); t[1] = T.translation().y(); t[2] = T.translation().z();
    }
    SE3 get() const {
        SE3 T = SE3::Identity();
        T.linear() = Eigen::Quaterniond(q[3], q[0], q[1], q[2]).normalized().toRotationMatrix();
        T.translation() = Vec3(t[0], t[1], t[2]);
        return T;
    }
};

}  // namespace

SE3 odomRelative(const KeyFrame* a, const KeyFrame* b, const SE3& tRobotCam) {
    // T_c1c2 = M^-1 * O1^-1 * O2 * M
    return tRobotCam.inverse() * (a->odom.inverse() * b->odom).toSE3() * tRobotCam;
}

// --------------------------------------------------------------------------- pose only

int optimizePose(Frame& F, const Map& map, const Pinhole& cam, const PosePrior& prior) {
    std::vector<int> idx;
    std::vector<Vec3> pts;
    for (size_t i = 0; i < F.mps.size(); ++i) {
        MapPoint* p = map.mp(F.mps[i]);
        if (!p || p->bad) {
            F.mps[i] = -1;
            continue;
        }
        F.outlier[i] = 0;
        idx.push_back((int)i);
        pts.push_back(p->pos);
    }
    if (idx.size() < 3 && !prior.use) return 0;
    const double huber = std::sqrt(kChi2Mono);
    SE3 T = F.tcw;
    std::vector<char> out(idx.size(), 0);
    int inliers = 0;
    for (int round = 0; round < 4; ++round) {
        for (int it = 0; it < 10; ++it) {
            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();
            Eigen::Matrix<double, 6, 1> g = Eigen::Matrix<double, 6, 1>::Zero();
            for (size_t k = 0; k < idx.size(); ++k) {
                if (out[k]) continue;
                const cv::Point2f& obs = F.f.un[idx[k]];
                float inv2 = F.f.invLevelSigma2[F.f.kps[idx[k]].octave];
                Vec3 pc = T * pts[k];
                if (pc.z() < 1e-6) continue;
                double iz = 1.0 / pc.z();
                Eigen::Vector2d e(obs.x - (cam.fx * pc.x() * iz + cam.cx), obs.y - (cam.fy * pc.y() * iz + cam.cy));
                Eigen::Matrix<double, 2, 3> jp;
                jp << cam.fx * iz, 0, -cam.fx * pc.x() * iz * iz, 0, cam.fy * iz, -cam.fy * pc.y() * iz * iz;
                Eigen::Matrix<double, 3, 6> dp;
                dp.leftCols<3>() = -skew(pc);
                dp.rightCols<3>() = Mat3::Identity();
                Eigen::Matrix<double, 2, 6> J = -jp * dp;
                double chi = std::sqrt(e.squaredNorm() * inv2);
                double w = inv2 * (chi <= huber ? 1.0 : huber / chi);
                H += w * J.transpose() * J;
                g += w * J.transpose() * e;
            }
            if (prior.use) {
                Mat3 dR = T.linear() * prior.tcw.linear().transpose();
                Eigen::Matrix<double, 6, 1> e0;
                e0.head<3>() = logSO3(dR);
                e0.tail<3>() = T.translation() - dR * prior.tcw.translation();
                Eigen::Matrix<double, 6, 1> info;
                info << Vec3::Constant(1 / (prior.sigmaR * prior.sigmaR)), Vec3::Constant(1 / (prior.sigmaT * prior.sigmaT));
                H += info.asDiagonal();
                g += info.asDiagonal() * e0;
            }
            H.diagonal().array() += 1e-9;
            Eigen::Matrix<double, 6, 1> dx = -H.ldlt().solve(g);
            if (!dx.allFinite()) break;
            SE3 D = SE3::Identity();
            D.linear() = expSO3(dx.head<3>());
            D.translation() = dx.tail<3>();
            T = D * T;
            if (dx.norm() < 1e-8) break;
        }
        inliers = 0;
        for (size_t k = 0; k < idx.size(); ++k) {
            const cv::Point2f& obs = F.f.un[idx[k]];
            float inv2 = F.f.invLevelSigma2[F.f.kps[idx[k]].octave];
            Vec3 pc = T * pts[k];
            double chi2 = 1e9;
            if (pc.z() > 1e-6) {
                double du = obs.x - (cam.fx * pc.x() / pc.z() + cam.cx), dv = obs.y - (cam.fy * pc.y() / pc.z() + cam.cy);
                chi2 = (du * du + dv * dv) * inv2;
            }
            out[k] = chi2 > kChi2Mono;
            if (!out[k]) ++inliers;
        }
        if (inliers < 10 && !prior.use) break;
    }
    F.tcw = T;
    for (size_t k = 0; k < idx.size(); ++k) F.outlier[idx[k]] = out[k];
    return inliers;
}

// --------------------------------------------------------------------------- local BA

static int bundle(Map& map, const std::vector<KeyFrame*>& opt, const std::vector<KeyFrame*>& fixed,
                  const std::vector<MapPoint*>& pts, const Pinhole& cam, bool useOdom, const SE3& tRobotCam,
                  const OdomEdgeParams& op, int iterations, bool removeOutliers) {
    std::map<int, PoseBlock> poses;
    std::set<int> fixedIds;
    for (KeyFrame* k : opt) poses[k->id].set(k->tcw);
    for (KeyFrame* k : fixed) {
        poses[k->id].set(k->tcw);
        fixedIds.insert(k->id);
    }
    if (map.origin >= 0 && poses.count(map.origin)) fixedIds.insert(map.origin);
    std::map<int, std::array<double, 3>> X;
    for (MapPoint* p : pts) X[p->id] = {p->pos.x(), p->pos.y(), p->pos.z()};

    ceres::Problem problem;
    ceres::Manifold* qm = new ceres::EigenQuaternionManifold();
    const double huber = std::sqrt(kChi2Mono);
    struct Obs {
        int kf, mp, idx;
    };
    std::vector<Obs> obs;
    for (MapPoint* p : pts) {
        for (const auto& o : p->obs) {
            auto it = poses.find(o.first);
            if (it == poses.end()) continue;
            KeyFrame* k = map.kf(o.first);
            const cv::Point2f& u = k->f.un[o.second];
            double w = std::sqrt(k->f.invLevelSigma2[k->f.kps[o.second].octave]);
            auto* cost = new ceres::AutoDiffCostFunction<ReprojError, 2, 4, 3, 3>(new ReprojError(u.x, u.y, w, cam));
            problem.AddResidualBlock(cost, new ceres::HuberLoss(huber), it->second.q, it->second.t, X[p->id].data());
            obs.push_back({o.first, p->id, o.second});
        }
    }
    if (obs.empty()) return 0;
    if (useOdom) {
        // Odometry factors between keyframes that are consecutive in time.
        std::vector<int> ids;
        for (const auto& e : poses) ids.push_back(e.first);
        std::sort(ids.begin(), ids.end());
        for (size_t i = 1; i < ids.size(); ++i) {
            KeyFrame* a = map.kf(ids[i - 1]);
            KeyFrame* b = map.kf(ids[i]);
            if (!a->hasOdom || !b->hasOdom) continue;
            // Only for neighbours in time (no other good keyframe in between).
            bool between = false;
            for (int k = ids[i - 1] + 1; k < ids[i]; ++k)
                if (map.kf(k) && !map.kf(k)->bad) between = true;
            if (between) continue;
            SE3 m = odomRelative(a, b, tRobotCam);
            double dist = m.translation().norm();
            double turn = logSO3(m.linear()).norm();
            auto* cost = new ceres::AutoDiffCostFunction<RelPoseError, 6, 4, 3, 4, 3>(
                new RelPoseError(m, op.sigmaTBase + op.sigmaTPerM * dist, op.sigmaRBase + op.sigmaRPerRad * turn));
            problem.AddResidualBlock(cost, nullptr, poses[a->id].q, poses[a->id].t, poses[b->id].q, poses[b->id].t);
        }
    }
    bool attached = false;
    for (auto& e : poses) {
        if (!problem.HasParameterBlock(e.second.q)) continue;
        problem.SetManifold(e.second.q, qm);
        attached = true;
        if (fixedIds.count(e.first)) {
            problem.SetParameterBlockConstant(e.second.q);
            problem.SetParameterBlockConstant(e.second.t);
        }
    }
    if (!attached) delete qm;
    ceres::Solver::Options so;
    so.linear_solver_type = ceres::DENSE_SCHUR;
    so.max_num_iterations = iterations;
    so.num_threads = 4;
    so.logging_type = ceres::SILENT;
    ceres::Solver::Summary sum;
    ceres::Solve(so, &problem, &sum);

    int removed = 0;
    if (removeOutliers) {
        for (const Obs& o : obs) {
            KeyFrame* k = map.kf(o.kf);
            MapPoint* p = map.mp(o.mp);
            if (!p || p->bad) continue;
            SE3 T = poses[o.kf].get();
            Vec3 pc = T * Vec3(X[o.mp][0], X[o.mp][1], X[o.mp][2]);
            const cv::Point2f& u = k->f.un[o.idx];
            double chi2 = 1e9;
            if (pc.z() > 0) {
                double du = u.x - (cam.fx * pc.x() / pc.z() + cam.cx), dv = u.y - (cam.fy * pc.y() / pc.z() + cam.cy);
                chi2 = (du * du + dv * dv) * k->f.invLevelSigma2[k->f.kps[o.idx].octave];
            }
            if (chi2 > kChi2Mono) {
                map.eraseObservation(p, k);
                ++removed;
            }
        }
    }
    for (KeyFrame* k : opt)
        if (!fixedIds.count(k->id)) k->tcw = poses[k->id].get();
    for (MapPoint* p : pts) {
        if (p->bad) continue;
        p->pos = Vec3(X[p->id][0], X[p->id][1], X[p->id][2]);
        map.updateNormalAndDepth(p);
    }
    return removed;
}

int localBundleAdjustment(KeyFrame* kf, Map& map, const Pinhole& cam, int window, bool useOdom, const SE3& tRobotCam,
                          const OdomEdgeParams& op, long baId) {
    std::vector<KeyFrame*> local = {kf};
    kf->localBaFor = baId;
    for (KeyFrame* k : map.bestCovisibles(kf, window)) {
        k->localBaFor = baId;
        local.push_back(k);
    }
    if (useOdom) {
        // Keep the most recent keyframes in the window too: odometry ties them together.
        int added = 0;
        for (int id = kf->id - 1; id >= 0 && added < 3; --id) {
            KeyFrame* k = map.kf(id);
            if (!k || k->bad) continue;
            if (k->localBaFor != baId) {
                k->localBaFor = baId;
                local.push_back(k);
            }
            ++added;
        }
    }
    std::vector<MapPoint*> pts;
    for (KeyFrame* k : local)
        for (MapPoint* p : map.pointsOf(k))
            if (p->localBaFor != baId) {
                p->localBaFor = baId;
                pts.push_back(p);
            }
    std::vector<KeyFrame*> fixed;
    for (MapPoint* p : pts)
        for (const auto& o : p->obs) {
            KeyFrame* k = map.kf(o.first);
            if (k && !k->bad && k->localBaFor != baId && k->fixedBaFor != baId) {
                k->fixedBaFor = baId;
                fixed.push_back(k);
            }
        }
    if (useOdom) {
        // The keyframe just before the oldest local one anchors the odometry chain.
        int oldest = kf->id;
        for (KeyFrame* k : local) oldest = std::min(oldest, k->id);
        for (int id = oldest - 1; id >= 0; --id) {
            KeyFrame* k = map.kf(id);
            if (!k || k->bad) continue;
            if (k->fixedBaFor != baId && k->localBaFor != baId) {
                k->fixedBaFor = baId;
                fixed.push_back(k);
            }
            break;
        }
    }
    // Monocular without odometry needs fixed keyframes (gauge); with none, hold the oldest.
    if (fixed.empty() && map.origin >= 0 && kf->id != map.origin) {
        bool hasOrigin = false;
        for (KeyFrame* k : local) hasOrigin |= k->id == map.origin;
        if (!hasOrigin) {
            std::sort(local.begin(), local.end(), [](KeyFrame* a, KeyFrame* b) { return a->id < b->id; });
            fixed.push_back(local.front());
            local.erase(local.begin());
        }
    }
    for (MapPoint* p : pts) p->inBackend = true;
    int removed = bundle(map, local, fixed, pts, cam, useOdom, tRobotCam, op, 5, false);
    removed += bundle(map, local, fixed, pts, cam, useOdom, tRobotCam, op, 10, true);
    return removed;
}

void globalBundleAdjustment(Map& map, const Pinhole& cam, int iterations, bool useOdom, const SE3& tRobotCam,
                            const OdomEdgeParams& op) {
    std::vector<KeyFrame*> kfs = map.keyFrames();
    std::vector<MapPoint*> pts = map.mapPoints();
    std::vector<KeyFrame*> fixed;
    bundle(map, kfs, fixed, pts, cam, useOdom, tRobotCam, op, iterations, true);
}

// ------------------------------------------------------------------------- pose graph

void optimizeEssentialGraph(Map& map, KeyFrame* loopKf, KeyFrame* curKf, const std::map<int, Sim3>& corrected,
                            const std::map<int, Sim3>& nonCorrected, const std::map<int, std::set<int>>& newLinks,
                            int minW, bool fixedScale, const SE3& tRobotCam, const OdomEdgeParams& op) {
    std::vector<KeyFrame*> kfs = map.keyFrames();
    struct Block {
        double q[4], t[3], s[1];
    };
    std::map<int, Block> B;
    std::map<int, Sim3> before;  // S_iw used for measurements
    for (KeyFrame* k : kfs) {
        Sim3 S = corrected.count(k->id) ? corrected.at(k->id) : Sim3(k->tcw);
        before[k->id] = S;
        Eigen::Quaterniond q(S.R);
        Block b;
        b.q[0] = q.x(); b.q[1] = q.y(); b.q[2] = q.z(); b.q[3] = q.w();
        b.t[0] = S.t.x(); b.t[1] = S.t.y(); b.t[2] = S.t.z();
        b.s[0] = std::log(S.s);
        B[k->id] = b;
    }
    auto meas = [&](int i, int j) {  // S_ij from the uncorrected poses where available
        Sim3 si = nonCorrected.count(i) ? nonCorrected.at(i) : before[i];
        Sim3 sj = nonCorrected.count(j) ? nonCorrected.at(j) : before[j];
        return si * sj.inverse();
    };
    ceres::Problem problem;
    std::set<std::pair<int, int>> done;
    auto addEdge = [&](int i, int j, const Sim3& m, double w) {
        if (i == j || !B.count(i) || !B.count(j)) return;
        auto key = std::make_pair(std::min(i, j), std::max(i, j));
        if (done.count(key)) return;
        done.insert(key);
        auto* cost = new ceres::AutoDiffCostFunction<Sim3Edge, 7, 4, 3, 1, 4, 3, 1>(new Sim3Edge(m, w));
        problem.AddResidualBlock(cost, nullptr, B[i].q, B[i].t, B[i].s, B[j].q, B[j].t, B[j].s);
    };
    // New loop links use the corrected poses (that is the loop constraint).
    for (const auto& e : newLinks)
        for (int j : e.second) {
            if (!B.count(e.first) || !B.count(j)) continue;
            auto key = std::make_pair(std::min(e.first, j), std::max(e.first, j));
            if (done.count(key)) continue;
            done.insert(key);
            Sim3 m = before[e.first] * before[j].inverse();
            auto* cost = new ceres::AutoDiffCostFunction<Sim3Edge, 7, 4, 3, 1, 4, 3, 1>(new Sim3Edge(m, 1.0));
            problem.AddResidualBlock(cost, nullptr, B[e.first].q, B[e.first].t, B[e.first].s, B[j].q, B[j].t, B[j].s);
        }
    for (KeyFrame* k : kfs) {
        if (k->parent >= 0 && B.count(k->parent)) addEdge(k->id, k->parent, meas(k->id, k->parent), 1.0);
        for (int l : k->loopEdges)
            if (l < k->id) addEdge(k->id, l, meas(k->id, l), 1.0);
        for (KeyFrame* c : map.covisiblesByWeight(k, minW)) addEdge(k->id, c->id, meas(k->id, c->id), 1.0);
    }
    if (fixedScale) {
        // Odometry edges between consecutive keyframes (metric, SE3).
        KeyFrame* prev = nullptr;
        for (KeyFrame* k : kfs) {
            if (prev && prev->hasOdom && k->hasOdom) {
                SE3 m = odomRelative(prev, k, tRobotCam);
                double sT = op.sigmaTBase + op.sigmaTPerM * m.translation().norm();
                addEdge(prev->id, k->id, Sim3(m), std::min(1.0, 0.01 / sT));
            }
            prev = k;
        }
    }
    ceres::Manifold* qm = new ceres::EigenQuaternionManifold();
    bool attached = false;
    for (auto& e : B) {
        if (!problem.HasParameterBlock(e.second.q)) continue;
        problem.SetManifold(e.second.q, qm);
        attached = true;
        if (fixedScale) problem.SetParameterBlockConstant(e.second.s);
        if (e.first == loopKf->id || e.first == map.origin) {
            problem.SetParameterBlockConstant(e.second.q);
            problem.SetParameterBlockConstant(e.second.t);
            problem.SetParameterBlockConstant(e.second.s);
        }
    }
    if (!attached) {
        delete qm;
        return;
    }
    ceres::Solver::Options so;
    so.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    so.sparse_linear_algebra_library_type = ceres::EIGEN_SPARSE;
    so.max_num_iterations = 20;
    so.num_threads = 4;
    so.logging_type = ceres::SILENT;
    ceres::Solver::Summary sum;
    ceres::Solve(so, &problem, &sum);
    // Apply: keyframe poses, then each point with its reference keyframe's correction.
    std::map<int, Sim3> after;
    for (KeyFrame* k : kfs) {
        const Block& b = B[k->id];
        Sim3 S(std::exp(b.s[0]), Eigen::Quaterniond(b.q[3], b.q[0], b.q[1], b.q[2]).normalized().toRotationMatrix(),
               Vec3(b.t[0], b.t[1], b.t[2]));
        after[k->id] = S;
    }
    for (MapPoint* p : map.mapPoints()) {
        // Points moved by the loop correction are expressed with their correcting keyframe's
        // corrected pose; the others with their reference keyframe's pose.
        int ref = p->correctedBy == curKf->id ? p->correctedRef : p->refKf;
        if (!after.count(ref)) continue;
        p->pos = after[ref].inverse().map(before[ref].map(p->pos));
    }
    for (KeyFrame* k : kfs) k->tcw = after[k->id].toSE3();
    for (MapPoint* p : map.mapPoints()) map.updateNormalAndDepth(p);
}

// ------------------------------------------------------------------------------ Sim3

int optimizeSim3(KeyFrame* k1, KeyFrame* k2, std::vector<int>& m12, Sim3& s12, const Pinhole& cam, const Map& map,
                 double th2, bool fixScale) {
    Eigen::Quaterniond q(s12.R);
    double qq[4] = {q.x(), q.y(), q.z(), q.w()};
    double tt[3] = {s12.t.x(), s12.t.y(), s12.t.z()};
    double ls[1] = {std::log(s12.s)};
    struct Pair {
        int i1;
        ceres::ResidualBlockId r1, r2;
        Vec3 p1, p2;
        int i2;
    };
    std::vector<Pair> pairs;
    ceres::Problem problem;
    double huber = std::sqrt(th2);
    for (size_t i1 = 0; i1 < m12.size(); ++i1) {
        if (m12[i1] < 0) continue;
        MapPoint* a = map.mp(k1->mps[i1]);
        MapPoint* b = map.mp(m12[i1]);
        if (!a || !b || a->bad || b->bad || !b->obs.count(k2->id)) {
            m12[i1] = -1;
            continue;
        }
        int i2 = b->obs.at(k2->id);
        Vec3 p1 = k1->tcw * a->pos, p2 = k2->tcw * b->pos;
        const cv::Point2f& u1 = k1->f.un[i1];
        const cv::Point2f& u2 = k2->f.un[i2];
        double w1 = std::sqrt(k1->f.invLevelSigma2[k1->f.kps[i1].octave]);
        double w2 = std::sqrt(k2->f.invLevelSigma2[k2->f.kps[i2].octave]);
        auto* c1 = new ceres::AutoDiffCostFunction<Sim3ProjError, 2, 4, 3, 1>(new Sim3ProjError(p2, u1.x, u1.y, w1, false, cam));
        auto* c2 = new ceres::AutoDiffCostFunction<Sim3ProjError, 2, 4, 3, 1>(new Sim3ProjError(p1, u2.x, u2.y, w2, true, cam));
        Pair pr;
        pr.i1 = (int)i1;
        pr.i2 = i2;
        pr.p1 = p1;
        pr.p2 = p2;
        pr.r1 = problem.AddResidualBlock(c1, new ceres::HuberLoss(huber), qq, tt, ls);
        pr.r2 = problem.AddResidualBlock(c2, new ceres::HuberLoss(huber), qq, tt, ls);
        pairs.push_back(pr);
    }
    if (pairs.size() < 10) return 0;
    problem.SetManifold(qq, new ceres::EigenQuaternionManifold());
    if (fixScale) problem.SetParameterBlockConstant(ls);
    ceres::Solver::Options so;
    so.linear_solver_type = ceres::DENSE_QR;
    so.max_num_iterations = 10;
    so.logging_type = ceres::SILENT;
    ceres::Solver::Summary sum;
    ceres::Solve(so, &problem, &sum);
    s12 = Sim3(std::exp(ls[0]), Eigen::Quaterniond(qq[3], qq[0], qq[1], qq[2]).normalized().toRotationMatrix(),
               Vec3(tt[0], tt[1], tt[2]));
    int inliers = 0;
    for (const Pair& pr : pairs) {
        Vec3 x1 = s12.map(pr.p2);
        Vec3 x2 = s12.inverse().map(pr.p1);
        auto err = [&](const Vec3& x, const cv::Point2f& u, float inv2) {
            if (x.z() <= 0) return 1e9;
            double du = u.x - (cam.fx * x.x() / x.z() + cam.cx), dv = u.y - (cam.fy * x.y() / x.z() + cam.cy);
            return (du * du + dv * dv) * inv2;
        };
        double e1 = err(x1, k1->f.un[pr.i1], k1->f.invLevelSigma2[k1->f.kps[pr.i1].octave]);
        double e2 = err(x2, k2->f.un[pr.i2], k2->f.invLevelSigma2[k2->f.kps[pr.i2].octave]);
        if (e1 > th2 || e2 > th2) m12[pr.i1] = -1;
        else ++inliers;
    }
    return inliers;
}

}  // namespace beacon
