#include "edges.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <map>
#include <opencv2/imgproc.hpp>

namespace beacon {

namespace {

// Bilinear sample of a CV_32F image; 0 outside.
inline float bilinear(const cv::Mat& m, double x, double y) {
    int x0 = (int)std::floor(x), y0 = (int)std::floor(y);
    if (x0 < 0 || y0 < 0 || x0 + 1 >= m.cols || y0 + 1 >= m.rows) return 0.f;
    double ax = x - x0, ay = y - y0;
    const float* r0 = m.ptr<float>(y0);
    const float* r1 = m.ptr<float>(y0 + 1);
    return (float)((1 - ay) * ((1 - ax) * r0[x0] + ax * r0[x0 + 1]) + ay * ((1 - ax) * r1[x0] + ax * r1[x0 + 1]));
}

}  // namespace

EdgeTracker::EdgeTracker(const FieldModel& field, const Camera& cam, const Params& params)
    : field_(field), cam_(cam), p_(params) {}

bool EdgeTracker::project(const SE2& pose, const SE3& tRobotCam, const Vec3& pw, Vec2& px) const {
    SE3 tCamField = cameraInField(pose, tRobotCam).inverse();
    Vec3 pc = tCamField * pw;
    if (pc.z() < 0.05) return false;
    px = cam_.project(pc);
    return true;
}

void EdgeTracker::makeSamples(const SE2& pose, const SE3& tRobotCam, std::vector<Sample>& out, int onlyPart,
                              int onlyState) const {
    out.clear();
    SE3 tFieldCam = cameraInField(pose, tRobotCam);
    SE3 tCamField = tFieldCam.inverse();
    Vec3 camCentre = tFieldCam.translation();
    const double nearZ = 0.10;
    for (size_t ei = 0; ei < field_.edges.size(); ++ei) {
        const ModelEdge& e = field_.edges[ei];
        if (e.type == EdgeType::Seam && p_.edgeSeamWeight <= 0) continue;
        if (onlyPart >= 0) {
            if (e.movingPart != onlyPart || e.state != onlyState) continue;
        } else if (!e.isStatic) {
            // Tipping hive parts anchor only in their estimated tilt state.
            if (e.movingPart < 0 || e.movingPart >= (int)partState_.size() || partState_[e.movingPart] != e.state) continue;
        }
        Vec3 a = tCamField * e.a, b = tCamField * e.b;
        // Clip the segment to the part in front of the near plane.
        if (a.z() < nearZ && b.z() < nearZ) continue;
        double s0 = 0, s1 = 1;
        if (a.z() < nearZ) s0 = (nearZ - a.z()) / (b.z() - a.z());
        if (b.z() < nearZ) s1 = (nearZ - a.z()) / (b.z() - a.z());
        Vec3 ca = a + s0 * (b - a), cb = a + s1 * (b - a);
        Vec2 pa = cam_.toPixelNoDistortion(Vec2(ca.x() / ca.z(), ca.y() / ca.z()));
        Vec2 pb = cam_.toPixelNoDistortion(Vec2(cb.x() / cb.z(), cb.y() / cb.z()));
        double lenPx = (pb - pa).norm();
        if (!std::isfinite(lenPx) || lenPx < 2 * p_.edgeSampleStepPx) continue;
        // Cap samples per edge so one long near edge (a hive leg next to the camera) cannot
        // outvote the rest of the model.
        int n = std::min(p_.edgeMaxSamplesPerEdge, (int)(lenPx / p_.edgeSampleStepPx));
        Vec3 wa = e.a + s0 * (e.b - e.a), wb = e.a + s1 * (e.b - e.a);
        Vec3 dir = wb - wa;
        for (int k = 0; k < n; ++k) {
            double s = (k + 0.5) / n;
            Vec3 pw = wa + s * dir;
            Vec3 pc = tCamField * pw;
            if (pc.z() < nearZ || pc.norm() > (e.type == EdgeType::Seam ? p_.edgeSeamMaxRangeM : p_.edgeMaxRangeM)) continue;
            Vec2 px = cam_.project(pc);
            if (!cam_.inImage(px, 4)) continue;
            // Hidden behind a hive face?
            Vec3 ray = pw - camCentre;
            double dist = ray.norm();
            bool hidden = false;
            for (const Surface& f : field_.hiveFaces) {
                if (!f.convex) continue;
                if (f.movingPart >= 0 &&
                    (f.movingPart >= (int)partState_.size() || partState_[f.movingPart] != f.state))
                    continue;
                double t = f.intersect(camCentre, ray / dist);
                if (t > 0 && t < dist - 0.01) {
                    hidden = true;
                    break;
                }
            }
            if (hidden) continue;
            Vec3 pc2 = tCamField * (pw + 1e-3 * dir.normalized());
            if (pc2.z() < nearZ) continue;
            Vec2 t = cam_.project(pc2) - px;
            if (t.norm() < 1e-9) continue;
            t.normalize();
            Sample smp;
            smp.world = pw;
            smp.normal = Vec2(-t.y(), t.x());
            smp.px = px;
            smp.edge = (int)ei;
            // Panel joints and tile seams are thin dark lines: matched as intensity valleys, so a
            // step edge in the venue behind the clear panel cannot stand in for a joint.
            smp.ridge = e.type == EdgeType::Seam || (e.type == EdgeType::Post && p_.edgePostAsRidge);
            if (smp.ridge) {
                double widthM = e.type == EdgeType::Seam ? p_.seamWidthM : p_.postWidthM;
                smp.ridgeOffsetPx = std::min(8.0, std::max(2.0, cam_.fx * widthM / (2.0 * pc.z()) + 1.0));
            }
            // Polarity: the perimeter rails are black, so each horizontal wall edge has a known
            // dark side. "Up" in the image at this point decides the sign along the normal.
            int darkAbove = 0;
            if (e.type == EdgeType::WallFloor || e.type == EdgeType::RailBottom) darkAbove = 1;
            if (e.type == EdgeType::WallTop || e.type == EdgeType::BottomRail) darkAbove = -1;
            if (darkAbove != 0 && p_.edgeUsePolarity) {
                Vec3 up = tCamField * (pw + Vec3(0, 0, 0.01));
                if (up.z() > nearZ) {
                    Vec2 dir = cam_.project(up) - px;
                    double side = smp.normal.dot(dir);
                    // Intensity falls toward the dark side: dark above -> gradient points down.
                    if (std::abs(side) > 1e-6) smp.polarity = (side > 0 ? -1 : 1) * darkAbove;
                }
            }
            out.push_back(smp);
        }
    }
}

void EdgeTracker::search(Sample& s, double range, const EdgeFrame& f) const {
    s.candidates.clear();
    int r = (int)std::ceil(range);
    std::vector<float> g(2 * r + 1);
    if (s.ridge) {
        // Dark line: intensity below the mean of both sides 2 px away.
        for (int k = -r; k <= r; ++k) {
            auto at = [&](double o) {
                return bilinear(f.blur, s.px.x() + o * s.normal.x(), s.px.y() + o * s.normal.y());
            };
            double o = s.ridgeOffsetPx;
            g[k + r] = std::max(0.f, 0.5f * (at(k - o) + at(k + o)) - at(k));
        }
        std::vector<std::pair<float, double>> peaks;
        for (int i = 1; i + 1 < (int)g.size(); ++i) {
            if (g[i] < p_.edgeSeamMinContrast || g[i] < g[i - 1] || g[i] < g[i + 1]) continue;
            double den = g[i - 1] - 2 * g[i] + g[i + 1];
            peaks.push_back({g[i], i - r + (den < 0 ? 0.5 * (g[i - 1] - g[i + 1]) / den : 0.0)});
        }
        std::sort(peaks.begin(), peaks.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
        for (size_t i = 0; i < peaks.size() && i < 2; ++i) s.candidates.push_back(peaks[i].second);
        return;
    }
    for (int k = -r; k <= r; ++k) {
        double x = s.px.x() + k * s.normal.x(), y = s.px.y() + k * s.normal.y();
        float gn = bilinear(f.gx, x, y) * (float)s.normal.x() + bilinear(f.gy, x, y) * (float)s.normal.y();
        g[k + r] = s.polarity == 0 ? std::abs(gn) : std::max(0.f, gn * (float)s.polarity);
    }
    // Local maxima above threshold, strongest first, at most 4.
    std::vector<std::pair<float, double>> peaks;
    for (int i = 1; i + 1 < (int)g.size(); ++i) {
        if (g[i] < p_.edgeMinGradient || g[i] < g[i - 1] || g[i] < g[i + 1]) continue;
        double den = g[i - 1] - 2 * g[i] + g[i + 1];
        double sub = den < 0 ? 0.5 * (g[i - 1] - g[i + 1]) / den : 0.0;
        peaks.push_back({g[i], i - r + sub});
    }
    std::sort(peaks.begin(), peaks.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    for (size_t i = 0; i < peaks.size() && i < 4; ++i) s.candidates.push_back(peaks[i].second);
}

EdgeFrame EdgeTracker::prepare(const cv::Mat& gray, const SE2& odom, int64_t tNs) const {
    EdgeFrame f;
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);
    cv::Sobel(blurred, f.gx, CV_32F, 1, 0, 3, 0.25);
    cv::Sobel(blurred, f.gy, CV_32F, 0, 1, 3, 0.25);
    blurred.convertTo(f.blur, CV_32F);
    f.odom = odom;
    f.tNs = tNs;
    return f;
}

// d(A * o)/d(A params), as in predict.cpp.
static Mat3 alignJacobian(const SE2& a, const SE2& o) {
    SE2 p = a * o;
    Mat3 j = Mat3::Identity();
    j(0, 2) = -(p.y - a.y);
    j(1, 2) = p.x - a.x;
    return j;
}

EdgeResult EdgeTracker::solveWindow(const std::vector<const EdgeFrame*>& frames, const SE2& a0, const Mat3& covA,
                                    const SE3& tRobotCam, bool wantOverlay) {
    EdgeResult res;
    if (frames.empty()) return res;
    const EdgeFrame& lastF = *frames.back();
    // Initial search range from the uncertainty of the newest pose (3 sigma, 1.5 m lever).
    Mat3 jl = alignJacobian(a0, lastF.odom);
    Mat3 covLast = jl * covA * jl.transpose();
    double sxy = std::sqrt(std::max(covLast(0, 0), covLast(1, 1)));
    double sth = std::sqrt(covLast(2, 2));
    double range = 3.0 * cam_.fx * (sth + sxy / 1.5);
    range = std::min(p_.edgeSearchMaxPx, std::max(p_.edgeSearchMinPx * 2, range));

    Mat3 priorInfo = (covA + Mat3::Identity() * 1e-8).inverse();
    SE2 A = a0;
    const size_t nf = frames.size();
    std::vector<std::vector<Sample>> samples(nf);
    std::vector<std::vector<Vec2>> anchors(nf);
    std::vector<std::vector<double>> resid(nf);
    Eigen::Matrix3d hData = Eigen::Matrix3d::Zero();
    const double eps = 1e-4;
    size_t total = 0;
    for (int outer = 0; outer < 4; ++outer) {
        total = 0;
        for (size_t f = 0; f < nf; ++f) {
            makeSamples(A * frames[f]->odom, tRobotCam, samples[f]);
            for (Sample& smp : samples[f]) search(smp, range, *frames[f]);
            anchors[f].resize(samples[f].size());
            for (size_t i = 0; i < samples[f].size(); ++i) anchors[f][i] = samples[f][i].px;
            total += samples[f].size();
        }
        if (total < 6) {
            res.samples = (int)total;
            return res;
        }
        double cutoff = std::max(3.0 * p_.edgeHuberPx, range);
        for (int inner = 0; inner < 8; ++inner) {
            Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
            Eigen::Vector3d g = Eigen::Vector3d::Zero();
            hData.setZero();
            for (size_t f = 0; f < nf; ++f) {
                const SE2& o = frames[f]->odom;
                SE2 pose = A * o;
                resid[f].assign(samples[f].size(), NAN);
                for (size_t i = 0; i < samples[f].size(); ++i) {
                    Sample& smp = samples[f][i];
                    if (smp.candidates.empty()) continue;
                    Vec2 px;
                    if (!project(pose, tRobotCam, smp.world, px)) continue;
                    double along = smp.normal.dot(px - anchors[f][i]);
                    double best = 1e9;
                    for (double c : smp.candidates)
                        if (std::abs(along - c) < std::abs(best)) best = along - c;
                    if (std::abs(best) > cutoff) continue;
                    resid[f][i] = best;
                    Eigen::RowVector3d j;
                    bool okJ = true;
                    for (int k = 0; k < 3 && okJ; ++k) {
                        SE2 ap = A, am = A;
                        double* fp = k == 0 ? &ap.x : k == 1 ? &ap.y : &ap.theta;
                        double* fm = k == 0 ? &am.x : k == 1 ? &am.y : &am.theta;
                        *fp += eps;
                        *fm -= eps;
                        Vec2 pa, pb;
                        okJ = project(ap * o, tRobotCam, smp.world, pa) && project(am * o, tRobotCam, smp.world, pb);
                        j(k) = okJ ? smp.normal.dot(pa - pb) / (2 * eps) : 0.0;
                    }
                    if (!okJ) continue;
                    double w = std::abs(best) <= p_.edgeHuberPx ? 1.0 : p_.edgeHuberPx / std::abs(best);
                    if (smp.ridge && field_.edges[smp.edge].type == EdgeType::Seam) w *= p_.edgeSeamWeight;
                    h += w * j.transpose() * j;
                    g += w * j.transpose() * best;
                    hData += w * j.transpose() * j;
                }
            }
            // Weak prior toward the predicted alignment keeps unobservable directions in place.
            Eigen::Vector3d dp(A.x - a0.x, A.y - a0.y, wrapAngle(A.theta - a0.theta));
            double ps = p_.edgeSigmaPx * p_.edgeSigmaPx;
            Eigen::Vector3d step = -(h + priorInfo * ps).ldlt().solve(g + priorInfo * ps * dp);
            if (!step.allFinite()) break;
            A.x += step(0);
            A.y += step(1);
            A.theta = wrapAngle(A.theta + step(2));
            if (step.cwiseAbs().maxCoeff() < 1e-5) break;
        }
        range = std::max(p_.edgeSearchMinPx, range * 0.4);
    }
    double ss = 0;
    for (size_t f = 0; f < nf; ++f) {
        for (size_t i = 0; i < samples[f].size(); ++i) {
            if (!samples[f][i].candidates.empty()) res.matched++;
            if (i < resid[f].size() && std::isfinite(resid[f][i]) && std::abs(resid[f][i]) <= 2.0 * p_.edgeHuberPx) {
                res.inliers++;
                ss += resid[f][i] * resid[f][i];
            }
        }
    }
    res.samples = (int)total;
    res.rmsPx = res.inliers ? std::sqrt(ss / res.inliers) : 0;
    // Part consistency on the newest frame: a wrong pose can still win on most edges while a
    // whole nearby part (the hive frame foot, a wall) disagrees and is dropped as outliers.
    {
        std::map<int, std::pair<int, int>> parts;  // part key -> (samples, inliers)
        size_t f = nf - 1;
        for (size_t i = 0; i < samples[f].size(); ++i) {
            const ModelEdge& e = field_.edges[samples[f][i].edge];
            if (e.type == EdgeType::Seam) continue;
            int key = e.type == EdgeType::Hive ? (e.movingPart >= 0 ? -20 - e.movingPart : -10) : e.wall;
            auto& c = parts[key];
            c.first++;
            if (i < resid[f].size() && std::isfinite(resid[f][i]) && std::abs(resid[f][i]) <= 2.0 * p_.edgeHuberPx) c.second++;
        }
        for (const auto& c : parts) {
            if (c.second.first < p_.partMinSamples) continue;
            double frac = (double)c.second.second / c.second.first;
            if (frac < res.worstPartFrac) {
                res.worstPartFrac = frac;
                res.worstPartSamples = c.second.first;
            }
        }
    }
    // Covariance of the alignment from the data alone, inflated for correlated samples and
    // for residuals larger than the nominal pixel noise; then mapped to the newest pose.
    double rmsInfl = std::max(1.0, (res.rmsPx * res.rmsPx) / (p_.edgeSigmaPx * p_.edgeSigmaPx));
    Eigen::Matrix3d info = hData / (p_.edgeSigmaPx * p_.edgeSigmaPx * p_.edgeCorrelationInflation * rmsInfl * nf);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(info);
    Mat3 cA = es.eigenvalues().minCoeff() > 1e-6 ? Mat3(info.inverse()) : Mat3(Mat3::Identity() * 1e6);
    Mat3 jf = alignJacobian(A, lastF.odom);
    res.cov = jf * cA * jf.transpose();
    res.pose = A * lastF.odom;
    res.alignment = A;
    res.ok = res.pose.finite();
    if (wantOverlay) {
        std::vector<Sample> fin;
        makeSamples(res.pose, tRobotCam, fin);
        for (size_t i = 1; i < fin.size(); ++i) {
            if (fin[i].edge != fin[i - 1].edge) continue;
            res.overlay.push_back({cv::Point2f((float)fin[i - 1].px.x(), (float)fin[i - 1].px.y()),
                                   cv::Point2f((float)fin[i].px.x(), (float)fin[i].px.y())});
        }
    }
    return res;
}

EdgeResult EdgeTracker::solveWindowWide(const std::vector<const EdgeFrame*>& frames, const SE2& a0, const Mat3& covA,
                                        const SE3& tRobotCam, bool wantOverlay) {
    // Hypotheses on the alignment, spread over about 2 sigma; each solved with a tight prior.
    double sxy = std::sqrt(std::max(covA(0, 0), covA(1, 1)));
    double sth = std::sqrt(covA(2, 2));
    double stepXY = std::max(0.03, 0.75 * sxy), stepTh = std::max(0.01, 0.75 * sth);
    Mat3 local = Mat3::Zero();
    local(0, 0) = local(1, 1) = stepXY * stepXY;
    local(2, 2) = stepTh * stepTh;
    // Rotations of the alignment pivot about the newest robot position, not the field origin.
    const SE2& o = frames.back()->odom;
    SE2 p0 = a0 * o;
    EdgeResult best;
    double bestScore = -1e18;
    std::vector<std::pair<double, SE2>> all;  // (score, pose) of every converged hypothesis
    for (int k = -2; k <= 2; ++k)
        for (int i = -1; i <= 1; ++i)
            for (int j = -1; j <= 1; ++j) {
                SE2 p(p0.x + i * stepXY * 1.5, p0.y + j * stepXY * 1.5, p0.theta + k * stepTh);
                SE2 a = p * o.inverse();
                EdgeResult r = solveWindow(frames, a, local, tRobotCam, false);
                if (!r.ok) continue;
                // Most inliers wins; the prior breaks near-ties (equally good explanations).
                Eigen::Vector3d d(r.pose.x - p0.x, r.pose.y - p0.y, wrapAngle(r.pose.theta - p0.theta));
                Mat3 cp = alignJacobian(a0, o) * covA * alignJacobian(a0, o).transpose();
                double score = r.inliers - 0.5 * d.dot(cp.ldlt().solve(d)) * p_.wideSearchPriorWeight;
                all.push_back({score, r.pose});
                if (score > bestScore) {
                    bestScore = score;
                    best = r;
                }
            }
    if (best.ok) {
        // Ambiguity: another, clearly different pose scores almost as well.
        bool amb = false;
        double margin = std::max(p_.wideAmbiguityMinInliers, p_.wideAmbiguityFraction * best.inliers);
        for (const auto& h : all) {
            double dpos = std::hypot(h.second.x - best.pose.x, h.second.y - best.pose.y);
            double dth = std::abs(wrapAngle(h.second.theta - best.pose.theta));
            if ((dpos > 0.03 || dth > 0.026) && h.first >= bestScore - margin) amb = true;
        }
        best = solveWindow(frames, best.alignment, local, tRobotCam, wantOverlay);
        best.ambiguous = amb;
    }
    return best;
}

EdgeResult EdgeTracker::solve(const cv::Mat& gray, int64_t tNs, const SE2& prior, const Mat3& priorCov,
                              const SE3& tRobotCam, bool wantOverlay) {
    EdgeFrame f = prepare(gray, SE2(), tNs);  // identity odometry: the alignment is the pose
    return solveWindow({&f}, prior, priorCov, tRobotCam, wantOverlay);
}

EdgeResult EdgeTracker::solveWide(const cv::Mat& gray, int64_t tNs, const SE2& prior, const Mat3& priorCov,
                                  const SE3& tRobotCam, bool wantOverlay) {
    EdgeFrame f = prepare(gray, SE2(), tNs);
    return solveWindowWide({&f}, prior, priorCov, tRobotCam, wantOverlay);
}

double EdgeTracker::partSupport(const EdgeFrame& f, const SE2& pose, const SE3& tRobotCam, int part, int state,
                                int& samples) {
    std::vector<Sample> smp;
    makeSamples(pose, tRobotCam, smp, part, state);
    samples = (int)smp.size();
    if (smp.empty()) return 0;
    int hits = 0;
    for (Sample& s : smp) {
        search(s, p_.edgeSearchMinPx, f);
        for (double c : s.candidates)
            if (std::abs(c) <= 2.0) {
                ++hits;
                break;
            }
    }
    return (double)hits / samples;
}

std::vector<SE2> EdgeTracker::hypotheses(const SE2& a0, const Mat3& covA, const SE2& o, Mat3& local) const {
    double sxy = std::sqrt(std::max(covA(0, 0), covA(1, 1)));
    double sth = std::sqrt(covA(2, 2));
    double stepXY = std::max(0.03, 0.75 * sxy), stepTh = std::max(0.01, 0.75 * sth);
    local = Mat3::Zero();
    local(0, 0) = local(1, 1) = stepXY * stepXY;
    local(2, 2) = stepTh * stepTh;
    SE2 p0 = a0 * o;
    std::vector<SE2> out;
    out.push_back(a0);  // the current alignment first
    for (int k = -2; k <= 2; ++k)
        for (int i = -1; i <= 1; ++i)
            for (int j = -1; j <= 1; ++j) {
                if (i == 0 && j == 0 && k == 0) continue;
                SE2 p(p0.x + i * stepXY * 1.5, p0.y + j * stepXY * 1.5, p0.theta + k * stepTh);
                out.push_back(p * o.inverse());
            }
    return out;
}

}  // namespace beacon
