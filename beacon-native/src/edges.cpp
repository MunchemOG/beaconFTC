#include "edges.h"

#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
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

void EdgeTracker::makeSamples(const SE2& pose, const SE3& tRobotCam, std::vector<Sample>& out) const {
    out.clear();
    SE3 tFieldCam = cameraInField(pose, tRobotCam);
    SE3 tCamField = tFieldCam.inverse();
    Vec3 camCentre = tFieldCam.translation();
    const double nearZ = 0.10;
    for (size_t ei = 0; ei < field_.edges.size(); ++ei) {
        const ModelEdge& e = field_.edges[ei];
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
        int n = std::min(400, (int)(lenPx / p_.edgeSampleStepPx));
        Vec3 wa = e.a + s0 * (e.b - e.a), wb = e.a + s1 * (e.b - e.a);
        Vec3 dir = wb - wa;
        for (int k = 0; k < n; ++k) {
            double s = (k + 0.5) / n;
            Vec3 pw = wa + s * dir;
            Vec3 pc = tCamField * pw;
            if (pc.z() < nearZ || pc.norm() > p_.edgeMaxRangeM) continue;
            Vec2 px = cam_.project(pc);
            if (!cam_.inImage(px, 4)) continue;
            // Hidden behind a hive face?
            Vec3 ray = pw - camCentre;
            double dist = ray.norm();
            bool hidden = false;
            for (const Surface& f : field_.hiveFaces) {
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
            out.push_back(smp);
        }
    }
}

void EdgeTracker::search(Sample& s, double range) const {
    s.candidates.clear();
    int r = (int)std::ceil(range);
    std::vector<float> g(2 * r + 1);
    for (int k = -r; k <= r; ++k) {
        double x = s.px.x() + k * s.normal.x(), y = s.px.y() + k * s.normal.y();
        g[k + r] = std::abs(bilinear(gx_, x, y) * (float)s.normal.x() + bilinear(gy_, x, y) * (float)s.normal.y());
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

EdgeResult EdgeTracker::solve(const cv::Mat& gray, int64_t, const SE2& prior, const Mat3& priorCov,
                              const SE3& tRobotCam, bool wantOverlay) {
    EdgeResult res;
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0);
    cv::Sobel(blurred, gx_, CV_32F, 1, 0, 3, 0.25);
    cv::Sobel(blurred, gy_, CV_32F, 0, 1, 3, 0.25);

    // Initial search range from the prediction's uncertainty (3 sigma, heading and a 1.5 m lever).
    double sxy = std::sqrt(std::max(priorCov(0, 0), priorCov(1, 1)));
    double sth = std::sqrt(priorCov(2, 2));
    double range = 3.0 * cam_.fx * (sth + sxy / 1.5);
    range = std::min(p_.edgeSearchMaxPx, std::max(p_.edgeSearchMinPx * 2, range));

    Mat3 priorInfo = (priorCov + Mat3::Identity() * 1e-8).inverse();
    SE2 pose = prior;
    std::vector<Sample> samples;
    Eigen::Matrix3d hData = Eigen::Matrix3d::Zero();
    std::vector<double> finalResid;
    const double eps[3] = {1e-4, 1e-4, 1e-4};

    for (int outer = 0; outer < 4; ++outer) {
        makeSamples(pose, tRobotCam, samples);
        if (samples.size() < 6) {
            res.samples = (int)samples.size();
            return res;
        }
        for (Sample& s : samples) search(s, range);
        std::vector<Vec2> anchors(samples.size());
        for (size_t i = 0; i < samples.size(); ++i) anchors[i] = samples[i].px;
        double cutoff = std::max(3.0 * p_.edgeHuberPx, range);
        for (int inner = 0; inner < 8; ++inner) {
            Eigen::Matrix3d h = Eigen::Matrix3d::Zero();
            Eigen::Vector3d g = Eigen::Vector3d::Zero();
            hData.setZero();
            finalResid.assign(samples.size(), NAN);
            for (size_t i = 0; i < samples.size(); ++i) {
                Sample& s = samples[i];
                if (s.candidates.empty()) continue;
                Vec2 px;
                if (!project(pose, tRobotCam, s.world, px)) continue;
                double along = s.normal.dot(px - anchors[i]);
                double best = 1e9;
                for (double c : s.candidates)
                    if (std::abs(along - c) < std::abs(best)) best = along - c;
                if (std::abs(best) > cutoff) continue;
                finalResid[i] = best;
                Eigen::RowVector3d j;
                for (int k = 0; k < 3; ++k) {
                    SE2 pp = pose, pm = pose;
                    double* fp = k == 0 ? &pp.x : k == 1 ? &pp.y : &pp.theta;
                    double* fm = k == 0 ? &pm.x : k == 1 ? &pm.y : &pm.theta;
                    *fp += eps[k];
                    *fm -= eps[k];
                    Vec2 a, b;
                    if (!project(pp, tRobotCam, s.world, a) || !project(pm, tRobotCam, s.world, b)) {
                        j.setZero();
                        break;
                    }
                    j(k) = s.normal.dot(a - b) / (2 * eps[k]);
                }
                double w = std::abs(best) <= p_.edgeHuberPx ? 1.0 : p_.edgeHuberPx / std::abs(best);
                h += w * j.transpose() * j;
                g += w * j.transpose() * best;
                hData += w * j.transpose() * j;
            }
            // Weak prior toward the prediction keeps unobservable directions in place.
            Eigen::Vector3d dp(pose.x - prior.x, pose.y - prior.y, wrapAngle(pose.theta - prior.theta));
            Eigen::Matrix3d hp = h + priorInfo * (p_.edgeSigmaPx * p_.edgeSigmaPx);
            Eigen::Vector3d gp = g + priorInfo * (p_.edgeSigmaPx * p_.edgeSigmaPx) * dp;
            Eigen::Vector3d step = -hp.ldlt().solve(gp);
            if (!step.allFinite()) break;
            pose.x += step(0);
            pose.y += step(1);
            pose.theta = wrapAngle(pose.theta + step(2));
            if (std::abs(step(0)) < 1e-5 && std::abs(step(1)) < 1e-5 && std::abs(step(2)) < 1e-5) break;
        }
        range = std::max(p_.edgeSearchMinPx, range * 0.4);
    }

    res.samples = (int)samples.size();
    double ss = 0;
    for (size_t i = 0; i < samples.size(); ++i) {
        if (!samples[i].candidates.empty()) res.matched++;
        if (std::isfinite(finalResid[i]) && std::abs(finalResid[i]) <= 2.0 * p_.edgeHuberPx) {
            res.inliers++;
            ss += finalResid[i] * finalResid[i];
        }
    }
    res.rmsPx = res.inliers ? std::sqrt(ss / res.inliers) : 0;
    Eigen::Matrix3d info = hData / (p_.edgeSigmaPx * p_.edgeSigmaPx * p_.edgeCorrelationInflation);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(info);
    if (es.eigenvalues().minCoeff() > 1e-6) {
        res.cov = info.inverse();
    } else {
        res.cov = Mat3::Identity() * 1e6;
    }
    res.pose = pose;
    res.ok = pose.finite();
    if (wantOverlay) {
        std::vector<Sample> fin;
        makeSamples(pose, tRobotCam, fin);
        for (size_t i = 1; i < fin.size(); ++i) {
            if (fin[i].edge != fin[i - 1].edge) continue;
            res.overlay.push_back({cv::Point2f((float)fin[i - 1].px.x(), (float)fin[i - 1].px.y()),
                                   cv::Point2f((float)fin[i].px.x(), (float)fin[i].px.y())});
        }
    }
    return res;
}

}  // namespace beacon
