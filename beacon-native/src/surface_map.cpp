#include "surface_map.h"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <numeric>
#include <Eigen/SVD>
#include <opencv2/imgproc.hpp>

namespace beacon {

namespace {

// ORB settings for surface points: fewer features and levels than explore mode (Hub time),
// a lower FAST threshold (foam tiles are low contrast).
Params pointOrbParams(const Params& p) {
    Params q = p;
    q.orbFeatures = p.pointFeatures;
    q.orbLevels = p.pointLevels;
    q.orbFastThreshold = p.pointFastThreshold;
    return q;
}

}  // namespace

SurfaceMap::SurfaceMap(const FieldModel& field, const Camera& cam, const Params& p)
    : field_(field), cam_(cam), p_(p), orbParams_(pointOrbParams(p)), orb_(orbParams_, cam) {
    Vec3 lo = Vec3::Constant(1e9), hi = Vec3::Constant(-1e9);
    for (size_t i = 0; i < field_.hiveFaces.size(); ++i) {
        const Surface& s = field_.hiveFaces[i];
        Vec3 c = Vec3::Zero();
        for (const Vec3& v : s.vertices) c += v;
        c /= std::max<size_t>(1, s.vertices.size());
        double r = 0;
        for (const Vec3& v : s.vertices) {
            r = std::max(r, (v - c).norm());
            lo = lo.cwiseMin(v);
            hi = hi.cwiseMax(v);
        }
        faceSphere_.push_back({c, r + 1e-3});
    }
    if (!field_.hiveFaces.empty()) {
        hiveCentre_ = 0.5 * (lo + hi);
        hiveRadius_ = 0.5 * (hi - lo).norm() + 1e-3;
    }
}

namespace {

// Ray (o, unit d) against a sphere: may the ray come within r of c (in front of o)?
inline bool raySphere(const Vec3& o, const Vec3& d, const Vec3& c, double r) {
    Vec3 oc = c - o;
    double along = oc.dot(d);
    if (along < -r) return false;
    return (oc - along * d).squaredNorm() <= r * r;
}

}  // namespace

int SurfaceMap::surfaceHit(const Vec3& o, const Vec3& d, double& tOut, bool rangeCheck) const {
    double best = 1e18;
    int kind = -2;  // -1 floor, -3 wall (blocks), >= 0 hive face (any)
    if (d.z() < -1e-9) {
        double t = -o.z() / d.z();
        if (t > 0) {
            best = t;
            kind = -1;
        }
    }
    double tw = field_.wallHit(o, d);
    if (tw > 0 && tw < best) {
        best = tw;
        kind = -3;
    }
    if (hiveRadius_ > 0 && raySphere(o, d, hiveCentre_, hiveRadius_)) {
        for (size_t i = 0; i < field_.hiveFaces.size(); ++i) {
            const Surface& f = field_.hiveFaces[i];
            if (!f.convex) continue;  // keyhole rib outlines: no inside test
            if (f.movingPart >= 0) {
                // Tipping parts occlude in their estimated tilt, in both tilts while unknown.
                int st = f.movingPart < (int)partState_.size() ? partState_[f.movingPart] : -1;
                if (st >= 0 && st != f.state) continue;
            }
            if (!raySphere(o, d, faceSphere_[i].first, faceSphere_[i].second)) continue;
            double t = f.intersect(o, d);
            if (t > 0 && t < best) {
                best = t;
                kind = (int)i;
            }
        }
    }
    tOut = best;
    if (kind == -2 || kind == -3) return -2;
    if (rangeCheck && (best < p_.pointMinRangeM || best > p_.pointMaxRangeM)) return -2;
    if (kind == -1) {
        Vec3 q = o + best * d;
        return field_.insidePerimeter(Vec2(q.x(), q.y()), p_.pointWallMarginM) ? -1 : -2;
    }
    const Surface& f = field_.hiveFaces[kind];
    return (f.movingPart < 0 && f.part == "frame") ? kind : -2;
}

Vec3 SurfaceMap::rayField(const SE3& tFieldCam, double u, double v) const {
    Vec3 n((u - cam_.cx) / cam_.fx, (v - cam_.cy) / cam_.fy, 1.0);
    return tFieldCam.linear() * n.normalized();
}

void SurfaceMap::buildMask(const SE2& pose, const SE3& tRobotCam, cv::Mat& mask) const {
    const int b = std::max(2, p_.pointMaskBlockPx);
    const int nx = (cam_.width + b - 1) / b, ny = (cam_.height + b - 1) / b;
    SE3 t = cameraInField(pose, tRobotCam);
    Vec3 o = t.translation();
    cv::Mat blk(ny, nx, CV_8U, cv::Scalar(0));
    for (int j = 0; j < ny; ++j)
        for (int i = 0; i < nx; ++i) {
            double u = std::min(cam_.width - 1.0, (i + 0.5) * b), v = std::min(cam_.height - 1.0, (j + 0.5) * b);
            Vec2 n = cam_.undistortPixel(u, v);
            Vec3 d = t.linear() * Vec3(n.x(), n.y(), 1.0).normalized();
            double th;
            if (surfaceHit(o, d, th) >= -1) blk.at<uint8_t>(j, i) = 255;
        }
    cv::erode(blk, blk, cv::Mat::ones(3, 3, CV_8U), cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));
    cv::resize(blk, mask, cv::Size(nx * b, ny * b), 0, 0, cv::INTER_NEAREST);
    mask = mask(cv::Rect(0, 0, cam_.width, cam_.height)).clone();
}

Vec2 SurfaceMap::undistort(const Vec2& px) const {
    Vec2 n = cam_.undistortPixel(px.x(), px.y());
    return cam_.toPixelNoDistortion(n);
}

bool SurfaceMap::toRef(const SurfacePoint& sp, const Plane& pl, const SE3& tFieldCam, const Vec2& px, Vec2& refPx) const {
    Vec2 n = cam_.undistortPixel(px.x(), px.y());
    Vec3 d = tFieldCam.linear() * Vec3(n.x(), n.y(), 1.0);
    Vec3 o = tFieldCam.translation();
    double den = pl.n.dot(d);
    if (std::abs(den) < 1e-9) return false;
    double t = -pl.n.dot(o - pl.o) / den;
    if (t <= 0) return false;
    Vec3 pc = sp.refCam.inverse() * (o + t * d);
    if (pc.z() < 0.05) return false;
    refPx = cam_.project(pc);
    return true;
}

void SurfaceMap::setReference(SurfacePoint& sp, const cv::Mat& gray, const Vec2& px, const SE3& tFieldCam) const {
    const int h = SurfacePoint::kRefHalf, n = SurfacePoint::kRefSize;
    int x0 = (int)std::lround(px.x()) - h, y0 = (int)std::lround(px.y()) - h;
    x0 = std::min(std::max(0, x0), gray.cols - n);
    y0 = std::min(std::max(0, y0), gray.rows - n);
    for (int r = 0; r < n; ++r) {
        const uint8_t* row = gray.ptr<uint8_t>(y0 + r) + x0;
        std::copy(row, row + n, sp.ref.begin() + r * n);
    }
    // Half resolution (2 x 2 means) over twice the extent, for views from farther away.
    int x1 = (int)std::lround(px.x()) - 2 * h - 1, y1 = (int)std::lround(px.y()) - 2 * h - 1;
    x1 = std::min(std::max(0, x1), gray.cols - 2 * n);
    y1 = std::min(std::max(0, y1), gray.rows - 2 * n);
    for (int r = 0; r < n; ++r) {
        const uint8_t* a = gray.ptr<uint8_t>(y1 + 2 * r) + x1;
        const uint8_t* b = gray.ptr<uint8_t>(y1 + 2 * r + 1) + x1;
        for (int c = 0; c < n; ++c) sp.ref1[r * n + c] = (uint8_t)((a[2 * c] + a[2 * c + 1] + b[2 * c] + b[2 * c + 1] + 2) / 4);
    }
    // Anchor the patch to the point: where the point projects in this view (at this pose) is
    // where its texture was matched (px), so a pose error at this moment does not move the
    // point's identity on the surface.
    Vec3 pc = tFieldCam.inverse() * sp.pos;
    Vec2 proj = pc.z() > 0.05 ? cam_.project(pc) : px;
    sp.refOrigin = Vec2(x0, y0) + (proj - px);
    sp.refOrigin1 = Vec2(x1, y1) + (proj - px);
    sp.refCam = tFieldCam;
}

void SurfaceMap::match(const cv::Mat& gray, const SE2& pred, const Mat3& predCov, const SE3& tRobotCam, int64_t tNs,
                       PointMatches& out) {
    out.mature.clear();
    out.candidates.clear();
    out.visibleMature = out.visibleCandidates = 0;
    if (pts_.empty() || gray.empty()) return;
    const SE3 tFieldCam = cameraInField(pred, tRobotCam);
    const SE3 tCamField = tFieldCam.inverse();
    const Vec3 camO = tFieldCam.translation();
    const int th = std::max(2, p_.pointPatchHalf), tw = 2 * th + 1;
    const double eps = 1e-4;
    cv::Mat tpl(tw, tw, CV_32F), roi, res, roiF;
    static int dbg[8] = {0};  // bench diagnostics: warp fail, outside ref, flat, roi, low ncc, ok (mature only)
    const bool debug = std::getenv("BEACON_POINTS_DEBUG") != nullptr;
    const auto start = std::chrono::steady_clock::now();
    // Visible points (in the image, facing the camera, not hidden by the hive), bucketed on a
    // coarse image grid so the per-frame cap keeps them spread out: mature points first.
    const int gcols = 8, grows = 6;
    std::vector<std::vector<int>> cellM(gcols * grows), cellC(gcols * grows);
    for (size_t i = 0; i < pts_.size(); ++i) {
        SurfacePoint& sp = pts_[i];
        if (sp.bad) continue;
        Vec3 pc = tCamField * sp.pos;
        if (pc.z() < 0.1 || pc.norm() > 1.15 * p_.pointMaxRangeM) continue;
        Vec2 px = cam_.project(pc);
        if (!cam_.inImage(px, th + 3)) continue;
        if (sp.surface >= 0) {
            // Hive face points are seen from the side they were created on only.
            const Surface& s = field_.hiveFaces[sp.surface];
            if ((s.normal.dot(camO) + s.d > 0) != (sp.side > 0)) continue;
        }
        Vec3 d = sp.pos - camO;
        double dist = d.norm(), th2;
        int hit = surfaceHit(camO, d / dist, th2, false);
        if (hit != sp.surface || std::abs(th2 - dist) > 0.03) continue;
        int c = std::min(gcols - 1, (int)(px.x() * gcols / cam_.width)) + gcols * std::min(grows - 1, (int)(px.y() * grows / cam_.height));
        (sp.mature ? cellM : cellC)[c].push_back((int)i);
        (sp.mature ? out.visibleMature : out.visibleCandidates)++;
    }
    std::vector<int> order;
    for (auto* cells : {&cellM, &cellC})
        for (size_t r = 0; (int)order.size() < p_.pointMaxMatchPerFrame; ++r) {
            bool any = false;
            for (auto& c : *cells)
                if (r < c.size() && (int)order.size() < p_.pointMaxMatchPerFrame) {
                    order.push_back(c[r]);
                    any = true;
                }
            if (!any) break;
        }
    for (int i : order) {
        // Robot: wall-clock budget (replays use the count cap only, so they stay deterministic).
        if (!p_.synchronous &&
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count() > p_.pointMatchBudgetMs)
            break;
        SurfacePoint& sp = pts_[i];
        Vec3 pc = tCamField * sp.pos;
        Vec2 px = cam_.project(pc);
        // Search radius: 3 sigma of the prediction, propagated numerically.
        Eigen::Matrix<double, 2, 3> j;
        for (int k = 0; k < 3; ++k) {
            SE2 a = pred, b = pred;
            double* fa = k == 0 ? &a.x : k == 1 ? &a.y : &a.theta;
            double* fb = k == 0 ? &b.x : k == 1 ? &b.y : &b.theta;
            *fa += eps;
            *fb -= eps;
            Vec3 qa = cameraInField(a, tRobotCam).inverse() * sp.pos, qb = cameraInField(b, tRobotCam).inverse() * sp.pos;
            j.col(k) = (Vec2(qa.x() / qa.z(), qa.y() / qa.z()) - Vec2(qb.x() / qb.z(), qb.y() / qb.z())) * cam_.fx / (2 * eps);
        }
        Eigen::Matrix2d s2 = j * predCov * j.transpose();
        int r = (int)std::ceil(std::min(p_.pointSearchMaxPx, std::max(p_.pointSearchMinPx, 3.0 * std::sqrt(std::max(s2(0, 0), s2(1, 1))))));
        sp.visible++;
        // Template: the reference patch warped into this view through the point's plane
        // (affine about the predicted pixel).
        Plane pl = plane(sp.surface);
        Vec2 c0, c1, c2;
        if (!toRef(sp, pl, tFieldCam, px, c0) || !toRef(sp, pl, tFieldCam, px + Vec2(1, 0), c1) ||
            !toRef(sp, pl, tFieldCam, px + Vec2(0, 1), c2)) {
            if (sp.mature) dbg[0]++;
            continue;
        }
        Eigen::Matrix2d a;
        a.col(0) = c1 - c0;
        a.col(1) = c2 - c0;
        bool inside = true;
        double sum = 0, sum2 = 0;
        // Level 0 unless the template would reach outside it (view from much farther away).
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(a);
        const bool half = svd.singularValues()(0) * th * 1.4143 > SurfacePoint::kRefHalf - 1.5;
        const uint8_t* refData = half ? sp.ref1.data() : sp.ref.data();
        const Vec2 qc = half ? Vec2((c0 - sp.refOrigin1 - Vec2(0.5, 0.5)) / 2.0) : Vec2(c0 - sp.refOrigin);
        const Eigen::Matrix2d qa = half ? Eigen::Matrix2d(a / 2.0) : a;
        for (int v = -th; v <= th && inside; ++v)
            for (int u = -th; u <= th; ++u) {
                Vec2 q = qc + qa * Vec2(u, v);
                int qx = (int)std::floor(q.x()), qy = (int)std::floor(q.y());
                if (qx < 0 || qy < 0 || qx + 1 >= SurfacePoint::kRefSize || qy + 1 >= SurfacePoint::kRefSize) {
                    inside = false;
                    break;
                }
                double ax = q.x() - qx, ay = q.y() - qy;
                const uint8_t* r0 = refData + qy * SurfacePoint::kRefSize + qx;
                const uint8_t* r1 = r0 + SurfacePoint::kRefSize;
                float val = (float)((1 - ay) * ((1 - ax) * r0[0] + ax * r0[1]) + ay * ((1 - ax) * r1[0] + ax * r1[1]));
                tpl.at<float>(v + th, u + th) = val;
                sum += val;
                sum2 += val * val;
            }
        if (!inside) {  // view too different from the reference (refreshed on keyframes)
            if (sp.mature) dbg[1]++;
            continue;
        }
        double var = sum2 / (tw * tw) - std::pow(sum / (tw * tw), 2);
        if (var < p_.pointMinPatchStd * p_.pointMinPatchStd) {
            if (sp.mature) dbg[2]++;
            continue;
        }
        // Correlate around the prediction.
        int cx = (int)std::lround(px.x()), cy = (int)std::lround(px.y());
        int x0 = std::max(0, cx - r - th), y0 = std::max(0, cy - r - th);
        int x1 = std::min(gray.cols, cx + r + th + 1), y1 = std::min(gray.rows, cy + r + th + 1);
        if (x1 - x0 < tw + 2 || y1 - y0 < tw + 2) {
            if (sp.mature) dbg[3]++;
            continue;
        }
        gray(cv::Range(y0, y1), cv::Range(x0, x1)).convertTo(roiF, CV_32F);
        cv::matchTemplate(roiF, tpl, res, cv::TM_CCOEFF_NORMED);
        double best;
        cv::Point bl;
        cv::minMaxLoc(res, nullptr, &best, nullptr, &bl);
        if (const char* md = std::getenv("BEACON_MATCH_DUMP")) {
            double tq = std::atof(md);
            if (sp.mature && tNs * 1e-9 >= tq && tNs * 1e-9 < tq + 0.04)
                std::fprintf(stderr, "pt %d range %.2f px %.0f,%.0f r %d ncc %.2f at %+d,%+d svd %.2f %.2f half %d age %.1f refdist %.2f\n", i,
                             pc.norm(), px.x(), px.y(), r, best, bl.x + x0 + th - cx, bl.y + y0 + th - cy,
                             svd.singularValues()(0), svd.singularValues()(1), (int)half, (tNs - sp.createdT) * 1e-9,
                             (camO - sp.refCam.translation()).norm());
        }
        if (best < p_.pointMinNcc) {
            if (sp.mature) dbg[4]++;
            continue;
        }
        if (sp.mature) dbg[5]++;
        // A second, distinct peak nearly as good (repeated seam zig-zags): keep both and let
        // the solve pick the one nearest the pose.
        float second = -1;
        cv::Point sl;
        for (int yy = 1; yy + 1 < res.rows; ++yy)
            for (int xx = 1; xx + 1 < res.cols; ++xx) {
                float v = res.at<float>(yy, xx);
                if (v <= second || std::abs(xx - bl.x) + std::abs(yy - bl.y) <= 3) continue;
                if (v < res.at<float>(yy, xx - 1) || v < res.at<float>(yy, xx + 1) || v < res.at<float>(yy - 1, xx) ||
                    v < res.at<float>(yy + 1, xx))
                    continue;
                second = v;
                sl = cv::Point(xx, yy);
            }
        auto subpix = [&](const cv::Point& p) {
            double dx = 0, dy = 0;
            if (p.x > 0 && p.x + 1 < res.cols) {
                double l = res.at<float>(p.y, p.x - 1), c = res.at<float>(p.y, p.x), rr = res.at<float>(p.y, p.x + 1);
                double den = l - 2 * c + rr;
                if (den < 0) dx = std::max(-0.5, std::min(0.5, 0.5 * (l - rr) / den));
            }
            if (p.y > 0 && p.y + 1 < res.rows) {
                double u = res.at<float>(p.y - 1, p.x), c = res.at<float>(p.y, p.x), d = res.at<float>(p.y + 1, p.x);
                double den = u - 2 * c + d;
                if (den < 0) dy = std::max(-0.5, std::min(0.5, 0.5 * (u - d) / den));
            }
            return Vec2(x0 + p.x + th + dx, y0 + p.y + th + dy);  // distorted pixel
        };
        PointObs ob;
        ob.world = sp.pos;
        ob.point = i;
        ob.scale = 1;
        ob.cand[ob.nCand++] = undistort(subpix(bl));
        bool ambiguous = second >= best - p_.pointNccAmbiguity && second >= p_.pointMinNcc;
        if (ambiguous) ob.cand[ob.nCand++] = undistort(subpix(sl));
        // The view drifts away from the reference (the robot drives toward or past the point):
        // take a new reference here. Safe at any pose: the patch is re-anchored where the point
        // projects, so only the warp shape, not the point's place, depends on the pose.
        double smax = svd.singularValues()(0), smin = svd.singularValues()(1);
        if (!ambiguous && best >= p_.pointRefreshNcc && (smax > p_.pointRefreshScale || smin < 1.0 / p_.pointRefreshScale))
            setReference(sp, gray, subpix(bl), tFieldCam);
        sp.found++;
        sp.lastSeenT = tNs;
        (sp.mature ? out.mature : out.candidates).push_back(ob);
    }
    if (debug && (tNs / 1000000000LL) % 10 == 0 && (tNs % 1000000000LL) < 40000000LL)
        std::fprintf(stderr, "match t=%.1f mature: warp %d outside %d flat %d roi %d lowncc %d ok %d\n", tNs * 1e-9, dbg[0], dbg[1],
                     dbg[2], dbg[3], dbg[4], dbg[5]);
}


bool SurfaceMap::keyframeDue(const SE2& odom, int64_t tNs) const {
    if (!haveKf_) return true;
    if ((tNs - lastKfT_) * 1e-9 < p_.pointKfMinS) return false;
    return std::hypot(odom.x - lastKfOdom_.x, odom.y - lastKfOdom_.y) >= p_.pointKfMinMoveM ||
           std::abs(wrapAngle(odom.theta - lastKfOdom_.theta)) >= p_.pointKfMinTurnRad;
}

SurfaceMap::Plane SurfaceMap::plane(int surface) const {
    Plane pl;
    if (surface < 0) {
        pl.o = Vec3::Zero();
        pl.e1 = Vec3::UnitX();
        pl.e2 = Vec3::UnitY();
        pl.n = Vec3::UnitZ();
        return pl;
    }
    const Surface& s = field_.hiveFaces[surface];
    pl.n = s.normal;
    pl.o = s.vertices[0] - s.normal * (s.normal.dot(s.vertices[0]) + s.d);
    Vec3 e = s.vertices[1] - s.vertices[0];
    e -= pl.n * pl.n.dot(e);
    pl.e1 = e.normalized();
    pl.e2 = pl.n.cross(pl.e1);
    return pl;
}

bool SurfaceMap::fit(SurfacePoint& sp, const SE2& align, const SE3& tRobotCam) const {
    Plane pl = plane(sp.surface);
    Vec3 x = sp.pos - pl.n * pl.n.dot(sp.pos - pl.o);
    double a = pl.e1.dot(x - pl.o), b = pl.e2.dot(x - pl.o);
    std::vector<SE3> tcf(sp.obs.size());
    for (size_t i = 0; i < sp.obs.size(); ++i) tcf[i] = cameraInField(align * sp.obs[i].odom, tRobotCam).inverse();
    double ss = 0;
    for (int it = 0; it < 6; ++it) {
        Vec3 X = pl.o + a * pl.e1 + b * pl.e2;
        Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
        Eigen::Vector2d g = Eigen::Vector2d::Zero();
        ss = 0;
        for (size_t i = 0; i < sp.obs.size(); ++i) {
            Vec3 pc = tcf[i] * X;
            if (pc.z() < 0.05) return false;
            const Vec3& bv = sp.obs[i].bearing;
            double k = cam_.fx;
            Vec2 r(k * (pc.x() / pc.z() - bv.x() / bv.z()), k * (pc.y() / pc.z() - bv.y() / bv.z()));
            Eigen::Matrix<double, 2, 3> dn;
            dn << 1 / pc.z(), 0, -pc.x() / (pc.z() * pc.z()), 0, 1 / pc.z(), -pc.y() / (pc.z() * pc.z());
            Eigen::Matrix<double, 3, 2> dx;
            dx.col(0) = pl.e1;
            dx.col(1) = pl.e2;
            Eigen::Matrix<double, 2, 2> jj = k * dn * tcf[i].linear() * dx;
            h += jj.transpose() * jj;
            g += jj.transpose() * r;
            ss += r.squaredNorm();
        }
        Eigen::Vector2d step = -(h + Eigen::Matrix2d::Identity() * 1e-9).ldlt().solve(g);
        if (!step.allFinite()) return false;
        a += step(0);
        b += step(1);
        if (step.norm() < 1e-6) break;
    }
    sp.consistPx = std::sqrt(ss / std::max<size_t>(1, sp.obs.size()));
    sp.pos = pl.o + a * pl.e1 + b * pl.e2;
    return sp.pos.allFinite();
}

void SurfaceMap::update(int64_t tNs, const cv::Mat& gray, const PointMatches& m, const std::vector<char>& matureInlier,
                        bool fixAccepted, bool fixAgreesWithPoints, const SE2& align, const SE2& odom,
                        double fixSigmaXY, const SE3& tRobotCam) {
    const bool kf = keyframeDue(odom, tNs);
    const SE2 pose = align * odom;
    const SE3 tFieldCam = cameraInField(pose, tRobotCam);
    const SE3 tCamField = tFieldCam.inverse();
    const Vec3 camO = tFieldCam.translation();
    int nGate = 0, nObs = 0, nMature = 0, nIncons = 0, nOff = 0, nExpire = 0, nRatio = 0, nStrike = 0, nWait = 0, nRefresh = 0;
    // Nearest matched position (undistorted px) to the point's projection at this pose, or -1.
    auto nearest = [&](const SurfacePoint& sp, const PointObs& ob, double& dist) {
        Vec3 pc = tCamField * sp.pos;
        dist = 1e9;
        if (pc.z() < 0.05) return -1;
        Vec2 px = cam_.toPixelNoDistortion(Vec2(pc.x() / pc.z(), pc.y() / pc.z()));
        int bestC = -1;
        for (int c = 0; c < ob.nCand; ++c) {
            double d = (px - ob.cand[c]).norm();
            if (d < dist) {
                dist = d;
                bestC = c;
            }
        }
        return bestC;
    };
    auto distorted = [&](const Vec2& un) { return cam_.project(Vec3((un.x() - cam_.cx) / cam_.fx, (un.y() - cam_.cy) / cam_.fy, 1.0)); };
    // Refresh a point's reference patch once the view has changed a lot (range or place).
    auto maybeRefresh = [&](SurfacePoint& sp, const Vec2& un) {
        Vec3 cr = sp.refCam.translation();
        double rNow = (sp.pos - camO).norm(), rRef = (sp.pos - cr).norm();
        if ((camO - cr).norm() > p_.pointRefRefreshM || std::abs(std::log(rNow / std::max(1e-3, rRef))) > 0.25) {
            setReference(sp, gray, distorted(un), tFieldCam);
            nRefresh++;
        }
    };
    // Mature points: frozen landmarks. A point that keeps disagreeing with fixes that most
    // points agree with is wrong (moved clutter, a bad early fit) and is dropped.
    for (size_t i = 0; i < m.mature.size(); ++i) {
        const PointObs& ob = m.mature[i];
        SurfacePoint& sp = pts_[ob.point];
        if (!fixAccepted || !fixAgreesWithPoints) continue;
        bool in = i < matureInlier.size() && matureInlier[i];
        sp.strikes = in ? std::max(0, sp.strikes - 1) : sp.strikes + 1;
        if (sp.strikes >= p_.pointMaxStrikes) {
            sp.bad = true;
            nStrike++;
            continue;
        }
        double dist;
        int c = nearest(sp, ob, dist);
        if (in && c >= 0 && kf) maybeRefresh(sp, ob.cand[c]);
    }
    // Candidates: on keyframes one more observation, then the surface test.
    for (const PointObs& ob : m.candidates) {
        SurfacePoint& sp = pts_[ob.point];
        double bestD;
        int bestC = nearest(sp, ob, bestD);
        if (bestC < 0 || bestD > p_.pointCandGatePx) {
            nGate++;
            continue;
        }
        if (!kf) continue;
        nObs++;
        const Vec2& un = ob.cand[bestC];
        Vec3 bearing((un.x() - cam_.cx) / cam_.fx, (un.y() - cam_.cy) / cam_.fy, 1.0);
        sp.obs.push_back({odom, bearing.normalized()});
        if ((int)sp.obs.size() > SurfacePoint::kMaxObs) sp.obs.erase(sp.obs.begin());
        if (!fit(sp, align, tRobotCam)) {
            sp.bad = true;
            continue;
        }
        // Baseline: spread of the camera centres (odometry world) that saw the point.
        double base = 0;
        for (const SurfacePoint::Obs& a : sp.obs) {
            Vec3 ca = cameraInField(a.odom, tRobotCam).translation();
            for (const SurfacePoint::Obs& b : sp.obs)
                base = std::max(base, (ca - cameraInField(b.odom, tRobotCam).translation()).norm());
        }
        bool onSurface = sp.surface < 0 ? field_.insidePerimeter(Vec2(sp.pos.x(), sp.pos.y()), p_.pointWallMarginM)
                                        : field_.hiveFaces[sp.surface].contains(sp.pos);
        if (!onSurface) {
            sp.bad = true;
            nOff++;
            continue;
        }
        if ((int)sp.obs.size() >= p_.pointMatureObs && base >= p_.pointMatureBaselineM) {
            if (sp.consistPx <= p_.pointConsistPx) {
                sp.mature = true;
                nMature++;
            } else if (sp.consistPx > p_.pointCullPx) {
                sp.bad = true;
                nIncons++;
            } else {
                nWait++;
            }
        } else if (sp.consistPx > p_.pointCullPx && (int)sp.obs.size() >= 2) {
            sp.bad = true;  // already inconsistent with little baseline: far off the surface
            nIncons++;
        } else {
            nWait++;
        }
        if (!sp.bad) maybeRefresh(sp, un);
    }
    for (SurfacePoint& sp : pts_) {
        if (sp.bad || sp.mature) continue;
        if ((tNs - sp.createdT) * 1e-9 > p_.pointCandidateMaxS) {
            sp.bad = true;
            nExpire++;
        } else if (sp.visible >= 8 && sp.found < p_.pointMinFoundRatio * sp.visible) {
            sp.bad = true;
            nRatio++;
        }
    }
    // New candidates: ORB keypoints (masked to field surfaces) whose ray meets an allowed
    // surface away from its border (rays a few pixels to each side meet the same surface),
    // not next to an existing point.
    int made = 0;
    if (fixAccepted && kf && fixSigmaXY <= p_.pointCreateMaxSigmaXY && (int)pts_.size() < 2 * p_.pointMaxPoints &&
        m.visibleMature + m.visibleCandidates < p_.pointTargetVisible && (tNs - lastCreateT_) * 1e-9 >= p_.pointCreateMinS) {
        lastCreateT_ = tNs;
        cv::Mat mask;
        buildMask(pose, tRobotCam, mask);
        Features f;
        orb_.extract(gray, f, mask);
        lastDetected_ = f.size();
        const int cell = 12, gc = (cam_.width + cell - 1) / cell, gr = (cam_.height + cell - 1) / cell;
        std::vector<char> occupied(gc * gr, 0);
        for (const SurfacePoint& sp : pts_) {
            if (sp.bad) continue;
            Vec3 pc = tCamField * sp.pos;
            if (pc.z() < 0.05) continue;
            Vec2 px = cam_.toPixelNoDistortion(Vec2(pc.x() / pc.z(), pc.y() / pc.z()));
            int cx = (int)std::floor(px.x() / cell), cy = (int)std::floor(px.y() / cell);
            if (cx >= 0 && cy >= 0 && cx < gc && cy < gr) occupied[cy * gc + cx] = 1;
        }
        std::vector<int> order(f.size());
        std::iota(order.begin(), order.end(), 0);
        std::sort(order.begin(), order.end(), [&](int a, int b) { return f.kps[a].response > f.kps[b].response; });
        const double mg = p_.pointEdgeMarginPx;
        for (int fi : order) {
            if (made >= p_.pointCreateMaxPerKf) break;
            double u = f.un[fi].x, v = f.un[fi].y;
            int cx = (int)std::floor(u / cell), cy = (int)std::floor(v / cell);
            if (cx < 0 || cy < 0 || cx >= gc || cy >= gr || occupied[cy * gc + cx]) continue;
            const Vec2 kp(f.kps[fi].pt.x, f.kps[fi].pt.y);
            if (kp.x() < SurfacePoint::kRefHalf || kp.y() < SurfacePoint::kRefHalf ||
                kp.x() >= cam_.width - SurfacePoint::kRefHalf - 1 || kp.y() >= cam_.height - SurfacePoint::kRefHalf - 1)
                continue;
            Vec3 d = rayField(tFieldCam, u, v);
            double t;
            int s = surfaceHit(camO, d, t);
            if (s < -1) continue;
            bool same = true;
            const double du[4] = {mg, -mg, 0, 0}, dv[4] = {0, 0, mg, -mg};
            for (int k = 0; k < 4 && same; ++k) {
                double t2;
                same = surfaceHit(camO, rayField(tFieldCam, u + du[k], v + dv[k]), t2) == s;
            }
            if (!same) continue;
            SurfacePoint sp;
            sp.pos = camO + t * d;
            sp.surface = s;
            if (s >= 0) sp.side = field_.hiveFaces[s].normal.dot(camO) + field_.hiveFaces[s].d > 0 ? 1 : -1;
            setReference(sp, gray, kp, tFieldCam);
            sp.createdT = sp.lastSeenT = tNs;
            Vec3 bearing((u - cam_.cx) / cam_.fx, (v - cam_.cy) / cam_.fy, 1.0);
            sp.obs.push_back({odom, bearing.normalized()});
            pts_.push_back(sp);
            occupied[cy * gc + cx] = 1;
            ++made;
        }
    }
    if (kf && std::getenv("BEACON_POINTS_DEBUG"))
        std::fprintf(stderr, "kf t=%.2f mature %zu cand %zu: gate %d obs %d -> mature %d wait %d incons %d off %d | expire %d ratio %d strike %d | refresh %d new %d (det %zu) sigma %.1f cm fix %d\n",
                     tNs * 1e-9, m.mature.size(), m.candidates.size(), nGate, nObs, nMature, nWait, nIncons, nOff, nExpire,
                     nRatio, nStrike, nRefresh, made, lastDetected_, fixSigmaXY * 100, fixAccepted);
    // Drop culled points; above the cap, forget the points seen longest ago (candidates first).
    pts_.erase(std::remove_if(pts_.begin(), pts_.end(), [](const SurfacePoint& s) { return s.bad; }), pts_.end());
    if ((int)pts_.size() > p_.pointMaxPoints) {
        std::stable_sort(pts_.begin(), pts_.end(), [](const SurfacePoint& a, const SurfacePoint& b) {
            if (a.mature != b.mature) return a.mature;
            return a.lastSeenT > b.lastSeenT;
        });
        pts_.resize(p_.pointMaxPoints);
    }
    if (kf) {
        haveKf_ = true;
        lastKfOdom_ = odom;
        lastKfT_ = tNs;
    }
}

void SurfaceMap::dropCandidates() {
    pts_.erase(std::remove_if(pts_.begin(), pts_.end(), [](const SurfacePoint& s) { return !s.mature; }), pts_.end());
    for (SurfacePoint& s : pts_) s.obs.clear();
    haveKf_ = false;
}

int SurfaceMap::matureCount() const {
    int n = 0;
    for (const SurfacePoint& s : pts_) n += s.mature && !s.bad;
    return n;
}

}  // namespace beacon

