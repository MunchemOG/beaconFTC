#include "init_mono.h"

#include <Eigen/SVD>
#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>

namespace beacon {

bool triangulate(const Vec2& n1, const Vec2& n2, const SE3& T1, const SE3& T2, Vec3& X) {
    Eigen::Matrix<double, 3, 4> P1 = T1.matrix().topRows<3>(), P2 = T2.matrix().topRows<3>();
    Eigen::Matrix4d A;
    A.row(0) = n1.x() * P1.row(2) - P1.row(0);
    A.row(1) = n1.y() * P1.row(2) - P1.row(1);
    A.row(2) = n2.x() * P2.row(2) - P2.row(0);
    A.row(3) = n2.y() * P2.row(2) - P2.row(1);
    Eigen::JacobiSVD<Eigen::Matrix4d> svd(A, Eigen::ComputeFullV);
    Eigen::Vector4d h = svd.matrixV().col(3);
    if (std::abs(h(3)) < 1e-12) return false;
    X = h.head<3>() / h(3);
    return X.allFinite();
}

static TwoViewResult checkAndTriangulate(const Features& f1, const Features& f2, const std::vector<int>& m12,
                                         const std::vector<char>& use, const SE3& t21, const Pinhole& cam,
                                         double minParallaxDeg) {
    TwoViewResult r;
    r.t21 = t21;
    r.points.assign(f1.size(), Vec3::Zero());
    r.good.assign(f1.size(), 0);
    SE3 I = SE3::Identity();
    Vec3 c2 = -(t21.linear().transpose() * t21.translation());
    std::vector<double> parallax;
    for (size_t i = 0; i < f1.size(); ++i) {
        if (m12[i] < 0 || !use[i]) continue;
        const cv::Point2f& a = f1.un[i];
        const cv::Point2f& b = f2.un[m12[i]];
        Vec2 n1((a.x - cam.cx) / cam.fx, (a.y - cam.cy) / cam.fy);
        Vec2 n2((b.x - cam.cx) / cam.fx, (b.y - cam.cy) / cam.fy);
        Vec3 X;
        if (!triangulate(n1, n2, I, t21, X)) continue;
        Vec3 X2 = t21 * X;
        if (X.z() <= 0 || X2.z() <= 0) continue;
        double cosPar = X.normalized().dot((X - c2).normalized());
        // Reprojection error in both views (4 px^2 at level 0, as a chi-square with sigma 1).
        double e1 = std::pow(cam.fx * X.x() / X.z() + cam.cx - a.x, 2) + std::pow(cam.fy * X.y() / X.z() + cam.cy - a.y, 2);
        double e2 = std::pow(cam.fx * X2.x() / X2.z() + cam.cx - b.x, 2) + std::pow(cam.fy * X2.y() / X2.z() + cam.cy - b.y, 2);
        float s1 = f1.scaleFactors[f1.kps[i].octave], s2 = f2.scaleFactors[f2.kps[m12[i]].octave];
        if (e1 > 4.0 * s1 * s1 || e2 > 4.0 * s2 * s2) continue;
        parallax.push_back(std::acos(std::min(1.0, cosPar)) * 180 / M_PI);
        if (cosPar > 0.99999) continue;  // under ~0.26 deg: depth is unreliable
        r.points[i] = X;
        r.good[i] = 1;
        r.nGood++;
    }
    if (!parallax.empty()) {
        std::sort(parallax.begin(), parallax.end());
        r.medianParallaxDeg = parallax[parallax.size() / 2];
    }
    return r;
}

TwoViewResult initializeMonocular(const Features& f1, const Features& f2, const std::vector<int>& m12,
                                  const Pinhole& cam, double minParallaxDeg, int minPoints) {
    TwoViewResult res;
    std::vector<cv::Point2f> p1, p2;
    std::vector<int> idx;
    for (size_t i = 0; i < m12.size(); ++i) {
        if (m12[i] < 0) continue;
        p1.push_back(f1.un[i]);
        p2.push_back(f2.un[m12[i]]);
        idx.push_back((int)i);
    }
    if ((int)p1.size() < minPoints) {
        res.why = "too few matches";
        return res;
    }
    cv::Mat K = (cv::Mat_<double>(3, 3) << cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1);
    cv::Mat maskE, maskH;
    cv::Mat E = cv::findEssentialMat(p1, p2, K, cv::RANSAC, 0.999, 1.0, maskE);
    if (E.empty() || E.rows != 3) {
        res.why = "no essential matrix";
        return res;
    }
    cv::Mat H = cv::findHomography(p1, p2, cv::RANSAC, 2.0, maskH);
    int inE = cv::countNonZero(maskE), inH = H.empty() ? 0 : cv::countNonZero(maskH);
    cv::Mat R, t, maskP = maskE.clone();
    cv::recoverPose(E, p1, p2, K, R, t, maskP);
    SE3 t21 = SE3::Identity();
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) t21.linear()(r, c) = R.at<double>(r, c);
        t21.translation()(r) = t.at<double>(r);
    }
    std::vector<char> use(f1.size(), 0);
    for (size_t k = 0; k < idx.size(); ++k)
        if (maskP.at<uchar>((int)k)) use[idx[k]] = 1;
    res = checkAndTriangulate(f1, f2, m12, use, t21, cam, minParallaxDeg);
    // Degenerate (mostly rotation or a plane seen with little parallax): wait for more motion.
    if (inH > 0.9 * inE && res.medianParallaxDeg < 2 * minParallaxDeg) {
        res.ok = false;
        res.why = "homography-like, low parallax";
        return res;
    }
    if (res.nGood < minPoints || res.medianParallaxDeg < minParallaxDeg || res.nGood < 0.6 * inE) {
        res.ok = false;
        res.why = "not enough parallax or points";
        return res;
    }
    res.ok = true;
    return res;
}

TwoViewResult triangulateKnown(const Features& f1, const Features& f2, const std::vector<int>& m12, const SE3& t21,
                               const Pinhole& cam, double minParallaxDeg) {
    std::vector<char> use(f1.size(), 1);
    TwoViewResult r = checkAndTriangulate(f1, f2, m12, use, t21, cam, minParallaxDeg);
    r.ok = r.nGood > 0;
    return r;
}

}  // namespace beacon
