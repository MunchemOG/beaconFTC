#include "eval.h"

#include <Eigen/SVD>
#include <cmath>

namespace beacon {

Alignment3 umeyama(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, bool withScale) {
    Alignment3 a;
    size_t n = std::min(src.size(), dst.size());
    if (n < 3) return a;
    Vec3 ms = Vec3::Zero(), md = Vec3::Zero();
    for (size_t i = 0; i < n; ++i) {
        ms += src[i];
        md += dst[i];
    }
    ms /= (double)n;
    md /= (double)n;
    Mat3 cov = Mat3::Zero();
    double varS = 0;
    for (size_t i = 0; i < n; ++i) {
        Vec3 s = src[i] - ms, d = dst[i] - md;
        cov += d * s.transpose();
        varS += s.squaredNorm();
    }
    cov /= (double)n;
    varS /= (double)n;
    Eigen::JacobiSVD<Mat3> svd(cov, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Mat3 S = Mat3::Identity();
    if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0) S(2, 2) = -1;
    a.R = svd.matrixU() * S * svd.matrixV().transpose();
    a.scale = withScale && varS > 0 ? (svd.singularValues().asDiagonal() * S).trace() / varS : 1.0;
    a.t = md - a.scale * a.R * ms;
    return a;
}

double rmsError(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, const Alignment3& a) {
    size_t n = std::min(src.size(), dst.size());
    if (n == 0) return NAN;
    double ss = 0;
    for (size_t i = 0; i < n; ++i) ss += (a.apply(src[i]) - dst[i]).squaredNorm();
    return std::sqrt(ss / (double)n);
}

}  // namespace beacon
