// Bench-only trajectory helpers: Umeyama alignment and ATE. These are for our own preliminary
// numbers; the plan's accuracy gates use tools/eval/eval_run.py (Field engineer).
#pragma once

#include <vector>

#include "geometry.h"

namespace beacon {

struct Alignment3 {
    double scale = 1;
    Mat3 R = Mat3::Identity();
    Vec3 t = Vec3::Zero();
    Vec3 apply(const Vec3& p) const { return scale * (R * p) + t; }
};

/** Least-squares similarity (or rigid if !withScale) mapping src onto dst (Umeyama 1991). */
Alignment3 umeyama(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, bool withScale);

/** RMS of |align(src_i) - dst_i|. */
double rmsError(const std::vector<Vec3>& src, const std::vector<Vec3>& dst, const Alignment3& a);

}  // namespace beacon
