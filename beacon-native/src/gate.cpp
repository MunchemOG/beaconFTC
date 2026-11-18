#include "gate.h"

#include <cmath>
#include <cstdio>

namespace beacon {

GateDecision gate(const GateInput& in, const Params& p) {
    GateDecision d;
    char buf[96];
    if (in.hasVelocity && std::abs(in.omega) > p.gateMaxOmegaRadS) {
        std::snprintf(buf, sizeof buf, "turning %.1f rad/s", std::abs(in.omega));
        d.reason = buf;
        return d;
    }
    if (in.hasVelocity && in.speed > p.gateMaxSpeedMps) {
        std::snprintf(buf, sizeof buf, "moving %.1f m/s", in.speed);
        d.reason = buf;
        return d;
    }
    const EdgeResult* r = in.result;
    if (r == nullptr || !r->ok) {
        d.reason = "no edges";
        return d;
    }
    if (r->inliers < p.gateMinInliers) {
        std::snprintf(buf, sizeof buf, "%d inliers", r->inliers);
        d.reason = buf;
        return d;
    }
    if (r->matched > 0 && (double)r->inliers / r->samples < p.gateMinInlierRatio) {
        std::snprintf(buf, sizeof buf, "inlier ratio %.2f", (double)r->inliers / r->samples);
        d.reason = buf;
        return d;
    }
    double sxy = std::sqrt(std::max(r->cov(0, 0), r->cov(1, 1)));
    double sth = std::sqrt(r->cov(2, 2));
    if (!std::isfinite(sxy) || sxy > p.gateMaxSigmaXY) {
        std::snprintf(buf, sizeof buf, "sigma %.1f cm", sxy * 100);
        d.reason = buf;
        return d;
    }
    if (!std::isfinite(sth) || sth > p.gateMaxSigmaHeading) {
        std::snprintf(buf, sizeof buf, "sigma %.1f deg", sth * 180 / M_PI);
        d.reason = buf;
        return d;
    }
    if (in.chi2 > p.gateMaxChi2) {
        std::snprintf(buf, sizeof buf, "jump chi2 %.0f", in.chi2);
        d.reason = buf;
        return d;
    }
    if (!r->pose.finite()) {
        d.reason = "non-finite pose";
        return d;
    }
    d.accept = true;
    return d;
}

}  // namespace beacon
