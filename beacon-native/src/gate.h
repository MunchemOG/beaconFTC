// Gate for field-mode fixes: a fix is published only when it is well constrained, enough of
// the image agreed, the robot was not moving too fast at capture time (blur, rolling shutter),
// and the fix is statistically consistent with the odometry prediction.
#pragma once

#include <string>

#include "edges.h"
#include "params.h"

namespace beacon {

struct GateInput {
    const EdgeResult* result = nullptr;
    SE2 predPose;
    Mat3 predCov = Mat3::Identity();
    double speed = 0, omega = 0;
    bool hasVelocity = false;
    double chi2 = 0;
};

struct GateDecision {
    bool accept = false;
    std::string reason;  // why not, for telemetry ("" when accepted)
};

GateDecision gate(const GateInput& in, const Params& p);

}  // namespace beacon
