// Tuning parameters, parsed from the optional params_json of beacon_create(). Every key is
// optional; unknown keys are an error so a typo never silently falls back to a default.
// The README lists every key with its default and unit.
#pragma once

#include <string>

namespace beacon {

enum class Mode { Field, Explore };

struct Params {
    Mode mode = Mode::Field;

    // Bench only: process each frame inside beacon_push_frame on the caller's thread, so a
    // replay is deterministic and never drops frames. Never set on the robot.
    bool synchronous = false;

    // Status: LOST (vision degraded) when no fix was accepted for this long.
    double lostAfterS = 1.0;

    // Odometry prediction.
    double odomMaxExtrapolateS = 0.10;  // beyond the newest sample, at most this far
    double odomHistoryS = 4.0;          // samples older than this are dropped
    double startSigmaXY = 0.15;         // m, trust in the typed start pose
    double startSigmaHeading = 0.10;    // rad
    double driftPerMetre = 0.01;        // alignment random walk, m per m travelled
    double driftHeadingPerRad = 0.005;  // rad per rad turned
    double driftHeadingPerS = 0.0005;   // rad per s (IMU drift)

    // Bench only, no odometry pushed at all: constant-velocity prediction from past fixes.
    double noOdomSigmaXY = 0.10;
    double noOdomSigmaHeading = 0.10;

    // ORB front end.
    int orbFeatures = 1000;
    double orbScaleFactor = 1.2;
    int orbLevels = 8;
    int orbFastThreshold = 20;
    int orbMinFastThreshold = 7;

    // Field mode: model-edge tracking (C2).
    double edgeSampleStepPx = 8.0;
    double edgeSearchMaxPx = 60.0;
    double edgeSearchMinPx = 6.0;
    double edgeMinGradient = 25.0;   // Sobel magnitude along the normal (8-bit image units)
    double edgeHuberPx = 2.0;
    double edgeSigmaPx = 1.5;        // per-sample pixel noise used for the covariance
    double edgeCorrelationInflation = 3.0;  // samples along one edge are correlated
    double edgeMaxRangeM = 5.0;

    // Field mode: gate for publishing a fix (C4).
    double gateMaxOmegaRadS = 1.5;
    double gateMaxSpeedMps = 2.0;
    int gateMinInliers = 25;
    double gateMinInlierRatio = 0.5;
    double gateMaxSigmaXY = 0.03;
    double gateMaxSigmaHeading = 0.03;
    double gateMaxChi2 = 16.0;       // Mahalanobis^2 of fix vs prediction (3 dof)

    // Field mode: surface-constrained ORB map (C3).
    bool usePoints = true;
    double pointMaxRangeM = 2.5;
    double pointSurfaceTolM = 0.02;

    // Explore mode (C6).
    std::string vocabularyPath;      // empty = loop closing off
    int exploreInitMinMatches = 100;
    double exploreInitMinParallaxDeg = 1.0;
    bool exploreLoopClosing = true;
    int exploreLocalWindow = 10;     // covisible keyframes in local BA
    bool exploreGlobalBA = true;     // full BA after each loop closure (bench)
    int exploreMaxFramesBetweenKf = 20;
    int exploreMinFramesBetweenKf = 3;

    /** Parses json (may be null/empty). Returns false and fills err on a bad document. */
    static bool parse(const char* json, Params& out, std::string& err);
};

}  // namespace beacon
