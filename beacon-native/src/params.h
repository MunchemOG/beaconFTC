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
    double driftHeadingPerS = 0.0006;   // rad per s (IMU drift, about 2 deg/min)

    // Bench only, no odometry pushed at all: constant-velocity prediction from past fixes.
    // Online odometry scale estimate (per robot axis) from pairs of accepted fixes.
    bool odomScaleEstimate = false;   // measured: no gain with 2-frame windows, worse on s2/s5 (round 4)
    double odomScaleMinPathM = 0.3;   // pair fixes this much travel apart
    double odomScaleMaxPairS = 3.0;
    double odomScaleForget = 0.98;
    double odomScaleMinInfo = 2.0;    // m^2 of accumulated travel per axis before trusting it

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
    int edgeMaxSamplesPerEdge = 80;
    bool edgeUsePolarity = true;     // black rails: wall edges have a known dark side
    double edgeSeamWeight = 0.5;     // tile seams (thin dark lines on the floor); 0 = off
    double edgeSeamMaxRangeM = 2.0;
    double edgeSeamMinContrast = 8.0;  // gray levels below the sides (seams and joints)
    bool edgePostAsRidge = true;     // panel joints (posts) are thin dark lines
    double postWidthM = 0.008;       // visible width of a joint
    double seamWidthM = 0.004;
    // Hive tilt (two stable states per hive): evidence = decay * evidence + gain * (support1 - support0).
    double hiveStartEvidence = 3.0;
    double hiveEvidenceDecay = 0.9;
    double hiveEvidenceGain = 4.0;
    double hiveDecide = 1.0;
    int hiveMinSamples = 20;
    double hiveSupportNeutral = 0.2;  // support fraction that is neither for nor against a tilt
    double wideSearchSigmaXY = 0.04;      // m: above this, try many pose hypotheses
    double wideSearchSigmaHeading = 0.03; // rad
    double wideSearchPriorWeight = 1.0;   // inliers per unit of prior chi2 when ranking hypotheses
    double wideAmbiguityMinInliers = 15;  // a different pose within this many inliers = ambiguous
    double wideAmbiguityFraction = 0.05;  // ... or within this fraction of the best
    int edgeWindowFrames = 2;             // frames tied by odometry when tracking (measured: 2 best)
    int edgeWideWindowFrames = 2;         // frames for the wide search and re-verification (6 measured worse)
    double wideMinSpreadM = 0.25;         // wide search needs this much travel in the window
    double wideMinTurnRad = 0.35;         // ... or this much turning
    double edgeWindowMinSpacingS = 0.15;  // keep window frames at least this far apart

    // Field mode: gate for publishing a fix (C4).
    double gateMaxOmegaRadS = 1.5;
    double gateMaxSpeedMps = 2.0;
    int gateMinInliers = 25;
    double gateMinInlierRatio = 0.5;
    double gateMaxSigmaXY = 0.03;
    double gateMaxSigmaHeading = 0.03;
    double gateMaxChi2 = 16.0;       // Mahalanobis^2 of fix vs prediction (3 dof)
    double gateChi2FloorXY = 0.0;    // systematic sigma added to the chi2 test only (m)
    double gateChi2FloorHeading = 0.0;  // rad
    double fixSigmaFloorXY = 0.008;      // m, lower bound on published sigma_xy
    double fixSigmaFloorHeading = 0.005; // rad
    double fixSigmaPerQXY = 0.0055;      // m per unit of q = rms_px / inlier_ratio^2 (calibrated)
    double fixSigmaPerQHeading = 0.004;  // rad per unit of q
    double fixSigmaQuadratic = 0.006;    // m per unit^2 of q above the knee
    double fixSigmaKneeQ = 2.4;
    int relockAfterRejects = 8;
    double relockCheckS = 2.0;       // re-verify the lock this often (0 = never)
    double relockGridXY = 0.05;      // m, re-verification grid step
    double relockGridHeading = 0.035; // rad
    double relockMargin = 0.08;      // switch only for this many more inliers (fraction) + 10
    double relockAgreeXY = 0.03;     // wide-search answers that fail only the chi2 test against the
    double relockAgreeHeading = 0.026; // prediction are taken when two in a row agree this well
    double relockAgreeMaxS = 3.0;    // ... within this time (0 XY = off)
    int searchSolvesPerFrame = 3;    // at most this many hypothesis solves per frame (45 per search)
    double searchBudgetMs = 8.0;     // ... and stop after this much time (measured on this machine)
    int searchRescoreTop = 2;
    int stillRelockMinInliers = 150;  // re-lock while standing still: stricter bar
    double stillRelockMaxSigmaXY = 0.015;
    int partMinSamples = 30;         // part-consistency check: parts with at least this many samples
    double partMinInlierFrac = 0.2;  // ... must have this inlier fraction, else no fix + re-check (0 = off)        // distinct candidates re-scored when a search completes      // consecutive prediction-disagreement rejections before widening
    double relockSigmaXY = 0.035;    // below the wide-search threshold: re-lock with a wider local search
    double relockSigmaHeading = 0.025;
    double relockWideSigmaXY = 0.06;   // after 3x relockAfterRejects: wide search
    double relockWideSigmaHeading = 0.05;

    // Field mode: surface-constrained ORB map (C3). Points only on the floor (inside the
    // perimeter) and static hive frame faces; candidates until several keyframes agree.
    bool usePoints = true;
    int pointFeatures = 200;          // ORB keypoints per creation keyframe, inside the surface mask only
    int pointLevels = 3;
    int pointFastThreshold = 12;      // foam tiles are low contrast
    double pointMinRangeM = 0.3;
    double pointMaxRangeM = 2.0;      // floor range precision falls with range^2 / camera height
    double pointWallMarginM = 0.05;   // floor points at least this far inside the perimeter
    double pointEdgeMarginPx = 6;     // rays this far to each side must meet the same surface
    int pointMaskBlockPx = 16;        // surface mask resolution (eroded by one block)
    double pointCreateMaxSigmaXY = 0.012;  // create points only from fixes at least this good
    int pointCreateMaxPerKf = 40;
    double pointCreateMinS = 0.5;     // new points at most this often
    int pointTargetVisible = 120;     // ... and only while fewer points than this are in view
    int pointMaxMatchPerFrame = 80;   // points correlated per frame (spread over the image, mature first)
    double pointMatchBudgetMs = 4.0;  // ... and on the robot at most this long (ignored in synchronous replays)
    int pointMaxPoints = 3000;
    double pointKfMinS = 0.2;         // keyframes (observations, creation): at most this often
    double pointKfMinMoveM = 0.05;    // ... and only after this much travel
    double pointKfMinTurnRad = 0.06;  // ... or turning
    int pointMatureObs = 3;           // keyframe observations before a point is used
    double pointMatureBaselineM = 0.10;  // camera travel among them (off-surface points show up)
    double pointConsistPx = 1.2;      // surface-constrained fit RMS (px / pyramid scale) to mature
    double pointCullPx = 2.5;         // above this the point is culled
    double pointCandidateMaxS = 4.0;  // candidates that do not mature in time are dropped
    double pointMinFoundRatio = 0.3;
    double pointCandGatePx = 4;       // a candidate's match must be this close at the frame's pose
    int pointMaxStrikes = 4;          // mature point dropped after this many net disagreements
    int pointPatchHalf = 7;           // correlation template 15 x 15 px
    double pointMinNcc = 0.6;         // normalized cross-correlation to accept a match (noise-limited ~0.8 on foam)
    double pointNccAmbiguity = 0.05;  // a second peak this close is kept as a second candidate
    double pointMinPatchStd = 3.0;    // gray levels: flatter patches are not matched
    double pointRefRefreshM = 0.3;    // new reference patch after the camera moved this far
    double pointRefreshScale = 1.4;   // ... or once the warp stretches or shrinks the patch this much
    double pointRefreshNcc = 0.85;    // ... taken only from a strong, unambiguous match
    double pointSearchMinPx = 12;
    double pointSearchMaxPx = 30;
    double pointWeight = 1.0;         // per point relative to one edge sample (both in px)
    double pointHuberPx = 2.0;
    double pointSigmaPx = 1.5;
    double pointCorrelationInflation = 4.0;  // point positions share the errors of their fixes
    int pointCheckMinMatched = 15;    // wrong-lock check: with this many mature points matched,
    double pointCheckMinInlierFrac = 0.35;   // ... fewer inliers than this rejects the fix
    int pointRelockMinMatched = 25;   // wide search: with this many mature points matched, try a
    double pointRelockMinFrac = 0.7;  // ... local re-lock; accepted with this inlier fraction
    double pointSurfaceTolM = 0.02;   // tests: a surviving point within this of its surface

    // Explore mode (C6).
    std::string vocabularyPath;      // empty = loop closing off
    int exploreInitMinMatches = 100;
    double exploreInitMinParallaxDeg = 1.0;
    int exploreInitWindow = 100;      // px: how far a feature may move between the two init frames
    bool exploreLoopClosing = true;
    int exploreLocalWindow = 10;     // covisible keyframes in local BA
    bool exploreGlobalBA = true;     // full BA after each loop closure (bench)
    int exploreGlobalBAIterations = 30;
    int exploreMaxFramesBetweenKf = 20;
    int exploreAtlasMaps = 3;          // stored maps kept after tracking is lost (0 = no multi-map, wait for relocalization)
    int exploreLostFramesNewMap = 4;   // lost this many frames in a row (map > 10 keyframes): start a new map
    int exploreAtlasEvery = 3;         // check the new map's frames against the stored maps every n-th frame
    double relocMinCommonFrac = 0.8;   // relocalization: keep keyframes sharing this fraction of the best one's words (ORB-SLAM: 0.8)
    int relocMinInliers = 50;          // relocalization accepted with this many pose inliers (ORB-SLAM: 50)
    int exploreAtlasCands = 4;         // keyframe candidates tried per stored map
    int exploreMinFramesBetweenKf = 0;
    int exploreReobsMinGapKf = 100;   // re-observation loop: keyframes this far apart in id

    /** Parses json (may be null/empty). Returns false and fills err on a bad document. */
    static bool parse(const char* json, Params& out, std::string& err);
};

}  // namespace beacon
