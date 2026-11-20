// ORB front end shared by both modes: oriented FAST + rotated BRIEF (OpenCV), spread evenly
// over the image, with undistorted keypoint positions and a grid for fast radius search.
#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <vector>

#include "geometry.h"
#include "params.h"

namespace beacon {

struct Features {
    std::vector<cv::KeyPoint> kps;   // distorted pixel positions (as detected)
    std::vector<cv::Point2f> un;     // undistorted pixel positions (pinhole fx, fy, cx, cy)
    cv::Mat desc;                    // N x 32 bytes
    std::vector<float> scaleFactors; // per pyramid level
    std::vector<float> invLevelSigma2;
    int width = 0, height = 0;

    // Grid over undistorted positions for radius queries.
    static constexpr int kCols = 48, kRows = 36;
    std::vector<std::vector<int>> grid;
    float minX = 0, minY = 0, cellW = 1, cellH = 1;

    size_t size() const { return kps.size(); }
    /** Indices of keypoints within r pixels of (x, y), optionally limited to octaves [minL, maxL]. */
    std::vector<int> inArea(float x, float y, float r, int minL = -1, int maxL = -1) const;
};

class OrbExtractor {
public:
    OrbExtractor(const Params& p, const Camera& cam);
    void extract(const cv::Mat& gray, Features& out) const;
    int levels() const { return levels_; }
    float scaleFactor() const { return scale_; }

private:
    const Camera& cam_;
    int nFeatures_, levels_, fast_;
    float scale_;
};

/** Hamming distance of two 32-byte ORB descriptors. */
int orbDistance(const uint8_t* a, const uint8_t* b);
inline int orbDistance(const cv::Mat& a, const cv::Mat& b) { return orbDistance(a.ptr<uint8_t>(), b.ptr<uint8_t>()); }

}  // namespace beacon
