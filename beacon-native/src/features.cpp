#include "features.h"

#include <algorithm>
#include <cmath>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

namespace beacon {

int orbDistance(const uint8_t* a, const uint8_t* b) {
    const uint64_t* pa = reinterpret_cast<const uint64_t*>(a);
    const uint64_t* pb = reinterpret_cast<const uint64_t*>(b);
    int d = 0;
    for (int i = 0; i < 4; ++i) d += __builtin_popcountll(pa[i] ^ pb[i]);
    return d;
}

OrbExtractor::OrbExtractor(const Params& p, const Camera& cam)
    : cam_(cam), nFeatures_(p.orbFeatures), levels_(p.orbLevels), fast_(p.orbFastThreshold),
      scale_((float)p.orbScaleFactor) {}

namespace {

constexpr int kEdge = 19;       // keep keypoints this far from each level's border
constexpr int kHalfPatch = 15;  // intensity-centroid patch radius

// Orientation by intensity centroid over a circular patch (Rublee et al. 2011).
float icAngle(const cv::Mat& img, const cv::Point2f& pt) {
    int cx = cvRound(pt.x), cy = cvRound(pt.y);
    double m01 = 0, m10 = 0;
    for (int v = -kHalfPatch; v <= kHalfPatch; ++v) {
        int span = (int)std::floor(std::sqrt((double)kHalfPatch * kHalfPatch - v * v));
        const uint8_t* row = img.ptr<uint8_t>(cy + v);
        for (int u = -span; u <= span; ++u) {
            int val = row[cx + u];
            m10 += u * val;
            m01 += v * val;
        }
    }
    float a = (float)(std::atan2(m01, m10) * 180.0 / M_PI);
    return a < 0 ? a + 360.f : a;
}

}  // namespace

void OrbExtractor::extract(const cv::Mat& gray, Features& f) const {
    f = Features();
    f.width = gray.cols;
    f.height = gray.rows;
    // Features per level: geometric series so each level gets its share of the image area.
    std::vector<int> perLevel(levels_);
    float inv = 1.0f / scale_;
    float desired = nFeatures_ * (1 - inv) / (1 - std::pow(inv, (float)levels_));
    int sum = 0;
    for (int l = 0; l < levels_ - 1; ++l) {
        perLevel[l] = cvRound(desired);
        sum += perLevel[l];
        desired *= inv;
    }
    perLevel[levels_ - 1] = std::max(nFeatures_ - sum, 0);
    // FAST per cell on each pyramid level; keep the strongest per grid cell, round-robin, so
    // the features cover the image even where contrast is low.
    std::vector<cv::KeyPoint> kps;
    cv::Mat level = gray;
    for (int l = 0; l < levels_; ++l) {
        float sc = std::pow(scale_, (float)l);
        if (l > 0) cv::resize(gray, level, cv::Size(cvRound(gray.cols / sc), cvRound(gray.rows / sc)), 0, 0, cv::INTER_LINEAR);
        const int cell = 30;
        int minX = kEdge - 3, minY = kEdge - 3, maxX = level.cols - kEdge + 3, maxY = level.rows - kEdge + 3;
        if (maxX - minX < cell || maxY - minY < cell) break;
        int nCols = (maxX - minX) / cell, nRows = (maxY - minY) / cell;
        std::vector<std::vector<cv::KeyPoint>> cells(nCols * nRows);
        for (int r = 0; r < nRows; ++r) {
            for (int c = 0; c < nCols; ++c) {
                int x0 = minX + c * cell, y0 = minY + r * cell;
                int x1 = std::min(x0 + cell + 6, maxX), y1 = std::min(y0 + cell + 6, maxY);
                cv::Mat roi = level(cv::Range(y0, y1), cv::Range(x0, x1));
                std::vector<cv::KeyPoint> cellKps;
                cv::FAST(roi, cellKps, fast_, true);
                if (cellKps.empty()) cv::FAST(roi, cellKps, std::min(fast_, 7), true);
                for (cv::KeyPoint& k : cellKps) {
                    k.pt.x += x0;
                    k.pt.y += y0;
                    if (k.pt.x < kEdge || k.pt.y < kEdge || k.pt.x >= level.cols - kEdge || k.pt.y >= level.rows - kEdge)
                        continue;
                    cells[r * nCols + c].push_back(k);
                }
            }
        }
        for (auto& c : cells)
            std::sort(c.begin(), c.end(), [](const cv::KeyPoint& a, const cv::KeyPoint& b) { return a.response > b.response; });
        int got = 0;
        for (size_t round = 0; got < perLevel[l]; ++round) {
            bool any = false;
            for (auto& c : cells) {
                if (round >= c.size()) continue;
                any = true;
                cv::KeyPoint k = c[round];
                k.angle = icAngle(level, k.pt);
                k.octave = l;
                k.size = 31 * sc;
                k.pt.x *= sc;
                k.pt.y *= sc;
                kps.push_back(k);
                if (++got >= perLevel[l]) break;
            }
            if (!any) break;
        }
    }
    // Descriptors: OpenCV's rotated BRIEF on its own (blurred) pyramid, same scale factor.
    cv::Ptr<cv::ORB> orb = cv::ORB::create(nFeatures_, scale_, levels_, kEdge, 0, 2, cv::ORB::HARRIS_SCORE, 31, fast_);
    cv::Mat desc;
    orb->detectAndCompute(gray, cv::noArray(), kps, desc, true);
    f.kps = kps;
    f.desc = desc;
    // Undistort.
    f.un.resize(f.kps.size());
    if (cam_.hasDistortion() && !f.kps.empty()) {
        std::vector<cv::Point2f> in(f.kps.size());
        for (size_t i = 0; i < in.size(); ++i) in[i] = f.kps[i].pt;
        cv::Mat k = (cv::Mat_<double>(3, 3) << cam_.fx, 0, cam_.cx, 0, cam_.fy, cam_.cy, 0, 0, 1);
        cv::Mat d = (cv::Mat_<double>(1, 5) << cam_.k1, cam_.k2, cam_.p1, cam_.p2, cam_.k3);
        cv::undistortPoints(in, f.un, k, d, cv::noArray(), k);
    } else {
        for (size_t i = 0; i < f.kps.size(); ++i) f.un[i] = f.kps[i].pt;
    }
    f.scaleFactors.resize(levels_);
    f.invLevelSigma2.resize(levels_);
    for (int l = 0; l < levels_; ++l) {
        f.scaleFactors[l] = std::pow(scale_, (float)l);
        f.invLevelSigma2[l] = 1.0f / (f.scaleFactors[l] * f.scaleFactors[l]);
    }
    // Grid over undistorted positions (bounds: undistorted image corners, with a margin).
    f.minX = -0.1f * gray.cols;
    f.minY = -0.1f * gray.rows;
    f.cellW = 1.2f * gray.cols / Features::kCols;
    f.cellH = 1.2f * gray.rows / Features::kRows;
    f.grid.assign(Features::kCols * Features::kRows, {});
    for (int i = 0; i < (int)f.un.size(); ++i) {
        int cx = (int)std::floor((f.un[i].x - f.minX) / f.cellW);
        int cy = (int)std::floor((f.un[i].y - f.minY) / f.cellH);
        if (cx < 0 || cy < 0 || cx >= Features::kCols || cy >= Features::kRows) continue;
        f.grid[cy * Features::kCols + cx].push_back(i);
    }
}

std::vector<int> Features::inArea(float x, float y, float r, int minL, int maxL) const {
    std::vector<int> out;
    int x0 = std::max(0, (int)std::floor((x - r - minX) / cellW));
    int x1 = std::min(kCols - 1, (int)std::floor((x + r - minX) / cellW));
    int y0 = std::max(0, (int)std::floor((y - r - minY) / cellH));
    int y1 = std::min(kRows - 1, (int)std::floor((y + r - minY) / cellH));
    if (x0 > x1 || y0 > y1) return out;
    for (int cy = y0; cy <= y1; ++cy)
        for (int cx = x0; cx <= x1; ++cx)
            for (int i : grid[cy * kCols + cx]) {
                const cv::KeyPoint& k = kps[i];
                if (minL >= 0 && k.octave < minL) continue;
                if (maxL >= 0 && k.octave > maxL) continue;
                float dx = un[i].x - x, dy = un[i].y - y;
                if (dx * dx + dy * dy < r * r) out.push_back(i);
            }
    return out;
}

}  // namespace beacon
