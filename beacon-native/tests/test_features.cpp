// ORB front end and matching on real images (EuRoC MH_01 if present) and synthetic renders.
#include <gtest/gtest.h>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "dataset.h"
#include "features.h"
#include "matcher.h"

using namespace beacon;

static std::string eurocDir() { return std::string(BEACON_REPO_DIR) + "/desktop-harness/data/euroc/MH_01_easy"; }

TEST(Features, shifted_frame_matches_most_features) {
    Dataset d;
    std::string err;
    if (!loadEuroc(eurocDir(), d, err)) GTEST_SKIP() << err;
    Camera cam;
    cam.width = d.camera.width;
    cam.height = d.camera.height;
    cam.fx = d.camera.fx;
    cam.fy = d.camera.fy;
    cam.cx = d.camera.cx;
    cam.cy = d.camera.cy;
    cam.k1 = d.camera.dist[0];
    cam.k2 = d.camera.dist[1];
    cam.p1 = d.camera.dist[2];
    cam.p2 = d.camera.dist[3];
    Params p;
    OrbExtractor orb(p, cam);
    cv::Mat a = cv::imread(d.frames[10].path, cv::IMREAD_GRAYSCALE);
    // The second view: the same frame shifted by (3.5, -2.25) px plus noise (MH_01 shakes a lot
    // at the start, so real consecutive frames are not a fair repeatability check).
    cv::Mat b;
    cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, 3.5, 0, 1, -2.25);
    cv::warpAffine(a, b, M, a.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    cv::Mat noise(b.size(), CV_8S);
    cv::randn(noise, 0, 2);
    cv::add(b, noise, b, cv::noArray(), CV_8U);
    Features fa, fb;
    orb.extract(a, fa);
    orb.extract(b, fb);
    int level0 = 0;
    for (auto& k : fa.kps) level0 += k.octave == 0;
    Map map;
    Matcher m(map, Pinhole{cam.fx, cam.fy, cam.cx, cam.cy, cam.width, cam.height});
    std::vector<cv::Point2f> prev = fa.un;
    std::vector<int> m12;
    int n = m.searchForInitialization(fa, fb, prev, m12, 100);
    std::vector<cv::Point2f> prevSelf = fa.un;
    std::vector<int> mSelf;
    int nSelf = m.searchForInitialization(fa, fa, prevSelf, mSelf, 100);
    int outside = 0;
    for (auto& u : fa.un)
        if (u.x < fa.minX || u.y < fa.minY || u.x >= fa.minX + Features::kCols * fa.cellW || u.y >= fa.minY + Features::kRows * fa.cellH) ++outside;
    std::printf("features %zu / %zu, level-0 %d, init matches %d, self %d, outside grid %d\n", fa.size(), fb.size(), level0, n, nSelf, outside);
    // Brute-force cross-checked matches within 3 px (repeatability of detector + descriptor).
    int bf = 0;
    for (size_t i = 0; i < fa.size(); ++i) {
        int best = 256, bi = -1;
        for (size_t j = 0; j < fb.size(); ++j) {
            int dd = orbDistance(fa.desc.ptr<uint8_t>((int)i), fb.desc.ptr<uint8_t>((int)j));
            if (dd < best) { best = dd; bi = (int)j; }
        }
        if (best < 50 && cv::norm(fa.kps[i].pt + cv::Point2f(3.5f, -2.25f) - fb.kps[bi].pt) < 3) ++bf;
    }
    std::printf("repeatable (brute force, < 50 bits, < 3 px): %d of %zu\n", bf, fa.size());
    Features fa2;
    orb.extract(a, fa2);
    int same = 0;
    for (size_t i = 0; i < fa.size() && i < fa2.size(); ++i) same += cv::norm(fa.kps[i].pt - fa2.kps[i].pt) < 1e-3 && orbDistance(fa.desc.ptr<uint8_t>((int)i), fa2.desc.ptr<uint8_t>((int)i)) == 0;
    std::printf("deterministic extraction: %d of %zu identical\n", same, fa.size());
    EXPECT_GT(bf, 0.5 * fa.size());
    EXPECT_GT(n, 0.3 * level0);
}
