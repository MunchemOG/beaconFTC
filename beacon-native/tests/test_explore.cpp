// C6: explore mode (monocular SLAM, optional odometry) and the Rerun viewer.
#include <gtest/gtest.h>

#include <cstdio>
#include <fstream>
#include <sstream>

#include "dataset.h"
#include "eval.h"
#include "render.h"
#include "replay.h"
#include "viewer_rerun.h"

using namespace beacon;

namespace {

std::string repo() { return BEACON_REPO_DIR; }
std::string eurocDir() { return repo() + "/desktop-harness/data/euroc/MH_01_easy"; }
std::string vocabPath() { return std::string(BEACON_SOURCE_DIR) + "/bench/vocab/beacon_orb_v1_01.bin"; }

std::string exploreParams() {
    return "{\"mode\": \"explore\", \"vocabularyPath\": \"" + vocabPath() + "\"}";
}

// Camera centres from the EuRoC ground truth (body * T_body_cam) at the estimate times.
void associate(const Dataset& d, const std::vector<std::pair<int64_t, Vec3>>& est, std::vector<Vec3>& e,
               std::vector<Vec3>& g) {
    size_t j = 0;
    for (const auto& p : est) {
        while (j + 1 < d.truth6.size() && d.truth6[j + 1].tNs <= p.first) ++j;
        if (j + 1 >= d.truth6.size()) break;
        const auto& a = d.truth6[j];
        if (std::llabs(a.tNs - p.first) > 10000000) continue;
        SE3 body = SE3::Identity();
        body.linear() = a.q.toRotationMatrix();
        body.translation() = a.p;
        e.push_back(p.second);
        g.push_back((body * d.tBodyCam).translation());
    }
}

}  // namespace

TEST(Explore, explore_euroc_mh01_tracks) {
    Dataset d;
    std::string err;
    if (!loadEuroc(eurocDir(), d, err)) GTEST_SKIP() << err;
    if (!fileExists(vocabPath())) GTEST_SKIP() << "train the vocabulary first (README: beacon_vocab_train)";
    ReplayOptions opt;
    opt.paramsJson = exploreParams();
    std::vector<Vec3> finalTraj;
    std::vector<int64_t> finalT;
    int firstTracked = -1, frameNo = 0;
    opt.onFrame = [&](const VizFrame& v) {
        if (firstTracked < 0 && v.status == BEACON_STATUS_TRACKING) firstTracked = frameNo;
        ++frameNo;
        if (v.mapChanged && !v.trajectory.empty()) {
            finalTraj = v.trajectory;
            finalT = v.trajectoryT;
        }
    };
    ReplayResult r;
    ASSERT_TRUE(runReplay(d, nullptr, opt, r, err)) << err;
    double pctAll = 100.0 * r.framesTracking / r.frames;
    double pctAfterInit = firstTracked >= 0 ? 100.0 * r.framesTracking / (r.frames - firstTracked) : 0;
    std::vector<std::pair<int64_t, Vec3>> live, fin;
    for (const auto& p : r.cameraPoses) live.push_back({p.first, p.second.translation()});
    for (size_t i = 0; i < finalTraj.size(); ++i) fin.push_back({finalT[i], finalTraj[i]});
    std::vector<Vec3> e1, g1, e2, g2;
    associate(d, live, e1, g1);
    associate(d, fin, e2, g2);
    double ateLive = rmsError(e1, g1, umeyama(e1, g1, true));
    double ateFinal = rmsError(e2, g2, umeyama(e2, g2, true));
    std::printf("MH_01: frames %d, tracking %d (%.1f %% of all, %.1f %% after init at frame %d), loop closures %d, "
                "keyframes %zu, points %zu\n",
                r.frames, r.framesTracking, pctAll, pctAfterInit, firstTracked, r.loopClosures, r.finalKeyframes.size(),
                r.finalMap.size());
    std::printf("MH_01: ATE after Sim(3): live poses %.3f m (%zu), final optimized trajectory %.3f m (%zu)\n", ateLive,
                e1.size(), ateFinal, e2.size());
    RecordProperty("ate_live_m", std::to_string(ateLive));
    RecordProperty("ate_final_m", std::to_string(ateFinal));
    EXPECT_GE(pctAll, 90.0);
    EXPECT_GE(r.loopClosures, 1);
    EXPECT_LT(ateLive, 0.10);
}

TEST(Explore, explore_with_odom_is_metric) {
    // Synthetic field run with Pinpoint-like odometry: the map must be metric (no scale
    // alignment needed) and the trajectory must match the truth.
    SynthOptions o;
    o.traj.kind = TrajKind::Match;
    o.traj.durationS = 20;
    o.traj.maxSpeed = 0.8;
    o.traj.maxOmega = 1.5;
    o.traj.seed = 4;
    o.fps = 15;
    if (std::getenv("BEACON_TEST_EXACT_ODOM")) {
        o.odom.scaleSigma = 0;
        o.odom.headingWalkDegPerMin = 0;
    }
    Dataset d;
    std::vector<cv::Mat> images;
    std::string err;
    ASSERT_TRUE(makeSynthRun(o, perimeterFieldJson(), d, &images, "", err)) << err;
    d.fieldJson.clear();  // explore mode: no field model
    ReplayOptions opt;
    opt.paramsJson = "{\"mode\": \"explore\", \"orbFeatures\": 1000}";
    ReplayResult r;
    ASSERT_TRUE(runReplay(d, &images, opt, r, err)) << err;
    // Odometry world -> field: the simulator's odometry starts at (0,0,0) at the truth start.
    SE2 s0(d.truth[0].x, d.truth[0].y, d.truth[0].heading);
    SE3 tFieldOdom = s0.toSE3();
    SE3 tRobotCam = mountToTransform(d.mount.x, d.mount.y, d.mount.z, d.mount.roll, d.mount.pitch, d.mount.yaw);
    std::vector<TimedPose> truth;
    for (const PoseRow& p : d.truth) truth.push_back({p.tNs, SE2(p.x, p.y, p.heading)});
    std::vector<Vec3> est, gt;
    for (const auto& p : r.cameraPoses) {
        est.push_back((tFieldOdom * p.second).translation());
        gt.push_back(cameraInField(sampleTrajectory(truth, p.first), tRobotCam).translation());
    }
    ASSERT_GT(est.size(), images.size() / 2);
    // Reference: the odometry alone, same frames.
    std::vector<TimedPose> odom;
    for (const PoseRow& p : d.odom) odom.push_back({p.tNs, SE2(p.x, p.y, p.heading)});
    std::vector<Vec3> odo;
    for (const auto& p : r.cameraPoses)
        odo.push_back((tFieldOdom * cameraInField(sampleTrajectory(odom, p.first), tRobotCam)).translation());
    Alignment3 simOdo = umeyama(odo, gt, true);
    std::printf("odometry alone: ATE without alignment %.3f m, Sim(3) scale %.4f\n", rmsError(odo, gt, Alignment3()),
                simOdo.scale);
    Alignment3 rigid;  // identity: no alignment at all
    double ate = rmsError(est, gt, rigid);
    Alignment3 sim = umeyama(est, gt, true);
    std::printf("explore+odom synthetic: tracked %d/%d frames, ATE without alignment %.3f m, Sim(3) scale %.4f\n",
                r.framesTracking, r.frames, ate, sim.scale);
    // Map scale: median ratio of estimated to true distances between camera positions at least
    // 0.5 m apart (robust to per-pose noise, unlike a Sim(3) fit over a 2 m trajectory).
    std::vector<double> ratios, ratiosOdo;
    for (size_t i = 0; i < est.size(); i += 3)
        for (size_t j = i + 3; j < est.size(); j += 3) {
            double dt = (gt[i] - gt[j]).norm();
            if (dt < 0.5) continue;
            ratios.push_back((est[i] - est[j]).norm() / dt);
            ratiosOdo.push_back((odo[i] - odo[j]).norm() / dt);
        }
    ASSERT_GT(ratios.size(), 100u);
    std::nth_element(ratios.begin(), ratios.begin() + ratios.size() / 2, ratios.end());
    std::nth_element(ratiosOdo.begin(), ratiosOdo.begin() + ratiosOdo.size() / 2, ratiosOdo.end());
    double scaleErr = ratios[ratios.size() / 2] - 1.0;
    std::printf("map scale error (median distance ratio - 1): explore+odom %+.2f %%, odometry alone %+.2f %%\n",
                100 * scaleErr, 100 * (ratiosOdo[ratiosOdo.size() / 2] - 1.0));
    EXPECT_LT(std::abs(scaleErr), 0.01);
    EXPECT_LT(ate, 0.05);
}

TEST(Viewer, viewer_logs_all_entities) {
    Dataset d;
    std::string err;
    if (!loadEuroc(eurocDir(), d, err)) GTEST_SKIP() << err;
    std::string rrd = std::string(BEACON_BINARY_DIR) + "/viewer_test.rrd";
    std::remove(rrd.c_str());
    {
        RerunViewer viewer;
        ViewerOptions vo;
        vo.saveRrd = rrd;
        vo.worldZUp = false;
        vo.imageEvery = 10;
        ASSERT_TRUE(viewer.open(vo, err)) << err;
        ReplayOptions opt;
        opt.paramsJson = exploreParams();
        opt.maxFrames = 300;
        opt.viewer = &viewer;
        ReplayResult r;
        ASSERT_TRUE(runReplay(d, nullptr, opt, r, err)) << err;
        viewer.flush();
    }
    std::string cli = std::string(BEACON_SOURCE_DIR) + "/third_party/src/rerun-cli-0.41.0";
    std::string out = std::string(BEACON_BINARY_DIR) + "/viewer_test.txt";
    ASSERT_EQ(std::system((cli + " rrd print -v " + rrd + " > " + out + " 2>&1").c_str()), 0);
    std::ifstream f(out);
    std::stringstream ss;
    ss << f.rdbuf();
    std::string text = ss.str();
    for (const std::string& e : viewerEntityPaths())
        EXPECT_NE(text.find("- /" + e + " - "), std::string::npos) << "missing entity " << e;
}
