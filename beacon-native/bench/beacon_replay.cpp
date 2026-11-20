// beacon_replay: runs the native core over a recorded or synthetic data set.
//
//   beacon_replay --run DIR [--out fixes.csv] [--mode field|explore] [--no-odom]
//   beacon_replay --euroc DIR [--traj traj.txt]       (EuRoC cam0, explore mode)
// Common options: --params FILE_OR_JSON  --latency-ms 40  --max-frames N  --async
//                 --view (live Rerun viewer)  --rrd FILE (save a Rerun recording)  --realtime
#include <cstdio>
#include <algorithm>
#include <cstring>
#include <string>

#include "dataset.h"
#include "eval.h"
#include "render.h"
#include "replay.h"
#include "viewer_rerun.h"

using namespace beacon;

static void usage() {
    std::fprintf(stderr,
                 "usage: beacon_replay (--run DIR | --euroc DIR) [--out fixes.csv] [--traj traj.txt]\n"
                 "       [--mode field|explore] [--no-odom] [--params FILE_OR_JSON] [--latency-ms MS]\n"
                 "       [--max-frames N] [--async] [--view] [--rrd FILE] [--realtime] [--image-every N]\n");
}

int main(int argc, char** argv) {
    std::string run, euroc, out, traj, mode, params, rrd;
    bool noOdom = false, async = false, view = false, realtime = false, rrdFinalOnly = false;
    double latencyMs = 40;
    int maxFrames = -1, imageEvery = 1, verbose = 0;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                usage();
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--run") run = next();
        else if (a == "--euroc") euroc = next();
        else if (a == "--out") out = next();
        else if (a == "--traj") traj = next();
        else if (a == "--mode") mode = next();
        else if (a == "--params") params = next();
        else if (a == "--latency-ms") latencyMs = std::stod(next());
        else if (a == "--max-frames") maxFrames = std::stoi(next());
        else if (a == "--image-every") imageEvery = std::stoi(next());
        else if (a == "--rrd") rrd = next();
        else if (a == "--verbose") verbose = std::stoi(next());
        else if (a == "--no-odom") noOdom = true;
        else if (a == "--async") async = true;
        else if (a == "--view") view = true;
        else if (a == "--realtime") realtime = true;
        else if (a == "--rrd-final-only") rrdFinalOnly = true;
        else {
            usage();
            return 2;
        }
    }
    if (run.empty() == euroc.empty()) {
        usage();
        return 2;
    }
    Dataset d;
    std::string err;
    bool ok = euroc.empty() ? loadDataset(run, d, err) : loadEuroc(euroc, d, err);
    if (!ok) {
        std::fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }
    if (mode.empty()) mode = euroc.empty() ? "field" : "explore";
    if (mode != "field" && mode != "explore") {
        usage();
        return 2;
    }
    if (!params.empty() && params[0] != '{') {
        std::string text = readFile(params);
        if (text.empty()) {
            std::fprintf(stderr, "error: cannot read params file %s\n", params.c_str());
            return 1;
        }
        params = text;
    }
    ReplayOptions opt;
    opt.paramsJson = mergeJson(params, "{\"mode\": \"" + mode + "\"}");
    opt.async = async;
    opt.latencyS = latencyMs / 1000.0;
    opt.useOdom = !noOdom;
    opt.maxFrames = maxFrames;
    opt.realtime = realtime;
    if (mode == "explore" && d.odom.empty()) d.fieldJson.clear();  // explore needs no field
    RerunViewer viewer;
    if (view || !rrd.empty()) {
        ViewerOptions vo;
        vo.spawn = view;
        vo.saveRrd = rrd;
        vo.blueprint = std::string(BEACON_SOURCE_DIR_BENCH) + "/bench/viewer/beacon.rbl";
        vo.worldZUp = !(mode == "explore" && (d.odom.empty() || noOdom));
        vo.imageEvery = imageEvery;
        vo.onlyLast = rrdFinalOnly;
        if (!viewer.open(vo, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
        opt.viewer = &viewer;
    }
    int frameNo = 0;
    std::vector<Vec3> finalTraj;
    std::vector<int64_t> finalT;
    opt.onFrame = [&](const VizFrame& v) {
            if (v.mapChanged && !v.trajectory.empty()) {
                finalTraj = v.trajectory;
                finalT = v.trajectoryT;
            }
            if (verbose > 0 && frameNo++ % verbose == 0)
                std::printf("%5d t=%.2f tracked %zu new %zu | %s\n", frameNo - 1, (v.tNs - d.frames[0].tNs) * 1e-9,
                            v.tracked.size(), v.fresh.size(), v.text.c_str());
        };
    ReplayResult res;
    if (!runReplay(d, nullptr, opt, res, err)) {
        std::fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }
    viewer.flush();
    std::printf("frames %d, tracking %d (%.1f %%), fixes %zu, loop closures %d, keyframes %zu, map points %zu\n",
                res.frames, res.framesTracking, res.frames ? 100.0 * res.framesTracking / res.frames : 0.0,
                res.fixes.size(), res.loopClosures, res.finalKeyframes.size(), res.finalMap.size());
    std::printf("last: %s\n", res.lastDebug.c_str());
    if (!async && !res.frameMs.empty()) {
        std::vector<double> ms = res.frameMs;
        std::sort(ms.begin(), ms.end());
        std::printf("core time per frame (this Mac, synchronous): p50 %.2f ms, p99 %.2f ms, max %.2f ms\n",
                    ms[ms.size() / 2], ms[ms.size() * 99 / 100], ms.back());
    }
    if (!out.empty() && !writeFixesCsv(out, res.fixes, err)) {
        std::fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }
    if (!traj.empty()) {
        FILE* f = std::fopen(traj.c_str(), "w");
        if (f) {
            std::fprintf(f, "# t_ns x y z qx qy qz qw (camera in the map frame)\n");
            for (const auto& p : res.cameraPoses) {
                Eigen::Quaterniond q(p.second.linear());
                Vec3 t = p.second.translation();
                std::fprintf(f, "%lld %.6f %.6f %.6f %.6f %.6f %.6f %.6f\n", (long long)p.first, t.x(), t.y(), t.z(),
                             q.x(), q.y(), q.z(), q.w());
            }
            std::fclose(f);
        }
    }
    // Preliminary error numbers (NOT the plan's gate; that is tools/eval/eval_run.py).
    if (mode == "field" && !d.truth.empty() && !res.fixes.empty()) {
        std::vector<TimedPose> truth;
        for (const PoseRow& r : d.truth) truth.push_back({r.tNs, SE2(r.x, r.y, r.heading)});
        double maxE = 0, ss = 0, maxH = 0;
        for (const beacon_fix& f : res.fixes) {
            SE2 t = sampleTrajectory(truth, f.t_ns);
            double e = std::hypot(f.x - t.x, f.y - t.y);
            maxE = std::max(maxE, e);
            ss += e * e;
            maxH = std::max(maxH, std::abs(wrapAngle(f.heading - t.theta)));
        }
        std::printf("PRELIMINARY (own scoring, fixes vs truth): rms %.2f cm, max %.2f cm, max heading %.2f deg\n",
                    100 * std::sqrt(ss / res.fixes.size()), 100 * maxE, maxH * 180 / M_PI);
    }
    if (!d.truth6.empty() && !res.cameraPoses.empty()) {
        std::vector<Vec3> est, gt;
        size_t j = 0;
        for (const auto& p : res.cameraPoses) {
            while (j + 1 < d.truth6.size() && d.truth6[j + 1].tNs <= p.first) ++j;
            if (j + 1 >= d.truth6.size()) break;
            const auto& g = std::llabs(d.truth6[j].tNs - p.first) < std::llabs(d.truth6[j + 1].tNs - p.first)
                                ? d.truth6[j]
                                : d.truth6[j + 1];
            if (std::llabs(g.tNs - p.first) > 10000000) continue;
            SE3 body = SE3::Identity();
            body.linear() = g.q.toRotationMatrix();
            body.translation() = g.p;
            est.push_back(p.second.translation());
            gt.push_back((body * d.tBodyCam).translation());
        }
        Alignment3 a = umeyama(est, gt, true);
        std::printf("ATE after Sim(3) alignment: %.3f m over %zu poses (scale %.3f)\n", rmsError(est, gt, a), est.size(),
                    a.scale);
        // Final (loop-corrected, BA-refined) trajectory: every frame re-expressed through its
        // reference keyframe at the end of the run.
        std::vector<Vec3> e2, g2;
        j = 0;
        for (size_t k = 0; k < finalTraj.size(); ++k) {
            int64_t t = finalT[k];
            while (j + 1 < d.truth6.size() && d.truth6[j + 1].tNs <= t) ++j;
            if (j + 1 >= d.truth6.size()) break;
            if (std::llabs(d.truth6[j].tNs - t) > 10000000) continue;
            SE3 body = SE3::Identity();
            body.linear() = d.truth6[j].q.toRotationMatrix();
            body.translation() = d.truth6[j].p;
            e2.push_back(finalTraj[k]);
            g2.push_back((body * d.tBodyCam).translation());
        }
        if (e2.size() > 3)
            std::printf("ATE of the final trajectory after Sim(3) alignment: %.3f m over %zu poses\n",
                        rmsError(e2, g2, umeyama(e2, g2, true)), e2.size());
    }
    return 0;
}
