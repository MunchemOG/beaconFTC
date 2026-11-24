#include "replay.h"

#include <chrono>
#include <cstdio>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

#include "viewer_rerun.h"

namespace beacon {

std::string mergeJson(const std::string& a, const std::string& b) {
    nlohmann::json ja = a.empty() ? nlohmann::json::object() : nlohmann::json::parse(a, nullptr, false, true);
    nlohmann::json jb = b.empty() ? nlohmann::json::object() : nlohmann::json::parse(b, nullptr, false, true);
    if (ja.is_discarded() || !ja.is_object()) ja = nlohmann::json::object();
    if (jb.is_object())
        for (auto it = jb.begin(); it != jb.end(); ++it) ja[it.key()] = it.value();
    return ja.dump();
}

bool writeFixesCsv(const std::string& path, const std::vector<beacon_fix>& fixes, std::string& err) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        err = "cannot write " + path;
        return false;
    }
    std::fprintf(f, "t_ns,x,y,heading,sigma_xy,sigma_heading,inliers,kind\n");
    for (const beacon_fix& x : fixes)
        std::fprintf(f, "%lld,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d\n", (long long)x.t_ns, x.x, x.y, x.heading, x.sigma_xy,
                     x.sigma_heading, x.inliers, x.kind);
    std::fclose(f);
    return true;
}

bool runReplay(const Dataset& d, const std::vector<cv::Mat>* images, const ReplayOptions& opt, ReplayResult& out,
               std::string& err) {
    out = ReplayResult();
    std::string params = opt.async ? opt.paramsJson : mergeJson(opt.paramsJson, "{\"synchronous\": true}");
    char msg[512] = {0};
    const char* field = d.fieldJson.empty() ? nullptr : d.fieldJson.c_str();
    beacon_ctx* ctx = beacon_create(field, &d.camera, &d.mount, params.c_str(), msg, sizeof msg);
    if (!ctx) {
        err = std::string("beacon_create: ") + msg;
        return false;
    }
    beacon_set_observer(ctx, [&](const VizFrame& v) {
        out.frames++;
        if (v.status == BEACON_STATUS_TRACKING) out.framesTracking++;
        if (v.poseValid) out.cameraPoses.push_back({v.tNs, v.tWorldCam});
        if (v.mapChanged) {
            out.loopClosures = (int)v.loopLinks.size();
            out.finalKeyframes = v.keyframes;
            out.finalMap = v.mapPoints;
        }
        if (opt.viewer) opt.viewer->log(v);
        if (opt.onFrame) opt.onFrame(v);
    });
    if (!d.start.empty()) {
        beacon_pose2 s{d.start[0].tNs, d.start[0].x, d.start[0].y, d.start[0].heading};
        beacon_set_start_pose(ctx, &s);
    }
    const int64_t latency = (int64_t)(opt.latencyS * 1e9);
    size_t oi = 0;
    size_t n = d.frames.size();
    if (opt.maxFrames >= 0 && (size_t)opt.maxFrames < n) n = (size_t)opt.maxFrames;
    auto wall0 = std::chrono::steady_clock::now();
    int64_t t0 = n ? d.frames[0].tNs : 0;
    cv::Mat img;
    for (size_t i = 0; i < n; ++i) {
        const FrameRow& f = d.frames[i];
        int64_t deliver = f.tNs + latency;
        while (opt.useOdom && oi < d.odom.size() && d.odom[oi].tNs <= deliver) {
            beacon_pose2 p{d.odom[oi].tNs, d.odom[oi].x, d.odom[oi].y, d.odom[oi].heading};
            beacon_push_odom(ctx, &p);
            ++oi;
        }
        if (images) {
            img = (*images)[i];
        } else {
            img = cv::imread(f.path, cv::IMREAD_GRAYSCALE);
            if (img.empty()) {
                err = "cannot read " + f.path;
                beacon_destroy(ctx);
                return false;
            }
        }
        if (opt.realtime) {
            std::this_thread::sleep_until(wall0 + std::chrono::nanoseconds(deliver - t0));
        }
        auto c0 = std::chrono::steady_clock::now();
        int rc = beacon_push_frame(ctx, f.tNs, img.data, img.cols, img.rows, (int)img.step);
        out.frameMs.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - c0).count());
        if (rc == BEACON_ERR) {
            err = "beacon_push_frame rejected frame " + std::to_string(f.idx) + " (size " + std::to_string(img.cols) +
                  "x" + std::to_string(img.rows) + ")";
            beacon_destroy(ctx);
            return false;
        }
        beacon_fix fx;
        if (beacon_latest_fix(ctx, &fx) == 1) out.fixes.push_back(fx);
    }
    if (opt.async) {
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        beacon_fix fx;
        if (beacon_latest_fix(ctx, &fx) == 1) out.fixes.push_back(fx);
    }
    out.statusAtEnd = beacon_status(ctx);
    char line[512];
    beacon_debug_line(ctx, line, sizeof line);
    out.lastDebug = line;
    beacon_destroy(ctx);
    return true;
}

}  // namespace beacon
