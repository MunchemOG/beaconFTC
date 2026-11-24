// beacon_live: runs the native core on the live Limelight MJPEG stream on a Mac and shows it in
// the Rerun viewer (camera panel with tracked/new features and the projected field model, 3-D
// panel with trajectory, keyframes, map, loop closures). Bench only: no odometry on a Mac, so
// explore mode has arbitrary scale and field mode predicts with constant velocity.
//
//   beacon_live [--url http://limelight.local:5800] [--camera camera.json]
//               [--mode explore|field] [--field FIELD.json --start x,y,heading_deg]
//               [--params FILE_OR_JSON] [--latency-ms 30] [--frames N] [--no-view] [--rrd FILE]
//               [--record DIR] [--no-mask-overlay]
// By default the Limelight's crosshair (centre) and text (top-left) are scrubbed from each frame.
// Exit code 2 when the stream cannot be reached.
#include <sys/stat.h>

#include <cmath>
#include <cstdio>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>

#include "dataset.h"
#include "mjpeg.h"
#include "render.h"
#include "replay.h"
#include "viewer_rerun.h"

using namespace beacon;

int main(int argc, char** argv) {
    std::string url = "http://limelight.local:5800", cameraPath, mode = "explore", fieldPath, start, params, rrd,
                record;
    double latencyMs = 30;
    int maxFrames = -1;
    bool view = true, maskOverlay = true;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--url") url = next();
        else if (a == "--camera") cameraPath = next();
        else if (a == "--mode") mode = next();
        else if (a == "--field") fieldPath = next();
        else if (a == "--start") start = next();
        else if (a == "--params") params = next();
        else if (a == "--latency-ms") latencyMs = std::stod(next());
        else if (a == "--frames") maxFrames = std::stoi(next());
        else if (a == "--rrd") rrd = next();
        else if (a == "--record") record = next();
        else if (a == "--no-view") view = false;
        else if (a == "--no-mask-overlay") maskOverlay = false;
        else {
            std::fprintf(stderr, "unknown option %s (see the header of bench/beacon_live.cpp)\n", a.c_str());
            return 2;
        }
    }
    if (mode != "explore" && mode != "field") {
        std::fprintf(stderr, "--mode must be explore or field\n");
        return 2;
    }
    // Camera: camera.json (I2 format) or the Limelight 3A defaults (640x480, HFOV 54.5 deg).
    Dataset cam;
    cam.camera = benchCamera(640, 480, 54.5);
    cam.mount = beacon_mount{0.0, 0.0, 0.3, 0.0, 0.0, 0.0};
    if (!cameraPath.empty()) {
        std::string err;
        if (!loadCameraJson(cameraPath, cam.camera, cam.mount, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            return 1;
        }
    }
    std::string fieldJson;
    if (mode == "field") {
        fieldJson = fieldPath.empty() ? perimeterFieldJson() : readFile(fieldPath);
        if (fieldJson.empty()) {
            std::fprintf(stderr, "error: cannot read field model %s\n", fieldPath.c_str());
            return 1;
        }
    }
    if (!params.empty() && params[0] != '{') params = readFile(params);
    std::string p = mergeJson(params, "{\"mode\": \"" + mode + "\"}");
    char msg[512];
    beacon_ctx* ctx = beacon_create(fieldJson.empty() ? nullptr : fieldJson.c_str(), &cam.camera, &cam.mount,
                                    p.c_str(), msg, sizeof msg);
    if (!ctx) {
        std::fprintf(stderr, "error: beacon_create: %s\n", msg);
        return 1;
    }
    RerunViewer viewer;
    if (view || !rrd.empty()) {
        ViewerOptions vo;
        vo.spawn = view;
        vo.saveRrd = rrd;
        vo.blueprint = std::string(BEACON_SOURCE_DIR_BENCH) + "/bench/viewer/beacon.rbl";
        vo.worldZUp = mode == "field";
        std::string err;
        if (!viewer.open(vo, err)) {
            std::fprintf(stderr, "error: %s\n", err.c_str());
            beacon_destroy(ctx);
            return 1;
        }
        // The observer runs on the core's worker thread; Rerun's stream is thread-safe.
        beacon_set_observer(ctx, [&viewer](const VizFrame& v) { viewer.log(v); });
    }
    MjpegClient client;
    std::string err;
    if (!client.open(url, 3000, err)) {
        std::fprintf(stderr, "error: Limelight stream not reachable: %s\n", err.c_str());
        beacon_destroy(ctx);
        return 2;
    }
    int64_t startT = 0;
    FILE* framesCsv = nullptr;
    if (!record.empty()) {
        mkdir(record.c_str(), 0755);
        mkdir((record + "/frames").c_str(), 0755);
        writeCameraJson(record + "/camera.json", cam.camera, cam.mount, err);
        framesCsv = std::fopen((record + "/frames.csv").c_str(), "w");
        if (framesCsv) std::fprintf(framesCsv, "idx,t_ns\n");
    }
    std::vector<uint8_t> jpg;
    int64_t recv;
    int n = 0, dropped = 0;
    bool warnedSize = false;
    int64_t lastPrint = monotonicNs();
    while (maxFrames < 0 || n < maxFrames) {
        if (!client.next(jpg, recv, err)) {
            std::fprintf(stderr, "error: stream: %s\n", err.c_str());
            break;
        }
        cv::Mat gray = cv::imdecode(jpg, cv::IMREAD_GRAYSCALE);
        if (gray.empty()) continue;
        if (gray.cols != cam.camera.width || gray.rows != cam.camera.height) {
            if (!warnedSize) {
                std::fprintf(stderr, "warning: stream is %dx%d, camera model is %dx%d; resizing\n", gray.cols, gray.rows,
                             cam.camera.width, cam.camera.height);
                warnedSize = true;
            }
            cv::resize(gray, gray, cv::Size(cam.camera.width, cam.camera.height), 0, 0, cv::INTER_AREA);
        }
        if (maskOverlay) {
            // Limelight draws a crosshair at the image centre and a small text block top-left on
            // every frame. They never move, so they would look like zero-parallax features. A
            // median filter over those small regions removes thin strokes and keeps the rest.
            auto scrub = [&](cv::Rect r) {
                r &= cv::Rect(0, 0, gray.cols, gray.rows);
                if (r.area() <= 0) return;
                cv::Mat roi = gray(r);
                cv::medianBlur(roi.clone(), roi, 15);
            };
            scrub(cv::Rect(gray.cols / 2 - 30, gray.rows / 2 - 30, 60, 60));
            scrub(cv::Rect(0, 0, 90, 50));
        }
        int64_t t = recv - (int64_t)(latencyMs * 1e6);
        if (n == 0 && mode == "field") {
            double x = 0, y = 0, hd = 0;
            if (!start.empty()) std::sscanf(start.c_str(), "%lf,%lf,%lf", &x, &y, &hd);
            beacon_pose2 s{t, x, y, hd * M_PI / 180};
            beacon_set_start_pose(ctx, &s);
            startT = t;
        }
        if (beacon_push_frame(ctx, t, gray.data, gray.cols, gray.rows, (int)gray.step) == BEACON_DROPPED) ++dropped;
        if (framesCsv) {
            char name[64];
            std::snprintf(name, sizeof name, "%s/frames/%06d.png", record.c_str(), n);
            cv::imwrite(name, gray);
            std::fprintf(framesCsv, "%d,%lld\n", n, (long long)t);
        }
        ++n;
        if (recv - lastPrint > 1000000000LL) {
            char line[512];
            beacon_debug_line(ctx, line, sizeof line);
            std::printf("%s | jpeg %zu B | dropped by core %d\n", line, jpg.size(), dropped);
            std::fflush(stdout);
            lastPrint = recv;
        }
    }
    (void)startT;
    if (framesCsv) std::fclose(framesCsv);
    char line[512];
    beacon_debug_line(ctx, line, sizeof line);
    std::printf("done: %d frames received. %s\n", n, line);
    beacon_destroy(ctx);
    viewer.flush();
    return 0;
}
