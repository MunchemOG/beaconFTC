#include "viewer_rerun.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <spawn.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <opencv2/imgcodecs.hpp>
#include <rerun.hpp>
#include <thread>

#include "dataset.h"

extern char** environ;

namespace beacon {

namespace {

rerun::Position3D pos(const Vec3& p) { return rerun::Position3D((float)p.x(), (float)p.y(), (float)p.z()); }
rerun::encodings::Vec3D v3(const Vec3& p) { return rerun::encodings::Vec3D((float)p.x(), (float)p.y(), (float)p.z()); }

rerun::Points3D points(const std::vector<Vec3>& pts, rerun::Color c, float radius) {
    std::vector<rerun::Position3D> p;
    p.reserve(pts.size());
    for (const Vec3& x : pts) p.push_back(pos(x));
    return rerun::Points3D(p).with_colors(c).with_radii(rerun::Radius::ui_points(radius));
}

rerun::LineStrips3D segments(const std::vector<std::pair<Vec3, Vec3>>& segs, rerun::Color c, float radius) {
    std::vector<rerun::LineStrip3D> s;
    s.reserve(segs.size());
    for (const auto& e : segs) s.push_back(rerun::LineStrip3D(std::vector<rerun::encodings::Vec3D>{v3(e.first), v3(e.second)}));
    return rerun::LineStrips3D(s).with_colors(c).with_radii(rerun::Radius::ui_points(radius));
}

// Short RGB axes for each pose (x red, y green, z blue).
rerun::Arrows3D axes(const std::vector<SE3>& poses, float len) {
    std::vector<rerun::Position3D> o;
    std::vector<rerun::Vector3D> v;
    std::vector<rerun::Color> c;
    for (const SE3& t : poses) {
        for (int k = 0; k < 3; ++k) {
            o.push_back(pos(t.translation()));
            Vec3 d = t.linear().col(k) * len;
            v.push_back(rerun::Vector3D((float)d.x(), (float)d.y(), (float)d.z()));
            c.push_back(k == 0 ? rerun::Color(230, 40, 40) : k == 1 ? rerun::Color(40, 200, 40) : rerun::Color(40, 80, 230));
        }
    }
    return rerun::Arrows3D::from_vectors(v).with_origins(o).with_colors(c);
}

// Camera frustum outline (optical frame: z forward, x right, y down).
std::vector<std::pair<Vec3, Vec3>> frustum(const SE3& t, double s) {
    Vec3 c = t.translation();
    Vec3 corners[4] = {t * Vec3(-s, -0.75 * s, s), t * Vec3(s, -0.75 * s, s), t * Vec3(s, 0.75 * s, s),
                       t * Vec3(-s, 0.75 * s, s)};
    std::vector<std::pair<Vec3, Vec3>> out;
    for (int i = 0; i < 4; ++i) {
        out.push_back({c, corners[i]});
        out.push_back({corners[i], corners[(i + 1) % 4]});
    }
    return out;
}

bool waitForPort(int port, int timeoutMs) {
    auto end = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < end) {
        int fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons((uint16_t)port);
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        int rc = connect(fd, (sockaddr*)&a, sizeof a);
        close(fd);
        if (rc == 0) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    return false;
}

}  // namespace

const std::vector<std::string>& viewerEntityPaths() {
    static const std::vector<std::string> paths = {
        "beacon/world/grid",       "beacon/world/tf",       "beacon/points",        "beacon/backend_pose_imu_viz",
        "beacon/image/cam0",       "beacon/image/cam0/tracked", "beacon/image/cam0/new", "beacon/loop_closures",
        "beacon/pose_imu",         "beacon/kfs",            "beacon/frontend_map",  "beacon/pose_graph",
        "beacon/lc_database_map",  "beacon/backend_points",
    };
    return paths;
}

RerunViewer::RerunViewer() = default;

RerunViewer::~RerunViewer() { flush(); }

bool RerunViewer::open(const ViewerOptions& opt, std::string& err) {
    opt_ = opt;
    rec_ = std::make_unique<rerun::RecordingStream>("beacon");
    if (opt.spawn) {
        std::string exe = opt.rerunExecutable;
        if (exe.empty()) {
            std::string pinned = std::string(BEACON_SOURCE_DIR_BENCH) + "/third_party/src/rerun-cli-0.41.0";
            exe = fileExists(pinned) ? pinned : "rerun";
        }
        std::string port = std::to_string(opt.port);
        std::vector<std::string> args = {exe, "--port", port, "--memory-limit", "50%"};
        if (!opt.blueprint.empty()) args.push_back(opt.blueprint);
        std::vector<char*> argv;
        for (std::string& a : args) argv.push_back(a.data());
        argv.push_back(nullptr);
        pid_t pid;
        if (posix_spawnp(&pid, exe.c_str(), nullptr, nullptr, argv.data(), environ) != 0) {
            err = "cannot start the Rerun viewer (" + exe + ")";
            return false;
        }
        viewerPid_ = (int)pid;
        if (!waitForPort(opt.port, 15000)) {
            err = "Rerun viewer did not open port " + port;
            return false;
        }
    }
    std::string url = "rerun+http://127.0.0.1:" + std::to_string(opt.port) + "/proxy";
    rerun::Error e;
    if (opt.spawn && !opt.saveRrd.empty()) {
        e = rec_->set_sinks(rerun::GrpcSink{url}, rerun::FileSink{opt.saveRrd});
    } else if (opt.spawn) {
        e = rec_->connect_grpc(url);
    } else if (!opt.saveRrd.empty()) {
        e = rec_->save(opt.saveRrd);
    } else {
        err = "viewer: nothing to do (no --view and no --rrd)";
        return false;
    }
    if (e.is_err()) {
        err = "Rerun: " + e.description;
        return false;
    }
    return true;
}

void RerunViewer::logStatic(const VizFrame& v) {
    const rerun::RecordingStream& r = *rec_;
    r.log_static("/", opt_.worldZUp ? rerun::ViewCoordinates::RIGHT_HAND_Z_UP : rerun::ViewCoordinates::RDF);
    // Ground grid: 1 m (or 1 map unit) squares, 12 x 12, in the ground plane.
    std::vector<std::pair<Vec3, Vec3>> g;
    const int n = 6;
    for (int i = -n; i <= n; ++i) {
        if (opt_.worldZUp) {
            g.push_back({Vec3(i, -n, 0), Vec3(i, n, 0)});
            g.push_back({Vec3(-n, i, 0), Vec3(n, i, 0)});
        } else {  // map = first camera frame, y down: ground about 1 unit below the camera
            g.push_back({Vec3(i, 1, -n), Vec3(i, 1, n)});
            g.push_back({Vec3(-n, 1, i), Vec3(n, 1, i)});
        }
    }
    r.log_static("beacon/world/grid", segments(g, rerun::Color(200, 200, 200), 0.5f));
}

void RerunViewer::log(const VizFrame& v) {
    if (!rec_) return;
    const rerun::RecordingStream& r = *rec_;
    if (!staticDone_) {
        logStatic(v);
        staticDone_ = true;
    }
    r.set_time_sequence("frame", frames_);
    r.set_time_duration_nanos("capture", v.tNs);
    if (!v.modelEdges.empty())
        r.log_static("beacon/world/field", segments(v.modelEdges, rerun::Color(255, 200, 0), 1.0f));

    // Camera panel: image + tracked (green) and new (blue) keypoints + field overlay.
    if (!v.image.empty() && frames_ % std::max(1, opt_.imageEvery) == 0) {
        std::vector<uint8_t> jpg;
        cv::imencode(".jpg", v.image, jpg, {cv::IMWRITE_JPEG_QUALITY, opt_.jpegQuality});
        r.log("beacon/image/cam0", rerun::EncodedImage::from_bytes(jpg, rerun::components::MediaType::jpeg()));
    }
    std::vector<rerun::Position2D> tr, fr;
    for (const cv::Point2f& p : v.tracked) tr.push_back(rerun::Position2D(p.x, p.y));
    for (const cv::Point2f& p : v.fresh) fr.push_back(rerun::Position2D(p.x, p.y));
    r.log("beacon/image/cam0/tracked",
          rerun::Points2D(tr).with_colors(rerun::Color(0, 230, 0)).with_radii(rerun::Radius::ui_points(2.5f)));
    r.log("beacon/image/cam0/new",
          rerun::Points2D(fr).with_colors(rerun::Color(40, 90, 255)).with_radii(rerun::Radius::ui_points(2.5f)));
    if (!v.projectedEdges.empty()) {
        std::vector<rerun::LineStrip2D> e;
        for (const auto& s : v.projectedEdges)
            e.push_back(rerun::LineStrip2D(std::vector<rerun::encodings::Vec2D>{{s.first.x, s.first.y}, {s.second.x, s.second.y}}));
        r.log("beacon/image/cam0/field", rerun::LineStrips2D(e).with_colors(rerun::Color(255, 200, 0)));
    }

    // 3-D panel.
    double scale = 0.15;
    if (v.poseValid) {
        r.log("beacon/world/camera", segments(frustum(v.tWorldCam, scale), rerun::Color(255, 120, 0), 1.0f));
        SE3 robot = v.tWorldCam * v.tCamRobot;
        r.log("beacon/world/tf", axes({v.tWorldCam, robot}, (float)(2 * scale)));
    }
    r.log("beacon/points", points(v.trackedPoints, rerun::Color(0, 220, 0), 2.0f));
    if (v.odomValid)
        r.log("beacon/pose_imu", axes({v.tWorldCamOdom}, (float)(2 * scale)));
    if (v.mapChanged) {
        std::vector<rerun::LineStrip3D> traj;
        std::vector<rerun::encodings::Vec3D> pts;
        for (const Vec3& p : v.trajectory) pts.push_back(v3(p));
        traj.push_back(rerun::LineStrip3D(pts));
        r.log("beacon/backend_pose_imu_viz",
              rerun::LineStrips3D(traj).with_colors(rerun::Color(30, 30, 200)).with_radii(rerun::Radius::ui_points(1.5f)));
        r.log("beacon/kfs", axes(v.keyframes, (float)scale));
        r.log("beacon/frontend_map", points(v.localPoints, rerun::Color(220, 60, 220), 1.5f));
        r.log("beacon/backend_points", points(v.backendPoints, rerun::Color(20, 20, 20), 1.5f));
        r.log("beacon/lc_database_map", points(v.mapPoints, rerun::Color(60, 60, 60), 1.0f));
        r.log("beacon/loop_closures", segments(v.loopLinks, rerun::Color(255, 0, 0), 2.0f));
        r.log("beacon/pose_graph", segments(v.graphEdges, rerun::Color(0, 160, 160), 0.6f));
    }
    if (!v.text.empty()) r.log("beacon/status", rerun::TextDocument(v.text));
    ++frames_;
}

void RerunViewer::flush() {
    if (rec_) (void)rec_->flush_blocking(10.0f);
}

}  // namespace beacon
