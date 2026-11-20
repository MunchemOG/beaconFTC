// beacon_overlay: draws the field model's edges projected with a pose over one frame of an I2
// run. Default pose: the truth (synthetic runs); --pose x,y,heading_rad overrides. Use it to
// check camera intrinsics, mount and field-model conventions by eye.
//
//   beacon_overlay --run DIR --frame N --out overlay.png [--pose x,y,h]
#include <cstdio>
#include <fstream>
#include <sstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "dataset.h"
#include "field_model.h"
#include "edges.h"
#include "render.h"

using namespace beacon;

int main(int argc, char** argv) {
    std::string run, out = "overlay.png", pose;
    int frame = 0;
    for (int i = 1; i + 1 < argc; i += 2) {
        std::string a = argv[i], v = argv[i + 1];
        if (a == "--run") run = v;
        else if (a == "--frame") frame = std::stoi(v);
        else if (a == "--out") out = v;
        else if (a == "--pose") pose = v;
    }
    Dataset d;
    std::string err;
    FieldModel fm;
    if (run.empty() || !loadDataset(run, d, err) || !FieldModel::parse(d.fieldJson, fm, err)) {
        std::fprintf(stderr, "usage: beacon_overlay --run DIR --frame N --out PNG [--pose x,y,h]\n%s\n", err.c_str());
        return 2;
    }
    const FrameRow& f = d.frames.at((size_t)frame);
    cv::Mat img = cv::imread(f.path, cv::IMREAD_GRAYSCALE), col;
    cv::cvtColor(img, col, cv::COLOR_GRAY2BGR);
    SE2 p;
    if (!pose.empty()) {
        std::sscanf(pose.c_str(), "%lf,%lf,%lf", &p.x, &p.y, &p.theta);
    } else {
        std::vector<TimedPose> tr;
        for (const PoseRow& r : d.truth) tr.push_back({r.tNs, SE2(r.x, r.y, r.heading)});
        p = sampleTrajectory(tr, f.tNs);
    }
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
    cam.k3 = d.camera.dist[4];
    SE3 tcf = cameraInField(p, mountToTransform(d.mount.x, d.mount.y, d.mount.z, d.mount.roll, d.mount.pitch, d.mount.yaw)).inverse();
    // Tipping hive parts: draw the state given by hive.csv (synthetic runs) if present.
    std::vector<int> state(fm.parts.size(), -1);
    {
        std::ifstream hf(run + "/hive.csv");
        std::string line, header;
        std::getline(hf, header);
        std::vector<std::string> cols;
        std::stringstream hs(header);
        for (std::string c; std::getline(hs, c, ',');) cols.push_back(c);
        while (std::getline(hf, line)) {
            std::stringstream ls(line);
            std::vector<std::string> v;
            for (std::string c; std::getline(ls, c, ',');) v.push_back(c);
            if (v.empty() || std::stoll(v[0]) > f.tNs) break;
            for (size_t k = 1; k < v.size() && k < cols.size(); ++k)
                for (size_t pi = 0; pi < fm.parts.size(); ++pi)
                    if (fm.parts[pi].name == cols[k]) {
                        double tilt = std::stod(v[k]);
                        state[pi] = std::abs(tilt - fm.parts[pi].tilts[0]) < std::abs(tilt - fm.parts[pi].tilts[1]) ? 0 : 1;
                    }
        }
    }
    for (const ModelEdge& e : fm.edges) {
        if (e.movingPart >= 0 && state[e.movingPart] >= 0 && e.state != state[e.movingPart]) continue;
        cv::Scalar c = e.isStatic ? cv::Scalar(0, 220, 255) : cv::Scalar(255, 80, 80);
        for (int k = 0; k < 100; ++k) {
            Vec3 a = tcf * (e.a + (e.b - e.a) * (k / 100.0)), b = tcf * (e.a + (e.b - e.a) * ((k + 1) / 100.0));
            if (a.z() < 0.05 || b.z() < 0.05) continue;
            Vec2 pa = cam.project(a), pb = cam.project(b);
            cv::line(col, cv::Point2d(pa.x(), pa.y()), cv::Point2d(pb.x(), pb.y()), c, 1, cv::LINE_AA);
        }
    }
    cv::imwrite(out, col);
    {
        // Score this pose: single-frame edge solve with a tight prior (1 cm, 0.3 deg).
        Params prm;
        EdgeTracker et(fm, cam, prm);
        std::vector<int> st;
        for (int v : state) st.push_back(v);
        et.setPartStates(st);
        Mat3 c = Mat3::Zero();
        c(0, 0) = c(1, 1) = 1e-4;
        c(2, 2) = 2.7e-5;
        EdgeResult r = et.solve(img, f.tNs, p, c,
                                mountToTransform(d.mount.x, d.mount.y, d.mount.z, d.mount.roll, d.mount.pitch, d.mount.yaw), false);
        std::printf("score: inliers %d / samples %d, rms %.2f px, solved pose %.3f %.3f %.3f\n", r.inliers, r.samples,
                    r.rmsPx, r.pose.x, r.pose.y, r.pose.theta);
    }
    std::printf("pose %.3f %.3f %.3f -> %s\n", p.x, p.y, p.theta, out.c_str());
    return 0;
}
