#include "render.h"

#include <sys/stat.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>

namespace beacon {

namespace {

inline uint32_t hash3(int32_t x, int32_t y, uint32_t s) {
    uint32_t h = (uint32_t)x * 374761393u + (uint32_t)y * 668265263u + s * 2246822519u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}
inline double hashUnit(int32_t x, int32_t y, uint32_t s) { return (hash3(x, y, s) & 0xffffff) / 16777215.0; }

// Smooth value noise in [0, 1] with cell size `cell`.
double valueNoise(double x, double y, double cell, uint32_t s) {
    double fx = x / cell, fy = y / cell;
    int32_t ix = (int32_t)std::floor(fx), iy = (int32_t)std::floor(fy);
    double ax = fx - ix, ay = fy - iy;
    ax = ax * ax * (3 - 2 * ax);
    ay = ay * ay * (3 - 2 * ay);
    double a = hashUnit(ix, iy, s), b = hashUnit(ix + 1, iy, s);
    double c = hashUnit(ix, iy + 1, s), d = hashUnit(ix + 1, iy + 1, s);
    return (1 - ay) * ((1 - ax) * a + ax * b) + ay * ((1 - ax) * c + ax * d);
}

// Blocky high-contrast pattern: random rectangles (banners, people, bleachers).
double blocks(double x, double y, double cell, uint32_t s) {
    int32_t ix = (int32_t)std::floor(x / cell), iy = (int32_t)std::floor(y / cell);
    double base = hashUnit(ix, iy, s);
    double fx = x / cell - ix, fy = y / cell - iy;
    double inner = hashUnit(ix, iy, s + 7);
    if (fx > 0.2 && fx < 0.2 + 0.6 * inner && fy > 0.15 && fy < 0.85) base = 1.0 - base;
    return base;
}

bool rayBox(const Vec3& o, const Vec3& d, const Box& b, double& tHit, Vec3& n) {
    double t0 = 0, t1 = 1e9;
    int axis = -1;
    for (int i = 0; i < 3; ++i) {
        if (std::abs(d[i]) < 1e-12) {
            if (o[i] < b.lo[i] || o[i] > b.hi[i]) return false;
            continue;
        }
        double ta = (b.lo[i] - o[i]) / d[i], tb = (b.hi[i] - o[i]) / d[i];
        if (ta > tb) std::swap(ta, tb);
        if (ta > t0) {
            t0 = ta;
            axis = i;
        }
        t1 = std::min(t1, tb);
        if (t0 > t1) return false;
    }
    if (axis < 0 || t0 <= 1e-6) return false;
    tHit = t0;
    n = Vec3::Zero();
    n[axis] = 1;
    return true;
}

}  // namespace

Renderer::Renderer(const FieldModel& field, const Camera& cam, const RenderOptions& opt)
    : field_(field), cam_(cam), opt_(opt) {
    int ss = std::max(1, opt_.supersample);
    rays_.resize((size_t)cam.width * cam.height * ss * ss);
    for (int v = 0; v < cam.height; ++v)
        for (int u = 0; u < cam.width; ++u)
            for (int sy = 0; sy < ss; ++sy)
                for (int sx = 0; sx < ss; ++sx) {
                    double pu = u + (sx + 0.5) / ss - 0.5, pv = v + (sy + 0.5) / ss - 0.5;
                    Vec2 n = cam.undistortPixel(pu, pv);
                    rays_[(((size_t)v * cam.width + u) * ss + sy) * ss + sx] = Vec3(n.x(), n.y(), 1).normalized();
                }
}

double Renderer::floorTexture(const Vec2& p) const {
    // Foam tile: mid gray, mottled at 1-6 cm, darker seams on the tile grid.
    double g = 0.42 + 0.10 * (valueNoise(p.x(), p.y(), 0.012, opt_.seed) - 0.5) +
               0.14 * (valueNoise(p.x(), p.y(), 0.05, opt_.seed + 3) - 0.5) +
               0.08 * (valueNoise(p.x(), p.y(), 0.25, opt_.seed + 5) - 0.5);
    // A few scuffs (dark dots) to give ORB something to hold on to.
    double sc = valueNoise(p.x(), p.y(), 0.03, opt_.seed + 11);
    if (sc > 0.82) g -= 0.18 * (sc - 0.82) / 0.18;
    double pitch = field_.tilePitchM > 0 ? field_.tilePitchM : 0.6096;
    double rx = std::fmod(std::abs(p.x() - field_.gridOrigin.x()), pitch);
    double ry = std::fmod(std::abs(p.y() - field_.gridOrigin.y()), pitch);
    double seam = 0.003;
    if (rx < seam || pitch - rx < seam || ry < seam || pitch - ry < seam) g *= 0.6;
    return g;
}

double Renderer::venue(const Vec3& p) const {
    // Venue: floor, walls and ceiling of a big hall with random banners and people.
    double a = std::atan2(p.y(), p.x());
    double g = 0.25 + 0.5 * blocks(a * 6.0, p.z(), 0.35, opt_.seed + 101);
    g += 0.15 * (valueNoise(a * 6.0, p.z(), 0.05, opt_.seed + 103) - 0.5);
    return g;
}

double Renderer::shade(const Vec3& o, const Vec3& d, HitKind* kind, Vec3* point) const {
    HitKind k = HitKind::None;
    double best = 1e9;
    double value = 0.3;
    Vec3 hitP;
    // Floor inside the perimeter (outside it the venue floor shows).
    if (d.z() < -1e-9) {
        double t = -o.z() / d.z();
        Vec3 p = o + t * d;
        if (t < best) {
            best = t;
            hitP = p;
            if (field_.insidePerimeter(Vec2(p.x(), p.y()))) {
                k = HitKind::Floor;
                value = floorTexture(Vec2(p.x(), p.y()));
            } else {
                k = HitKind::Venue;
                value = 0.3 + 0.2 * valueNoise(p.x(), p.y(), 0.4, opt_.seed + 201);
            }
        }
    }
    // Hive faces.
    for (const Surface& f : field_.hiveFaces) {
        double t = f.intersect(o, d);
        if (t > 0 && t < best) {
            best = t;
            hitP = o + t * d;
            k = HitKind::Hive;
            Vec3 q = hitP;
            value = f.textured ? 0.55 + 0.35 * (valueNoise(q.x() + q.z(), q.y() - q.z(), 0.02, opt_.seed + 301) - 0.5)
                               : 0.65;
        }
    }
    // Clutter boxes.
    for (const Box& b : opt_.clutter) {
        double t;
        Vec3 n;
        if (rayBox(o, d, b, t, n) && t < best) {
            best = t;
            hitP = o + t * d;
            k = HitKind::Clutter;
            double u = n.x() != 0 ? hitP.y() : hitP.x();
            value = 0.2 + 0.6 * blocks(u, hitP.z(), 0.05, b.seed);
        }
    }
    // Walls: opaque bands and posts, clear panels in between.
    int wi = -1;
    double tw = field_.wallHit(o, d, &wi);
    bool throughPanel = false;
    if (tw > 0 && tw < best) {
        const Wall& w = field_.walls[wi];
        Vec3 p = o + tw * d;
        Vec2 e = w.p1 - w.p0;
        double s = (Vec2(p.x(), p.y()) - w.p0).dot(e) / e.norm();
        bool post = false;
        for (const Vec2& q : w.posts)
            if (std::abs((q - w.p0).dot(e) / e.norm() - s) < opt_.postHalfWidthM) post = true;
        double railLo = w.railHeightM > 0 ? w.railHeightM : w.heightM - 0.025;
        if (post) {
            best = tw;
            hitP = p;
            k = HitKind::Post;
            value = 0.12;
        } else if (p.z() < opt_.bottomBandM || p.z() > railLo) {
            best = tw;
            hitP = p;
            k = HitKind::Wall;
            value = p.z() > railLo ? 0.15 : 0.18;
        } else {
            throughPanel = true;
        }
    }
    if (k == HitKind::None || (throughPanel && k == HitKind::Venue) || (best > 1e8)) {
        // Venue backdrop: box of half size 6 m, height 5 m.
        double t = 1e9;
        for (int i = 0; i < 2; ++i) {
            if (std::abs(d[i]) > 1e-12) {
                double ti = ((d[i] > 0 ? 6.0 : -6.0) - o[i]) / d[i];
                t = std::min(t, ti);
            }
        }
        if (d.z() > 1e-12) t = std::min(t, (5.0 - o.z()) / d.z());
        Vec3 p = o + t * d;
        if (k == HitKind::None || best > t) {
            k = HitKind::Venue;
            hitP = p;
            value = p.z() > 4.99 ? 0.6 + 0.3 * blocks(p.x(), p.y(), 1.0, opt_.seed + 401) : venue(p);
        }
    }
    if (throughPanel) value = 0.85 * value + 0.15 * 0.7;  // slight polycarbonate haze
    if (kind) *kind = k;
    if (point) *point = hitP;
    return value;
}

cv::Mat Renderer::render(const std::function<SE3(int)>& poseAtRow, uint32_t frameSeed) const {
    cv::Mat img(cam_.height, cam_.width, CV_8UC1);
    int ss = std::max(1, opt_.supersample);
    std::vector<SE3> rowPose(cam_.height);
    for (int v = 0; v < cam_.height; ++v) rowPose[v] = poseAtRow(v);
    cv::parallel_for_(cv::Range(0, cam_.height), [&](const cv::Range& r) {
        std::mt19937 rng(frameSeed * 7919u + (uint32_t)r.start);
        std::normal_distribution<double> noise(0.0, opt_.noiseSigma);
        for (int v = r.start; v < r.end; ++v) {
            const SE3& t = rowPose[v];
            Mat3 rot = t.linear();
            Vec3 o = t.translation();
            uint8_t* out = img.ptr<uint8_t>(v);
            for (int u = 0; u < cam_.width; ++u) {
                double acc = 0;
                for (int k = 0; k < ss * ss; ++k) {
                    Vec3 d = rot * rays_[((size_t)v * cam_.width + u) * ss * ss + k];
                    acc += shade(o, d, nullptr, nullptr);
                }
                double g = 255.0 * acc / (ss * ss) + noise(rng);
                out[u] = (uint8_t)std::min(255.0, std::max(0.0, std::round(g)));
            }
        }
    });
    return img;
}

HitKind Renderer::trueHit(const SE3& t, double u, double v, Vec3& point) const {
    Vec2 n = cam_.undistortPixel(u, v);
    Vec3 d = t.linear() * Vec3(n.x(), n.y(), 1).normalized();
    HitKind k;
    shade(t.translation(), d, &k, &point);
    return k;
}

// ----------------------------------------------------------------------------- trajectories

std::vector<TimedPose> makeTrajectory(const TrajOptions& o, double half, int64_t t0) {
    std::mt19937 rng(o.seed);
    std::uniform_real_distribution<double> uni(0, 1);
    double lim = half - o.marginM;
    // Keyframes of the motion: (time, pose); min-jerk interpolation in between.
    struct Key {
        double t;
        SE2 p;
    };
    std::vector<Key> keys;
    SE2 cur(uni(rng) * lim - lim / 2, uni(rng) * lim - lim / 2, (uni(rng) * 2 - 1) * M_PI);
    if (o.kind == TrajKind::Square) cur = SE2(-0.6, -0.6, 0);
    keys.push_back({0, cur});
    double t = 0;
    int leg = 0;
    while (t < o.durationS) {
        SE2 next = cur;
        double dur;
        if (o.kind == TrajKind::Still) {
            dur = o.durationS;
        } else if (o.kind == TrajKind::Spin) {
            // Alternate: a short drive, then an in-place spin at maxOmega.
            if (leg % 2 == 0) {
                next.x = (uni(rng) * 2 - 1) * lim * 0.7;
                next.y = (uni(rng) * 2 - 1) * lim * 0.7;
                dur = std::max(1.0, std::hypot(next.x - cur.x, next.y - cur.y) / (0.6 * o.maxSpeed) * 1.875);
            } else {
                double turn = (uni(rng) < 0.5 ? -1 : 1) * (M_PI + uni(rng) * 2 * M_PI);
                next.theta = cur.theta + turn;
                dur = std::abs(turn) / o.maxOmega * 1.875;  // min-jerk peak = 1.875 x mean
            }
        } else if (o.kind == TrajKind::Square) {
            static const double pts[4][2] = {{0.6, -0.6}, {0.6, 0.6}, {-0.6, 0.6}, {-0.6, -0.6}};
            next.x = pts[leg % 4][0];
            next.y = pts[leg % 4][1];
            next.theta = cur.theta + M_PI / 2;
            dur = 1.2 / (0.5 * o.maxSpeed) * 1.875;
        } else {
            next.x = (uni(rng) * 2 - 1) * lim;
            next.y = (uni(rng) * 2 - 1) * lim;
            double turn = (uni(rng) * 2 - 1) * M_PI;
            next.theta = cur.theta + turn;
            double dist = std::hypot(next.x - cur.x, next.y - cur.y);
            double speed = o.maxSpeed * (0.4 + 0.6 * uni(rng));
            dur = std::max({dist / speed * 1.875, std::abs(turn) / o.maxOmega * 1.875, 0.8});
        }
        double pause = (o.kind == TrajKind::Match && uni(rng) < 0.3) ? 0.5 + uni(rng) : 0.0;
        t += dur;
        keys.push_back({t, next});
        if (pause > 0) {
            t += pause;
            keys.push_back({t, next});
        }
        cur = next;
        ++leg;
    }
    std::vector<TimedPose> out;
    size_t n = (size_t)(o.durationS * o.rateHz) + 1;
    size_t k = 0;
    for (size_t i = 0; i < n; ++i) {
        double ti = i / o.rateHz;
        while (k + 2 < keys.size() && keys[k + 1].t < ti) ++k;
        const Key& a = keys[k];
        const Key& b = keys[std::min(k + 1, keys.size() - 1)];
        double u = b.t > a.t ? std::min(1.0, std::max(0.0, (ti - a.t) / (b.t - a.t))) : 1.0;
        double s = u * u * u * (10 - 15 * u + 6 * u * u);  // min-jerk
        SE2 p(a.p.x + s * (b.p.x - a.p.x), a.p.y + s * (b.p.y - a.p.y), a.p.theta + s * (b.p.theta - a.p.theta));
        out.push_back({t0 + (int64_t)std::llround(ti * 1e9), p});
    }
    return out;
}

SE2 sampleTrajectory(const std::vector<TimedPose>& tr, int64_t t) {
    if (tr.empty()) return SE2();
    if (t <= tr.front().tNs) return SE2(tr.front().pose.x, tr.front().pose.y, wrapAngle(tr.front().pose.theta));
    if (t >= tr.back().tNs) return SE2(tr.back().pose.x, tr.back().pose.y, wrapAngle(tr.back().pose.theta));
    auto it = std::upper_bound(tr.begin(), tr.end(), t, [](int64_t v, const TimedPose& p) { return v < p.tNs; });
    const TimedPose& b = *it;
    const TimedPose& a = *(it - 1);
    double u = (double)(t - a.tNs) / (double)(b.tNs - a.tNs);
    return SE2(a.pose.x + u * (b.pose.x - a.pose.x), a.pose.y + u * (b.pose.y - a.pose.y),
               wrapAngle(a.pose.theta + u * wrapAngle(b.pose.theta - a.pose.theta)));
}

std::vector<TimedPose> simulateOdometry(const std::vector<TimedPose>& truth, const OdomOptions& o) {
    std::vector<TimedPose> out;
    if (truth.empty()) return out;
    std::mt19937 rng(o.seed * 31u + 17u);
    std::normal_distribution<double> gauss(0, 1);
    std::uniform_real_distribution<double> uni(0, 1);
    double sx = 1 + o.scaleSigma * gauss(rng), sy = 1 + o.scaleSigma * gauss(rng);
    double walk = o.headingWalkDegPerMin * M_PI / 180 / std::sqrt(60.0);  // rad / sqrt(s)
    SE2 odom(0, 0, 0);
    double unwrapped = 0;
    out.push_back({truth[0].tNs, odom});
    for (size_t i = 1; i < truth.size(); ++i) {
        double dt = (truth[i].tNs - truth[i - 1].tNs) * 1e-9;
        SE2 a(truth[i - 1].pose.x, truth[i - 1].pose.y, truth[i - 1].pose.theta);
        SE2 b(truth[i].pose.x, truth[i].pose.y, truth[i].pose.theta);
        SE2 d = a.inverse() * b;
        double dth = truth[i].pose.theta - truth[i - 1].pose.theta;
        d.x *= sx;
        d.y *= sy;
        d.theta = dth + walk * std::sqrt(dt) * gauss(rng);
        if (o.slipProbPerS > 0 && uni(rng) < o.slipProbPerS * dt) {
            double mag = 0.01 + 0.02 * uni(rng);
            double dir = uni(rng) * 2 * M_PI;
            d.x += mag * std::cos(dir);
            d.y += mag * std::sin(dir);
        }
        unwrapped += d.theta;
        double c = std::cos(odom.theta), s = std::sin(odom.theta);
        odom = SE2(odom.x + c * d.x - s * d.y, odom.y + s * d.x + c * d.y, unwrapped);
        out.push_back({truth[i].tNs, SE2(odom.x, odom.y, wrapAngle(unwrapped))});
        odom.theta = unwrapped;
    }
    return out;
}

// ------------------------------------------------------------------------------ data sets

beacon_camera benchCamera(int w, int h, double hfovDeg) {
    beacon_camera c{};
    c.width = w;
    c.height = h;
    c.fx = c.fy = (w / 2.0) / std::tan(hfovDeg * M_PI / 360.0);
    c.cx = w / 2.0;
    c.cy = h / 2.0;
    return c;
}

std::string perimeterFieldJson() {
    // 12 ft square perimeter (inner half size 6 ft), 12 in walls with a 1 in top rail, posts
    // every 2 ft. Hand-made for the bench; NOT the official BIOBUZZ model (no hive).
    const double h = 1.8288;
    std::string posts[4];
    const double s[] = {-1.8288, -1.2192, -0.6096, 0.0, 0.6096, 1.2192, 1.8288};
    char buf[64];
    for (int w = 0; w < 4; ++w) {
        std::string p = "[";
        for (int i = 0; i < 7; ++i) {
            double x = 0, y = 0;
            if (w == 0) { x = s[i]; y = -h; }
            if (w == 1) { x = h; y = s[i]; }
            if (w == 2) { x = -s[i]; y = h; }
            if (w == 3) { x = -h; y = -s[i]; }
            std::snprintf(buf, sizeof buf, "%s[%.4f, %.4f]", i ? ", " : "", x, y);
            p += buf;
        }
        posts[w] = p + "]";
    }
    std::string j = "{\n  \"schema\": 1, \"season\": \"bench perimeter-only (not BIOBUZZ)\",\n"
                    "  \"source\": \"hand-made bench model, beacon-native/bench/render.cpp\",\n"
                    "  \"toleranceM\": 0.0254,\n"
                    "  \"floor\": {\"halfSizeM\": 1.8288, \"tilePitchM\": 0.6096, \"gridOriginM\": [0.0, 0.0]},\n"
                    "  \"walls\": [\n";
    const char* names[] = {"south", "east", "north", "west"};
    const double p0[4][2] = {{-h, -h}, {h, -h}, {h, h}, {-h, h}};
    const double p1[4][2] = {{h, -h}, {h, h}, {-h, h}, {-h, -h}};
    for (int w = 0; w < 4; ++w) {
        std::snprintf(buf, sizeof buf, "    {\"name\": \"%s\", ", names[w]);
        j += buf;
        std::snprintf(buf, sizeof buf, "\"p0\": [%.4f, %.4f], \"p1\": [%.4f, %.4f], ", p0[w][0], p0[w][1], p1[w][0], p1[w][1]);
        j += buf;
        j += "\"heightM\": 0.3048, \"railHeightM\": 0.2794,\n     \"posts\": " + posts[w] + "}";
        j += w < 3 ? ",\n" : "\n";
    }
    j += "  ],\n  \"hive\": null\n}\n";
    return j;
}

static bool mkdirs(const std::string& path) {
    std::string cur;
    for (size_t i = 0; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/') {
            if (!cur.empty()) mkdir(cur.c_str(), 0755);
        }
        if (i < path.size()) cur += path[i];
    }
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

bool makeSynthRun(const SynthOptions& o, const std::string& fieldJson, Dataset& out,
                  std::vector<cv::Mat>* images, const std::string& outDir, std::string& err) {
    FieldModel field;
    if (!FieldModel::parse(fieldJson, field, err)) return false;
    out = Dataset();
    out.camera = benchCamera(o.width, o.height, o.hfovDeg);
    out.camera.rs_readout_s = o.rsReadoutS;
    out.mount = beacon_mount{o.camForwardM, 0.0, o.camHeightM, 0.0, o.camPitchDeg * M_PI / 180, 0.0};
    out.fieldJson = fieldJson;
    const int64_t t0 = 1000000000LL;  // 1 s, so times are clearly not zero
    std::vector<TimedPose> truth = makeTrajectory(o.traj, field.halfSizeM, t0);
    std::vector<TimedPose> odom = simulateOdometry(truth, o.odom);
    for (const TimedPose& p : truth) out.truth.push_back({p.tNs, p.pose.x, p.pose.y, wrapAngle(p.pose.theta)});
    if (o.withOdom)
        for (const TimedPose& p : odom) out.odom.push_back({p.tNs, p.pose.x, p.pose.y, wrapAngle(p.pose.theta)});
    SE2 s0 = truth.front().pose;
    out.start.push_back({truth.front().tNs, s0.x + o.startErrorM, s0.y,
                         wrapAngle(s0.theta + o.startErrorDeg * M_PI / 180)});

    Camera cam;
    cam.width = out.camera.width;
    cam.height = out.camera.height;
    cam.fx = out.camera.fx;
    cam.fy = out.camera.fy;
    cam.cx = out.camera.cx;
    cam.cy = out.camera.cy;
    SE3 tRobotCam = mountToTransform(out.mount.x, out.mount.y, out.mount.z, out.mount.roll, out.mount.pitch, out.mount.yaw);
    Renderer r(field, cam, o.render);
    if (!outDir.empty()) {
        if (!mkdirs(outDir + "/frames")) {
            err = "cannot create " + outDir;
            return false;
        }
    }
    int nFrames = (int)std::floor(o.traj.durationS * o.fps);
    FILE* fcsv = nullptr;
    if (!outDir.empty()) {
        fcsv = std::fopen((outDir + "/frames.csv").c_str(), "w");
        std::fprintf(fcsv, "idx,t_ns\n");
    }
    for (int i = 0; i < nFrames; ++i) {
        int64_t t = t0 + (int64_t)std::llround((i + 0.5) / o.fps * 1e9);
        // Capture time = first row's exposure; later rows are read out rsReadoutS later.
        auto poseAtRow = [&](int row) {
            int64_t tr = t + (int64_t)std::llround(o.rsReadoutS * 1e9 * row / std::max(1, cam.height - 1));
            return cameraInField(sampleTrajectory(truth, tr), tRobotCam);
        };
        cv::Mat img = r.render(poseAtRow, (uint32_t)(o.render.seed * 100003u + i));
        FrameRow fr{i, t, ""};
        if (!outDir.empty()) {
            char name[32];
            std::snprintf(name, sizeof name, "%06d.png", i);
            fr.path = outDir + "/frames/" + name;
            cv::imwrite(fr.path, img);
            std::fprintf(fcsv, "%d,%lld\n", i, (long long)t);
        }
        out.frames.push_back(fr);
        if (images) images->push_back(img);
    }
    if (fcsv) std::fclose(fcsv);
    if (!outDir.empty()) {
        out.dir = outDir;
        if (!writeCameraJson(outDir + "/camera.json", out.camera, out.mount, err)) return false;
        std::ofstream(outDir + "/field.json") << fieldJson;
        if (o.withOdom && !writePoseCsv(outDir + "/odom.csv", out.odom, err)) return false;
        if (!writePoseCsv(outDir + "/start.csv", out.start, err)) return false;
        if (!writePoseCsv(outDir + "/truth.csv", out.truth, err)) return false;
    }
    return true;
}

}  // namespace beacon
