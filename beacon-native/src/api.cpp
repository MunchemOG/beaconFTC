// The C API (I3): argument checking, the frame hand-off to the worker thread, and the fix
// slot. Everything numeric is checked for finiteness here, before it reaches the engine.
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <thread>

#include "beacon/beacon.h"
#include "engine.h"

using namespace beacon;

struct beacon_ctx {
    explicit beacon_ctx(const Params& p) : shared(p) {}
    Shared shared;
    FieldModel field;
    bool hasField = false;
    std::unique_ptr<Engine> engine;

    // Frame hand-off: one pending slot. The worker swaps it out under frameMu.
    std::mutex frameMu;
    std::condition_variable frameCv;
    cv::Mat pending, working;
    int64_t pendingT = 0;
    bool hasPending = false;
    bool stop = false;
    std::thread worker;
    std::mutex engineMu;  // serializes engine calls (worker vs synchronous mode)
};

namespace {

void writeErr(char* err, int len, const std::string& msg) {
    if (err == nullptr || len <= 0) return;
    std::snprintf(err, (size_t)len, "%s", msg.c_str());
}

bool isFiniteNum(double v) { return std::isfinite(v); }

void runEngine(beacon_ctx* c, int64_t t, const cv::Mat& img) {
    std::lock_guard<std::mutex> lock(c->engineMu);
    auto t0 = std::chrono::steady_clock::now();
    try {
        c->engine->process(t, img, c->shared);
    } catch (const std::exception& e) {
        std::lock_guard<std::mutex> l(c->shared.mu);
        c->shared.debug = std::string("beacon: frame failed: ") + e.what();
    } catch (...) {
        std::lock_guard<std::mutex> l(c->shared.mu);
        c->shared.debug = "beacon: frame failed (unknown error)";
    }
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    std::lock_guard<std::mutex> l(c->shared.mu);
    c->shared.framesDone++;
    c->shared.lastProcessMs = ms;
}

void workerLoop(beacon_ctx* c) {
    for (;;) {
        int64_t t;
        {
            std::unique_lock<std::mutex> lock(c->frameMu);
            c->frameCv.wait(lock, [c] { return c->stop || c->hasPending; });
            if (c->stop) return;
            std::swap(c->pending, c->working);
            t = c->pendingT;
            c->hasPending = false;
        }
        runEngine(c, t, c->working);
    }
}

}  // namespace

namespace beacon {
void publishFix(Shared& s, const beacon_fix& fix) {
    std::lock_guard<std::mutex> lock(s.mu);
    s.fix = fix;
    s.hasFix = true;
    s.fixNew = true;
    s.fixes++;
}
}  // namespace beacon

extern "C" beacon_ctx* beacon_create(const char* fieldJson, const beacon_camera* cam,
                                     const beacon_mount* mount, const char* paramsJson, char* err,
                                     int errLen) {
    try {
        Params params;
        std::string msg;
        if (!Params::parse(paramsJson, params, msg)) {
            writeErr(err, errLen, msg);
            return nullptr;
        }
        if (cam == nullptr || mount == nullptr) {
            writeErr(err, errLen, "camera and mount must not be NULL");
            return nullptr;
        }
        const double camNums[] = {cam->fx, cam->fy, cam->cx, cam->cy, cam->dist[0], cam->dist[1],
                                  cam->dist[2], cam->dist[3], cam->dist[4], cam->rs_readout_s};
        for (double v : camNums) {
            if (!isFiniteNum(v)) {
                writeErr(err, errLen, "camera has a non-finite number");
                return nullptr;
            }
        }
        if (cam->width < 16 || cam->height < 16 || cam->width > 4096 || cam->height > 4096 ||
            cam->fx <= 0 || cam->fy <= 0) {
            writeErr(err, errLen, "camera size or focal length out of range");
            return nullptr;
        }
        const double mountNums[] = {mount->x, mount->y, mount->z, mount->roll, mount->pitch, mount->yaw};
        for (double v : mountNums) {
            if (!isFiniteNum(v)) {
                writeErr(err, errLen, "mount has a non-finite number");
                return nullptr;
            }
        }
        auto c = std::make_unique<beacon_ctx>(params);
        if (fieldJson != nullptr) {
            if (!FieldModel::parse(fieldJson, c->field, msg)) {
                writeErr(err, errLen, msg);
                return nullptr;
            }
            c->hasField = true;
        } else if (params.mode == Mode::Field) {
            writeErr(err, errLen, "field mode needs a field model (field_json is NULL)");
            return nullptr;
        }
        Camera& k = c->shared.camera;
        k.width = cam->width;
        k.height = cam->height;
        k.fx = cam->fx;
        k.fy = cam->fy;
        k.cx = cam->cx;
        k.cy = cam->cy;
        k.k1 = cam->dist[0];
        k.k2 = cam->dist[1];
        k.p1 = cam->dist[2];
        k.p2 = cam->dist[3];
        k.k3 = cam->dist[4];
        k.rsReadoutS = cam->rs_readout_s;
        c->shared.tRobotCam = mountToTransform(mount->x, mount->y, mount->z, mount->roll, mount->pitch, mount->yaw);
        c->engine = params.mode == Mode::Field ? makeFieldEngine(c->field, c->shared)
                                               : makeExploreEngine(c->hasField ? &c->field : nullptr, c->shared);
        c->pending.create(cam->height, cam->width, CV_8UC1);
        c->working.create(cam->height, cam->width, CV_8UC1);
        if (!params.synchronous) {
            beacon_ctx* raw = c.get();
            c->worker = std::thread([raw] { workerLoop(raw); });
        }
        return c.release();
    } catch (const std::exception& e) {
        writeErr(err, errLen, std::string("beacon_create failed: ") + e.what());
        return nullptr;
    } catch (...) {
        writeErr(err, errLen, "beacon_create failed");
        return nullptr;
    }
}

extern "C" void beacon_set_start_pose(beacon_ctx* c, const beacon_pose2* p) {
    if (c == nullptr || p == nullptr) return;
    if (!isFiniteNum(p->x) || !isFiniteNum(p->y) || !isFiniteNum(p->heading)) return;
    std::lock_guard<std::mutex> lock(c->shared.mu);
    c->shared.start = *p;
    c->shared.hasStart = true;
    c->shared.startSeq++;
}

extern "C" void beacon_push_odom(beacon_ctx* c, const beacon_pose2* p) {
    if (c == nullptr || p == nullptr) return;
    if (!isFiniteNum(p->x) || !isFiniteNum(p->y) || !isFiniteNum(p->heading)) return;
    c->shared.odom.push(p->t_ns, SE2(p->x, p->y, p->heading));
}

extern "C" int beacon_push_frame(beacon_ctx* c, int64_t t, const uint8_t* gray, int w, int h,
                                 int stride) {
    if (c == nullptr || gray == nullptr) return BEACON_ERR;
    if (w != c->shared.camera.width || h != c->shared.camera.height || stride < w) return BEACON_ERR;
    {
        std::lock_guard<std::mutex> l(c->shared.mu);
        c->shared.framesIn++;
    }
    if (c->shared.params.synchronous) {
        for (int r = 0; r < h; ++r) std::memcpy(c->working.ptr(r), gray + (size_t)r * stride, (size_t)w);
        runEngine(c, t, c->working);
        return BEACON_OK;
    }
    std::unique_lock<std::mutex> lock(c->frameMu, std::try_to_lock);
    if (!lock.owns_lock() || c->hasPending) {
        lock = std::unique_lock<std::mutex>();
        std::lock_guard<std::mutex> l(c->shared.mu);
        c->shared.framesDropped++;
        return BEACON_DROPPED;
    }
    for (int r = 0; r < h; ++r) std::memcpy(c->pending.ptr(r), gray + (size_t)r * stride, (size_t)w);
    c->pendingT = t;
    c->hasPending = true;
    lock.unlock();
    c->frameCv.notify_one();
    return BEACON_OK;
}

extern "C" int beacon_latest_fix(beacon_ctx* c, beacon_fix* out) {
    if (c == nullptr || out == nullptr) return 0;
    std::lock_guard<std::mutex> lock(c->shared.mu);
    if (!c->shared.hasFix) return 0;
    *out = c->shared.fix;
    int isNew = c->shared.fixNew ? 1 : 0;
    c->shared.fixNew = false;
    return isNew;
}

extern "C" int beacon_status(beacon_ctx* c) {
    if (c == nullptr) return BEACON_STATUS_INVALID;
    std::lock_guard<std::mutex> lock(c->shared.mu);
    return c->shared.status;
}

extern "C" void beacon_debug_line(beacon_ctx* c, char* buf, int len) {
    if (buf == nullptr || len <= 0) return;
    if (c == nullptr) {
        std::snprintf(buf, (size_t)len, "beacon: no context");
        return;
    }
    std::lock_guard<std::mutex> lock(c->shared.mu);
    std::snprintf(buf, (size_t)len, "%s | frames %llu in, %llu dropped, %llu done, %.1f ms, fixes %llu",
                  c->shared.debug.c_str(), (unsigned long long)c->shared.framesIn,
                  (unsigned long long)c->shared.framesDropped, (unsigned long long)c->shared.framesDone,
                  c->shared.lastProcessMs, (unsigned long long)c->shared.fixes);
}

extern "C" void beacon_destroy(beacon_ctx* c) {
    if (c == nullptr) return;
    {
        std::lock_guard<std::mutex> lock(c->frameMu);
        c->stop = true;
    }
    c->frameCv.notify_all();
    if (c->worker.joinable()) c->worker.join();
    delete c;
}

void beacon_set_observer(beacon_ctx* c, VizObserver observer) {
    if (c == nullptr) return;
    std::lock_guard<std::mutex> lock(c->engineMu);
    c->engine->setObserver(std::move(observer));
}
