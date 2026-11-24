// beacon_make_testdata: writes a minimal synthetic I2 data set (perimeter-only bench field)
// until the Field engineer's tools/sim/make_dataset.py is available.
//
//   beacon_make_testdata --out DIR [--traj match|spin|still|square|clutter] [--duration S]
//       [--seed N] [--fps F] [--cam-height M] [--cam-pitch DEG] [--max-speed MPS]
//       [--max-omega RADPS] [--start-error-cm CM] [--start-error-deg DEG] [--no-odom]
//       [--rs-readout-s S] [--field FIELD.json]
#include <cstdio>
#include <string>

#include "dataset.h"
#include "render.h"

using namespace beacon;

int main(int argc, char** argv) {
    SynthOptions o;
    std::string out, fieldPath;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", a.c_str());
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--out") out = next();
        else if (a == "--traj") {
            std::string t = next();
            if (t == "match") o.traj.kind = TrajKind::Match;
            else if (t == "spin") o.traj.kind = TrajKind::Spin;
            else if (t == "still") o.traj.kind = TrajKind::Still;
            else if (t == "square") o.traj.kind = TrajKind::Square;
            else if (t == "clutter") {
                o.traj.kind = TrajKind::Match;
                o.render.clutter = {{Vec3(0.3, -0.2, 0), Vec3(0.75, 0.25, 0.35), 5}, {Vec3(-1.0, 0.8, 0), Vec3(-0.6, 1.2, 0.3), 6}};
            } else {
                std::fprintf(stderr, "unknown --traj %s\n", t.c_str());
                return 2;
            }
        } else if (a == "--duration") o.traj.durationS = std::stod(next());
        else if (a == "--seed") {
            o.traj.seed = o.odom.seed = o.render.seed = (uint32_t)std::stoul(next());
        } else if (a == "--fps") o.fps = std::stod(next());
        else if (a == "--cam-height") o.camHeightM = std::stod(next());
        else if (a == "--cam-pitch") o.camPitchDeg = std::stod(next());
        else if (a == "--max-speed") o.traj.maxSpeed = std::stod(next());
        else if (a == "--max-omega") o.traj.maxOmega = std::stod(next());
        else if (a == "--start-error-cm") o.startErrorM = std::stod(next()) / 100.0;
        else if (a == "--start-error-deg") o.startErrorDeg = std::stod(next());
        else if (a == "--rs-readout-s") o.rsReadoutS = std::stod(next());
        else if (a == "--no-odom") o.withOdom = false;
        else if (a == "--field") fieldPath = next();
        else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
    }
    if (out.empty()) {
        std::fprintf(stderr, "usage: beacon_make_testdata --out DIR [options]\n");
        return 2;
    }
    std::string field = fieldPath.empty() ? perimeterFieldJson() : readFile(fieldPath);
    Dataset d;
    std::string err;
    if (!makeSynthRun(o, field, d, nullptr, out, err)) {
        std::fprintf(stderr, "error: %s\n", err.c_str());
        return 1;
    }
    std::printf("wrote %s: %zu frames, %zu odom, %zu truth\n", out.c_str(), d.frames.size(), d.odom.size(),
                d.truth.size());
    return 0;
}
