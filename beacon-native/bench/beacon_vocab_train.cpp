// beacon_vocab_train: trains Beacon's own ORB vocabulary (k-majority tree) from images.
//
//   beacon_vocab_train --out vocab.bin [--euroc DIR]... [--run DIR]... [--every N] [--k 10] [--levels 4]
// For the robot, train on practice-field images (I2 runs); for the EuRoC bench, train on a
// sequence that is NOT the one you test on (we train on V1_01 and test on MH_01).
#include <cstdio>
#include <opencv2/imgcodecs.hpp>
#include <string>
#include <vector>

#include "dataset.h"
#include "features.h"
#include "vocab.h"

using namespace beacon;

int main(int argc, char** argv) {
    std::string out;
    std::vector<std::string> euroc, runs;
    int every = 5, k = 10, levels = 4;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (i + 1 >= argc) {
            std::fprintf(stderr, "missing value for %s\n", a.c_str());
            return 2;
        }
        std::string v = argv[++i];
        if (a == "--out") out = v;
        else if (a == "--euroc") euroc.push_back(v);
        else if (a == "--run") runs.push_back(v);
        else if (a == "--every") every = std::stoi(v);
        else if (a == "--k") k = std::stoi(v);
        else if (a == "--levels") levels = std::stoi(v);
        else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            return 2;
        }
    }
    if (out.empty() || (euroc.empty() && runs.empty())) {
        std::fprintf(stderr, "usage: beacon_vocab_train --out vocab.bin (--euroc DIR | --run DIR)... [--every N]\n");
        return 2;
    }
    std::vector<cv::Mat> descs;
    Params p;
    for (int pass = 0; pass < 2; ++pass) {
        const std::vector<std::string>& dirs = pass == 0 ? euroc : runs;
        for (const std::string& d : dirs) {
            Dataset ds;
            std::string err;
            if (!(pass == 0 ? loadEuroc(d, ds, err) : loadDataset(d, ds, err))) {
                std::fprintf(stderr, "error: %s\n", err.c_str());
                return 1;
            }
            Camera cam;
            cam.width = ds.camera.width;
            cam.height = ds.camera.height;
            cam.fx = ds.camera.fx;
            cam.fy = ds.camera.fy;
            cam.cx = ds.camera.cx;
            cam.cy = ds.camera.cy;
            OrbExtractor orb(p, cam);
            for (size_t i = 0; i < ds.frames.size(); i += (size_t)every) {
                cv::Mat img = cv::imread(ds.frames[i].path, cv::IMREAD_GRAYSCALE);
                if (img.empty()) continue;
                Features f;
                orb.extract(img, f);
                if (f.desc.rows > 0) descs.push_back(f.desc.clone());
            }
            std::printf("%s: %zu images so far\n", d.c_str(), descs.size());
        }
    }
    Vocabulary voc;
    voc.train(descs, k, levels, 1);
    if (!voc.save(out)) {
        std::fprintf(stderr, "error: cannot write %s\n", out.c_str());
        return 1;
    }
    std::printf("wrote %s: %d words from %zu images\n", out.c_str(), voc.words(), descs.size());
    return 0;
}
