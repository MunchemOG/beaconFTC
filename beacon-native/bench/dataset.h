// Dataset directory (shared interface I2) reader/writer, and a EuRoC cam0 reader. Bench only.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "beacon/beacon.h"
#include "geometry.h"

namespace beacon {

struct PoseRow {
    int64_t tNs;
    double x, y, heading;
};

struct FrameRow {
    int idx;
    int64_t tNs;
    std::string path;  // absolute path of the PNG
};

struct Dataset {
    std::string dir;
    beacon_camera camera{};
    beacon_mount mount{};
    std::string fieldJson;  // empty when the run has no field.json
    std::vector<FrameRow> frames;
    std::vector<PoseRow> odom, start, truth, marks;
    // EuRoC only: full 6-DoF ground truth of the body (t, position, quaternion w x y z).
    struct Pose6 {
        int64_t tNs;
        Vec3 p;
        Eigen::Quaterniond q;
    };
    std::vector<Pose6> truth6;
    SE3 tBodyCam = SE3::Identity();  // EuRoC: cam0 in the body (IMU) frame
};

/** Loads an I2 directory. Missing optional files leave their vectors empty. */
bool loadDataset(const std::string& dir, Dataset& out, std::string& err);

/** Loads a EuRoC sequence (the directory that contains mav0/), cam0 only. */
bool loadEuroc(const std::string& dir, Dataset& out, std::string& err);

/** Reads camera.json (I2 layout). */
bool loadCameraJson(const std::string& path, beacon_camera& cam, beacon_mount& mount, std::string& err);

/** Writes camera.json (I2 layout). */
bool writeCameraJson(const std::string& path, const beacon_camera& cam, const beacon_mount& mount,
                     std::string& err);

bool writePoseCsv(const std::string& path, const std::vector<PoseRow>& rows, std::string& err);
bool readPoseCsv(const std::string& path, std::vector<PoseRow>& rows, std::string& err);

std::string readFile(const std::string& path);
bool fileExists(const std::string& path);

}  // namespace beacon
