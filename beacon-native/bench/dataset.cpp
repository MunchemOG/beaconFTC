#include "dataset.h"

#include <sys/stat.h>

#include <cmath>
#include <cstdio>
#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>

namespace beacon {

using json = nlohmann::json;

std::string readFile(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return "";
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

bool fileExists(const std::string& path) {
    struct stat st;
    return stat(path.c_str(), &st) == 0;
}

static std::vector<std::string> splitCsv(const std::string& line) {
    std::vector<std::string> out;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        while (!cell.empty() && (cell.back() == '\r' || cell.back() == ' ')) cell.pop_back();
        while (!cell.empty() && cell.front() == ' ') cell.erase(cell.begin());
        out.push_back(cell);
    }
    return out;
}

bool readPoseCsv(const std::string& path, std::vector<PoseRow>& rows, std::string& err) {
    rows.clear();
    std::ifstream f(path);
    if (!f) {
        err = "cannot open " + path;
        return false;
    }
    std::string line;
    int n = 0;
    while (std::getline(f, line)) {
        ++n;
        if (line.empty() || line[0] == '#' || (n == 1 && line.find("t_ns") != std::string::npos)) continue;
        auto c = splitCsv(line);
        if (c.size() < 4) {
            err = path + ":" + std::to_string(n) + ": expected t_ns,x,y,heading";
            return false;
        }
        rows.push_back({std::stoll(c[0]), std::stod(c[1]), std::stod(c[2]), std::stod(c[3])});
    }
    return true;
}

bool writePoseCsv(const std::string& path, const std::vector<PoseRow>& rows, std::string& err) {
    FILE* f = std::fopen(path.c_str(), "w");
    if (!f) {
        err = "cannot write " + path;
        return false;
    }
    std::fprintf(f, "t_ns,x,y,heading\n");
    for (const PoseRow& r : rows) std::fprintf(f, "%lld,%.6f,%.6f,%.6f\n", (long long)r.tNs, r.x, r.y, r.heading);
    std::fclose(f);
    return true;
}

bool writeCameraJson(const std::string& path, const beacon_camera& c, const beacon_mount& m, std::string& err) {
    json j;
    j["width"] = c.width;
    j["height"] = c.height;
    j["fx"] = c.fx;
    j["fy"] = c.fy;
    j["cx"] = c.cx;
    j["cy"] = c.cy;
    j["dist"] = {c.dist[0], c.dist[1], c.dist[2], c.dist[3], c.dist[4]};
    j["mount"] = {{"x", m.x}, {"y", m.y}, {"z", m.z}, {"roll", m.roll}, {"pitch", m.pitch}, {"yaw", m.yaw}};
    j["rollingShutterReadoutS"] = c.rs_readout_s;
    std::ofstream f(path);
    if (!f) {
        err = "cannot write " + path;
        return false;
    }
    f << j.dump(2) << "\n";
    return true;
}

static bool parseCamera(const std::string& text, beacon_camera& c, beacon_mount& m, std::string& err) {
    json j = json::parse(text, nullptr, false, true);
    if (j.is_discarded() || !j.is_object()) {
        err = "camera.json: not a JSON object";
        return false;
    }
    try {
        c.width = j.at("width").get<int>();
        c.height = j.at("height").get<int>();
        c.fx = j.at("fx").get<double>();
        c.fy = j.at("fy").get<double>();
        c.cx = j.at("cx").get<double>();
        c.cy = j.at("cy").get<double>();
        for (int i = 0; i < 5; ++i) c.dist[i] = 0;
        if (j.contains("dist"))
            for (size_t i = 0; i < j["dist"].size() && i < 5; ++i) c.dist[i] = j["dist"][i].get<double>();
        c.rs_readout_s = j.value("rollingShutterReadoutS", 0.0);
        m = beacon_mount{0, 0, 0, 0, 0, 0};
        if (j.contains("mount")) {
            const json& mj = j["mount"];
            m.x = mj.value("x", 0.0);
            m.y = mj.value("y", 0.0);
            m.z = mj.value("z", 0.0);
            m.roll = mj.value("roll", 0.0);
            m.pitch = mj.value("pitch", 0.0);
            m.yaw = mj.value("yaw", 0.0);
        }
    } catch (const std::exception& e) {
        err = std::string("camera.json: ") + e.what();
        return false;
    }
    return true;
}

bool loadCameraJson(const std::string& path, beacon_camera& cam, beacon_mount& mount, std::string& err) {
    std::string text = readFile(path);
    if (text.empty()) {
        err = "cannot read " + path;
        return false;
    }
    return parseCamera(text, cam, mount, err);
}

bool loadDataset(const std::string& dir, Dataset& d, std::string& err) {
    d = Dataset();
    d.dir = dir;
    std::string cam = readFile(dir + "/camera.json");
    if (cam.empty()) {
        err = "missing " + dir + "/camera.json";
        return false;
    }
    if (!parseCamera(cam, d.camera, d.mount, err)) return false;
    d.fieldJson = readFile(dir + "/field.json");
    std::ifstream f(dir + "/frames.csv");
    if (!f) {
        err = "missing " + dir + "/frames.csv";
        return false;
    }
    std::string line;
    int n = 0;
    while (std::getline(f, line)) {
        ++n;
        if (line.empty() || line[0] == '#' || (n == 1 && line.find("idx") != std::string::npos)) continue;
        auto c = splitCsv(line);
        if (c.size() < 2) {
            err = "frames.csv:" + std::to_string(n) + ": expected idx,t_ns";
            return false;
        }
        FrameRow r;
        r.idx = std::stoi(c[0]);
        r.tNs = std::stoll(c[1]);
        char name[32];
        std::snprintf(name, sizeof name, "%06d.png", r.idx);
        r.path = dir + "/frames/" + name;
        d.frames.push_back(r);
    }
    if (fileExists(dir + "/odom.csv") && !readPoseCsv(dir + "/odom.csv", d.odom, err)) return false;
    if (fileExists(dir + "/start.csv") && !readPoseCsv(dir + "/start.csv", d.start, err)) return false;
    if (fileExists(dir + "/truth.csv") && !readPoseCsv(dir + "/truth.csv", d.truth, err)) return false;
    if (fileExists(dir + "/marks.csv") && !readPoseCsv(dir + "/marks.csv", d.marks, err)) return false;
    return true;
}

// Reads the "data: [...]" list of a EuRoC sensor.yaml key (flow style).
static std::vector<double> yamlList(const std::string& text, const std::string& key) {
    std::vector<double> out;
    size_t k = text.find(key);
    if (k == std::string::npos) return out;
    size_t a = text.find('[', k), b = text.find(']', a);
    if (a == std::string::npos || b == std::string::npos) return out;
    std::string body = text.substr(a + 1, b - a - 1);
    for (char& ch : body)
        if (ch == '\n') ch = ' ';
    for (const std::string& s : splitCsv(body))
        if (!s.empty()) out.push_back(std::stod(s));
    return out;
}

bool loadEuroc(const std::string& dir, Dataset& d, std::string& err) {
    d = Dataset();
    d.dir = dir;
    std::string base = dir + "/mav0/cam0";
    std::string yaml = readFile(base + "/sensor.yaml");
    if (yaml.empty()) {
        err = "missing " + base + "/sensor.yaml";
        return false;
    }
    auto intr = yamlList(yaml, "intrinsics:");
    auto dist = yamlList(yaml, "distortion_coefficients:");
    auto res = yamlList(yaml, "resolution:");
    auto tbs = yamlList(yaml, "data:");
    if (intr.size() != 4 || dist.size() < 4 || res.size() != 2 || tbs.size() != 16) {
        err = "cannot parse " + base + "/sensor.yaml";
        return false;
    }
    d.camera.width = (int)res[0];
    d.camera.height = (int)res[1];
    d.camera.fx = intr[0];
    d.camera.fy = intr[1];
    d.camera.cx = intr[2];
    d.camera.cy = intr[3];
    for (int i = 0; i < 5; ++i) d.camera.dist[i] = i < (int)dist.size() ? dist[i] : 0.0;
    Eigen::Matrix4d m;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) m(r, c) = tbs[r * 4 + c];
    d.tBodyCam.matrix() = m;
    std::ifstream f(base + "/data.csv");
    std::string line;
    int idx = 0;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto c = splitCsv(line);
        if (c.size() < 2) continue;
        d.frames.push_back({idx++, std::stoll(c[0]), base + "/data/" + c[1]});
    }
    std::ifstream g(dir + "/mav0/state_groundtruth_estimate0/data.csv");
    while (std::getline(g, line)) {
        if (line.empty() || line[0] == '#') continue;
        auto c = splitCsv(line);
        if (c.size() < 8) continue;
        Dataset::Pose6 p;
        p.tNs = std::stoll(c[0]);
        p.p = Vec3(std::stod(c[1]), std::stod(c[2]), std::stod(c[3]));
        p.q = Eigen::Quaterniond(std::stod(c[4]), std::stod(c[5]), std::stod(c[6]), std::stod(c[7]));
        d.truth6.push_back(p);
    }
    if (d.frames.empty()) {
        err = "no frames in " + base;
        return false;
    }
    return true;
}

}  // namespace beacon
