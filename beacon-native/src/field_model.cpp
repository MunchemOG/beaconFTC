#include "field_model.h"

#include <cmath>
#include <nlohmann/json.hpp>

namespace beacon {

namespace {

using json = nlohmann::json;

bool num(const json& j, const char* key, double& out, std::string& err, const std::string& where) {
    if (!j.contains(key) || !j[key].is_number() || !std::isfinite(j[key].get<double>())) {
        err = "field model: " + where + "." + key + " missing or not a finite number";
        return false;
    }
    out = j[key].get<double>();
    return true;
}

bool vecN(const json& j, int n, double* out, std::string& err, const std::string& where) {
    if (!j.is_array() || (int)j.size() != n) {
        err = "field model: " + where + " must be an array of " + std::to_string(n) + " numbers";
        return false;
    }
    for (int i = 0; i < n; ++i) {
        if (!j[i].is_number() || !std::isfinite(j[i].get<double>())) {
            err = "field model: " + where + " has a non-finite entry";
            return false;
        }
        out[i] = j[i].get<double>();
    }
    return true;
}

bool makePlane(Surface& s, std::string& err) {
    if (s.vertices.size() < 3) {
        err = "field model: surface \"" + s.name + "\" needs at least 3 vertices";
        return false;
    }
    Vec3 n = Vec3::Zero();
    // Newell's method: robust normal for a (nearly) planar polygon.
    for (size_t i = 0; i < s.vertices.size(); ++i) {
        const Vec3& a = s.vertices[i];
        const Vec3& b = s.vertices[(i + 1) % s.vertices.size()];
        n += Vec3((a.y() - b.y()) * (a.z() + b.z()), (a.z() - b.z()) * (a.x() + b.x()),
                  (a.x() - b.x()) * (a.y() + b.y()));
    }
    if (n.norm() < 1e-9) {
        err = "field model: surface \"" + s.name + "\" is degenerate";
        return false;
    }
    s.normal = n.normalized();
    Vec3 c = Vec3::Zero();
    for (const Vec3& v : s.vertices) c += v;
    c /= (double)s.vertices.size();
    s.d = -s.normal.dot(c);
    return true;
}

}  // namespace

Vec3 MovingPart::moved(const Vec3& p, int s) const {
    double ang = -(tilts[s] - startTilt);
    return point + Eigen::AngleAxisd(ang, axis.normalized()) * (p - point);
}

double Surface::intersect(const Vec3& o, const Vec3& dir) const {
    double den = normal.dot(dir);
    if (std::abs(den) < 1e-12) return -1;
    double t = -(normal.dot(o) + d) / den;
    if (t <= 0) return -1;
    return contains(o + t * dir) ? t : -1;
}

bool Surface::contains(const Vec3& p) const {
    // Convex polygon: p is inside when it is on the same side of every edge.
    int sign = 0;
    for (size_t i = 0; i < vertices.size(); ++i) {
        const Vec3& a = vertices[i];
        const Vec3& b = vertices[(i + 1) % vertices.size()];
        double s = normal.dot((b - a).cross(p - a));
        int sg = s > 1e-12 ? 1 : (s < -1e-12 ? -1 : 0);
        if (sg == 0) continue;
        if (sign == 0) sign = sg;
        else if (sg != sign) return false;
    }
    return true;
}

bool FieldModel::parse(const std::string& text, FieldModel& m, std::string& err) {
    m = FieldModel();
    json doc = json::parse(text, nullptr, false, true);
    if (doc.is_discarded() || !doc.is_object()) {
        err = "field model: not a JSON object";
        return false;
    }
    if (!doc.contains("schema") || !doc["schema"].is_number_integer() || doc["schema"].get<int>() != 1) {
        err = "field model: \"schema\" must be 1";
        return false;
    }
    if (doc.contains("season") && doc["season"].is_string()) m.season = doc["season"].get<std::string>();
    if (doc.contains("toleranceM") && !num(doc, "toleranceM", m.toleranceM, err, "root")) return false;

    if (!doc.contains("floor") || !doc["floor"].is_object()) {
        err = "field model: \"floor\" missing";
        return false;
    }
    const json& fl = doc["floor"];
    if (!num(fl, "halfSizeM", m.halfSizeM, err, "floor") || !num(fl, "tilePitchM", m.tilePitchM, err, "floor"))
        return false;
    if (m.halfSizeM < 0.5 || m.halfSizeM > 5) {
        err = "field model: floor.halfSizeM out of range";
        return false;
    }
    if (fl.contains("gridOriginM") && !vecN(fl["gridOriginM"], 2, m.gridOrigin.data(), err, "floor.gridOriginM"))
        return false;
    double h = m.halfSizeM;
    m.floor.name = "floor";
    m.floor.isFloor = true;
    m.floor.vertices = {Vec3(-h, -h, 0), Vec3(h, -h, 0), Vec3(h, h, 0), Vec3(-h, h, 0)};
    if (!makePlane(m.floor, err)) return false;

    if (!doc.contains("walls") || !doc["walls"].is_array() || doc["walls"].size() < 3) {
        err = "field model: \"walls\" must list at least 3 walls";
        return false;
    }
    for (size_t i = 0; i < doc["walls"].size(); ++i) {
        const json& wj = doc["walls"][i];
        std::string where = "walls[" + std::to_string(i) + "]";
        Wall w;
        if (wj.contains("name") && wj["name"].is_string()) w.name = wj["name"].get<std::string>();
        if (!wj.contains("p0") || !vecN(wj["p0"], 2, w.p0.data(), err, where + ".p0")) {
            if (err.empty()) err = "field model: " + where + ".p0 missing";
            return false;
        }
        if (!wj.contains("p1") || !vecN(wj["p1"], 2, w.p1.data(), err, where + ".p1")) {
            if (err.empty()) err = "field model: " + where + ".p1 missing";
            return false;
        }
        if (!num(wj, "heightM", w.heightM, err, where)) return false;
        if (wj.contains("railHeightM") && !num(wj, "railHeightM", w.railHeightM, err, where)) return false;
        if (wj.contains("bottomRailTopM") && !num(wj, "bottomRailTopM", w.bottomRailTopM, err, where)) return false;
        if (w.heightM <= 0 || w.heightM > 2 || (w.p1 - w.p0).norm() < 0.1) {
            err = "field model: " + where + " has a bad size";
            return false;
        }
        if (wj.contains("posts")) {
            if (!wj["posts"].is_array()) {
                err = "field model: " + where + ".posts must be an array";
                return false;
            }
            for (size_t k = 0; k < wj["posts"].size(); ++k) {
                Vec2 p;
                if (!vecN(wj["posts"][k], 2, p.data(), err, where + ".posts")) return false;
                w.posts.push_back(p);
            }
        }
        int wi = (int)m.walls.size();
        Vec3 a0(w.p0.x(), w.p0.y(), 0), a1(w.p1.x(), w.p1.y(), 0);
        Vec3 up(0, 0, 1);
        m.edges.push_back({a0, a1, EdgeType::WallFloor, wi});
        m.edges.push_back({a0 + w.heightM * up, a1 + w.heightM * up, EdgeType::WallTop, wi});
        if (w.railHeightM > 0 && w.railHeightM < w.heightM) {
            double z = w.heightM - w.railHeightM;
            m.edges.push_back({a0 + z * up, a1 + z * up, EdgeType::RailBottom, wi});
        }
        if (w.bottomRailTopM > 0 && w.bottomRailTopM < w.heightM)
            m.edges.push_back({a0 + w.bottomRailTopM * up, a1 + w.bottomRailTopM * up, EdgeType::BottomRail, wi});
        for (const Vec2& p : w.posts)
            m.edges.push_back({Vec3(p.x(), p.y(), 0), Vec3(p.x(), p.y(), w.heightM), EdgeType::Post, wi});
        m.walls.push_back(w);
    }

    // Tile seams: the interior lines of the tile grid (dark, thin). Used as low-weight ridge
    // features near the robot (edgeSeamWeight, edgeSeamMaxRangeM; 0 weight = off).
    if (m.tilePitchM > 0.1) {
        for (int axis = 0; axis < 2; ++axis) {
            double o = m.gridOrigin[axis];
            int k0 = (int)std::ceil((-h - o) / m.tilePitchM), k1 = (int)std::floor((h - o) / m.tilePitchM);
            for (int k = k0; k <= k1; ++k) {
                double c = o + k * m.tilePitchM;
                if (std::abs(c) > h - 0.05) continue;  // the outermost lines are the walls
                Vec3 a = axis == 0 ? Vec3(c, -h, 0) : Vec3(-h, c, 0);
                Vec3 b = axis == 0 ? Vec3(c, h, 0) : Vec3(h, c, 0);
                m.edges.push_back({a, b, EdgeType::Seam, -1});
            }
        }
    }

    if (doc.contains("hive") && !doc["hive"].is_null()) {
        const json& hj = doc["hive"];
        double np[3] = {0, 0, 0};
        if (hj.contains("nominalPose") && !vecN(hj["nominalPose"], 3, np, err, "hive.nominalPose")) return false;
        m.hiveNominal = SE2(np[0], np[1], np[2]);
        SE3 t = m.hiveNominal.toSE3();
        // Tipping parts: each of their faces and edges is kept once per stable tilt.
        if (hj.contains("pivots") && hj["pivots"].is_array()) {
            for (const json& pj : hj["pivots"]) {
                MovingPart mp;
                mp.name = pj.value("part", std::string());
                double pt[3], ax[3] = {0, 1, 0};
                if (!pj.contains("point") || !vecN(pj["point"], 3, pt, err, "hive.pivots.point")) {
                    if (err.empty()) err = "field model: hive pivot without point";
                    return false;
                }
                if (pj.contains("axis") && !vecN(pj["axis"], 3, ax, err, "hive.pivots.axis")) return false;
                mp.point = t * Vec3(pt[0], pt[1], pt[2]);
                mp.axis = t.linear() * Vec3(ax[0], ax[1], ax[2]);
                mp.startTilt = pj.value("startTiltRad", 0.0);
                if (!pj.contains("tiltRad") || !vecN(pj["tiltRad"], 2, mp.tilts, err, "hive.pivots.tiltRad")) {
                    if (err.empty()) err = "field model: hive pivot without tiltRad";
                    return false;
                }
                mp.startState = std::abs(mp.tilts[1] - mp.startTilt) < std::abs(mp.tilts[0] - mp.startTilt) ? 1 : 0;
                m.parts.push_back(mp);
            }
        }
        auto partIndex = [&m](const std::string& name) {
            for (size_t i = 0; i < m.parts.size(); ++i)
                if (m.parts[i].name == name) return (int)i;
            return -1;
        };
        if (hj.contains("faces")) {
            for (size_t i = 0; i < hj["faces"].size(); ++i) {
                const json& fj = hj["faces"][i];
                Surface s;
                s.name = fj.contains("name") && fj["name"].is_string() ? fj["name"].get<std::string>()
                                                                       : "face" + std::to_string(i);
                if (fj.contains("textured") && fj["textured"].is_boolean()) s.textured = fj["textured"].get<bool>();
                if (fj.contains("part") && fj["part"].is_string()) s.part = fj["part"].get<std::string>();
                s.convex = s.name.find("rib") == std::string::npos;
                if (!fj.contains("vertices") || !fj["vertices"].is_array()) {
                    err = "field model: hive face \"" + s.name + "\" has no vertices";
                    return false;
                }
                for (const json& vj : fj["vertices"]) {
                    Vec3 v;
                    if (!vecN(vj, 3, v.data(), err, "hive.faces.vertices")) return false;
                    s.vertices.push_back(t * v);
                }
                if (!makePlane(s, err)) return false;
                int mpi = partIndex(s.part);
                if (mpi < 0) {
                    m.hiveFaces.push_back(s);
                } else {
                    for (int st = 0; st < 2; ++st) {
                        Surface c = s;
                        for (Vec3& v : c.vertices) v = m.parts[mpi].moved(v, st);
                        if (!makePlane(c, err)) return false;
                        c.movingPart = mpi;
                        c.state = st;
                        m.hiveFaces.push_back(c);
                    }
                }
            }
        }
        std::vector<std::string> labels;
        if (hj.contains("edgeLabels") && hj["edgeLabels"].is_array())
            for (const json& l : hj["edgeLabels"]) labels.push_back(l.is_string() ? l.get<std::string>() : "");
        if (hj.contains("edges")) {
            size_t ei = 0;
            for (const json& ej : hj["edges"]) {
                std::string label = ei < labels.size() ? labels[ei] : "";
                ++ei;
                if (!ej.is_array() || ej.size() != 2) {
                    err = "field model: hive edge must be [[x,y,z],[x,y,z]]";
                    return false;
                }
                Vec3 a, b;
                if (!vecN(ej[0], 3, a.data(), err, "hive.edges") || !vecN(ej[1], 3, b.data(), err, "hive.edges"))
                    return false;
                ModelEdge e{t * a, t * b, EdgeType::Hive, -1};
                // Edge part from its label prefix; unlabelled hive edges are treated as moving.
                if (label.rfind("frame", 0) == 0) e.part = "frame";
                else if (label.rfind("red", 0) == 0) e.part = "hive_red";
                else if (label.rfind("blue", 0) == 0) e.part = "hive_blue";
                e.isStatic = e.part == "frame" || (labels.empty() && hj.value("pivots", json::array()).empty());
                int mpi = partIndex(e.part);
                if (mpi < 0) {
                    m.edges.push_back(e);
                } else {
                    for (int st = 0; st < 2; ++st) {
                        ModelEdge c = e;
                        c.a = m.parts[mpi].moved(e.a, st);
                        c.b = m.parts[mpi].moved(e.b, st);
                        c.movingPart = mpi;
                        c.state = st;
                        m.edges.push_back(c);
                    }
                }
            }
        }
    }
    return true;
}

double FieldModel::wallHit(const Vec3& o, const Vec3& dir, int* wallIndex) const {
    double best = -1;
    for (size_t i = 0; i < walls.size(); ++i) {
        const Wall& w = walls[i];
        Vec2 e = w.p1 - w.p0;
        Vec2 n(-e.y(), e.x());  // in-plane normal of the wall line
        double den = n.x() * dir.x() + n.y() * dir.y();
        if (std::abs(den) < 1e-12) continue;
        double t = n.dot(w.p0 - Vec2(o.x(), o.y())) / den;
        if (t <= 1e-9) continue;
        Vec3 p = o + t * dir;
        double s = (Vec2(p.x(), p.y()) - w.p0).dot(e) / e.squaredNorm();
        if (s < 0 || s > 1 || p.z() < 0 || p.z() > w.heightM) continue;
        if (best < 0 || t < best) {
            best = t;
            if (wallIndex) *wallIndex = (int)i;
        }
    }
    return best;
}

bool FieldModel::insidePerimeter(const Vec2& p, double margin) const {
    return std::abs(p.x()) <= halfSizeM - margin && std::abs(p.y()) <= halfSizeM - margin;
}

}  // namespace beacon
