// Field model (shared interface I1): perimeter walls, floor tile grid and hive, in the field
// frame. The core turns it into model edges (for edge tracking) and planar surfaces (the only
// places where field-mode map points may live).
#pragma once

#include <string>
#include <vector>

#include "geometry.h"

namespace beacon {

enum class EdgeType { WallFloor, WallTop, RailBottom, Post, Hive };

/** A straight 3-D model edge in the field frame. */
struct ModelEdge {
    Vec3 a, b;
    EdgeType type;
    int wall = -1;  // wall index for wall edges, -1 otherwise
};

/** A convex planar polygon in the field frame (floor, hive face). */
struct Surface {
    std::string name;
    std::vector<Vec3> vertices;
    Vec3 normal;    // unit, pointing to the side the polygon is seen from is not assumed
    double d = 0;   // plane: normal . p + d = 0
    bool textured = true;
    bool isFloor = false;

    /** Ray (origin o, direction dir) hit distance along dir, or < 0 if no hit inside polygon. */
    double intersect(const Vec3& o, const Vec3& dir) const;
    double distance(const Vec3& p) const { return std::abs(normal.dot(p) + d); }
    bool contains(const Vec3& pOnPlane) const;
};

struct Wall {
    std::string name;
    Vec2 p0, p1;
    double heightM = 0;
    double railHeightM = 0;  // z of the lower edge of the opaque top rail (< heightM), 0 = none
    std::vector<Vec2> posts;
};

struct FieldModel {
    std::string season;
    double toleranceM = 0;
    double halfSizeM = 0;
    double tilePitchM = 0;
    Vec2 gridOrigin = Vec2::Zero();
    std::vector<Wall> walls;
    SE2 hiveNominal;
    std::vector<Surface> hiveFaces;  // in the field frame (nominal pose applied)
    std::vector<ModelEdge> edges;    // all model edges, field frame
    Surface floor;

    /** Parses I1 JSON text. Returns false with a message on any schema problem. */
    static bool parse(const std::string& text, FieldModel& out, std::string& err);

    /** First hit of a ray with a wall plane (inside its extent, 0..heightM), or < 0. */
    double wallHit(const Vec3& o, const Vec3& dir, int* wallIndex = nullptr) const;
    /** True when (x, y) is inside the perimeter. */
    bool insidePerimeter(const Vec2& p, double margin = 0) const;
};

}  // namespace beacon
