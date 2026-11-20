// Field model (shared interface I1): perimeter walls, floor tile grid and hive, in the field
// frame. The core turns it into model edges (for edge tracking) and planar surfaces (the only
// places where field-mode map points may live).
#pragma once

#include <string>
#include <vector>

#include "geometry.h"

namespace beacon {

enum class EdgeType { WallFloor, WallTop, RailBottom, BottomRail, Post, Hive, Seam };

/** A straight 3-D model edge in the field frame. */
struct ModelEdge {
    Vec3 a, b;
    EdgeType type;
    int wall = -1;          // wall index for wall edges, -1 otherwise
    std::string part;       // hive edges: "frame" (static), "hive_red", "hive_blue" (these tip)
    bool isStatic = true;   // false for parts that move during a match
    int movingPart = -1;    // index into FieldModel::parts for tipping parts
    int state = -1;         // which of the part's two stable tilts this copy is drawn in
};

/** A convex planar polygon in the field frame (floor, hive face). */
struct Surface {
    std::string name;
    std::vector<Vec3> vertices;
    Vec3 normal;    // unit, pointing to the side the polygon is seen from is not assumed
    double d = 0;   // plane: normal . p + d = 0
    bool textured = true;
    bool isFloor = false;
    std::string part;       // hive faces: frame / hive_red / hive_blue
    bool convex = true;     // false for keyhole polygons (rib outlines): no inside test
    int movingPart = -1;    // as in ModelEdge
    int state = -1;

    /** Ray (origin o, direction dir) hit distance along dir, or < 0 if no hit inside polygon. */
    double intersect(const Vec3& o, const Vec3& dir) const;
    double distance(const Vec3& p) const { return std::abs(normal.dot(p) + d); }
    bool contains(const Vec3& pOnPlane) const;
};

struct Wall {
    std::string name;
    Vec2 p0, p1;
    double heightM = 0;
    double railHeightM = 0;     // vertical size of the opaque top rail; its bottom edge is at heightM - railHeightM
    double bottomRailTopM = 0;  // top edge of the bottom rail (0 = none)
    std::vector<Vec2> posts;
};

/** A hive part that tips between two stable tilts about a pivot (BIOBUZZ hive_red / hive_blue). */
struct MovingPart {
    std::string name;
    Vec3 point = Vec3::Zero(), axis = Vec3::UnitY();
    double startTilt = 0;
    double tilts[2] = {0, 0};
    int startState = 0;  // index of startTilt in tilts
    /** Moves a point given in the start pose to tilt state s. */
    Vec3 moved(const Vec3& p, int s) const;
};

struct FieldModel {
    std::string season;
    double toleranceM = 0;
    double halfSizeM = 0;
    double tilePitchM = 0;
    Vec2 gridOrigin = Vec2::Zero();
    std::vector<Wall> walls;
    SE2 hiveNominal;
    std::vector<MovingPart> parts;  // tipping hive parts (from hive.pivots)
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
