"""Load and check a Beacon field model (plan I1, beacon-field/<season>.json).

    field = load_field("beacon-field/biobuzz-2026.json")
    problems = field.validate()          # [] means OK
    tipped = field.hive_at({"hive_red": -0.5236})   # red hive tipped to its other state

Keys beyond the I1 example are optional (a reader that ignores them still works):
face "material" and "part", hive "edgeLabels" and "pivots", wall "bottomRailTopM" and
"panelInsetM", floor "tiles" and "tape", top-level "flowers", "toleranceSource", "fieldFrame".
See beacon-field/README.md.
"""
import copy
import json
import math
from dataclasses import dataclass, field as dc_field

import numpy as np

PLANAR_TOL_M = 0.001
CLOSE_TOL_M = 0.001


class FieldError(Exception):
    """The file cannot be read as a field model at all."""


@dataclass
class Floor:
    half_size_m: float
    tile_pitch_m: float
    grid_origin_m: tuple
    tiles: tuple = (6, 6)
    tape: list = dc_field(default_factory=list)


@dataclass
class Wall:
    name: str
    p0: tuple
    p1: tuple
    height_m: float
    rail_height_m: float
    posts: list
    bottom_rail_top_m: float = 0.0
    panel_inset_m: float = 0.0


@dataclass
class Face:
    name: str
    vertices: list
    textured: bool
    material: str = ""
    part: str = "hive"


@dataclass
class Pivot:
    part: str
    point: tuple
    axis: tuple
    start_tilt_rad: float
    tilt_rad: tuple


@dataclass
class Field:
    raw: dict
    season: str
    source: str
    tolerance_m: float
    floor: Floor
    walls: list
    hive_faces: list
    hive_edges: list
    edge_labels: list
    pivots: list
    flowers: list
    tilts: dict = dc_field(default_factory=dict)  # current tilt per part (start state if absent)

    @property
    def is_placeholder(self):
        return "PLACEHOLDER" in (self.source or "").upper()

    # ------------------------------------------------------------------ helpers

    def tile_seams(self):
        """Seam coordinates (same on x and y) inside the field.

        Grid lines within a quarter tile of a wall are where the trimmed outer tiles meet the
        wall, not seams, and are left out.
        """
        fl = self.floor
        o = fl.grid_origin_m[0]
        limit = fl.half_size_m - fl.tile_pitch_m / 4.0
        k0 = math.ceil((-limit - o) / fl.tile_pitch_m)
        k1 = math.floor((limit - o) / fl.tile_pitch_m)
        return [o + k * fl.tile_pitch_m for k in range(k0, k1 + 1)]

    def hive_edges_named(self, label):
        return [e for e, lab in zip(self.hive_edges, self.edge_labels) if lab == label]

    def hive_at(self, tilts):
        """A copy with hive parts rotated to the given tilts (radians, + = audience end up).

        A point p of part P moves to point + Ry(-(tilt - startTilt)) (p - point), Ry being the
        right-handed rotation about the pivot axis (+y).
        """
        out = copy.copy(self)
        rot = {}
        for pv in self.pivots:
            if pv.part in tilts:
                delta = float(tilts[pv.part]) - pv.start_tilt_rad
                rot[pv.part] = (np.asarray(pv.point, float), _axis_rotation(pv.axis, -delta))
        def move(part, pts):
            if part not in rot:
                return pts
            c, r = rot[part]
            return ((r @ (np.asarray(pts, float) - c).T).T + c).tolist()
        out.hive_faces = [Face(f.name, move(f.part, f.vertices), f.textured, f.material, f.part)
                          for f in self.hive_faces]
        edge_parts = [_part_of_label(lab, self.pivots) for lab in self.edge_labels]
        if len(edge_parts) != len(self.hive_edges):
            edge_parts = ["frame"] * len(self.hive_edges)
        out.hive_edges = [move(p, e) for e, p in zip(self.hive_edges, edge_parts)]
        out.tilts = dict(self.tilts, **tilts)
        return out

    # ------------------------------------------------------------------ validation

    def validate(self):
        """Return a list of problems (empty = OK)."""
        errors = []
        raw = self.raw
        if raw.get("schema") != 1:
            errors.append("schema must be 1 (found %r)" % raw.get("schema"))
        if not isinstance(raw.get("season"), str) or not raw.get("season"):
            errors.append("season must be a non-empty string")
        if not isinstance(raw.get("source"), str) or not raw.get("source"):
            errors.append("source must name the drawing (name, revision, URL)")
        _positive(errors, "toleranceM", raw.get("toleranceM"))
        if not _all_finite(raw):
            errors.append("every number must be finite (found NaN or infinity)")

        fl = raw.get("floor")
        if not isinstance(fl, dict):
            errors.append("floor section missing")
        else:
            _positive(errors, "floor.halfSizeM", fl.get("halfSizeM"))
            _positive(errors, "floor.tilePitchM", fl.get("tilePitchM"))
            if not _vec(fl.get("gridOriginM"), 2):
                errors.append("floor.gridOriginM must be [x, y]")

        walls = raw.get("walls")
        if not isinstance(walls, list) or len(walls) < 3:
            errors.append("walls must list at least 3 walls")
        else:
            for i, w in enumerate(walls):
                if not (_vec(w.get("p0"), 2) and _vec(w.get("p1"), 2)):
                    errors.append("walls[%d]: p0 and p1 must be [x, y]" % i)
                    continue
                _positive(errors, "walls[%d].heightM" % i, w.get("heightM"))
                if "railHeightM" in w:
                    _positive(errors, "walls[%d].railHeightM" % i, w.get("railHeightM"))
                for p in w.get("posts", []):
                    if not _vec(p, 2):
                        errors.append("walls[%d].posts entries must be [x, y]" % i)
            if all(_vec(w.get("p0"), 2) and _vec(w.get("p1"), 2) for w in walls):
                for i, w in enumerate(walls):
                    nxt = walls[(i + 1) % len(walls)]
                    if np.linalg.norm(np.subtract(w["p1"], nxt["p0"])) > CLOSE_TOL_M:
                        errors.append("walls do not form a closed loop: walls[%d].p1 != walls[%d].p0"
                                      % (i, (i + 1) % len(walls)))

        hive = raw.get("hive")
        if not isinstance(hive, dict):
            errors.append("hive section missing (the field model needs the hive geometry)")
        else:
            faces = hive.get("faces")
            if not isinstance(faces, list) or not faces:
                errors.append("hive has no faces")
            else:
                for i, f in enumerate(faces):
                    v = f.get("vertices")
                    if not isinstance(v, list) or len(v) < 3 or not all(_vec(p, 3) for p in v):
                        errors.append("hive face %d (%s): needs >= 3 [x, y, z] vertices"
                                      % (i, f.get("name")))
                        continue
                    if not isinstance(f.get("textured"), bool):
                        errors.append("hive face %d (%s): textured must be true or false"
                                      % (i, f.get("name")))
                    off = _planarity(np.asarray(v, float))
                    if not math.isfinite(off) or off > PLANAR_TOL_M:
                        errors.append("hive face %d (%s) is not planar (%.4f m off its plane)"
                                      % (i, f.get("name"), off))
            edges = hive.get("edges", [])
            if not isinstance(edges, list) or not all(
                    isinstance(e, list) and len(e) == 2 and _vec(e[0], 3) and _vec(e[1], 3)
                    for e in edges):
                errors.append("hive edges must be [[x,y,z],[x,y,z]] pairs")
            labels = hive.get("edgeLabels")
            if labels is not None and len(labels) != len(edges):
                errors.append("hive edgeLabels must have one label per edge")
            if not _vec(hive.get("nominalPose"), 3):
                errors.append("hive.nominalPose must be [x, y, yaw]")
            parts = {f.get("part") for f in (faces or []) if isinstance(f, dict)}
            for pv in hive.get("pivots", []):
                if pv.get("part") not in parts:
                    errors.append("hive pivot for unknown part %r" % pv.get("part"))
                if not (_vec(pv.get("point"), 3) and _vec(pv.get("axis"), 3)):
                    errors.append("hive pivot %r needs point and axis" % pv.get("part"))
        return errors


def load_field(path):
    """Read a field model file. Raises FieldError only if it is not a JSON object."""
    try:
        with open(path) as f:
            raw = json.load(f)
    except (OSError, ValueError) as e:
        raise FieldError("cannot read field model %s: %s" % (path, e)) from None
    if not isinstance(raw, dict):
        raise FieldError("field model %s is not a JSON object" % path)
    return from_dict(raw)


def from_dict(raw):
    fl = raw.get("floor") or {}
    floor = Floor(_f(fl.get("halfSizeM")), _f(fl.get("tilePitchM")),
                  tuple(fl.get("gridOriginM") or (0.0, 0.0)), tuple(fl.get("tiles") or (6, 6)),
                  list(fl.get("tape") or []))
    walls = []
    for w in raw.get("walls") or []:
        walls.append(Wall(w.get("name", ""), tuple(w.get("p0") or ()), tuple(w.get("p1") or ()),
                          _f(w.get("heightM")), _f(w.get("railHeightM", 0.0)),
                          [tuple(p) for p in w.get("posts", [])],
                          _f(w.get("bottomRailTopM", 0.0)), _f(w.get("panelInsetM", 0.0))))
    hive = raw.get("hive") or {}
    faces = [Face(f.get("name", "face%d" % i), f.get("vertices") or [], bool(f.get("textured")),
                  f.get("material", ""), f.get("part", "hive"))
             for i, f in enumerate(hive.get("faces") or []) if isinstance(f, dict)]
    edges = list(hive.get("edges") or [])
    labels = list(hive.get("edgeLabels") or [""] * len(edges))
    pivots = [Pivot(p.get("part"), tuple(p.get("point") or (0, 0, 0)),
                    tuple(p.get("axis") or (0, 1, 0)), _f(p.get("startTiltRad", 0.0)),
                    tuple(p.get("tiltRad") or ()))
              for p in hive.get("pivots", []) if isinstance(p, dict)]
    return Field(raw, raw.get("season", ""), raw.get("source", ""), _f(raw.get("toleranceM")),
                 floor, walls, faces, edges, labels, pivots, list(raw.get("flowers") or []))


# ---------------------------------------------------------------------- internals


def _part_of_label(label, pivots):
    for pv in pivots:
        color = pv.part.replace("hive_", "")
        if label.startswith(color + "_"):
            return pv.part
    return "frame"


def _axis_rotation(axis, angle):
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * (k @ k)


def _planarity(v):
    c = v.mean(axis=0)
    _, _, vt = np.linalg.svd(v - c, full_matrices=False)
    return float(np.abs((v - c) @ vt[2]).max())


def _f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def _vec(v, n):
    return (isinstance(v, (list, tuple)) and len(v) == n
            and all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v))


def _positive(errors, name, value):
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value) \
            or value <= 0:
        errors.append("%s must be a positive finite number (found %r)" % (name, value))


def _all_finite(obj):
    if isinstance(obj, float):
        return math.isfinite(obj)
    if isinstance(obj, dict):
        return all(_all_finite(v) for v in obj.values())
    if isinstance(obj, list):
        return all(_all_finite(v) for v in obj)
    return True
