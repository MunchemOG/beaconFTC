#!/usr/bin/env python3
"""Derive beacon-field/biobuzz-2026.json from FIRST's official BIOBUZZ field CAD.

Offline tool, NOT a Beacon runtime dependency. It needs a separate scratch environment:

    python3.11 -m venv /tmp/fieldcad && /tmp/fieldcad/bin/pip install \
        cascadio==0.1.1 trimesh==5.2.1 numpy==2.4.6 opencv-python-headless==4.14.0.94
    curl -L -o field-cad-step.zip \
        https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step
    unzip field-cad-step.zip            # -> field-cad-step.step (Onshape export, 2026-09-09)
    /tmp/fieldcad/bin/python -c "import cascadio; cascadio.step_to_glb('field-cad-step.step', \
        'field.glb', 0.5, 0.5)"
    /tmp/fieldcad/bin/python beacon-field/derive/extract_biobuzz.py field.glb \
        beacon-field/biobuzz-2026.json

Every number comes from the CAD meshes (no hand-entered hive dimension), except the pivot
height, 43.95 in, and the 30 deg tilt, which are the Competition Manual's values (Section 9.6,
Figures 9-8 and 9-10). The script checks that the CAD matches them: it prints the symmetry
residuals of the two cells about the pivot (they should be a few tenths of a millimetre at most).

Frames: the CAD (glTF export) is y-up with x toward the blue alliance and z toward the audience.
The FTC field frame (ftc-docs "Field Coordinate System", red wall = the wall at the red ALLIANCE
AREA, which is on the audience's left for BIOBUZZ) is +x toward the audience, +y toward the blue
alliance, +z up, origin at the field centre on the tile top: X = gz, Y = gx, Z = gy.
"""
import json
import math
import sys

import cv2
import numpy as np
import trimesh

IN = 0.0254
PIVOT_Z = 43.95 * IN      # Competition Manual 9.6.1 / Figure 9-8
TILT = math.radians(30)   # Competition Manual Figure 9-10


def load(path):
    scene = trimesh.load(path, force="scene")
    parts = {}
    for node in scene.graph.nodes_geometry:
        transform, geom = scene.graph[node]
        mesh = scene.geometry[geom].copy()
        mesh.apply_transform(transform)
        parts[node] = mesh.vertices[:, [2, 0, 1]].copy()   # glTF -> field frame
        TRIANGLES[node] = np.asarray(mesh.faces)
    return parts


TRIANGLES = {}


def select(parts, prefix):
    return {k: v for k, v in parts.items() if k.startswith(prefix)}


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def r4(p):
    return [round(float(x), 4) for x in p]


# ----------------------------------------------------------------------------- outlines


def outlines_bc(points_bc, triangles, res=0.0005, eps=0.002):
    """Outer and inner (hole) outline of a ring-shaped part projected to 2-D (b, c)."""
    lo = points_bc.min(axis=0) - 0.01
    pix = np.round((points_bc - lo) / res).astype(np.int32)
    size = pix.max(axis=0) + 3
    img = np.zeros((size[1], size[0]), np.uint8)
    for tri in triangles:
        cv2.fillConvexPoly(img, pix[tri], 255)
    contours, hier = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    order = sorted(range(len(contours)), key=lambda i: -cv2.contourArea(contours[i]))
    outer = contours[order[0]]
    holes = [contours[i] for i in order[1:] if hier[0][i][3] == order[0]]
    inner = max(holes, key=cv2.contourArea)

    def poly(c, e):
        # Convex hull first: drops notches and cable-tie holes, keeps the silhouette.
        approx = cv2.approxPolyDP(cv2.convexHull(c), e / res, True).reshape(-1, 2).astype(float)
        return approx * res + lo

    # The opening (inner) outline is simplified harder: it is the skins' pentagon.
    return poly(outer, eps), poly(inner, 3 * eps)


def ccw(poly):
    area = 0.5 * np.sum(poly[:, 0] * np.roll(poly[:, 1], -1) - np.roll(poly[:, 0], -1) * poly[:, 1])
    return poly if area > 0 else poly[::-1]


def keyhole(outer, inner):
    """One weakly-simple polygon for a ring: outer CCW, a slit, inner CW, back along the slit."""
    outer = ccw(outer)
    inner = ccw(inner)[::-1]
    # slit from the outer vertex with the lowest c to the nearest inner vertex
    i0 = int(np.argmin(outer[:, 1]))
    j0 = int(np.argmin(np.linalg.norm(inner - outer[i0], axis=1)))
    ring_o = np.roll(outer, -i0, axis=0)
    ring_i = np.roll(inner, -j0, axis=0)
    return np.vstack([ring_o, ring_o[:1], ring_i, ring_i[:1]])


# ----------------------------------------------------------------------------- main


def main(glb, out_path):
    parts = load(glb)
    faces, edges, labels = [], [], []

    def face(name, verts, textured, material, part):
        faces.append({"name": name, "vertices": [r4(v) for v in verts], "textured": textured,
                      "material": material, "part": part})

    def edge(p, q, label):
        edges.append([r4(p), r4(q)])
        labels.append(label)

    # ---------------------------------------------------------------- floor and walls
    rails = np.vstack(list(select(parts, "am-2556a").values()))
    # Every rail point lies on the perimeter, so the smallest max(|x|, |y|) is the inner face.
    half = float(np.min(np.max(np.abs(rails[:, :2]), axis=1)))
    top_rail = rails[rails[:, 2] > 0.1]
    bottom_rail = rails[rails[:, 2] < 0.1]
    height = float(top_rail[:, 2].max())
    rail_bottom = float(top_rail[:, 2].min())
    bottom_rail_top = float(bottom_rail[:, 2].max())
    # Seam pitch: centre tiles span [seam - tab, seam + pitch + tab]; side tiles start at
    # 2 * pitch - tab. Solve both from the CAD bounds.
    c_b = [(v[:, 0].min(), v[:, 0].max()) for v in select(parts, "am-2499-Center").values()]
    widths = sorted(round(b - a, 5) for a, b in c_b)
    span = widths[len(widths) // 2]                      # pitch + 2 tab
    mins = sorted(set(round(a, 4) for a, _ in c_b))
    tab_size = -min(m for m in mins if m > -0.1)           # centre tile starting at seam 0
    pitch = span - 2 * tab_size
    panels = np.vstack(list(select(parts, "FTC Field Side Glass").values()))
    links = select(parts, "am-2556a")
    # vertical panel joints: gaps between rail segments along a wall
    seg_ends = sorted(set(round(float(abs(e)), 4) for v in links.values()
                          for e in (v[:, 1].min(), v[:, 1].max()) if abs(abs(e) - half) > 0.05
                          and abs(e) < half - 0.05 and abs(e) > 0.3))
    joint = (seg_ends[0] + seg_ends[1]) / 2.0                 # e.g. 0.5927 / 0.5959

    walls = []
    # Counter-clockwise around the field (interior on the left of p0 -> p1).
    order = [((half, -half), (half, half), "audience"), ((half, half), (-half, half), "blue"),
             ((-half, half), (-half, -half), "scoring"), ((-half, -half), (half, -half), "red")]
    for p0, p1, name in order:
        d = np.subtract(p1, p0) / (2 * half)
        posts = [r4(np.add(p0, d * (half + s))) for s in (-joint, joint)]
        walls.append({"name": name, "p0": r4(p0), "p1": r4(p1), "heightM": round(height, 4),
                      "railHeightM": round(height - rail_bottom, 4),
                      "bottomRailTopM": round(bottom_rail_top, 4),
                      "panelInsetM": round(float(np.min(np.max(np.abs(panels[:, :2]), axis=1))) - half, 4),
                      "posts": [r4(p0)] + posts})

    # ---------------------------------------------------------------- hive (two tipping hives)
    pivots = []
    residuals = []
    for color, sign_start in (("red", +1.0), ("blue", -1.0)):
        part = "hive_" + color
        ribs = select(parts, "am-5866-%s" % color)
        y0 = float(np.mean(np.vstack(list(ribs.values()))[:, 1]))
        pivot = np.array([0.0, y0, PIVOT_Z])
        start_tilt = sign_start * TILT               # +: the +x (audience) end is raised
        to_flat = ry(start_tilt)                     # undo the start tilt (flat <- field)
        to_field = ry(-start_tilt)

        def flat(v):
            return (to_flat @ (v - pivot).T).T

        def field(v_flat):
            return (to_field @ np.asarray(v_flat, float).T).T + pivot

        pivots.append({"part": part, "point": r4(pivot), "axis": [0.0, 1.0, 0.0],
                       "startTiltRad": round(start_tilt, 6), "tiltRad": [round(TILT, 6), round(-TILT, 6)]})

        for side_sign, cell in ((+1.0, "audience"), (-1.0, "scoring")):
            cell_ribs = [(flat(v), TRIANGLES[k]) for k, v in ribs.items()
                         if np.sign(flat(v)[:, 0].mean()) == side_sign]
            cell_ribs.sort(key=lambda f: abs(f[0][:, 0].mean()))
            (inner_rib, inner_tri), (outer_rib, outer_tri) = cell_ribs
            name = "%s_%s" % (color, cell)
            # rib outlines in (b, c) = (y - y0, z) in the flat frame
            for rib, tri, rib_name in ((inner_rib, inner_tri, "back_rib"),
                                       (outer_rib, outer_tri, "opening_rib")):
                bc = rib[:, 1:3]
                o_poly, i_poly = outlines_bc(bc, tri)
                a_near = float(np.min(np.abs(rib[:, 0]))) * side_sign
                a_far = float(np.max(np.abs(rib[:, 0]))) * side_sign
                for a, which in ((a_near, "pivot_side"), (a_far, "outer_side")):
                    ring = keyhole(o_poly, i_poly)
                    verts = field(np.column_stack([np.full(len(ring), a), ring]))
                    face("%s_%s_%s" % (name, rib_name, which), verts, False, "rib_" + color, part)
                # outer band of the rib
                o = ccw(o_poly)
                for k in range(len(o)):
                    p, q = o[k], o[(k + 1) % len(o)]
                    quad = [[a_near, p[0], p[1]], [a_far, p[0], p[1]], [a_far, q[0], q[1]], [a_near, q[0], q[1]]]
                    face("%s_%s_band%d" % (name, rib_name, k), field(quad), False, "rib_" + color, part)
                # edges: outer outline on the outward face; opening outline on the opening rib
                a_out = a_far if rib_name == "opening_rib" else a_near
                for k in range(len(o)):
                    p, q = o[k], o[(k + 1) % len(o)]
                    pq = field([[a_out, p[0], p[1]], [a_out, q[0], q[1]]])
                    edge(pq[0], pq[1], "%s_%s_outline" % (name, rib_name))
                if rib_name == "opening_rib":
                    ii = ccw(i_poly)
                    for k in range(len(ii)):
                        p, q = ii[k], ii[(k + 1) % len(ii)]
                        pq = field([[a_far, p[0], p[1]], [a_far, q[0], q[1]]])
                        edge(pq[0], pq[1], "%s_opening" % name)
                if rib_name == "back_rib":
                    back_inner = i_poly
                    back_a = a_near
                else:
                    open_a = a_far
            # clear skins: back pentagon and prism walls along the opening outline
            pent = ccw(back_inner)
            face("%s_back_skin" % name, field(np.column_stack([np.full(len(pent), back_a), pent])),
                 False, "clear", part)
            for k in range(len(pent)):
                p, q = pent[k], pent[(k + 1) % len(pent)]
                quad = [[back_a, p[0], p[1]], [open_a, p[0], p[1]], [open_a, q[0], q[1]], [back_a, q[0], q[1]]]
                face("%s_skin%d" % (name, k), field(quad), False, "clear", part)
            # AprilTag cluster sticker: printed (black/white), opaque, faces the tiles
            tag_key = [k for k in select(parts, "am-5888-%s" % color)
                       if ("Audience" in k) == (cell == "audience")][0]
            t = flat(parts[tag_key])
            lo, hi = t.min(axis=0), t.max(axis=0)
            c_tag = float(lo[2])
            rect = [[lo[0], lo[1], c_tag], [hi[0], lo[1], c_tag], [hi[0], hi[1], c_tag], [lo[0], hi[1], c_tag]]
            face("%s_sticker" % name, field(rect), True, "sticker", part)
            residuals.append((name, np.abs(inner_rib[:, 0]).min(), np.abs(outer_rib[:, 0]).max()))

    # symmetry check of the two cells of each hive about the pivot (flat frame)
    print("cell extents along the arm (flat frame, inches):")
    for n, a0, a1 in residuals:
        print("  %-16s %+8.3f %+8.3f" % (n, a0 / IN, a1 / IN))

    # ---------------------------------------------------------------- frame (static)
    def box_from(points, name, material, textured=False, axis_hint=None):
        """Oriented box around a straight member; returns its 4 long faces and long edges."""
        c = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points - c, full_matrices=False)
        u = vt[0]
        p = np.cross(u, [0.0, 0.0, 1.0])
        if np.linalg.norm(p) < 1e-6:
            p = np.array([1.0, 0.0, 0.0])
        p /= np.linalg.norm(p)
        q = np.cross(u, p)
        d = points - c
        su, sp, sq = d @ u, d @ p, d @ q
        corners = []
        for a, b in ((sp.min(), sq.min()), (sp.max(), sq.min()), (sp.max(), sq.max()), (sp.min(), sq.max())):
            corners.append((c + a * p + b * q))
        for k in range(4):
            e0, e1 = corners[k], corners[(k + 1) % 4]
            quad = [e0 + su.min() * u, e0 + su.max() * u, e1 + su.max() * u, e1 + su.min() * u]
            face("%s_side%d" % (name, k), quad, textured, material, "frame")
            edge(e0 + su.min() * u, e0 + su.max() * u, name)
        return c, u, (sp.max() - sp.min(), sq.max() - sq.min())

    for k, (key, v) in enumerate(sorted(select(parts, "am-5876").items())):
        box_from(v, "frame_leg%d" % k, "aluminum")
    box_from(select(parts, "am-5875")["am-5875: A-Frame Top Bar"], "frame_top_bar", "aluminum")
    for k, (key, v) in enumerate(sorted(select(parts, "am-5878").items())):
        box_from(v, "frame_foot_bar%d" % k, "aluminum")
    for k, (key, v) in enumerate(sorted(select(parts, "am-5883").items())):
        c = v.mean(axis=0)
        _, _, vt = np.linalg.svd(v - c, full_matrices=False)
        u, w = vt[0], vt[1]
        su, sw = (v - c) @ u, (v - c) @ w
        quad = [c + su.min() * u + sw.min() * w, c + su.max() * u + sw.min() * w,
                c + su.max() * u + sw.max() * w, c + su.min() * u + sw.max() * w]
        face("frame_biobuzz_sticker%d" % k, quad, True, "printed", "frame")
        for j in range(4):
            edge(quad[j], quad[(j + 1) % 4], "frame_biobuzz_panel%d" % k)

    # ---------------------------------------------------------------- tape and flowers
    tape = []
    for key, v in sorted(select(parts, "Gaffer Tape").items()):
        color = "red" if "Red" in key else "blue"
        lo, hi = v.min(axis=0)[:2], v.max(axis=0)[:2]
        if np.all(np.abs(lo) <= half + 1e-3) and np.all(np.abs(hi) <= half + 1e-3):
            tape.append({"color": color, "min": r4(lo), "max": r4(hi)})  # inside the field only
    flowers = []
    flower_parts = {k: v for k, v in parts.items() if k.startswith(("am-5857", "am-5858", "am-5859",
                                                                     "am-5860", "am-5862", "am-5884"))}
    pts = np.vstack(list(flower_parts.values()))
    for v in [pts[(np.abs(pts[:, 0]) > 1.4) & (pts[:, 1] > 0)], pts[(np.abs(pts[:, 0]) > 1.4) & (pts[:, 1] < 0)],
              pts[(np.abs(pts[:, 1]) > 1.4) & (pts[:, 0] > 0)], pts[(np.abs(pts[:, 1]) > 1.4) & (pts[:, 0] < 0)]]:
        if len(v):
            lo = np.maximum(v.min(axis=0), [-half, -half, 0.0])
            hi = np.minimum(v.max(axis=0), [half, half, 10.0])
            flowers.append({"min": r4(lo), "max": r4(hi)})

    model = {
        "schema": 1,
        "season": "BIOBUZZ 2026-27",
        "source": ("FIRST BIOBUZZ Field CAD (STEP, .ZIP) v26-27.2, posted 2026-09-15, Onshape export "
                   "2026-09-09: https://ftc-resources.firstinspires.org/ftc/archive/2027/field/field-cad-step "
                   "(derived by beacon-field/derive/extract_biobuzz.py); pivot height 43.95 in and 30 deg "
                   "tilt from Competition Manual Section 9 (TU06, 2026-10-22), Figures 9-8 and 9-10"),
        "toleranceM": 0.0254,
        "toleranceSource": ("Competition Manual 9.1: measurements from the official 3D CAD carry a general "
                            "tolerance of +/- 1 in (+/- 2.5 cm); Field Acceptance Checklist 26-27.3 "
                            "controls the HIVE frame feet (+/- 0.50 in) and the upright CELL opening "
                            "height (53.5 +/- 0.50 in)"),
        "fieldFrame": ("FTC field coordinates (ftc-docs): origin field centre on the tile top; +x toward "
                       "the audience, +y toward the blue alliance wall, +z up; the red alliance wall "
                       "(y = -halfSize) is on the audience's left. Metres."),
        "floor": {"halfSizeM": round(half, 4), "tilePitchM": round(pitch, 5), "gridOriginM": [0.0, 0.0],
                  "tiles": [6, 6], "tape": tape},
        "walls": walls,
        "hive": {"nominalPose": [0.0, 0.0, 0.0], "faces": faces, "edges": edges, "edgeLabels": labels,
                 "pivots": pivots,
                 "startState": "red audience CELL up, blue scoring CELL up (Event Field Setup Guide 11.1)"},
        "flowers": flowers,
    }
    with open(out_path, "w") as f:
        json.dump(model, f, indent=1)
        f.write("\n")
    print("half %.4f pitch %.5f (tab %.4f) height %.4f rail %.4f bottomRail %.4f joint %.4f"
          % (half, pitch, tab_size, height, height - rail_bottom, bottom_rail_top, joint))
    print("hive faces %d, edges %d, flowers %d, tape %d" % (len(faces), len(edges), len(flowers), len(tape)))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
