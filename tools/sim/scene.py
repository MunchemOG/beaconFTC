"""Turn a field model (I1) plus clutter and a venue into renderable surfaces.

What is drawn (all seeded, so a seed gives the same scene):
- floor: dark grey foam tiles with fine foam texture, interlocking seams (zig-zag) on the
  field model's grid, gaffer-tape zones;
- walls: black top and bottom rails, clear polycarbonate panels (mostly see-through) with the
  venue behind them, panel joints;
- hive: every field-model face with its material (clear skins see-through, coloured ribs,
  aluminium frame, AprilTag-cluster stickers and BIOBUZZ panels as printed textures), tipped to
  the current hive tilts; FLOWERs as pale see-through boxes;
- venue (must be rejected by Beacon): venue floor, far walls with banners, bleachers and people,
  a ceiling with lights, people and tables just outside the field;
- clutter: other robots (textured boxes) and game pieces (POLLEN and NECTAR as small boxes).

Brightness is 8-bit grey; colours are converted with Rec. 601 luma.
"""
import math
import zlib

import cv2
import numpy as np

from render import ImageTexture, Surface, noise_image

LIGHT = np.array([0.3, 0.2, 0.93]) / np.linalg.norm([0.3, 0.2, 0.93])


def luma(rgb):
    r, g, b = rgb
    return 0.299 * r + 0.587 * g + 0.114 * b


RED_TAPE = luma((200, 30, 35))
BLUE_TAPE = luma((25, 70, 210))
MATERIAL_GREY = {"rib_red": luma((220, 20, 20)), "rib_blue": luma((20, 40, 200)),
                 "aluminum": 170.0, "clear": 215.0}


def shaded(shade, normal, ambient=0.7):
    """Lambert-like factor so faces with different normals separate visually."""
    f = ambient + (1 - ambient) * abs(float(np.dot(normal, LIGHT)))

    def s(a, b, foot):
        return shade(a, b, foot) * f
    return s


def textured_constant(value, rng, contrast=6.0, mpt=0.004, size_m=1.0):
    n = max(16, int(size_m / mpt))
    img = noise_image(rng, n, n, ((1, 1.0), (4, 0.5)), mean=value, contrast=contrast)
    return ImageTexture(img, mpt, wrap=True)


# ----------------------------------------------------------------------------- floor


def floor_texture(field, rng, mpt=0.002):
    fl = field.floor
    half = fl.half_size_m
    n = int(math.ceil(2 * half / mpt))
    img = noise_image(rng, n, n, ((1, 1.0), (3, 0.8), (12, 0.4), (40, 0.25)), mean=62.0,
                      contrast=11.0)
    coords = (np.arange(n) + 0.5) * mpt - half      # metres at texel centres
    # per-tile brightness variation
    k = np.floor((coords - fl.grid_origin_m[0]) / fl.tile_pitch_m).astype(int)
    offs = rng.normal(0.0, 4.0, (20, 20))
    img += offs[(k[:, None] + 10) % 20, (k[None, :] + 10) % 20]
    # interlocking seams: a triangle-wave line, dark, ~4 mm wide
    amp, period, width = 0.0103, fl.tile_pitch_m / 10.0, 0.004
    yy, xx = np.meshgrid(coords, coords, indexing="ij")    # row = y, col = x
    for s in field.tile_seams():
        for along, across in ((yy, xx), (xx, yy)):
            zig = amp * (2 / math.pi) * np.arcsin(np.sin(2 * math.pi * along / period))
            m = np.abs(across - s - zig) < width / 2 * 1.6
            img[m] = img[m] * 0.45
    # tape
    for tp in fl.tape:
        lo, hi = tp["min"], tp["max"]
        m = (xx >= lo[0]) & (xx <= hi[0]) & (yy >= lo[1]) & (yy <= hi[1])
        val = RED_TAPE if tp.get("color") == "red" else BLUE_TAPE
        img[m] = val + rng.normal(0, 2.5, m.sum())
    return ImageTexture(img, mpt, s0=-half, t0=-half)


def floor_surfaces(field, rng):
    half = field.floor.half_size_m
    tex = floor_texture(field, rng)
    # s = x, t = y: origin at (-half, -half) so the texture offsets match
    surf = Surface([0.0, 0.0, 0.0], [1, 0, 0], [0, 1, 0],
                   [(-half, -half), (half, -half), (half, half), (-half, half)],
                   lambda s, t, f: tex(s, t, f), name="floor", kind="floor")
    return [surf]


# ----------------------------------------------------------------------------- walls


def wall_surfaces(field, rng):
    out = []
    rail = textured_constant(22.0, rng, contrast=4.0)
    panel = constant_noise(215.0)
    for w in field.walls:
        p0 = np.array([w.p0[0], w.p0[1], 0.0])
        p1 = np.array([w.p1[0], w.p1[1], 0.0])
        d = p1 - p0
        length = np.linalg.norm(d)
        d /= length
        inward = np.array([-d[1], d[0], 0.0])
        up = np.array([0.0, 0.0, 1.0])
        h, rh, brt = w.height_m, w.rail_height_m, w.bottom_rail_top_m
        inset = w.panel_inset_m
        # top rail: inner face and top face
        out.append(Surface(p0 + up * (h - rh), d, up, [(0, 0), (length, 0), (length, rh), (0, rh)],
                           rail, name=w.name + "_top_rail", kind="wall"))
        out.append(Surface(p0 + up * h, d, -inward, [(0, 0), (length, 0), (length, 0.027), (0, 0.027)],
                           shaded(rail, up), name=w.name + "_rail_top", kind="wall"))
        if brt > 0:
            out.append(Surface(p0, d, up, [(0, 0), (length, 0), (length, brt), (0, brt)], rail,
                               name=w.name + "_bottom_rail", kind="wall"))
        # clear panel, slightly behind the rails
        base = p0 - inward * inset + up * brt
        ph = h - rh - brt
        out.append(Surface(base, d, up, [(0, 0), (length, 0), (length, ph), (0, ph)], panel,
                           alpha=0.12, name=w.name + "_panel", kind="wall_panel"))
        # panel joints: dark vertical strips (3 mm gap + edge) at the posts after the corner
        for post in w.posts[1:]:
            s = float(np.dot(np.array([post[0], post[1], 0.0]) - p0, d))
            out.append(Surface(p0 + d * (s - 0.004) - inward * 0.0005, d, up,
                               [(0, 0), (0.008, 0), (0.008, h), (0, h)], constant_noise(35.0),
                               name=w.name + "_joint", kind="wall"))
    return out


def constant_noise(value):
    def shade(s, t, foot):
        return np.full(s.shape, value, dtype=np.float32)
    return shade


# ----------------------------------------------------------------------------- hive


def sticker_texture(s_len, t_len, rng, mpt=0.001):
    """AprilTag-cluster-like print: four black-bordered random bit squares on white."""
    ns, nt = max(8, int(s_len / mpt)), max(8, int(t_len / mpt))
    img = np.full((nt, ns), 235.0, np.float32)            # rows = t, cols = s
    long_s = s_len >= t_len
    L, S = (ns, nt) if long_s else (nt, ns)
    tag = int(0.0825 / mpt)
    gap = max(1, (L - 4 * tag) // 5)
    for k in range(4):
        a0 = gap + k * (tag + gap)
        b0 = max(0, (S - tag) // 2)
        bits = rng.integers(0, 2, (8, 8))
        bits[0, :] = bits[-1, :] = bits[:, 0] = bits[:, -1] = 0
        block = np.kron(bits, np.ones((tag // 8 + 1, tag // 8 + 1)))[:tag, :tag] * 215 + 15
        if long_s:
            img[b0:b0 + tag, a0:a0 + tag] = block[: min(tag, nt - b0), : min(tag, ns - a0)]
        else:
            img[a0:a0 + tag, b0:b0 + tag] = block[: min(tag, nt - a0), : min(tag, ns - b0)]
    img += rng.normal(0, 2, img.shape)
    return ImageTexture(img, mpt)            # rows index t, columns index s


def printed_texture(s_len, t_len, rng, mpt=0.002):
    """BIOBUZZ panel: yellow with honeycomb outlines and dark text blocks."""
    ns, nt = max(8, int(s_len / mpt)), max(8, int(t_len / mpt))
    img = np.full((nt, ns), luma((240, 215, 40)), np.float32)
    for _ in range(40):
        c = (int(rng.uniform(0, ns)), int(rng.uniform(0, nt)))
        r = int(rng.uniform(8, 25))
        pts = np.array([[c[0] + r * math.cos(a), c[1] + r * math.sin(a)]
                        for a in np.arange(6) * math.pi / 3], np.int32)
        cv2.polylines(img, [pts], True, 60.0, 2)
    for _ in range(25):
        x0, y0 = int(rng.uniform(0, ns)), int(rng.uniform(0, nt))
        cv2.rectangle(img, (x0, y0), (x0 + int(rng.uniform(4, 30)), y0 + int(rng.uniform(4, 14))),
                      25.0, -1)
    return ImageTexture(img, mpt)


def hive_surfaces(field, rng):
    out = []
    tex_cache = {}
    for f in field.hive_faces:
        v = np.asarray(f.vertices, float)
        mat = f.material or ("printed" if f.textured else "aluminum")
        alpha = 1.0
        surf = Surface.from_vertices(v, None, name=f.name, kind="hive_" + f.part)
        ext = surf.polygon.max(axis=0) - surf.polygon.min(axis=0)
        s0, t0 = surf.polygon.min(axis=0)
        if mat == "sticker":
            tex = sticker_texture(ext[0], ext[1], rng)
            shade = _offset(tex, s0, t0)
        elif mat == "printed":
            tex = printed_texture(ext[0], ext[1], rng)
            shade = _offset(tex, s0, t0)
        elif mat == "clear":
            shade = constant_noise(MATERIAL_GREY["clear"])
            alpha = 0.10
        else:
            key = mat
            if key not in tex_cache:
                tex_cache[key] = textured_constant(MATERIAL_GREY.get(mat, 150.0), rng,
                                                   contrast=7.0, mpt=0.002)
            shade = tex_cache[key]
        surf.shade = shaded(shade, surf.normal) if alpha == 1.0 else shade
        surf.alpha = alpha
        out.append(surf)
    for i, fl in enumerate(field.flowers):
        out += box_surfaces(np.asarray(fl["min"]), np.asarray(fl["max"]), constant_noise(200.0),
                            "flower%d" % i, alpha=0.45)
    return out


def _offset(tex, s0, t0):
    def shade(s, t, foot):
        return tex(s - s0, t - t0, foot)
    return shade


# ----------------------------------------------------------------------------- boxes


def box_surfaces(lo, hi, shade, name, alpha=1.0, yaw=0.0, centre=None):
    """Axis-aligned (or yawed about centre) box: 4 sides and the top."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c = (lo + hi) / 2 if centre is None else np.asarray(centre, float)
    hx, hy = (hi[0] - lo[0]) / 2, (hi[1] - lo[1]) / 2
    z0, z1 = lo[2], hi[2]
    cy, sy = math.cos(yaw), math.sin(yaw)

    def P(x, y, z):
        return [c[0] + cy * x - sy * y, c[1] + sy * x + cy * y, z]
    corners = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)]
    out = []
    for k in range(4):
        (ax, ay), (bx, by) = corners[k], corners[(k + 1) % 4]
        quad = [P(ax, ay, z0), P(bx, by, z0), P(bx, by, z1), P(ax, ay, z1)]
        s = Surface.from_vertices(quad, None, alpha, name="%s_side%d" % (name, k), kind="clutter")
        s.shade = shaded(_offset_any(shade, s), s.normal) if alpha == 1.0 else shade
        out.append(s)
    top = [P(x, y, z1) for x, y in corners]
    s = Surface.from_vertices(top, None, alpha, name=name + "_top", kind="clutter")
    s.shade = shaded(_offset_any(shade, s), s.normal) if alpha == 1.0 else shade
    out.append(s)
    return out


def _offset_any(shade, surf):
    s0, t0 = surf.polygon.min(axis=0)

    def f(s, t, foot):
        return shade(s - s0, t - t0, foot)
    return f


def robot_texture(rng, mpt=0.003):
    n = int(0.5 / mpt)
    img = noise_image(rng, n, n, ((1, 0.6), (6, 1.0)), mean=rng.uniform(60, 160), contrast=25)
    for _ in range(30):
        x0, y0 = int(rng.uniform(0, n)), int(rng.uniform(0, n))
        cv2.rectangle(img, (x0, y0), (x0 + int(rng.uniform(3, 40)), y0 + int(rng.uniform(3, 40))),
                      float(rng.uniform(10, 245)), -1)
    return ImageTexture(img, mpt, wrap=True)


def piece_value(kind, rng):
    if kind == "pollen":
        return luma((235, 210, 40))
    return luma((210, 30, 30)) if rng.random() < 0.5 else luma((30, 50, 200))


def clutter_box_surfaces(box, pose, shade):
    x, y, yaw = pose
    sx, sy, sz = box.size
    return box_surfaces([x - sx / 2, y - sy / 2, 0.0], [x + sx / 2, y + sy / 2, sz], shade,
                        box.name, yaw=yaw, centre=[x, y, 0.0])


# ----------------------------------------------------------------------------- venue


def venue_surfaces(field, rng):
    half = field.floor.half_size_m
    out = []
    dx, dy = rng.uniform(6, 15), rng.uniform(6, 15)
    height = rng.uniform(5, 10)
    vf = textured_constant(rng.uniform(70, 150), rng, contrast=15.0, mpt=0.01, size_m=4.0)
    edge = half + 0.03                               # just outside the rails
    rects = [(-dx, -dy, dx, -edge), (-dx, edge, dx, dy), (-dx, -edge, -edge, edge),
             (edge, -edge, dx, edge)]
    for i, (x0, y0, x1, y1) in enumerate(rects):
        out.append(Surface([0, 0, -0.0005], [1, 0, 0], [0, 1, 0],
                           [(x0, y0), (x1, y0), (x1, y1), (x0, y1)], vf, name="venue_floor%d" % i,
                           kind="venue"))
    walls = [((-dx, dy), (dx, dy)), ((dx, dy), (dx, -dy)), ((dx, -dy), (-dx, -dy)),
             ((-dx, -dy), (-dx, dy))]
    for i, (a, b) in enumerate(walls):
        length = math.hypot(b[0] - a[0], b[1] - a[1])
        tex = venue_wall_texture(length, height, rng)
        d = np.array([b[0] - a[0], b[1] - a[1], 0.0]) / length
        out.append(Surface([a[0], a[1], 0.0], d, [0, 0, 1], [(0, 0), (length, 0), (length, height),
                                                              (0, height)], tex,
                           name="venue_wall%d" % i, kind="venue"))
    ceil = venue_ceiling_texture(2 * dx, 2 * dy, rng)
    out.append(Surface([-dx, -dy, height], [1, 0, 0], [0, 1, 0],
                       [(0, 0), (2 * dx, 0), (2 * dx, 2 * dy), (0, 2 * dy)], ceil, name="ceiling",
                       kind="venue"))
    # people and tables just outside the field
    for i in range(int(rng.integers(4, 9))):
        side = rng.integers(0, 4)
        along = rng.uniform(-half, half)
        off = half + rng.uniform(0.4, 2.0)
        x, y = [(along, -off), (along, off), (-off, along), (off, along)][side]
        tall = rng.uniform(1.5, 1.9) if rng.random() < 0.7 else 0.75
        out += box_surfaces([x - 0.2, y - 0.2, 0.0], [x + 0.2, y + 0.2, tall],
                            robot_texture(rng), "venue_person%d" % i)
    return out


def venue_wall_texture(length, height, rng, mpt=0.02):
    n_s, n_t = int(length / mpt), int(height / mpt)
    img = noise_image(rng, n_t, n_s, ((2, 0.5), (20, 1.0)), mean=rng.uniform(60, 140), contrast=20)
    # bleacher rows
    for k in range(0, int(n_t * 0.4), 6):
        img[k:k + 2, :] *= 0.6
    # people blobs on the bleachers
    for _ in range(int(length * 6)):
        cx, cy = int(rng.uniform(0, n_s)), int(rng.uniform(0, n_t * 0.4))
        cv2.ellipse(img, (cx, cy), (int(rng.uniform(5, 12)), int(rng.uniform(8, 20))), 0, 0, 360,
                    float(rng.uniform(20, 230)), -1)
    # banners
    for _ in range(int(length / 3)):
        x0, y0 = int(rng.uniform(0, n_s)), int(rng.uniform(n_t * 0.45, n_t * 0.9))
        cv2.rectangle(img, (x0, y0), (x0 + int(rng.uniform(40, 200)), y0 + int(rng.uniform(20, 60))),
                      float(rng.uniform(10, 250)), -1)
    return ImageTexture(img, mpt)            # rows = height (t), columns = along the wall (s)


def venue_ceiling_texture(sx, sy, rng, mpt=0.05):
    n_s, n_t = int(sx / mpt), int(sy / mpt)
    img = noise_image(rng, n_t, n_s, ((4, 1.0),), mean=40, contrast=8)
    for _ in range(int(sx * sy / 15)):
        x0, y0 = int(rng.uniform(0, n_s)), int(rng.uniform(0, n_t))
        cv2.rectangle(img, (x0, y0), (x0 + 12, y0 + 4), 250.0, -1)
    return ImageTexture(img, mpt)


# ----------------------------------------------------------------------------- whole scene


class Scene:
    """Static surfaces built once; hive and clutter rebuilt for each frame's state."""

    def __init__(self, field, seed=0, venue=True):
        rng = np.random.default_rng(seed + 7919)
        self.field = field
        self.static = floor_surfaces(field, rng) + wall_surfaces(field, rng)
        if venue:
            self.static += venue_surfaces(field, rng)
        self.hive_rng_seed = seed + 104729
        self._hive_cache = {}
        self.clutter_shades = {}
        self.rng = rng

    def hive(self, tilts):
        key = tuple(sorted((k, round(v, 4)) for k, v in tilts.items()))
        if key not in self._hive_cache:
            if len(self._hive_cache) > 8:
                self._hive_cache.clear()
            f = self.field.hive_at(dict(tilts)) if tilts else self.field
            self._hive_cache[key] = hive_surfaces(f, np.random.default_rng(self.hive_rng_seed))
        return self._hive_cache[key]

    def clutter(self, boxes_with_poses):
        out = []
        for box, pose in boxes_with_poses:
            if box.name not in self.clutter_shades:
                rng = np.random.default_rng(zlib.crc32(box.name.encode()))
                if box.kind in ("robot", "occluder"):
                    self.clutter_shades[box.name] = robot_texture(rng)
                else:
                    self.clutter_shades[box.name] = constant_noise(piece_value(box.kind, rng))
            out += clutter_box_surfaces(box, pose, self.clutter_shades[box.name])
        return out

    def surfaces(self, tilts=None, boxes_with_poses=()):
        return self.static + self.hive(tilts or {}) + self.clutter(boxes_with_poses)
