"""Ray-cast renderer for scenes made of textured planar polygons.

Every pixel's ray is intersected exactly with every surface plane, so a 3-D point on a surface
lands exactly where Camera.project() puts it (the renderer and the projection share one camera
model, including lens distortion). Options: k x k supersampling (anti-aliasing), rolling
shutter (one camera pose per image row), motion blur (average of sub-frames over the exposure),
sensor noise.

A Surface is a polygon in a plane: points origin + s * u_axis + t * v_axis with (s, t) inside
`polygon`. Its `shade(s, t, footprint_m)` returns brightness 0..255; footprint_m is the size of
one pixel on the surface (for texture filtering). alpha < 1 makes it see-through (clear
polycarbonate panels): it is blended over whatever lies behind it.
"""
import math

import numpy as np


class Surface:
    def __init__(self, origin, u_axis, v_axis, polygon, shade, alpha=1.0, name="", kind=""):
        self.origin = np.asarray(origin, dtype=float)
        u = np.asarray(u_axis, dtype=float)
        v = np.asarray(v_axis, dtype=float)
        self.u = u / np.linalg.norm(u)
        v = v - self.u * float(self.u @ v)  # make v orthogonal to u
        self.v = v / np.linalg.norm(v)
        self.normal = np.cross(self.u, self.v)
        self.polygon = np.asarray(polygon, dtype=float).reshape(-1, 2)
        self.shade = shade
        self.alpha = float(alpha)
        self.name = name
        self.kind = kind

    @classmethod
    def from_vertices(cls, vertices, shade, alpha=1.0, name="", kind=""):
        """Planar polygon from 3-D vertices (in order). u along the first edge."""
        p = np.asarray(vertices, dtype=float)
        origin = p[0]
        u = p[1] - p[0]
        # Newell normal is robust for any simple polygon.
        n = np.zeros(3)
        for i in range(len(p)):
            a, b = p[i], p[(i + 1) % len(p)]
            n += np.array([(a[1] - b[1]) * (a[2] + b[2]),
                           (a[2] - b[2]) * (a[0] + b[0]),
                           (a[0] - b[0]) * (a[1] + b[1])])
        n /= np.linalg.norm(n)
        u /= np.linalg.norm(u)
        v = np.cross(n, u)
        poly = np.stack([(p - origin) @ u, (p - origin) @ v], axis=1)
        return cls(origin, u, v, poly, shade, alpha, name, kind)

    def vertices3d(self):
        return self.origin + self.polygon[:, :1] * self.u + self.polygon[:, 1:] * self.v

    def inside(self, s, t):
        """Even-odd point-in-polygon test, vectorised over points."""
        inside = np.zeros(s.shape, dtype=bool)
        poly = self.polygon
        n = len(poly)
        for i in range(n):
            x0, y0 = poly[i]
            x1, y1 = poly[(i + 1) % n]
            if y0 == y1:
                continue
            crosses = (y0 > t) != (y1 > t)
            x_at = x0 + (t - y0) * (x1 - x0) / (y1 - y0)
            inside ^= crosses & (s < x_at)
        return inside


def _rays(camera, supersample):
    """Optical-frame rays (H, W, k*k, 3) for sub-pixel sample positions."""
    k = supersample
    v, u = np.mgrid[0:camera.height, 0:camera.width].astype(float)
    offsets = (np.arange(k) + 0.5) / k - 0.5
    us, vs = [], []
    for dv in offsets:
        for du in offsets:
            us.append(u + du)
            vs.append(v + dv)
    us = np.stack(us, axis=-1)
    vs = np.stack(vs, axis=-1)
    x, y = camera.undistort_pixels(us, vs)
    return np.stack([x, y, np.ones_like(x)], axis=-1)


_RAY_CACHE = {}


def camera_rays(camera, supersample=1):
    key = (camera.width, camera.height, camera.fx, camera.fy, camera.cx, camera.cy,
           tuple(camera.dist), supersample)
    rays = _RAY_CACHE.get(key)
    if rays is None:
        if len(_RAY_CACHE) > 4:
            _RAY_CACHE.clear()
        rays = _rays(camera, supersample)
        _RAY_CACHE[key] = rays
    return rays


def render_pose(surfaces, camera, rotations, positions, supersample=1, background=0.0,
                near=0.02):
    """Render one exposure.

    rotations: (3,3) R_field_cam for a global shutter, or (H,3,3) one per image row.
    positions: (3,) or (H,3) camera centre in the field.
    Returns float32 image (H, W), brightness 0..255 (not clipped, not quantised).

    Each surface is only intersected with the rays inside its projected bounding box (plus a
    margin), so small surfaces cost little; a surface straddling the camera plane uses all rays.
    """
    rays = camera_rays(camera, supersample)            # (H, W, S, 3)
    h, w, s_count, _ = rays.shape
    rot = np.asarray(rotations, dtype=float)
    pos = np.asarray(positions, dtype=float)
    rolling = rot.ndim == 3
    if not rolling:
        dirs = (rays @ rot.T).reshape(-1, 3)
        origins = pos                                    # one origin for all rays
        check = [(rot, pos)]
    else:
        dirs = np.einsum("hij,hwsj->hwsi", rot, rays).reshape(-1, 3)
        origins = np.broadcast_to(pos[:, None, None, :], (h, w, s_count, 3)).reshape(-1, 3)
        check = [(rot[r], pos[r]) for r in (0, h // 2, h - 1)]
    ray_len = np.linalg.norm(dirs, axis=1)
    # One pixel spans about 1/f of a unit-z ray; per sample it is 1/(f k).
    pixel_angle = 1.0 / (max(camera.fx, camera.fy) * supersample)

    color = np.full(dirs.shape[0], background, dtype=np.float32)
    depth = np.full(dirs.shape[0], np.inf)
    see_through = []
    for surf in surfaces:
        box = _pixel_box(surf, camera, check, near)
        if box is False:
            continue
        if box is None:
            sel = None
        else:
            r0, r1, c0, c1 = box
            rr = np.arange(r0, r1)[:, None, None]
            cc = np.arange(c0, c1)[None, :, None]
            ss_ = np.arange(s_count)[None, None, :]
            sel = ((rr * w + cc) * s_count + ss_).ravel()
        if sel is None:
            hit = _intersect(surf, origins, dirs, ray_len, pixel_angle, near)
        else:
            org = origins if not rolling else origins[sel]
            hit = _intersect(surf, org, dirs[sel], ray_len[sel], pixel_angle, near)
        if hit is None:
            continue
        idx, tt, ss, st, foot = hit
        if sel is not None:
            idx = sel[idx]
        if surf.alpha < 1.0:
            see_through.append((surf, idx, tt, ss, st, foot))
            continue
        closer = tt < depth[idx]
        if not np.any(closer):
            continue
        idx, tt, ss, st, foot = idx[closer], tt[closer], ss[closer], st[closer], foot[closer]
        depth[idx] = tt
        color[idx] = surf.shade(ss, st, foot)
    for surf, idx, tt, ss, st, foot in see_through:
        front = tt < depth[idx]
        if not np.any(front):
            continue
        idx = idx[front]
        panel = surf.shade(ss[front], st[front], foot[front])
        color[idx] = surf.alpha * panel + (1.0 - surf.alpha) * color[idx]
    return color.reshape(h, w, s_count).mean(axis=2)


def _pixel_box(surf, camera, poses, near, margin=4):
    """Pixel rows/cols that can see the surface: (r0, r1, c0, c1); None = all; False = none."""
    pts = surf.__dict__.get("_outline")
    if pts is None:
        v = surf.vertices3d()
        nxt = np.roll(v, -1, axis=0)
        # subdivide edges so lens distortion's bending of long edges stays inside the box
        f = np.linspace(0.0, 1.0, 5)[:-1]
        pts = (v[:, None, :] + (nxt - v)[:, None, :] * f[None, :, None]).reshape(-1, 3)
        surf._outline = pts
    u_min = v_min = np.inf
    u_max = v_max = -np.inf
    visible = False
    for rot, pos in poses:
        pc = (pts - pos) @ rot                          # field -> camera
        z = pc[:, 2]
        if np.all(z <= near):
            continue
        if np.any(z <= near):
            pc = _clip_near(pc, near)                    # keep the part in front of the camera
            if len(pc) == 0:
                continue
        uv = camera.project(pc)
        if not np.all(np.isfinite(uv)):
            return None
        visible = True
        u_min = min(u_min, uv[:, 0].min())
        u_max = max(u_max, uv[:, 0].max())
        v_min = min(v_min, uv[:, 1].min())
        v_max = max(v_max, uv[:, 1].max())
    if not visible:
        return False
    c0 = max(0, int(np.floor(u_min)) - margin)
    c1 = min(camera.width, int(np.ceil(u_max)) + margin + 1)
    r0 = max(0, int(np.floor(v_min)) - margin)
    r1 = min(camera.height, int(np.ceil(v_max)) + margin + 1)
    if c0 >= c1 or r0 >= r1:
        return False
    return r0, r1, c0, c1


def _clip_near(ring, near):
    """Sutherland-Hodgman clip of a closed polygon (camera frame) to z >= near."""
    out = []
    n = len(ring)
    for i in range(n):
        a, b = ring[i], ring[(i + 1) % n]
        a_in, b_in = a[2] >= near, b[2] >= near
        if a_in:
            out.append(a)
        if a_in != b_in:
            f = (near - a[2]) / (b[2] - a[2])
            out.append(a + f * (b - a))
    return np.asarray(out).reshape(-1, 3)


def _intersect(surf, origins, dirs, ray_len, pixel_angle, near):
    denom = dirs @ surf.normal
    ok = np.abs(denom) > 1e-12
    num = (surf.origin - origins) @ surf.normal          # scalar for a single origin
    with np.errstate(divide="ignore", invalid="ignore"):
        tt = np.where(ok, num / denom, -1.0)
    cand = np.nonzero(tt * ray_len > near)[0]
    if cand.size == 0:
        return None
    tt = tt[cand]
    org = origins if origins.ndim == 1 else origins[cand]
    p = org + dirs[cand] * tt[:, None] - surf.origin
    ss = p @ surf.u
    st = p @ surf.v
    inside = surf.inside(ss, st)
    if not np.any(inside):
        return None
    cand, tt, ss, st = cand[inside], tt[inside], ss[inside], st[inside]
    dist = tt * ray_len[cand]
    cos_inc = np.abs(denom[cand]) / ray_len[cand]
    foot = dist * pixel_angle / np.maximum(cos_inc, 0.05)
    return cand, tt, ss, st, foot


def render_frame(surfaces, camera, pose_at, t_s, supersample=1, exposure_s=0.0,
                 blur_samples=1, noise_sigma=0.0, rng=None, background=0.0):
    """Render one 8-bit gray frame.

    pose_at(times_s ndarray) -> (x, y, heading) arrays: the robot's field pose.
    t_s: frame time (centre row, mid-exposure). Rolling shutter if camera.rs_readout_s > 0.
    Motion blur: blur_samples sub-frames spread over exposure_s.
    """
    if blur_samples > 1 and exposure_s > 0:
        offsets = (np.arange(blur_samples) + 0.5) / blur_samples * exposure_s - exposure_s / 2
    else:
        offsets = np.array([0.0])
    acc = None
    for off in offsets:
        if camera.rs_readout_s > 0:
            times = t_s + off + camera.row_time_offsets_s()
            xs, ys, hs = pose_at(times)
            rots = np.empty((camera.height, 3, 3))
            poss = np.empty((camera.height, 3))
            for r in range(camera.height):
                rots[r], poss[r] = camera.field_from_camera(xs[r], ys[r], hs[r])
        else:
            xs, ys, hs = pose_at(np.array([t_s + off]))
            rots, poss = camera.field_from_camera(xs[0], ys[0], hs[0])
        img = render_pose(surfaces, camera, rots, poss, supersample, background)
        acc = img if acc is None else acc + img
    img = acc / len(offsets)
    if noise_sigma > 0:
        rng = rng or np.random.default_rng(0)
        img = img + rng.normal(0.0, noise_sigma, img.shape)
    return np.clip(np.rint(img), 0, 255).astype(np.uint8)


# ---------------------------------------------------------------------- textures


def checker_shade(square_m, dark=20.0, light=235.0):
    """Checkerboard brightness with square size square_m; square (0,0) is dark."""
    def shade(s, t, foot):
        i = np.floor(s / square_m).astype(np.int64)
        j = np.floor(t / square_m).astype(np.int64)
        return np.where((i + j) % 2 == 0, dark, light).astype(np.float32)
    return shade


def constant_shade(value):
    def shade(s, t, foot):
        return np.full(s.shape, value, dtype=np.float32)
    return shade


class ImageTexture:
    """A gray texture image mapped onto (s, t) with a mip pyramid against aliasing."""

    def __init__(self, image, metres_per_texel, s0=0.0, t0=0.0, wrap=False):
        img = np.asarray(image, dtype=np.float32)
        self.levels = [img]
        while min(self.levels[-1].shape) > 8:
            a = self.levels[-1]
            h2, w2 = a.shape[0] // 2 * 2, a.shape[1] // 2 * 2
            a = a[:h2, :w2]
            self.levels.append(0.25 * (a[0::2, 0::2] + a[1::2, 0::2] + a[0::2, 1::2] + a[1::2, 1::2]))
        self.mpt = float(metres_per_texel)
        self.s0, self.t0 = float(s0), float(t0)
        self.wrap = wrap

    def __call__(self, s, t, foot):
        level = np.clip(np.floor(np.log2(np.maximum(foot / self.mpt, 1e-9))), 0,
                        len(self.levels) - 1).astype(np.int64)
        out = np.empty(s.shape, dtype=np.float32)
        for lv in np.unique(level):
            m = level == lv
            out[m] = self._bilinear(self.levels[lv], (s[m] - self.s0) / (self.mpt * 2 ** lv),
                                    (t[m] - self.t0) / (self.mpt * 2 ** lv))
        return out

    def _bilinear(self, img, x, y):
        h, w = img.shape
        x = x - 0.5
        y = y - 0.5
        x0 = np.floor(x)
        y0 = np.floor(y)
        fx = (x - x0).astype(np.float32)
        fy = (y - y0).astype(np.float32)
        x0 = x0.astype(np.int64)
        y0 = y0.astype(np.int64)
        if self.wrap:
            xa, xb = x0 % w, (x0 + 1) % w
            ya, yb = y0 % h, (y0 + 1) % h
        else:
            xa, xb = np.clip(x0, 0, w - 1), np.clip(x0 + 1, 0, w - 1)
            ya, yb = np.clip(y0, 0, h - 1), np.clip(y0 + 1, 0, h - 1)
        top = img[ya, xa] * (1 - fx) + img[ya, xb] * fx
        bot = img[yb, xa] * (1 - fx) + img[yb, xb] * fx
        return top * (1 - fy) + bot * fy


def noise_image(rng, height, width, octaves=((1, 1.0),), mean=128.0, contrast=40.0):
    """Multi-scale value noise. octaves: (cell size in texels, weight)."""
    acc = np.zeros((height, width), dtype=np.float32)
    total = 0.0
    for cell, weight in octaves:
        gh = int(math.ceil(height / cell)) + 2
        gw = int(math.ceil(width / cell)) + 2
        grid = rng.standard_normal((gh, gw)).astype(np.float32)
        if cell == 1:
            layer = grid[:height, :width]
        else:
            import cv2  # local import: only the noise generator needs OpenCV
            layer = cv2.resize(grid, ((gw - 1) * cell, (gh - 1) * cell),
                               interpolation=cv2.INTER_CUBIC)[:height, :width]
        acc += weight * layer
        total += weight * weight
    acc /= math.sqrt(total)
    return mean + contrast * acc


def checkerboard_surface(cols_inner, rows_inner, square_m, margin_squares=1.0, name="board"):
    """A printed checkerboard lying in its own plane z = 0 (board frame), white margin.

    Inner corners are at (i * square_m, j * square_m), i < cols_inner, j < rows_inner, and the
    board has cols_inner + 1 by rows_inner + 1 squares, square (-1, -1) dark.
    Returns (shade function, (s, t) polygon) for a Surface placed by the caller.
    """
    s_min = -square_m * (1 + margin_squares)
    t_min = -square_m * (1 + margin_squares)
    s_max = square_m * (cols_inner + margin_squares)
    t_max = square_m * (rows_inner + margin_squares)
    checker = checker_shade(square_m)

    def shade(s, t, foot):
        out = checker(s, t, foot)
        outside = ((s < -square_m) | (s > square_m * cols_inner) |
                   (t < -square_m) | (t > square_m * rows_inner))
        out[outside] = 235.0
        return out

    poly = [(s_min, t_min), (s_max, t_min), (s_max, t_max), (s_min, t_max)]
    return shade, poly
