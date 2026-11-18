"""Synthetic data with a known answer, used by the tests.

SyntheticCamera renders a Target through a pinhole-radtan or equidistant-fisheye model by
casting a ray through every pixel and intersecting it with the board plane. Nothing here is
imported by the tools: the tools must recover the numbers from the pictures alone.
"""
import csv
import os

import cv2
import numpy as np

SUPERSAMPLE = 2


def rot_xyz(ax, ay, az):
    cx, sx = np.cos(ax), np.sin(ax)
    cy, sy = np.cos(ay), np.sin(ay)
    cz, sz = np.cos(az), np.sin(az)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return rz @ ry @ rx


class SyntheticCamera:
    def __init__(self, width, height, fx, fy, cx, cy, dist, model="radtan"):
        self.w, self.h = width, height
        self.K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1.0]])
        self.dist = np.array(dist, dtype=np.float64)
        self.model = model
        s = SUPERSAMPLE
        us = (np.arange(width * s) + 0.5) / s - 0.5
        vs = (np.arange(height * s) + 0.5) / s - 0.5
        uu, vv = np.meshgrid(us, vs)
        pts = np.stack([uu.ravel(), vv.ravel()], axis=1).reshape(-1, 1, 2).astype(np.float64)
        if model == "equidistant":
            n = cv2.fisheye.undistortPoints(pts, self.K, self.dist.reshape(4, 1))
        else:
            n = cv2.undistortPoints(pts, self.K, self.dist)
        self.rays = n.reshape(height * s, width * s, 2)

    def project(self, obj_pts, R, t):
        rvec, _ = cv2.Rodrigues(R)
        pts = obj_pts.reshape(-1, 1, 3).astype(np.float64)
        if self.model == "equidistant":
            img, _ = cv2.fisheye.projectPoints(pts, rvec, t.reshape(3, 1), self.K,
                                               self.dist.reshape(4, 1))
        else:
            img, _ = cv2.projectPoints(pts, rvec, t.reshape(3, 1), self.K, self.dist)
        return img.reshape(-1, 2)

    def render(self, target, tex, ppm, origin, R, t, blur_sigma=0.6, noise=1.5, rng=None,
               motion_len=0):
        """Render the board texture (from Target.render) seen with board->camera pose R, t."""
        s = SUPERSAMPLE
        hh, ww = self.rays.shape[:2]
        rays = np.concatenate([self.rays, np.ones((hh, ww, 1))], axis=2).reshape(-1, 3)
        rt = R.T
        origin_c = rt @ t  # camera centre expressed in the board frame, negated below
        dirs_b = np.stack([rays[:, 0] * R[0, k] + rays[:, 1] * R[1, k] + rays[:, 2] * R[2, k]
                           for k in range(3)], axis=1)  # rows: R^T r (no BLAS: avoids spurious warnings)
        lam = origin_c[2] / dirs_b[:, 2]
        xb = (lam[:, None] * dirs_b - origin_c[None, :])
        valid = lam > 0
        px = origin[0] + xb[:, 0] * ppm - 0.5
        py = origin[1] + xb[:, 1] * ppm - 0.5
        px[~valid] = -1e6
        py[~valid] = -1e6
        img = cv2.remap(tex, px.reshape(hh, ww).astype(np.float32),
                        py.reshape(hh, ww).astype(np.float32), cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=70)
        img = cv2.resize(img, (self.w, self.h), interpolation=cv2.INTER_AREA).astype(np.float32)
        if blur_sigma > 0:
            img = cv2.GaussianBlur(img, (0, 0), blur_sigma)
        if motion_len > 1:
            k = np.zeros((motion_len, motion_len), np.float32)
            k[motion_len // 2, :] = 1.0 / motion_len
            img = cv2.filter2D(img, -1, k)
        if noise > 0:
            rng = rng or np.random.default_rng(0)
            img = img + rng.normal(0.0, noise, img.shape).astype(np.float32)
        return np.clip(img, 0, 255).astype(np.uint8)


def random_pose(rng, target, cam, dist_range=(0.30, 0.55), tilt_deg=40.0, center_frac=0.9,
                spread=1.0):
    """Board->camera pose with the board centre landing somewhere in the image.

    spread=1 puts the board centre anywhere over the image; smaller keeps it near the middle.
    """
    d = rng.uniform(*dist_range)
    tilt = np.radians(tilt_deg)
    R = rot_xyz(rng.uniform(-tilt, tilt), rng.uniform(-tilt, tilt), rng.uniform(-0.6, 0.6))
    u = cam.K[0, 2] + spread * rng.uniform(-center_frac, center_frac) * cam.w / 2
    v = cam.K[1, 2] + spread * rng.uniform(-center_frac, center_frac) * cam.h / 2
    ray = np.array([(u - cam.K[0, 2]) / cam.K[0, 0], (v - cam.K[1, 2]) / cam.K[1, 1], 1.0])
    centre_b = np.array([target.width / 2, target.height / 2, 0.0])
    # place the board centre at depth d along `ray`
    t = d * ray - R @ centre_b
    return R, t


def write_dataset(directory, frames, fps=30.0, imu=None, meta=None):
    """Write a dataset directory in the layout of calibration/README.md."""
    os.makedirs(os.path.join(directory, "images"), exist_ok=True)
    t0 = 1_000_000_000_000
    with open(os.path.join(directory, "cam_timestamps.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame_id", "t_ns", "exposure_ns", "iso"])
        for i, img in enumerate(frames):
            cv2.imwrite(os.path.join(directory, "images", f"{i:06d}.pgm"), img)
            w.writerow([i, int(t0 + i * 1e9 / fps), 3_000_000, 100])
    if meta is not None:
        import yaml
        with open(os.path.join(directory, "meta.yaml"), "w") as f:
            yaml.safe_dump(meta, f)


GOOD_META = {
    "device": "synthetic", "width": 640, "height": 480, "ae_mode": "off", "af_mode": "off",
    "ois": "off", "timestamp_source": "REALTIME", "exposure_ns": 3000000, "iso": 100,
}
