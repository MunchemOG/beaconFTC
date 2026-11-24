"""Camera model shared by the simulator, the Limelight tools and the evaluation scripts.

Conventions (same as BeaconConfig / FrameMath on the robot, and I2 camera.json):
- Pixels: OpenCV convention. Pixel (u, v) = column, row; the centre of the top-left pixel is
  (0, 0). Lens distortion is OpenCV's 5-parameter Brown-Conrady model [k1, k2, p1, p2, k3].
- Camera optical frame: x right in the image, y down, z out of the lens.
- Robot frame: +x forward, +y left, +z up, origin at the centre of rotation (on the floor).
- Mount (T_robot_cam): R_robot_cam = Rz(yaw) * Ry(pitch) * Rx(roll) * R_FORWARD, where all zeros
  means "looking straight forward, level"; yaw > 0 looks left, pitch > 0 looks down, roll > 0
  raises the image's left edge. Metres and radians.
- Field frame: origin field centre, +z up, heading CCW from +x.
"""
import json
import math
from dataclasses import dataclass, field

import numpy as np

# Columns are the optical axes written in robot axes: optical x (right) = robot -y,
# optical y (down) = robot -z, optical z (forward) = robot +x.
R_FORWARD = np.array([[0.0, 0.0, 1.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0]])

# Limelight 3A datasheet values (docs.limelightvision.io, read 2026-11-17): OV5647 rolling
# shutter, 640x480, FOV H 54.5 deg, V 42 deg. Not calibrated: override with camera.json.
DEFAULT_WIDTH = 640
DEFAULT_HEIGHT = 480
DEFAULT_HFOV_DEG = 54.5


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


@dataclass
class Mount:
    """Camera optical frame in the robot frame (T_robot_cam). Metres, radians."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0

    def rotation(self):
        """R_robot_cam (3x3): maps optical-frame vectors into the robot frame."""
        return rot_z(self.yaw) @ rot_y(self.pitch) @ rot_x(self.roll) @ R_FORWARD

    def translation(self):
        return np.array([self.x, self.y, self.z])

    def to_json(self):
        return {"x": self.x, "y": self.y, "z": self.z,
                "roll": self.roll, "pitch": self.pitch, "yaw": self.yaw}


@dataclass
class Camera:
    """Pinhole camera with OpenCV distortion. fx, fy, cx, cy in pixels."""
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist: tuple = (0.0, 0.0, 0.0, 0.0, 0.0)
    rs_readout_s: float = 0.0
    mount: Mount = field(default_factory=Mount)

    @classmethod
    def from_hfov(cls, width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, hfov_deg=DEFAULT_HFOV_DEG,
                  dist=(0.0, 0.0, 0.0, 0.0, 0.0), rs_readout_s=0.0, mount=None):
        """Square pixels, principal point at the image centre, focal length from the HFOV."""
        fx = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(width, height, fx, fx, (width - 1) / 2.0, (height - 1) / 2.0,
                   tuple(float(d) for d in dist), float(rs_readout_s), mount or Mount())

    def matrix(self):
        return np.array([[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]])

    # ------------------------------------------------------------------ projection

    def distort_normalized(self, x, y):
        """Normalized undistorted coordinates -> normalized distorted coordinates."""
        k1, k2, p1, p2, k3 = self.dist
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return xd, yd

    def project(self, points_cam):
        """Points in the optical frame (N,3) -> pixels (N,2). Points with z <= 0 give NaN."""
        p = np.asarray(points_cam, dtype=float).reshape(-1, 3)
        z = p[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            x = np.where(z > 0, p[:, 0] / z, np.nan)
            y = np.where(z > 0, p[:, 1] / z, np.nan)
        xd, yd = self.distort_normalized(x, y)
        return np.stack([self.fx * xd + self.cx, self.fy * yd + self.cy], axis=1)

    def undistort_pixels(self, u, v, iterations=30):
        """Pixels -> normalized undistorted coordinates (fixed-point iteration, as OpenCV)."""
        xd = (np.asarray(u, dtype=float) - self.cx) / self.fx
        yd = (np.asarray(v, dtype=float) - self.cy) / self.fy
        k1, k2, p1, p2, k3 = self.dist
        x, y = xd.copy(), yd.copy()
        if not any(self.dist):
            return x, y
        for _ in range(iterations):
            r2 = x * x + y * y
            radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
            dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
            dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
            x = (xd - dx) / radial
            y = (yd - dy) / radial
        return x, y

    def pixel_rays(self):
        """Unit-z rays (H, W, 3) in the optical frame for every pixel centre."""
        v, u = np.mgrid[0:self.height, 0:self.width].astype(float)
        x, y = self.undistort_pixels(u, v)
        return np.stack([x, y, np.ones_like(x)], axis=-1)

    def row_time_offsets_s(self):
        """Exposure time of each row relative to the frame time (rolling shutter).

        Convention (recorded in DECISIONS.md): a frame's t_ns is the exposure time of the
        image's centre row; rows are read top to bottom over rs_readout_s.
        """
        r = np.arange(self.height, dtype=float)
        return (r - (self.height - 1) / 2.0) * (self.rs_readout_s / self.height)

    # ------------------------------------------------------------------ poses

    def field_from_camera(self, x, y, heading):
        """Camera pose in the field for a robot pose: (R_field_cam 3x3, position 3)."""
        rz = rot_z(heading)
        r = rz @ self.mount.rotation()
        t = np.array([x, y, 0.0]) + rz @ self.mount.translation()
        return r, t

    # ------------------------------------------------------------------ I2 camera.json

    def to_json(self):
        return {"width": int(self.width), "height": int(self.height),
                "fx": float(self.fx), "fy": float(self.fy),
                "cx": float(self.cx), "cy": float(self.cy),
                "dist": [float(d) for d in self.dist],
                "mount": self.mount.to_json(),
                "rollingShutterReadoutS": float(self.rs_readout_s)}

    @classmethod
    def from_json(cls, obj):
        m = obj.get("mount", {})
        mount = Mount(*(float(m.get(k, 0.0)) for k in ("x", "y", "z", "roll", "pitch", "yaw")))
        dist = list(obj.get("dist", [0.0] * 5)) + [0.0] * 5
        return cls(int(obj["width"]), int(obj["height"]), float(obj["fx"]), float(obj["fy"]),
                   float(obj["cx"]), float(obj["cy"]), tuple(float(d) for d in dist[:5]),
                   float(obj.get("rollingShutterReadoutS", 0.0)), mount)

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_json(), f, indent=2)
            f.write("\n")

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_json(json.load(f))
