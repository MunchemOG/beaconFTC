"""Pinpoint-like odometry from a true trajectory.

What the goBILDA Pinpoint reports is modelled as (defaults from the plan, Task F2):
- per-axis scale error: forward and strafe distances scaled by (1 + s), s drawn uniformly from
  [-0.5 %, +0.5 %] per run (pod wheel diameter / calibration error);
- heading random walk: the IMU heading picks up white-noise increments with a standard deviation
  of 0.5 deg after 60 s (it grows with the square root of time);
- slip bursts: at each collision the pods miss a 1-3 cm displacement in a random direction,
  spread over 0.1 s (and, in the hard profile, the heading jumps by up to +/- 2 deg);
- output at 200 Hz in the Pinpoint's own world frame. The Pinpoint is reset where the robot
  starts, so the first sample is (0, 0, 0), not the field start pose.

Positions are integrated from robot-frame increments rotated by the ODOMETRY heading, as the
Pinpoint does, so heading error turns into position error the way it does on the robot.
"""
import math
from dataclasses import dataclass

import numpy as np


@dataclass
class OdomParams:
    rate_hz: float = 200.0
    scale_error_max: float = 0.005        # +/- 0.5 % per axis
    heading_rw_deg_at_60s: float = 0.5    # std of heading error after 60 s
    slip_m: tuple = (0.01, 0.03)          # slip burst size range
    slip_duration_s: float = 0.1
    heading_jump_deg_max: float = 0.0     # heading jump at each collision, uniform +/- this
    position_noise_m: float = 0.0001      # white noise on each reported sample


PROFILES = {
    # Plan F2 defaults.
    "nominal": OdomParams(),
    # Plan F2b: harsh enough that odometry alone fails the Stage 1 gate.
    "hard": OdomParams(scale_error_max=0.015, heading_rw_deg_at_60s=2.0, slip_m=(0.02, 0.05),
                       heading_jump_deg_max=2.0),
}
# Collisions per 150 s and default start-pose error (cm, deg) for each profile.
PROFILE_COLLISIONS_PER_150S = {"nominal": None, "hard": (4, 8)}
PROFILE_START_ERROR = {"nominal": (0.0, 0.0), "hard": (5.0, 3.0)}


@dataclass
class OdomResult:
    t: np.ndarray      # seconds, same base as the trajectory
    x: np.ndarray      # Pinpoint world frame
    y: np.ndarray
    h: np.ndarray      # unwrapped
    scale_x: float
    scale_y: float
    slips: list        # (time, dx_robot, dy_robot, heading_jump_rad)


def simulate(traj_t, traj_x, traj_y, traj_h, seed=0, params=None, collisions=()):
    """Odometry samples at params.rate_hz over the trajectory's time span."""
    p = params or OdomParams()
    rng = np.random.default_rng(seed)
    sx = 1.0 + rng.uniform(-p.scale_error_max, p.scale_error_max)
    sy = 1.0 + rng.uniform(-p.scale_error_max, p.scale_error_max)
    dt = 1.0 / p.rate_hz
    t = np.arange(0.0, traj_t[-1] + 1e-9, dt)
    x = np.interp(t, traj_t, traj_x)
    y = np.interp(t, traj_t, traj_y)
    h = np.interp(t, traj_t, traj_h)

    # true increments in the robot frame (midpoint heading)
    dx, dy, dh = np.diff(x), np.diff(y), np.diff(h)
    hm = h[:-1] + dh / 2
    c, s = np.cos(hm), np.sin(hm)
    fwd = c * dx + s * dy
    left = -s * dx + c * dy

    # slip bursts: the pods miss part of the motion
    slips = []
    jumps = []
    for tc in collisions:
        mag = rng.uniform(*p.slip_m)
        ang = rng.uniform(-math.pi, math.pi)
        ex, ey = mag * math.cos(ang), mag * math.sin(ang)
        k0 = int(tc / dt)
        k1 = min(len(fwd), k0 + max(1, int(round(p.slip_duration_s / dt))))
        if k0 >= len(fwd):
            continue
        n = k1 - k0
        fwd[k0:k1] -= ex / n
        left[k0:k1] -= ey / n
        jump = 0.0
        if p.heading_jump_deg_max > 0:  # drawn only here, so nominal runs keep their numbers
            jump = math.radians(rng.uniform(-p.heading_jump_deg_max, p.heading_jump_deg_max))
        jumps.append((k0, k1, jump))
        slips.append((float(tc), ex, ey, jump))

    # heading: true change plus random walk
    sigma_step = math.radians(p.heading_rw_deg_at_60s) * math.sqrt(dt / 60.0)
    dh_odom = dh + rng.normal(0.0, sigma_step, dh.shape)
    for k0, k1, jump in jumps:
        dh_odom[k0:k1] += jump / (k1 - k0)
    ho = np.concatenate([[0.0], np.cumsum(dh_odom)])
    hmo = ho[:-1] + dh_odom / 2
    co, so = np.cos(hmo), np.sin(hmo)
    fx, fy = sx * fwd, sy * left
    xo = np.concatenate([[0.0], np.cumsum(co * fx - so * fy)])
    yo = np.concatenate([[0.0], np.cumsum(so * fx + co * fy)])
    if p.position_noise_m > 0:
        xo = xo + rng.normal(0.0, p.position_noise_m, xo.shape)
        yo = yo + rng.normal(0.0, p.position_noise_m, yo.shape)
        xo[0] = yo[0] = 0.0
    return OdomResult(t, xo, yo, ho, sx, sy, slips)
