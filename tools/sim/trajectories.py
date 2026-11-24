"""Robot trajectories for the synthetic bench: match, session, spin, blind, clutter.

A trajectory is the robot's TRUE field pose sampled at 1 kHz (x, y in metres, heading in
radians, unwrapped). It is built from moves between waypoints with a minimum-jerk profile, so
speed, acceleration and turn rate stay within the limits given (design speeds 2.7 m/s and
7 rad/s). The robot (an 18 in square, so a 0.33 m circle when turning) never crosses the walls,
the hive frame or the FLOWERs. Everything is seeded and repeatable.

Kinds:
- match   (default 150 s): 3 s still (INIT), 30 s autonomous at up to 1.5 m/s, 8 s still
          (transition), then teleop at up to 2.7 m/s and 7 rad/s with stops and quick spins.
- session (default 1200 s): teleop-like driving with pauses, for drift over a practice session.
- spin    (default 60 s): spins up to 7 rad/s in open floor, with short drives between.
- blind   (default 45 s): normal driving, then 10 s parked facing a corner where a partner robot
          blocks the view of the walls (see Trajectory.static_boxes), then driving again.
- clutter (default 150 s): like match, plus three other robots driving around and game pieces
          on the floor (Trajectory.movers, Trajectory.static_boxes); contacts count as collisions.
"""
import math
from dataclasses import dataclass, field as dc_field

import numpy as np

RATE_HZ = 1000.0
ROBOT_HALF = 0.2286          # 18 in robot
ROBOT_RADIUS = 0.33          # turning circle of the 18 in square, rounded up
DEFAULT_DURATIONS = {"match": 150.0, "session": 1200.0, "spin": 60.0, "blind": 45.0,
                     "clutter": 150.0}
KINDS = tuple(DEFAULT_DURATIONS)


@dataclass
class Limits:
    v_max: float = 2.7       # m/s
    a_max: float = 3.5       # m/s^2
    w_max: float = 7.0       # rad/s
    alpha_max: float = 25.0  # rad/s^2


@dataclass
class Box:
    """An axis-aligned-in-yaw box on the floor (another robot, a game piece, an occluder)."""
    name: str
    size: tuple              # (length x, width y, height z), metres
    kind: str = "robot"      # robot | pollen | nectar | occluder
    pose: tuple = (0.0, 0.0, 0.0)


@dataclass
class Mover:
    """A box that drives: its own 1 kHz pose track on the same time base."""
    box: Box
    t: np.ndarray
    x: np.ndarray
    y: np.ndarray
    h: np.ndarray

    def pose_at(self, ts):
        return (np.interp(ts, self.t, self.x), np.interp(ts, self.t, self.y),
                np.interp(ts, self.t, self.h))


@dataclass
class Trajectory:
    kind: str
    t: np.ndarray            # seconds from the start of the run
    x: np.ndarray
    y: np.ndarray
    h: np.ndarray            # unwrapped heading
    still_until_s: float     # robot motionless (INIT) until this time
    collisions: list = dc_field(default_factory=list)   # times (s) of contacts
    static_boxes: list = dc_field(default_factory=list)
    movers: list = dc_field(default_factory=list)
    blind_window_s: tuple = None

    def pose_at(self, ts):
        ts = np.asarray(ts, dtype=float)
        return (np.interp(ts, self.t, self.x), np.interp(ts, self.t, self.y),
                np.interp(ts, self.t, self.h))

    @property
    def duration_s(self):
        return float(self.t[-1])

    def speeds(self):
        dt = 1.0 / RATE_HZ
        v = np.hypot(np.gradient(self.x, dt), np.gradient(self.y, dt))
        w = np.gradient(self.h, dt)
        return v, w


# ----------------------------------------------------------------------------- free space


class FreeSpace:
    """Where the robot centre may be: inside the walls, clear of the hive frame and FLOWERs."""

    def __init__(self, field, radius=ROBOT_RADIUS, extra_boxes=()):
        self.half = field.floor.half_size_m
        self.radius = radius
        self.keep_out = []                   # (xmin, ymin, xmax, ymax), already inflated
        frame = [np.asarray(f.vertices) for f in field.hive_faces if f.part == "frame"]
        if frame:
            pts = np.vstack(frame)
            low = pts[pts[:, 2] < 0.6]       # legs and feet near the floor
            self.keep_out.append((low[:, 0].min() - radius, low[:, 1].min() - radius,
                                  low[:, 0].max() + radius, low[:, 1].max() + radius))
        for fl in field.flowers:
            self.keep_out.append((fl["min"][0] - radius, fl["min"][1] - radius,
                                  fl["max"][0] + radius, fl["max"][1] + radius))
        for b in extra_boxes:
            r = math.hypot(b.size[0], b.size[1]) / 2 + radius
            self.keep_out.append((b.pose[0] - r, b.pose[1] - r, b.pose[0] + r, b.pose[1] + r))

    def ok(self, x, y, wall_radius=None):
        r = self.radius if wall_radius is None else wall_radius
        if abs(x) > self.half - r or abs(y) > self.half - r:
            return False
        for x0, y0, x1, y1 in self.keep_out:
            if x0 < x < x1 and y0 < y < y1:
                return False
        return True

    def segment_ok(self, p, q, wall_radius=None):
        """Exact test: both ends inside the walls and the segment misses every keep-out box."""
        r = self.radius if wall_radius is None else wall_radius
        for x, y in (p, q):
            if abs(x) > self.half - r or abs(y) > self.half - r:
                return False
        return not any(_segment_hits_box(p, q, box) for box in self.keep_out)

    def random_point(self, rng, tries=1000):
        for _ in range(tries):
            x = rng.uniform(-self.half, self.half)
            y = rng.uniform(-self.half, self.half)
            if self.ok(x, y):
                return x, y
        raise RuntimeError("no free point found")


def _segment_hits_box(p, q, box):
    """Liang-Barsky: does the segment p-q pass through the open box (x0, y0, x1, y1)?"""
    x0, y0, x1, y1 = box
    dx, dy = q[0] - p[0], q[1] - p[1]
    t0, t1 = 0.0, 1.0
    for d, lo, hi, s in ((dx, x0, x1, p[0]), (dy, y0, y1, p[1])):
        if abs(d) < 1e-12:
            if not (lo < s < hi):
                return False
            continue
        a, b = (lo - s) / d, (hi - s) / d
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t0 >= t1:
            return False
    return True


# ----------------------------------------------------------------------------- builder


class Builder:
    """Appends moves and holds to a 1 kHz pose track."""

    def __init__(self, x, y, h):
        self.xs, self.ys, self.hs = [np.array([x])], [np.array([y])], [np.array([h])]
        self.x, self.y, self.h = x, y, h
        self.n = 1

    @property
    def t(self):
        return (self.n - 1) / RATE_HZ

    def hold(self, seconds):
        k = int(round(seconds * RATE_HZ))
        if k > 0:
            self.xs.append(np.full(k, self.x))
            self.ys.append(np.full(k, self.y))
            self.hs.append(np.full(k, self.h))
            self.n += k

    def move(self, x, y, h, limits):
        """Minimum-jerk move to (x, y, h); h is taken as the nearest equivalent angle."""
        h = self.h + _wrap(h - self.h)
        d = math.hypot(x - self.x, y - self.y)
        dh = abs(h - self.h)
        # min-jerk: peak speed 1.875 D/T, peak acceleration 5.774 D/T^2
        t_lin = max(1.875 * d / limits.v_max, math.sqrt(5.774 * d / limits.a_max)) if d > 0 else 0
        t_rot = max(1.875 * dh / limits.w_max, math.sqrt(5.774 * dh / limits.alpha_max)) if dh > 0 else 0
        T = max(t_lin, t_rot, 0.05)
        k = max(1, int(math.ceil(T * RATE_HZ)))
        s = np.arange(1, k + 1) / k
        prof = 10 * s ** 3 - 15 * s ** 4 + 6 * s ** 5
        self.xs.append(self.x + (x - self.x) * prof)
        self.ys.append(self.y + (y - self.y) * prof)
        self.hs.append(self.h + (h - self.h) * prof)
        self.x, self.y, self.h = x, y, h
        self.n += k

    def arrays(self):
        x = np.concatenate(self.xs)
        return (np.arange(len(x)) / RATE_HZ, x, np.concatenate(self.ys), np.concatenate(self.hs))


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _escape(b, free, limits):
    """From a pose touching a wall (the start pose), back straight out toward the centre."""
    if free.ok(b.x, b.y):
        return
    for f in np.linspace(0.02, 1.0, 50):
        x, y = b.x * (1 - f), b.y * (1 - f)
        if free.ok(x, y):
            b.move(x, y, b.h, limits)
            return


def _random_drive(b, rng, free, limits, until_s, dwell=(0.0, 1.5), spin_prob=0.15,
                  max_leg=2.5):
    """Random waypoint driving until time until_s."""
    _escape(b, free, limits)
    while b.t < until_s:
        for _ in range(200):
            x, y = free.random_point(rng)
            if math.hypot(x - b.x, y - b.y) <= max_leg and free.segment_ok((b.x, b.y), (x, y)):
                break
        else:
            x, y = b.x, b.y
        h = rng.uniform(-math.pi, math.pi)
        b.move(x, y, h, limits)
        if rng.random() < spin_prob:
            turns = rng.choice([-1, 1]) * rng.uniform(0.5, 1.5) * 2 * math.pi
            b.move(b.x, b.y, b.h + turns, limits)
        b.hold(rng.uniform(*dwell))


def _start_pose(field, rng, side=None):
    """Robot against the red or blue wall, facing into the field."""
    half = field.floor.half_size_m
    side = side if side is not None else rng.choice([-1, 1])
    x = rng.choice([-1, 1]) * rng.uniform(0.9, 1.4)
    y = side * (half - ROBOT_HALF - 0.005)
    h = -side * math.pi / 2  # red side (y < 0) faces +y
    return x, y, h


def make_trajectory(kind, field, duration_s=None, seed=0, limits=None):
    if kind not in KINDS:
        raise ValueError("unknown trajectory kind %r (choose from %s)" % (kind, ", ".join(KINDS)))
    rng = np.random.default_rng(seed)
    duration = float(duration_s or DEFAULT_DURATIONS[kind])
    limits = limits or Limits()
    slow = Limits(1.5, 2.5, 3.0, 12.0)
    static_boxes, movers = [], []
    blind_window = None
    if kind == "clutter":
        static_boxes = _game_pieces(field, rng, 30)
    free = FreeSpace(field, extra_boxes=[b for b in static_boxes if b.kind == "occluder"])
    x0, y0, h0 = _start_pose(field, rng)
    b = Builder(x0, y0, h0)
    still = 3.0
    b.hold(still)
    if kind in ("match", "clutter"):
        _random_drive(b, rng, free, slow, min(still + 30.0, duration), dwell=(0.5, 1.5),
                      spin_prob=0.0, max_leg=1.8)
        if b.t < duration:
            b.hold(min(8.0, duration - b.t))
        _random_drive(b, rng, free, limits, duration)
    elif kind == "session":
        while b.t < duration:
            _random_drive(b, rng, free, limits, min(b.t + rng.uniform(60, 150), duration))
            b.hold(min(rng.uniform(3, 15), max(duration - b.t, 0)))
    elif kind == "spin":
        while b.t < duration:
            _random_drive(b, rng, free, slow, b.t + 0.1, dwell=(0.2, 0.5), spin_prob=0.0,
                          max_leg=1.0)
            turns = rng.choice([-1, 1]) * rng.uniform(1.0, 3.0) * 2 * math.pi
            b.move(b.x, b.y, b.h + turns, limits)
            b.hold(rng.uniform(0.2, 1.0))
    elif kind == "blind":
        _random_drive(b, rng, free, slow, still + 10.0, dwell=(0.3, 1.0), spin_prob=0.0)
        static_boxes, park, face_h = _corner_occluder(field, rng)
        _drive_to(b, rng, free, park, limits=slow)
        b.move(park[0], park[1], face_h, slow)
        t0 = b.t
        b.hold(10.0)
        blind_window = (t0, b.t)
        # back out of the corner along the diagonal, then drive on
        for f in np.linspace(0.1, 0.5, 9)[::-1]:
            out = (park[0] * (1 - f), park[1] * (1 - f))
            if free.ok(*out) and free.segment_ok(park, out, wall_radius=ROBOT_HALF):
                b.move(out[0], out[1], b.h, slow)
                break
        _random_drive(b, rng, free, slow, duration, dwell=(0.3, 1.0), spin_prob=0.0)
    t, x, y, h = b.arrays()
    keep = t <= duration + 1e-9
    t, x, y, h = t[keep], x[keep], y[keep], h[keep]
    collisions = []
    if kind == "clutter":
        movers = _other_robots(field, rng, duration, 3)
        collisions = _contacts(t, x, y, movers)
    elif kind in ("match", "session"):
        n = max(1, int(round(duration / 50.0)))   # a few bumps per match
        collisions = sorted(rng.uniform(still + 5, max(duration - 1, still + 6), n).tolist())
    return Trajectory(kind, t, x, y, h, still, collisions, static_boxes, movers, blind_window)


def _drive_to(b, rng, free, target, limits):
    """Drive to a target through intermediate random points when the straight line is blocked."""
    for _ in range(50):
        if free.segment_ok((b.x, b.y), target, wall_radius=ROBOT_HALF):
            b.move(target[0], target[1], b.h, limits)
            return
        x, y = free.random_point(rng)
        if free.segment_ok((b.x, b.y), (x, y)):
            b.move(x, y, b.h, limits)
    b.move(target[0], target[1], b.h, limits)


def _corner_occluder(field, rng):
    """A partner robot parked in a free corner, and where our robot parks facing it."""
    half = field.floor.half_size_m
    sx, sy = rng.choice([-1, 1]), rng.choice([-1, 1])
    occ = Box("partner", (0.457, 0.457, 0.40), "occluder",
              (sx * (half - 0.25), sy * (half - 0.25), math.pi / 4))
    # park on the diagonal, 0.05 m gap between the two robots, facing the corner
    d = 0.25 * math.sqrt(2) + 0.457 / math.sqrt(2) + ROBOT_HALF + 0.05
    cx = sx * (half - 0.25) - sx * d / math.sqrt(2)
    cy = sy * (half - 0.25) - sy * d / math.sqrt(2)
    face = math.atan2(sy, sx)
    return [occ], (cx, cy), face


def _game_pieces(field, rng, count):
    half = field.floor.half_size_m - 0.05
    out = []
    for i in range(count):
        kind = "pollen" if rng.random() < 0.8 else "nectar"
        d = 0.071 if kind == "pollen" else 0.091
        out.append(Box("%s%d" % (kind, i), (d, d, d), kind,
                       (rng.uniform(-half, half), rng.uniform(-half, half), rng.uniform(0, math.pi))))
    return out


def _other_robots(field, rng, duration, count):
    movers = []
    for i in range(count):
        free = FreeSpace(field)
        x, y = free.random_point(rng)
        b = Builder(x, y, rng.uniform(-math.pi, math.pi))
        _random_drive(b, rng, free, Limits(1.8, 2.5, 4.0, 15.0), duration, dwell=(0.5, 3.0),
                      spin_prob=0.05)
        t, xs, ys, hs = b.arrays()
        movers.append(Mover(Box("robot%d" % i, (0.457, 0.457, rng.uniform(0.25, 0.45)), "robot"),
                            t, xs, ys, hs))
    return movers


def _contacts(t, x, y, movers, distance=0.47):
    """Times when another robot comes into contact (first touch of each encounter)."""
    times = []
    for m in movers:
        mx, my, _ = m.pose_at(t)
        close = np.hypot(mx - x, my - y) < distance
        starts = np.nonzero(close[1:] & ~close[:-1])[0]
        times.extend(t[starts + 1].tolist())
    return sorted(times)
