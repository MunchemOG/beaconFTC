"""Tests for the synthetic dataset generator (tools/sim)."""
import csv
import json
import math
import os
import sys

import cv2
import numpy as np
import pytest

from conftest import TOOLS_DIR, run_tool

sys.path.insert(0, os.path.join(TOOLS_DIR, "sim"))

import fieldmodel  # noqa: E402
import odomsim  # noqa: E402
import trajectories  # noqa: E402
from camera import Camera, Mount, rot_x, rot_y, rot_z  # noqa: E402
from render import Surface, checkerboard_surface, render_pose  # noqa: E402

FIELD = os.path.join(TOOLS_DIR, "..", "beacon-field", "biobuzz-2026.json")


@pytest.fixture(scope="module")
def field():
    return fieldmodel.load_field(FIELD)


def read_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    return rows[0], np.array([[float(v) for v in r] for r in rows[1:]])


# ----------------------------------------------------------------------------- projection


def test_projection_matches_pinhole():
    cam = Camera(640, 480, 500.0, 510.0, 320.5, 240.25)
    p = np.array([[0.2, -0.1, 2.0]])
    u, v = cam.project(p)[0]
    assert abs(u - (500.0 * 0.1 + 320.5)) < 1e-9 and abs(v - (510.0 * -0.05 + 240.25)) < 1e-9
    # with distortion: the OpenCV model, checked against cv2.projectPoints
    cam.dist = (-0.12, 0.05, 0.001, -0.002, 0.01)
    pts = np.array([[0.3, 0.2, 1.5], [-0.4, 0.1, 1.0], [0.0, -0.3, 2.5]])
    ours = cam.project(pts)
    ref, _ = cv2.projectPoints(pts, np.zeros(3), np.zeros(3), cam.matrix(), np.array(cam.dist))
    assert np.abs(ours - ref.reshape(-1, 2)).max() < 1e-6
    # undistortion inverts distortion
    x, y = cam.undistort_pixels(ours[:, 0], ours[:, 1])
    assert np.allclose(x, pts[:, 0] / pts[:, 2], atol=1e-7)
    assert np.allclose(y, pts[:, 1] / pts[:, 2], atol=1e-7)


def test_rendered_corners_land_on_projected_points():
    """Rendered checkerboard corners sit at the analytic pixels: no bias, sub-0.1 px scatter.

    project() itself is exact (test above). Here the image is rendered and the corners are found
    by OpenCV, whose own corner finder scatters by about 0.06 px; so we check that the mean error
    (a bias, e.g. a half-pixel convention slip) is under 0.03 px and the RMS under 0.1 px.
    """
    cam = Camera(640, 480, 520.0, 520.0, 319.5, 239.5, (-0.08, 0.02, 0, 0, 0))
    sq = 0.04
    shade, poly = checkerboard_surface(9, 6, sq)
    r = rot_y(0.3) @ rot_x(-0.2)
    t = np.array([0.0, 0.0, 0.75]) - r @ np.array([4 * sq, 2.5 * sq, 0.0])
    surf = Surface(t, r[:, 0], r[:, 1], poly, shade)
    img = render_pose([surf], cam, np.eye(3), np.zeros(3), supersample=8, background=128.0)
    img = np.clip(np.rint(img), 0, 255).astype(np.uint8)
    ok, corners = cv2.findChessboardCorners(img, (9, 6))
    assert ok
    corners = cv2.cornerSubPix(img, corners, (7, 7), (-1, -1),
                               (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-5))
    obj = np.array([[i * sq, j * sq, 0.0] for j in range(6) for i in range(9)])
    expected = cam.project((r @ obj.T).T + t)
    found = corners.reshape(-1, 2)
    if np.linalg.norm(found[0] - expected[0]) > np.linalg.norm(found[0] - expected[-1]):
        found = found[::-1]  # OpenCV may list the corners from the other end
    d = found - expected
    assert np.abs(d.mean(axis=0)).max() < 0.03, d.mean(axis=0)
    assert math.sqrt((d ** 2).sum(axis=1).mean()) < 0.1


def test_mount_convention_matches_beacon_config():
    """All zeros looks along robot +x; pitch > 0 looks down; yaw > 0 looks left."""
    m = Mount()
    assert np.allclose(m.rotation() @ [0, 0, 1], [1, 0, 0])      # optical z -> robot forward
    assert np.allclose(m.rotation() @ [1, 0, 0], [0, -1, 0])     # image right -> robot right
    down = Mount(pitch=0.3).rotation() @ [0, 0, 1]
    assert down[2] < 0
    left = Mount(yaw=0.3).rotation() @ [0, 0, 1]
    assert left[1] > 0


# ----------------------------------------------------------------------------- odometry


def straight_truth(seconds=20.0, speed=1.0):
    t = np.arange(0, seconds, 0.001)
    x = speed * t
    return t, x, np.zeros_like(t), np.zeros_like(t)


def test_odom_drift_statistics():
    params = odomsim.OdomParams()
    t, x, y, h = straight_truth()
    scales = []
    for seed in range(100):
        r = odomsim.simulate(t, x, y, h, seed=seed, params=params)
        assert r.x[0] == 0 and r.y[0] == 0 and r.h[0] == 0
        dist = math.hypot(r.x[-1], r.y[-1])
        scales.append(dist / np.interp(r.t[-1], t, x) - 1.0)
        assert abs(r.t[1] - r.t[0] - 1 / 200.0) < 1e-12        # 200 Hz
    scales = np.array(scales)
    assert np.all(np.abs(scales) <= 0.005 + 2e-5), "scale error within +/- 0.5 %"
    assert scales.max() > 0.003 and scales.min() < -0.003, "uses the configured range"
    # heading random walk: std of the heading error after 60 s is 0.5 deg
    t = np.arange(0, 60.0001, 0.001)
    still = np.zeros_like(t)
    errs = [odomsim.simulate(t, still, still, still, seed=s).h[-1] for s in range(400)]
    sd = math.degrees(float(np.std(errs)))
    assert 0.43 < sd < 0.57, sd


def test_odom_slip_burst():
    t, x, y, h = straight_truth(10.0)
    clean = odomsim.simulate(t, x, y, h, seed=3)
    slip = odomsim.simulate(t, x, y, h, seed=3, collisions=[5.0])
    d = math.hypot(slip.x[-1] - clean.x[-1], slip.y[-1] - clean.y[-1])
    assert 0.0095 < d < 0.031, d
    assert len(slip.slips) == 1


# ----------------------------------------------------------------------------- trajectories


@pytest.mark.parametrize("kind", ["match", "spin", "blind", "clutter"])
def test_trajectory_limits(field, kind):
    tr = trajectories.make_trajectory(kind, field, seed=2)
    v, w = tr.speeds()
    assert v.max() <= 2.7 * 1.001 and np.abs(w).max() <= 7.0 * 1.001
    half = field.floor.half_size_m
    assert np.abs(tr.x).max() <= half - trajectories.ROBOT_HALF + 1e-6
    assert np.abs(tr.y).max() <= half - trajectories.ROBOT_HALF + 1e-6
    free = trajectories.FreeSpace(field)
    for x0, y0, x1, y1 in free.keep_out[:1]:  # the hive frame
        inside = (tr.x > x0) & (tr.x < x1) & (tr.y > y0) & (tr.y < y1)
        assert not inside.any()
    still = tr.t <= tr.still_until_s
    assert np.ptp(tr.x[still]) == 0 and np.ptp(tr.h[still]) == 0, "motionless during INIT"
    assert abs(tr.duration_s - trajectories.DEFAULT_DURATIONS[kind]) < 0.01
    if kind == "match":
        assert v.max() > 2.0 and np.abs(w).max() > 4.0, "exercises the design speeds"
    if kind == "spin":
        assert np.abs(w).max() > 6.5
    if kind == "blind":
        a, b = tr.blind_window_s
        assert b - a >= 10.0 - 1e-6 and tr.static_boxes
    if kind == "clutter":
        assert len(tr.movers) == 3 and len(tr.static_boxes) >= 20


def test_trajectory_is_repeatable(field):
    a = trajectories.make_trajectory("match", field, 30, seed=5)
    b = trajectories.make_trajectory("match", field, 30, seed=5)
    c = trajectories.make_trajectory("match", field, 30, seed=6)
    assert np.array_equal(a.x, b.x) and not np.array_equal(a.x, c.x)


# ----------------------------------------------------------------------------- dataset


def test_dataset_layout(tmp_path):
    out = tmp_path / "run"
    r = run_tool("sim/make_dataset.py",
                 ["--field", FIELD, "--traj", "match", "--duration", "6", "--seed", "4",
                  "--fps", "5", "--width", "160", "--height", "120", "--workers", "2",
                  "--start-error-cm", "10", "--start-error-deg", "5", "--out", str(out)],
                 tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    for name in ("camera.json", "field.json", "frames.csv", "odom.csv", "start.csv", "truth.csv",
                 "hive.csv", "meta.json"):
        assert (out / name).is_file(), name
    cam = json.loads((out / "camera.json").read_text())
    assert set(cam) == {"width", "height", "fx", "fy", "cx", "cy", "dist", "mount",
                        "rollingShutterReadoutS"}
    assert set(cam["mount"]) == {"x", "y", "z", "roll", "pitch", "yaw"}
    assert (out / "field.json").read_bytes() == open(FIELD, "rb").read()
    head, frames = read_csv(out / "frames.csv")
    assert head == ["idx", "t_ns"] and len(frames) == 31
    assert np.all(np.diff(frames[:, 1]) > 0)
    for k in (0, 30):
        img = cv2.imread(str(out / "frames" / ("%06d.png" % k)), cv2.IMREAD_UNCHANGED)
        assert img.shape == (120, 160) and img.dtype == np.uint8 and img.std() > 5
    for name in ("odom.csv", "truth.csv", "start.csv"):
        head, rows = read_csv(out / name)
        assert head == ["t_ns", "x", "y", "heading"], name
        assert np.all(np.diff(rows[:, 0]) > 0), name
        assert np.all(np.abs(rows[:, 3]) <= math.pi + 1e-9), name
    _, odom = read_csv(out / "odom.csv")
    _, truth = read_csv(out / "truth.csv")
    _, start = read_csv(out / "start.csv")
    assert len(odom) >= 6 * 200 and np.allclose(np.diff(odom[:, 0]), 5e6)
    assert np.allclose(odom[0, 1:], 0.0), "Pinpoint world starts at zero"
    err = math.hypot(start[0, 1] - truth[0, 1], start[0, 2] - truth[0, 2])
    assert abs(err - 0.10) < 1e-6
    dh = (start[0, 3] - truth[0, 3] + math.pi) % (2 * math.pi) - math.pi
    assert abs(abs(math.degrees(dh)) - 5.0) < 1e-3
    assert frames[0, 1] == truth[0, 0] == start[0, 0]


def test_dataset_refuses_existing_and_bad_field(tmp_path):
    out = tmp_path / "run"
    out.mkdir()
    (out / "frames.csv").write_text("idx,t_ns\n")
    r = run_tool("sim/make_dataset.py", ["--traj", "spin", "--out", str(out)], tmp_path)
    assert r.returncode == 2 and "Error:" in r.stderr and "Traceback" not in r.stderr
    bad = tmp_path / "bad.json"
    bad.write_text('{"schema": 1}')
    r = run_tool("sim/make_dataset.py", ["--traj", "spin", "--field", str(bad), "--out",
                                         str(tmp_path / "x")], tmp_path)
    assert r.returncode == 2 and "not valid" in r.stderr


# ----------------------------------------------------------------------------- F2b hard profile


def _tiny_run(tmp_path, name, seed, extra=()):
    out = tmp_path / name
    r = run_tool("sim/make_dataset.py",
                 ["--field", FIELD, "--traj", "match", "--seed", str(seed), "--fps", "0.2",
                  "--width", "32", "--height", "24", "--workers", "1", "--out", str(out),
                  *extra], tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    return out


def _odometry_only_stage1(tmp_path, run):
    est = run / "odom_only.csv"
    r = run_tool("eval/odom_only.py", ["--run", str(run), "--out", str(est)], tmp_path)
    assert r.returncode == 0, r.stderr
    return run_tool("eval/eval_run.py", ["--run", str(run), "--est", str(est), "--gate", "stage1"],
                    tmp_path)


def test_hard_profile_fails_odometry_only(tmp_path):
    """Plan F2b: with the hard profile, pure odometry fails Stage 1 on 5 seeds, even with an
    exactly known start pose (so it is the drift, not only the start error, that fails)."""
    for seed in range(1, 6):
        run = _tiny_run(tmp_path, "hard%d" % seed, seed,
                        ["--odom-profile", "hard", "--start-error-cm", "0", "--start-error-deg", "0"])
        r = _odometry_only_stage1(tmp_path, run)
        assert r.returncode == 1 and "Gate stage1: FAIL" in r.stdout, (seed, r.stdout)


def test_hard_profile_parameters(tmp_path):
    run = _tiny_run(tmp_path, "hard", 7, ["--odom-profile", "hard"])
    meta = json.loads((run / "meta.json").read_text())
    assert meta["odomProfile"] == "hard"
    assert meta["startErrorCmDeg"] == [5.0, 3.0]
    assert 4 <= len(meta["collisionsS"]) <= 8
    assert all(0.02 <= math.hypot(s["dxRobotM"], s["dyRobotM"]) <= 0.05 for s in meta["slips"])
    assert all(abs(s["headingJumpRad"]) <= math.radians(2) + 1e-12 for s in meta["slips"])
    assert any(abs(s["headingJumpRad"]) > 0 for s in meta["slips"])
    assert all(abs(v - 1) <= 0.015 for v in meta["odomScale"])
    _, truth = read_csv(run / "truth.csv")
    _, start = read_csv(run / "start.csv")
    assert abs(math.hypot(start[0, 1] - truth[0, 1], start[0, 2] - truth[0, 2]) - 0.05) < 1e-6
    # the existing flags still override the profile's start error
    run = _tiny_run(tmp_path, "hard_exact", 7, ["--odom-profile", "hard", "--start-error-cm", "0",
                                                 "--start-error-deg", "0"])
    _, truth = read_csv(run / "truth.csv")
    _, start = read_csv(run / "start.csv")
    assert np.allclose(start[0, 1:], truth[0, 1:], atol=1e-6)


def test_hard_heading_random_walk_statistics():
    t = np.arange(0, 60.0001, 0.001)
    still = np.zeros_like(t)
    p = odomsim.PROFILES["hard"]
    errs = [odomsim.simulate(t, still, still, still, seed=s, params=p).h[-1] for s in range(400)]
    sd = math.degrees(float(np.std(errs)))
    assert 1.7 < sd < 2.3, sd


def test_nominal_profile_unchanged_by_f2b():
    """Adding the hard profile must not change nominal runs (no extra random draws)."""
    t, x, y, h = straight_truth(10.0)
    r = odomsim.simulate(t, x, y, h, seed=3, collisions=[5.0])
    assert r.slips[0][3] == 0.0
    assert odomsim.PROFILES["nominal"] == odomsim.OdomParams()
