"""Shared helpers for the tools tests. Tests create their own input files under tmp_path."""
import csv
import math
import os
import subprocess
import sys

import pytest

TOOLS_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, TOOLS_DIR)


def run_tool(script, args, cwd):
    """Run one tool as a student would, with the same Python as the test run."""
    env = dict(os.environ, MPLBACKEND="Agg")
    return subprocess.run([sys.executable, os.path.join(TOOLS_DIR, script), *args],
                          cwd=str(cwd), env=env, capture_output=True, text=True, timeout=120)


def write_csv(path, header, rows):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(rows)
    return str(path)


def square_points(steps_per_side=10, side=1.0):
    """Corners of a square traversed once, with points on each side; returns (x, y) list."""
    corners = [(0.0, 0.0), (side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
    points = []
    for (x0, y0), (x1, y1) in zip(corners, corners[1:]):
        for i in range(steps_per_side):
            f = i / steps_per_side
            points.append((x0 + f * (x1 - x0), y0 + f * (y1 - y0)))
    points.append(corners[-1])
    return points


def yaw_quaternion(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@pytest.fixture
def square_log(tmp_path):
    """A 41-row pose log tracing a 1 m square at 10 Hz (duration 4.0 s, path 4.0 m), all status 2."""
    rows = []
    for i, (x, y) in enumerate(square_points()):
        rows.append([i * 100_000_000, x, y, 0.0, 0.0, 0.0, 0.0, 1.0, 2])
    return write_csv(tmp_path / "square.csv",
                     ["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "status"], rows)
