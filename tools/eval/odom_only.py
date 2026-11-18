#!/usr/bin/env python3
"""Odometry-only baseline: what pure Pinpoint (no vision) would report in the field frame.

Usage:
    tools/.venv/bin/python tools/eval/odom_only.py --run RUN --out odom_only.csv
    tools/.venv/bin/python tools/eval/eval_run.py --run RUN --est odom_only.csv --gate stage1

Aligns RUN/odom.csv (Pinpoint world) to the field once, using RUN/start.csv (the start pose the
team typed): field = start * inverse(odom at the start time) * odom. Then it never corrects
again, like Beacon with vision switched off. Writes t_ns,x,y,heading (heading wrapped to
(-pi, pi]). Exit status: 0 OK; 2 bad input.
"""
import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from eval_run import InputError, read_poses, wrap  # noqa: E402


def odom_only(run):
    ot, ox, oy, oh = read_poses(os.path.join(run, "odom.csv"), "odometry")
    st, sx, sy, sh = read_poses(os.path.join(run, "start.csv"), "start")
    t0 = st[0]
    if not ot[0] <= t0 <= ot[-1]:
        raise InputError("start.csv's t_ns is outside odom.csv's time span")
    ohu = np.unwrap(oh)
    x0, y0, h0 = (np.interp(t0, ot, ox), np.interp(t0, ot, oy), np.interp(t0, ot, ohu))
    rot = sh[0] - h0                       # odom world -> field rotation
    c, s = math.cos(rot), math.sin(rot)
    dx, dy = ox - x0, oy - y0
    fx = sx[0] + c * dx - s * dy
    fy = sy[0] + s * dx + c * dy
    fh = wrap(ohu + rot)
    return ot, fx, fy, fh


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--run", required=True)
    p.add_argument("--out", required=True)
    args = p.parse_args(argv)
    try:
        t, x, y, h = odom_only(os.path.abspath(args.run))
    except InputError as e:
        print("Error: " + str(e), file=sys.stderr)
        return 2
    with open(args.out, "w") as f:
        f.write("t_ns,x,y,heading\n")
        for row in zip(t, x, y, h):
            f.write("%d,%.6f,%.6f,%.6f\n" % row)
    print("Wrote %d odometry-only poses to %s" % (len(t), args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
