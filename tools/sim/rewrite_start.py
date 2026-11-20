#!/usr/bin/env python3
"""Rewrite start.csv (and meta.json's startErrorCmDeg) of existing synthetic runs, without
re-rendering frames.

Usage:
    tools/.venv/bin/python tools/sim/rewrite_start.py tools/sim/runs/hard-match-s1 ...
    tools/.venv/bin/python tools/sim/rewrite_start.py RUN --start-error-cm 0 --start-error-deg 0

The typed start pose depends only on the true start (meta.json's startPoseTruth, else
truth.csv's first row), the run's seed
(meta.json) and the error size, so start.csv comes out exactly as make_dataset.py would write
it today for the same seed and arguments. Without --start-error-*, it uses the size
make_dataset.py would use now: the run's own --start-error-* arguments if it was made with them,
otherwise the current default of its odometry profile. Exit status: 0 OK; 2 bad input.
"""
import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import odomsim  # noqa: E402
from make_dataset import T0_NS, typed_start_pose, write_pose_csv  # noqa: E402


def rewrite(run, err_cm=None, err_deg=None):
    meta_path = os.path.join(run, "meta.json")
    with open(meta_path) as f:
        meta = json.load(f)
    args = meta["args"]
    profile = meta.get("odomProfile", args.get("odom_profile", "nominal"))
    d_cm, d_deg = odomsim.PROFILE_START_ERROR[profile]
    cm = err_cm if err_cm is not None else (args.get("start_error_cm") if args.get("start_error_cm") is not None else d_cm)
    deg = err_deg if err_deg is not None else (args.get("start_error_deg") if args.get("start_error_deg") is not None else d_deg)
    with open(os.path.join(run, "truth.csv")) as f:
        f.readline()
        first = f.readline().strip().split(",")
    t_ns, x, y, h = int(first[0]), float(first[1]), float(first[2]), float(first[3])
    if t_ns != T0_NS:
        raise ValueError("%s: truth.csv starts at %d, expected %d" % (run, t_ns, T0_NS))
    if "startPoseTruth" in meta:  # full precision (truth.csv is rounded to 6 decimals)
        x, y, h = (float(v) for v in meta["startPoseTruth"])
    start = typed_start_pose(x, y, h, int(args["seed"]), cm, deg)
    write_pose_csv(os.path.join(run, "start.csv"), "t_ns,x,y,heading", [t_ns], [[v] for v in start])
    old = meta.get("startErrorCmDeg")
    meta["startErrorCmDeg"] = [cm, deg]
    meta.setdefault("startRewrites", []).append({"from": old, "to": [cm, deg],
                                                 "by": "tools/sim/rewrite_start.py"})
    with open(meta_path, "w") as f:
        json.dump(meta, f, indent=1)
        f.write("\n")
    return old, [cm, deg]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("runs", nargs="+")
    p.add_argument("--start-error-cm", type=float, default=None)
    p.add_argument("--start-error-deg", type=float, default=None)
    a = p.parse_args(argv)
    for run in a.runs:
        try:
            old, new = rewrite(run, a.start_error_cm, a.start_error_deg)
        except (OSError, ValueError, KeyError, IndexError) as e:
            print("Error: %s: %s" % (run, e), file=sys.stderr)
            return 2
        print("%s: start error %s -> %s (cm, deg)" % (run, old, new))
    return 0


if __name__ == "__main__":
    sys.exit(main())
