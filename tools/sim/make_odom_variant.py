#!/usr/bin/env python3
"""Make a copy of a synthetic run with a different odometry profile, without re-rendering.

Usage:
    tools/.venv/bin/python tools/sim/make_odom_variant.py --from tools/sim/runs/hard-match-s1 \
        --odom-profile nominal --out tools/sim/runs/nominal-match-s1

It re-runs make_dataset.py with the source run's own arguments (from its meta.json), but with
--odom-profile changed and --no-render. So odom.csv, start.csv and meta.json are exactly what
make_dataset.py writes for that seed and profile. The start error falls back to the new
profile's default unless the source run set --start-error-* explicitly. frames/ is a symlink to
the source run's frames. It then checks that frames.csv, camera.json, field.json, truth.csv and
hive.csv are byte-identical to the source's, which proves the trajectory and frame times are
the same. On a mismatch it stops with exit status 2.
"""
import argparse
import filecmp
import json
import os
import shutil
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import make_dataset  # noqa: E402

SHARED = ("frames.csv", "camera.json", "field.json", "truth.csv", "hive.csv")


def to_argv(args, profile, out):
    """make_dataset.py arguments reproducing a run from its meta.json 'args'."""
    argv = []
    skip = {"out", "odom_profile", "no_render", "workers"}
    for key, value in args.items():
        if key in skip or value is None or value is False:
            continue
        flag = "--" + key.replace("_", "-")
        argv += [flag] if value is True else [flag, str(value)]
    return argv + ["--odom-profile", profile, "--no-render", "--workers", "1", "--out", out]


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--from", dest="src", required=True, help="existing rendered run")
    p.add_argument("--odom-profile", required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args(argv)
    src = os.path.abspath(a.src)
    out = os.path.abspath(a.out)
    try:
        with open(os.path.join(src, "meta.json")) as f:
            meta = json.load(f)
    except (OSError, ValueError) as e:
        print("Error: cannot read %s/meta.json: %s" % (src, e), file=sys.stderr)
        return 2
    if os.path.exists(out):
        print("Error: %s already exists" % out, file=sys.stderr)
        return 2
    rc = make_dataset.main(to_argv(meta["args"], a.odom_profile, out))
    if rc:
        return rc
    os.rmdir(os.path.join(out, "frames"))          # empty: --no-render
    os.symlink(os.path.join(src, "frames"), os.path.join(out, "frames"))
    bad = [n for n in SHARED if not filecmp.cmp(os.path.join(src, n), os.path.join(out, n),
                                                 shallow=False)]
    if bad:
        print("Error: %s differ from %s; the variant would not match its frames. Removed %s."
              % (", ".join(bad), src, out), file=sys.stderr)
        shutil.rmtree(out)
        return 2
    print("Variant %s: %s profile, frames -> %s (shared files identical)"
          % (out, a.odom_profile, os.path.join(src, "frames")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
