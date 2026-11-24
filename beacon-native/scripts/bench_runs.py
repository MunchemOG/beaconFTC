#!/usr/bin/env python3
"""Replay recorded Limelight runs through beacon_replay (explore mode) and print one table.

No ground truth exists for hand-held runs, so the table reports what the user feels:
tracking %, longest stretch without a pose, keyframes, map points, and core cost per frame.

usage: bench_runs.py [--params FILE_OR_JSON] [--runs DIR ...] [--bin PATH] [--keep]
Default runs: every ~/beacon-runs/live* that has frames.csv. A frames.csv cut off by Ctrl-C
(last line partial) is repaired in a temp copy; the recording itself is never modified.
"""
import argparse, csv, glob, os, re, subprocess, sys, tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_BIN = os.path.join(HERE, "..", "build", "beacon_replay")


def repaired_copy(run, tmp):
    """Temp run dir with symlinked frames and a frames.csv without a partial last line."""
    d = os.path.join(tmp, os.path.basename(run.rstrip("/")))
    os.makedirs(d, exist_ok=True)
    for name in ("camera.json", "frames"):
        dst = os.path.join(d, name)
        if not os.path.exists(dst):
            os.symlink(os.path.join(run, name), dst)
    good = []
    with open(os.path.join(run, "frames.csv")) as f:
        for line in f:
            if line.endswith("\n") and (line.startswith("idx") or re.match(r"^\d+,\d+$", line.strip())):
                good.append(line)
    # also drop a final line that parses but whose PNG is missing
    while len(good) > 1:
        idx = good[-1].split(",")[0]
        if os.path.exists(os.path.join(run, "frames", "%06d.png" % int(idx))):
            break
        good.pop()
    with open(os.path.join(d, "frames.csv"), "w") as f:
        f.writelines(good)
    return d


def run_one(binary, run, params, tmp, stride=1):
    d = repaired_copy(run, tmp)
    out = os.path.join(tmp, os.path.basename(run.rstrip("/")) + ".fixes.csv")
    cmd = [binary, "--run", d, "--mode", "explore", "--out", out]
    if params:
        cmd += ["--params", params]
    if stride > 1:
        cmd += ["--stride", str(stride)]
    p = subprocess.run(cmd, capture_output=True, text=True)
    text = p.stdout + p.stderr
    m = re.search(r"frames (\d+), tracking (\d+) \(([\d.]+) %\), fixes \d+, loop closures (\d+), keyframes (\d+), map points (\d+)", text)
    t = re.search(r"p50 ([\d.]+) ms, p99 ([\d.]+) ms, max ([\d.]+) ms", text)
    if not m:
        return None, text[-300:]
    gap = 0.0
    if os.path.exists(out):
        ts = [int(r["t_ns"]) / 1e9 for r in csv.DictReader(open(out))]
        if ts:
            gap = max([b - a for a, b in zip(ts, ts[1:])] + [0.0])
            dur = ts[-1] - ts[0]
        else:
            dur = 0.0
    return dict(frames=int(m[1]), track=float(m[3]), loops=int(m[4]), kfs=int(m[5]), pts=int(m[6]),
                gap=gap, p50=float(t[1]) if t else 0, p99=float(t[2]) if t else 0), ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params", default="")
    ap.add_argument("--runs", nargs="*")
    ap.add_argument("--bin", default=DEFAULT_BIN)
    ap.add_argument("--stride", type=int, default=1, help="keep every n-th frame (n times faster motion)")
    a = ap.parse_args()
    runs = a.runs or sorted(d for d in glob.glob(os.path.expanduser("~/beacon-runs/live*"))
                            if os.path.exists(os.path.join(d, "frames.csv")))
    print("%-8s %6s %7s %9s %5s %6s %7s %7s" % ("run", "frames", "track%", "maxgap_s", "kfs", "points", "p50ms", "p99ms"))
    tot = []
    with tempfile.TemporaryDirectory() as tmp:
        for r in runs:
            res, err = run_one(a.bin, r, a.params, tmp, a.stride)
            name = os.path.basename(r.rstrip("/"))
            if not res:
                print("%-8s FAILED: %s" % (name, err.strip().splitlines()[-1] if err.strip() else "?"))
                continue
            tot.append(res)
            print("%-8s %6d %7.1f %9.1f %5d %6d %7.1f %7.1f" % (name, res["frames"], res["track"], res["gap"],
                                                                res["kfs"], res["pts"], res["p50"], res["p99"]))
    if tot:
        n = len(tot)
        print("%-8s %6s %7.1f %9.1f %5s %6s %7.1f %7.1f" % ("MEAN", "", sum(r["track"] for r in tot) / n,
                                                           sum(r["gap"] for r in tot) / n, "", "",
                                                           sum(r["p50"] for r in tot) / n, sum(r["p99"] for r in tot) / n))


if __name__ == "__main__":
    sys.exit(main())
