#!/usr/bin/env python3
"""Tuning sweep: run the driver on EuRoC with config variants and tabulate ATE + time.

    sweep.py [--repeats 3] [--sequences MH_01_easy V1_01_easy] [--only name ...] [--out runs/sweep]

Each variant is the base config (config/euroc) with a few keys overridden. ATE is deterministic
for a given config (serial processing, fixed RNG seed), so it is computed once per run and checked
to be identical across repeats; timing is not (it depends on machine load), so each variant is run
--repeats times and the run with the median mean-time is reported. The load average at the start
of every run is recorded with it.

Writes <out>/<variant>/<sequence>_r<k>/ (driver output + ate.json + speed.json) and prints a
Markdown table to stdout and to <out>/table.md.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PY = os.path.join(ROOT, ".venv", "bin", "python")
DRIVER = os.path.join(ROOT, "build", "driver", "euroc_driver")
BASE = os.path.join(ROOT, "config", "euroc")

ZUPT = {"try_zupt": "true"}
VARIANTS = [
    ("baseline", {}),                                   # upstream euroc_mav values, mono
    ("num_pts_150", {"num_pts": "150"}),
    ("num_pts_100", {"num_pts": "100"}),
    ("max_clones_8", {"max_clones": "8"}),
    ("max_slam_25", {"max_slam": "25"}),
    ("max_slam_0", {"max_slam": "0"}),
    ("grid_5x3_px15", {"grid_x": "5", "grid_y": "3", "min_px_dist": "15"}),
    ("downsample", {"downsample_cameras": "true"}),
    ("calib_off", {"calib_cam_extrinsics": "false", "calib_cam_intrinsics": "false", "calib_cam_timeoffset": "false"}),
    ("zupt_disp", dict(ZUPT)),                          # upstream zupt params: disparity-only test
    ("zupt_imu_disp", dict(ZUPT, zupt_chi2_multipler="1")),  # IMU chi2 test AND disparity
]


def write_config(dst, overrides):
    os.makedirs(dst, exist_ok=True)
    for f in os.listdir(BASE):
        if f.endswith(".yaml"):
            shutil.copy(os.path.join(BASE, f), dst)
    p = os.path.join(dst, "estimator_config.yaml")
    s = open(p).read()
    for k, v in overrides.items():
        s, n = re.subn(r"(?m)^%s:\s*[^#\n]*" % re.escape(k), "%s: %s " % (k, v), s)
        if n != 1:
            raise SystemExit("key %s found %d times in %s" % (k, n, p))
    open(p, "w").write(s)
    return p


def run_one(cfg, seq, out):
    os.makedirs(out, exist_ok=True)
    load = os.getloadavg()[0]
    with open(os.path.join(out, "driver.log"), "w") as log:
        rc = subprocess.call([DRIVER, cfg, os.path.join(ROOT, "data", "euroc", seq), out], stdout=log, stderr=subprocess.STDOUT)
    if rc not in (0,):
        return {"error": "driver exit %d" % rc, "load": load}
    subprocess.check_call([PY, os.path.join(HERE, "ate.py"), os.path.join(out, "trajectory.csv"),
                           os.path.join(ROOT, "data", "euroc", seq), "--json", os.path.join(out, "ate.json")],
                          stdout=subprocess.DEVNULL)
    subprocess.check_call([PY, os.path.join(HERE, "speed.py"), out, "--json", os.path.join(out, "speed.json")],
                          stdout=subprocess.DEVNULL)
    ate = json.load(open(os.path.join(out, "ate.json")))
    spd = json.load(open(os.path.join(out, "speed.json")))
    return {"ate": ate, "speed": spd, "load": load}


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--repeats", type=int, default=3)
    ap.add_argument("--sequences", nargs="+", default=["MH_01_easy", "V1_01_easy"])
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--out", default=os.path.join(ROOT, "runs", "sweep"))
    args = ap.parse_args()

    rows = []
    for name, ov in VARIANTS:
        if args.only and name not in args.only:
            continue
        cfg = write_config(os.path.join(args.out, name, "config"), ov)
        for seq in args.sequences:
            runs = []
            for k in range(args.repeats):
                r = run_one(cfg, seq, os.path.join(args.out, name, "%s_r%d" % (seq, k)))
                runs.append(r)
            ok = [r for r in runs if "ate" in r]
            if not ok:
                rows.append((name, seq, None))
                print("%-16s %-11s FAILED: %s" % (name, seq, runs[0].get("error")), flush=True)
                continue
            rmses = sorted({round(r["ate"]["rmse_m"], 6) for r in ok})
            ok.sort(key=lambda r: r["speed"]["after_init"]["mean_ms"])
            med = ok[len(ok) // 2]
            med["rmse_set"] = rmses
            med["n_ok"] = len(ok)
            rows.append((name, seq, med))
            a, s = med["ate"], med["speed"]["after_init"]
            print("%-16s %-11s ATE %.4f (mean %.4f max %.4f) | %.2f ms mean, p95 %.2f, max %.1f | zupt %d | load %.1f | rmse over repeats %s"
                  % (name, seq, a["rmse_m"], a["mean_m"], a["max_m"], s["mean_ms"], s["p95_ms"], s["max_ms"],
                     med["speed"]["zupt_frames"], med["load"], rmses), flush=True)

    lines = ["| variant | sequence | ATE RMSE m | mean m | max m | tracked/assoc | ms mean | ms p95 | ms max | fps | ZUPT frames | load |",
             "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
    for name, seq, m in rows:
        if m is None:
            lines.append("| %s | %s | FAILED | | | | | | | | | |" % (name, seq))
            continue
        a, s = m["ate"], m["speed"]["after_init"]
        lines.append("| %s | %s | %.3f | %.3f | %.3f | %d/%d | %.2f | %.2f | %.1f | %.0f | %d | %.1f |"
                     % (name, seq, a["rmse_m"], a["mean_m"], a["max_m"], a["rows_tracking"], a["associated"],
                        s["mean_ms"], s["p95_ms"], s["max_ms"], s["fps"], m["speed"]["zupt_frames"], m["load"]))
    table = "\n".join(lines)
    print()
    print(table)
    with open(os.path.join(args.out, "table.md"), "w") as f:
        f.write(table + "\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
