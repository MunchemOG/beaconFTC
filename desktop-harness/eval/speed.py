#!/usr/bin/env python3
"""Per-frame processing time from the driver's timing log.

    speed.py <run_dir or timing.csv> [--json out.json]

timing.csv columns: t_ns,frame_ms,imu_ms,num_feats,zupt,status
  frame_ms  wall time inside VioManager::feed_measurement_camera (tracking + propagation + update)
  imu_ms    wall time inside feed_measurement_imu for the IMU samples fed since the previous frame
The headline per-frame time is frame_ms + imu_ms (everything the estimator costs per image).
Image decoding (PNG read) is excluded: on the phone, frames arrive already decoded.

Two sets of statistics are printed: all frames, and frames after initialisation (status 2 or 3),
which is the steady-state cost that matters on the robot.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np


def stats(ms):
    ms = np.asarray(ms, dtype=float)
    if ms.size == 0:
        return None
    mean = float(ms.mean())
    return {
        "frames": int(ms.size),
        "mean_ms": mean,
        "median_ms": float(np.median(ms)),
        "p95_ms": float(np.percentile(ms, 95)),
        "max_ms": float(ms.max()),
        "fps": 1000.0 / mean if mean > 0 else float("inf"),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path")
    ap.add_argument("--json")
    args = ap.parse_args()
    path = os.path.join(args.path, "timing.csv") if os.path.isdir(args.path) else args.path

    t, total, frame, status, zupt = [], [], [], [], []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            t.append(int(row["t_ns"]))
            fm = float(row["frame_ms"])
            im = float(row["imu_ms"])
            frame.append(fm)
            total.append(fm + im)
            status.append(int(row["status"]))
            zupt.append(int(row["zupt"]))
    if not t:
        raise SystemExit("%s: empty" % path)
    total = np.array(total)
    frame = np.array(frame)
    status = np.array(status)
    zupt = np.array(zupt)
    live = status >= 2
    seq_s = (t[-1] - t[0]) * 1e-9
    data_hz = (len(t) - 1) / seq_s if seq_s > 0 else 0.0

    out = {
        "timing_csv": os.path.abspath(path),
        "input_rate_hz": data_hz,
        "sequence_s": seq_s,
        "all": stats(total),
        "after_init": stats(total[live]),
        "after_init_camera_only": stats(frame[live]),
        "zupt_frames": int(zupt.sum()),
        "realtime_factor": seq_s / (total.sum() * 1e-3) if total.sum() > 0 else float("inf"),
    }

    def line(name, s):
        if s is None:
            print("%-22s (no frames)" % name)
            return
        print("%-22s %5d frames | mean %6.2f ms | median %6.2f | p95 %6.2f | max %7.2f | %6.1f fps"
              % (name, s["frames"], s["mean_ms"], s["median_ms"], s["p95_ms"], s["max_ms"], s["fps"]))

    print("speed  %s" % path)
    print("input %.1f Hz over %.1f s; processed %.1fx faster than real time" % (data_hz, seq_s, out["realtime_factor"]))
    line("all frames", out["all"])
    line("after init", out["after_init"])
    line("after init (cam only)", out["after_init_camera_only"])
    print("zero-velocity-update frames: %d" % out["zupt_frames"])
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
