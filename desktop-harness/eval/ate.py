#!/usr/bin/env python3
"""Absolute trajectory error of a Beacon pose log against ground truth.

    ate.py <est.csv> <gt.csv> [--png out.png] [--json out.json] [--scale] [--all-status]

<est.csv>  project log format (00-PROJECT.md, Logging): header "t_ns,x,y,z,qx,qy,qz,qw,status".
           Written by the desktop driver and, later, by the phone. Only rows with status == 2
           (TRACKING) are evaluated unless --all-status is given.
<gt.csv>   either an EuRoC ASL ground-truth file (mav0/state_groundtruth_estimate0/data.csv:
           "#timestamp, p_RS_R_x, ..., q_RS_w, q_RS_x, ...") or another project-format log.
           A sequence directory (containing mav0/) is also accepted.

Method (evo): associate by timestamp (max 10 ms apart), align the estimate to ground truth with an
SE(3) Umeyama fit (rotation + translation, NO scale; with an IMU, scale is observable, so unscaled
is the headline), then report the translation error statistics in metres. --scale additionally
reports a Sim(3) alignment for reference.
"""
import argparse
import csv
import json
import os
import sys

import numpy as np

from evo.core import metrics, sync
from evo.core.trajectory import PoseTrajectory3D

TRACKING = 2


def read_project_log(path, all_status=False):
    """Project log format -> PoseTrajectory3D (quaternions are stored x,y,z,w; evo wants w,x,y,z)."""
    t, xyz, wxyz = [], [], []
    n_rows = 0
    n_bad = 0
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        need = ["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "status"]
        if reader.fieldnames is None or [c.strip() for c in reader.fieldnames][:9] != need:
            raise SystemExit("%s: header must be %s, got %s" % (path, ",".join(need), reader.fieldnames))
        for row in reader:
            row = {k.strip(): v for k, v in row.items()}
            n_rows += 1
            if not all_status and int(row["status"]) != TRACKING:
                continue
            vals = [float(row[k]) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
            if not np.all(np.isfinite(vals)) or abs(np.linalg.norm(vals[3:]) - 1.0) > 1e-3:
                n_bad += 1
                continue
            t.append(int(row["t_ns"]) * 1e-9)
            xyz.append(vals[0:3])
            wxyz.append([vals[6], vals[3], vals[4], vals[5]])
    if not t:
        raise SystemExit("%s: no usable rows (status==2) out of %d" % (path, n_rows))
    traj = PoseTrajectory3D(np.array(xyz), np.array(wxyz), np.array(t))
    return traj, n_rows, n_bad


def read_euroc_gt(path):
    """EuRoC ASL ground truth: t_ns, p_RS_R (3), q_RS (w,x,y,z), ... (body = IMU frame)."""
    data = np.loadtxt(path, delimiter=",", comments="#", usecols=range(8))
    t = data[:, 0] * 1e-9
    return PoseTrajectory3D(data[:, 1:4], data[:, 4:8], t)


def read_gt(path):
    if os.path.isdir(path):
        path = os.path.join(path, "mav0", "state_groundtruth_estimate0", "data.csv")
    with open(path) as f:
        first = f.readline()
    if first.startswith("t_ns"):
        traj, _, _ = read_project_log(path, all_status=True)
        return traj, path
    return read_euroc_gt(path), path


def ape_stats(ref, est):
    m = metrics.APE(metrics.PoseRelation.translation_part)
    m.process_data((ref, est))
    s = m.get_all_statistics()
    r = metrics.APE(metrics.PoseRelation.rotation_angle_deg)
    r.process_data((ref, est))
    s["rot_rmse_deg"] = r.get_statistic(metrics.StatisticsType.rmse)
    return s, m.error


def top_down_png(ref, est, err, out_png, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(7, 6))
    ax.plot(ref.positions_xyz[:, 0], ref.positions_xyz[:, 1], color="0.6", lw=2, label="ground truth")
    sc = ax.scatter(est.positions_xyz[:, 0], est.positions_xyz[:, 1], c=err, s=3, cmap="viridis", label="estimate")
    fig.colorbar(sc, ax=ax, label="position error [m]")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title(title)
    ax.legend(loc="best")
    ax.grid(True, alpha=0.3)
    fig.tight_layout()
    fig.savefig(out_png, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("est")
    ap.add_argument("gt")
    ap.add_argument("--png", help="top-down plot (default: <est dir>/ate_topdown.png)")
    ap.add_argument("--json", help="write the statistics as JSON here")
    ap.add_argument("--scale", action="store_true", help="also report Sim(3) (scale-corrected) ATE")
    ap.add_argument("--all-status", action="store_true", help="evaluate every row, not just status==2")
    ap.add_argument("--max-diff", type=float, default=0.01, help="max timestamp difference for association [s]")
    args = ap.parse_args()

    est, n_rows, n_bad = read_project_log(args.est, args.all_status)
    ref, gt_path = read_gt(args.gt)
    n_est = est.num_poses
    ref_a, est_a = sync.associate_trajectories(ref, est, max_diff=args.max_diff)
    if est_a.num_poses < 10:
        raise SystemExit("only %d poses associated with ground truth; check timestamps/units" % est_a.num_poses)

    est_se3 = PoseTrajectory3D(poses_se3=est_a.poses_se3, timestamps=est_a.timestamps)
    est_se3.align(ref_a, correct_scale=False)
    stats, err = ape_stats(ref_a, est_se3)

    out = {
        "est": os.path.abspath(args.est),
        "gt": os.path.abspath(gt_path),
        "rows": n_rows,
        "rows_tracking": n_est,
        "rows_rejected_nonfinite": n_bad,
        "associated": est_a.num_poses,
        "alignment": "SE3 (Umeyama, no scale)",
        "rmse_m": stats["rmse"],
        "mean_m": stats["mean"],
        "median_m": stats["median"],
        "max_m": stats["max"],
        "std_m": stats["std"],
        "rot_rmse_deg": stats["rot_rmse_deg"],
        "path_length_m": ref_a.path_length,
    }
    print("ATE  est=%s" % args.est)
    print("     gt =%s" % gt_path)
    print("poses: %d rows, %d tracking, %d associated with ground truth (max dt %.3f s)"
          % (n_rows, n_est, est_a.num_poses, args.max_diff))
    print("alignment: SE(3), unscaled   path length %.2f m" % ref_a.path_length)
    print("ATE RMSE %.4f m | mean %.4f m | median %.4f m | max %.4f m | rot RMSE %.2f deg"
          % (stats["rmse"], stats["mean"], stats["median"], stats["max"], stats["rot_rmse_deg"]))

    if args.scale:
        est_sim3 = PoseTrajectory3D(poses_se3=est_a.poses_se3, timestamps=est_a.timestamps)
        r, t, s = est_sim3.align(ref_a, correct_scale=True)
        st3, _ = ape_stats(ref_a, est_sim3)
        out["sim3_rmse_m"] = st3["rmse"]
        out["sim3_scale"] = float(s)
        print("Sim(3) for reference: RMSE %.4f m (scale %.4f)" % (st3["rmse"], s))

    png = args.png or os.path.join(os.path.dirname(os.path.abspath(args.est)), "ate_topdown.png")
    top_down_png(ref_a, est_se3, err, png, "ATE RMSE %.3f m (SE3)" % stats["rmse"])
    print("plot: %s" % png)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)
    return 0


if __name__ == "__main__":
    sys.exit(main())
