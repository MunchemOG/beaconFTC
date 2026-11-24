#!/usr/bin/env python3
"""Replay a desktop-harness run in Rerun.

    view.py <run_dir> [--save out.rrd] [--image-stride N] [--max-frames N] [--gt <gt.csv>]

Opens the Rerun viewer (default) or, with --save, writes an .rrd file instead (no window needed;
open it later with `rerun out.rrd`).

Entities (mirroring the demo; all 3D data is in OpenVINS's gravity-aligned global frame {G}):
  beacon/image/cam0              the cam0 image (re-encoded as JPEG to keep recordings small)
  beacon/image/cam0/features     tracked features: green = long-tracked (seen >= 2 frames),
                                 blue = newly detected this frame
  beacon/points                  sparse map: every point triangulated for an MSCKF update so far,
                                 5 cm voxel-downsampled, coloured by height (z)
  beacon/backend_points          SLAM landmarks currently held in the filter state (orange)
  beacon/pose_imu                IMU pose (Transform3D + axes)
  beacon/trajectory              estimated IMU path so far
  beacon/groundtruth             ground truth in grey, SE(3)-aligned onto the estimate
Timelines: "frame" (sequence) and "sensor_time" (dataset clock).
"""
import argparse
import csv
import io
import os
import sys
from collections import defaultdict

import numpy as np
import rerun as rr
import rerun.blueprint as rrb

HERE = os.path.dirname(os.path.abspath(__file__))
GREEN = (40, 220, 60)
BLUE = (40, 120, 255)
ORANGE = (255, 150, 30)
GREY = (150, 150, 150)
MAP_VOXEL_M = 0.05
MAP_EVERY = 10    # re-log the accumulated map every N frames
TRAIL_EVERY = 5   # re-log the trajectory line every N frames


def read_run_info(run_dir):
    info = {}
    p = os.path.join(run_dir, "run_info.txt")
    if os.path.exists(p):
        with open(p) as f:
            for line in f:
                if "=" in line:
                    k, v = line.rstrip("\n").split("=", 1)
                    info[k] = v
    return info


def read_trajectory(path):
    rows = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            rows.append((int(r["t_ns"]), float(r["x"]), float(r["y"]), float(r["z"]),
                         float(r["qx"]), float(r["qy"]), float(r["qz"]), float(r["qw"]), int(r["status"])))
    return rows


def group_by_t(path, parse):
    out = defaultdict(list)
    if not os.path.exists(path):
        return out
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            out[int(r["t_ns"])].append(parse(r))
    return out


def height_colors(z):
    """Map heights to a blue-to-yellow ramp (viridis-like) without matplotlib."""
    if len(z) == 0:
        return np.zeros((0, 3), dtype=np.uint8)
    lo, hi = np.percentile(z, 2), np.percentile(z, 98)
    s = np.clip((z - lo) / max(1e-6, hi - lo), 0, 1)[:, None]
    a = np.array([68, 1, 84], dtype=float)
    b = np.array([33, 145, 140], dtype=float)
    c = np.array([253, 231, 37], dtype=float)
    col = np.where(s < 0.5, a + (b - a) * (s * 2), b + (c - b) * ((s - 0.5) * 2))
    return col.astype(np.uint8)


def aligned_groundtruth(traj_rows, gt_path):
    """Ground-truth positions expressed in the estimate's frame (inverse of ate.py's alignment)."""
    sys.path.insert(0, os.path.join(HERE, "..", "eval"))
    from evo.core import sync
    from evo.core.trajectory import PoseTrajectory3D
    import ate  # reuse the same readers

    tracked = [r for r in traj_rows if r[8] == 2]
    if len(tracked) < 10:
        return None
    est = PoseTrajectory3D(np.array([r[1:4] for r in tracked]),
                           np.array([[r[7], r[4], r[5], r[6]] for r in tracked]),
                           np.array([r[0] * 1e-9 for r in tracked]))
    gt_full, _ = ate.read_gt(gt_path)
    gt_a, est_a = sync.associate_trajectories(gt_full, est, max_diff=0.01)
    if est_a.num_poses < 10:
        return None
    r, t, _ = gt_a.align(est_a, correct_scale=False)  # gt -> estimate frame
    return (r @ gt_full.positions_xyz.T).T + t


def jpeg_bytes(png_path, quality):
    from PIL import Image
    with Image.open(png_path) as im:
        buf = io.BytesIO()
        im.convert("L").save(buf, format="JPEG", quality=quality)
        return buf.getvalue()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir")
    ap.add_argument("--save", help="write an .rrd file instead of opening the viewer")
    ap.add_argument("--image-stride", type=int, default=1, help="log every Nth image (features every frame)")
    ap.add_argument("--jpeg-quality", type=int, default=75)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--gt", help="ground truth csv or sequence dir (default: from run_info.txt)")
    args = ap.parse_args()

    run_dir = os.path.abspath(args.run_dir)
    info = read_run_info(run_dir)
    seq_dir = info.get("sequence_dir", "")
    if seq_dir and not os.path.isabs(seq_dir):
        seq_dir = os.path.normpath(os.path.join(HERE, "..", seq_dir))
    traj = read_trajectory(os.path.join(run_dir, "trajectory.csv"))
    feats = group_by_t(os.path.join(run_dir, "features.csv"),
                       lambda r: (float(r["u"]), float(r["v"]), int(r["track_len"])))
    pts = group_by_t(os.path.join(run_dir, "points.csv"),
                     lambda r: (r["kind"], float(r["x"]), float(r["y"]), float(r["z"])))

    # Image file for each logged frame. The trajectory t_ns is the camera time shifted by the
    # camera-IMU offset, so match each row to the nearest cam0 timestamp.
    cam_t, cam_file = [], []
    cam_csv = os.path.join(seq_dir, "mav0", "cam0", "data.csv") if seq_dir else ""
    if cam_csv and os.path.exists(cam_csv):
        with open(cam_csv) as f:
            for line in f:
                if line.startswith("#") or "," not in line:
                    continue
                a, b = line.strip().split(",", 1)
                cam_t.append(int(a))
                cam_file.append(os.path.join(seq_dir, "mav0", "cam0", "data", b.strip()))
    cam_t = np.array(cam_t, dtype=np.int64)

    blueprint = rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(name="cam0", origin="beacon/image/cam0"),
            rrb.Spatial3DView(name="map", origin="beacon", contents=["+ beacon/**", "- beacon/image/**"]),
            column_shares=[2, 3],
        ),
        collapse_panels=True,
    )
    rr.init("beacon_desktop_harness", spawn=False)
    if args.save:
        rr.save(args.save, default_blueprint=blueprint)
    else:
        rr.spawn(default_blueprint=blueprint)

    gt_src = args.gt or seq_dir
    if gt_src:
        try:
            gt_xyz = aligned_groundtruth(traj, gt_src)
        except Exception as e:  # ground truth is optional (phone logs have none)
            print("ground truth not shown: %s" % e)
            gt_xyz = None
        if gt_xyz is not None:
            rr.log("beacon/groundtruth", rr.LineStrips3D([gt_xyz], colors=[GREY], radii=[0.01]), static=True)

    map_vox = {}
    trail = []
    n = len(traj) if args.max_frames <= 0 else min(len(traj), args.max_frames)
    for i in range(n):
        t_ns, x, y, z, qx, qy, qz, qw, status = traj[i]
        rr.set_time("frame", sequence=i)
        rr.set_time("sensor_time", timestamp=np.datetime64(t_ns, "ns"))

        if cam_t.size and i % max(1, args.image_stride) == 0:
            j = int(np.argmin(np.abs(cam_t - t_ns)))
            if abs(int(cam_t[j]) - t_ns) < 20_000_000:
                rr.log("beacon/image/cam0", rr.EncodedImage(contents=jpeg_bytes(cam_file[j], args.jpeg_quality),
                                                            media_type="image/jpeg"))
        fr = feats.get(t_ns, [])
        if fr:
            uv = np.array([(u, v) for u, v, _ in fr])
            col = [BLUE if L <= 1 else GREEN for _, _, L in fr]
            rr.log("beacon/image/cam0/features", rr.Points2D(uv, colors=col, radii=3.0))
        else:
            rr.log("beacon/image/cam0/features", rr.Clear(recursive=False))

        if status >= 2:
            rr.log("beacon/pose_imu", rr.Transform3D(translation=[x, y, z], quaternion=[qx, qy, qz, qw]),
                   rr.TransformAxes3D(0.3))
            trail.append((x, y, z))
            if len(trail) > 1 and (i % TRAIL_EVERY == 0 or i == n - 1):
                rr.log("beacon/trajectory", rr.LineStrips3D([np.array(trail)], colors=[ORANGE], radii=[0.015]))

        pr = pts.get(t_ns, [])
        slam = np.array([p[1:] for p in pr if p[0] == "slam"]).reshape(-1, 3)
        msckf = np.array([p[1:] for p in pr if p[0] == "msckf"]).reshape(-1, 3)
        rr.log("beacon/backend_points", rr.Points3D(slam, colors=[ORANGE], radii=0.03))
        for p in msckf:
            map_vox[tuple(np.floor(p / MAP_VOXEL_M).astype(int))] = p
        if map_vox and (i % MAP_EVERY == 0 or i == n - 1):
            m = np.array(list(map_vox.values()))
            rr.log("beacon/points", rr.Points3D(m, colors=height_colors(m[:, 2]), radii=0.02))

    print("logged %d frames, %d map points%s" % (n, len(map_vox), (" -> " + args.save) if args.save else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
