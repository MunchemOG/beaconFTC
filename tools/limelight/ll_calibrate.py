#!/usr/bin/env python3
"""Work out the Limelight's lens intrinsics from a recording of a printed checkerboard.

Usage:
    tools/.venv/bin/python tools/limelight/ll_calibrate.py --run runs/board1
    tools/.venv/bin/python tools/limelight/ll_calibrate.py --run runs/board1 --board 9x6 \
        --square-mm 25

--run is a directory written by ll_record.py (frames.csv + frames/). --board is the number of
INNER corners, columns x rows (OpenCV's convention: a 9x6 board has 10 x 7 squares).
--square-mm is the printed square size: measure it with a ruler after printing.

Writes DIR/camera.json (plan I2: width, height, fx, fy, cx, cy, dist [k1,k2,p1,p2,k3], mount,
rollingShutterReadoutS) and DIR/calibration_report.json (RMS error, views used). If DIR already
has a camera.json, its mount and rollingShutterReadoutS are kept; otherwise they are written as
zeros and must be filled in later (the mount is measured on the robot).

Exit status: 0 calibrated; 2 bad input or too few checkerboard views.
"""
import argparse
import csv
import json
import math
import os
import sys

MIN_VIEWS = 8


def fail(msg):
    print("Error: " + msg, file=sys.stderr)
    sys.exit(2)


def parse_board(text):
    try:
        cols, rows = (int(v) for v in text.lower().split("x"))
    except ValueError:
        fail("--board must look like 9x6 (inner corners, columns x rows); got %r" % text)
    if cols < 3 or rows < 3:
        fail("--board needs at least 3x3 inner corners; got %r" % text)
    return cols, rows


def read_frames(run):
    path = os.path.join(run, "frames.csv")
    if not os.path.isdir(run):
        fail("run directory not found: %s" % run)
    if not os.path.isfile(path):
        fail("%s has no frames.csv; record one with ll_record.py first." % run)
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    if not rows or [c.strip() for c in rows[0]] != ["idx", "t_ns"]:
        fail("%s must start with the header idx,t_ns" % path)
    frames = []
    for line_no, row in enumerate(rows[1:], start=2):
        if not row:
            continue
        try:
            frames.append(int(row[0]))
        except (ValueError, IndexError):
            fail("%s line %d is not idx,t_ns: %r" % (path, line_no, row))
    if not frames:
        fail("%s lists no frames." % path)
    return frames


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--run", required=True, help="recording directory (frames.csv + frames/)")
    p.add_argument("--board", default="9x6", help="inner corners, columns x rows (default 9x6)")
    p.add_argument("--square-mm", type=float, default=25.0,
                   help="printed square size in mm (default 25)")
    p.add_argument("--max-views", type=int, default=40, help="most views to use (default 40)")
    p.add_argument("--scan", type=int, default=400,
                   help="most frames to search for the board, evenly spaced (default 400)")
    p.add_argument("--free-k3", action="store_true",
                   help="also fit k3 (default: k3 = 0, which is steadier with few views)")
    args = p.parse_args(argv)
    if not (args.square_mm > 0 and math.isfinite(args.square_mm)):
        fail("--square-mm must be a positive number")

    import cv2
    import numpy as np

    cols, rows = parse_board(args.board)
    run = os.path.abspath(args.run)
    indices = read_frames(run)
    step = max(1, len(indices) // max(args.scan, 1))
    scan = indices[::step]

    obj = np.zeros((rows * cols, 3), np.float32)
    obj[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * (args.square_mm / 1000.0)

    found = []  # (idx, corners)
    size = None
    missing = 0
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
    for idx in scan:
        path = os.path.join(run, "frames", "%06d.png" % idx)
        img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if img is None:
            missing += 1
            continue
        if size is None:
            size = (img.shape[1], img.shape[0])
        elif size != (img.shape[1], img.shape[0]):
            fail("frame %s is %dx%d but earlier frames are %dx%d; record one resolution per run."
                 % (path, img.shape[1], img.shape[0], size[0], size[1]))
        ok, corners = cv2.findChessboardCorners(
            img, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK)
        if not ok:
            continue
        corners = cv2.cornerSubPix(img, corners, (5, 5), (-1, -1), criteria)
        found.append((idx, corners))
    if size is None:
        fail("none of the frames listed in frames.csv could be read from %s/frames/" % run)

    views = select_views(found, args.max_views, size)
    print("Searched %d of %d frames: checkerboard %dx%d found in %d, using %d views."
          % (len(scan), len(indices), cols, rows, len(found), len(views)))
    if missing:
        print("Warning: %d frame files were missing or unreadable." % missing)
    if len(views) < MIN_VIEWS:
        fail("found the %dx%d checkerboard in only %d frames; need at least %d. Check --board "
             "(INNER corners, e.g. 9x6 for a board of 10 x 7 squares), keep the whole board in "
             "view and in focus, and move it to different places and angles."
             % (cols, rows, len(views), MIN_VIEWS))

    flags = 0 if args.free_k3 else cv2.CALIB_FIX_K3
    rms, k, dist, rvecs, tvecs = cv2.calibrateCamera(
        [obj] * len(views), [c for _, c in views], size, None, None, flags=flags)
    dist = [float(d) for d in np.asarray(dist).ravel()[:5]] + [0.0] * 5
    dist = dist[:5]
    per_view = []
    for (idx, c), rv, tv in zip(views, rvecs, tvecs):
        proj, _ = cv2.projectPoints(obj, rv, tv, k, np.asarray(dist))
        per_view.append((idx, float(np.sqrt(np.mean(np.sum((proj - c) ** 2, axis=2))))))

    cam_path = os.path.join(run, "camera.json")
    camera = {"mount": {"x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0},
              "rollingShutterReadoutS": 0.0}
    kept = False
    if os.path.isfile(cam_path):
        try:
            with open(cam_path) as f:
                old = json.load(f)
            camera["mount"] = old.get("mount", camera["mount"])
            camera["rollingShutterReadoutS"] = old.get("rollingShutterReadoutS", 0.0)
            kept = True
        except (OSError, ValueError):
            print("Warning: existing %s is not valid JSON; writing a new one." % cam_path)
    camera.update({"width": size[0], "height": size[1],
                   "fx": float(k[0, 0]), "fy": float(k[1, 1]),
                   "cx": float(k[0, 2]), "cy": float(k[1, 2]), "dist": dist})
    ordered = {key: camera[key] for key in ("width", "height", "fx", "fy", "cx", "cy", "dist",
                                            "mount", "rollingShutterReadoutS")}
    with open(cam_path, "w") as f:
        json.dump(ordered, f, indent=2)
        f.write("\n")
    report = {"rmsPx": float(rms), "views": len(views), "board": [cols, rows],
              "squareMm": args.square_mm, "fitK3": bool(args.free_k3),
              "perView": [{"idx": i, "rmsPx": e} for i, e in per_view]}
    with open(os.path.join(run, "calibration_report.json"), "w") as f:
        json.dump(report, f, indent=2)
        f.write("\n")

    hfov = math.degrees(2 * math.atan(size[0] / (2 * k[0, 0])))
    vfov = math.degrees(2 * math.atan(size[1] / (2 * k[1, 1])))
    print("RMS reprojection error %.3f px over %d views" % (rms, len(views)))
    print("fx %.2f  fy %.2f  cx %.2f  cy %.2f  (image %dx%d, FOV %.1f x %.1f deg)"
          % (k[0, 0], k[1, 1], k[0, 2], k[1, 2], size[0], size[1], hfov, vfov))
    print("dist k1 %.4f  k2 %.4f  p1 %.5f  p2 %.5f  k3 %.4f" % tuple(dist))
    worst = sorted(per_view, key=lambda x: -x[1])[:3]
    print("Worst views (frame idx: RMS px): " + ", ".join("%d: %.2f" % w for w in worst))
    if rms > 1.0:
        print("Warning: RMS above 1 px. Re-record with a flat, well-lit board held still.")
    if len(views) < 15:
        print("Warning: fewer than 15 views; more positions and angles give a steadier result.")
    print("Wrote %s" % cam_path)
    if kept:
        print("Kept mount and rollingShutterReadoutS from the existing camera.json.")
    else:
        print("mount and rollingShutterReadoutS were written as 0: set them once the camera is "
              "on the robot (see README).")
    return 0


def select_views(found, max_views, size):
    """Spread the chosen views: skip a view whose corners are nearly where an earlier one's were."""
    import numpy as np
    if len(found) <= max_views:
        return found
    diag = math.hypot(*size)
    chosen = []
    threshold = 0.08 * diag
    while True:
        chosen = []
        for idx, c in found:
            if all(np.mean(np.linalg.norm(c - d, axis=2)) > threshold for _, d in chosen):
                chosen.append((idx, c))
        if len(chosen) <= max_views:
            break
        threshold *= 1.3
    if len(chosen) < max_views:  # top up evenly from the rest
        rest = [f for f in found if all(f[0] != c[0] for c in chosen)]
        step = max(1, len(rest) // (max_views - len(chosen)))
        chosen += rest[::step][:max_views - len(chosen)]
    return chosen


if __name__ == "__main__":
    sys.exit(main())
