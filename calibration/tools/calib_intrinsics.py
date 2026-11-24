#!/usr/bin/env python3
"""Camera intrinsic calibration from a recorded dataset or a video, with a trust verdict.

Usage:
    calib_intrinsics.py DATASET_DIR|VIDEO|IMAGE_DIR --target targets/aprilgrid_6x6.yaml \\
        --out phones/galaxy_s22/intrinsics.yaml [--model radtan|equidistant] [--strict]

What it does: detects the board in every frame (OpenCV aruco), throws away blurred frames and
near-duplicate frames, calibrates pinhole-radtan (or equidistant fisheye), then judges the result.

The judgement is the point. The tool prints a verdict and exits non-zero when the numbers should
not be used:
  FAIL                    exit 1  too few views, board never reached the image corners/edges,
                                  reprojection error above 0.3 px, capture settings wrong, ...
  PASS_WITH_WARNINGS      exit 0  usable but something is marginal (exit 1 with --strict)
  PASS                    exit 0
  input problem           exit 2  file missing, unreadable, no frames
A FAIL writes no output YAML unless --write-even-if-failed is given, and that file then carries
an UNTRUSTED banner and trusted: false.

Every threshold is a constant near the top of this file, with the reason beside it.
"""
import argparse
import datetime
import glob
import os
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import board  # noqa: E402
import dataset  # noqa: E402

# ---- thresholds -------------------------------------------------------------------------
MAX_RMS_PX = 0.30          # spec 05: reprojection error <= 0.3 px. Judged on the RMS of the 2-D
#                            distance (OpenCV's definition), which is sqrt(2) times the per-axis RMS.
HOLDOUT_FACTOR = 1.5       # held-out error more than 1.5x the fit error suggests over-fitting
FAIL_VIEWS = 20            # fewer accepted views than this: calibration is not constrained
WARN_VIEWS = 40            # fewer than this: marginal
GRID = 5                   # coverage grid is GRID x GRID cells over the image
CELL_MIN_POINTS = 10       # a cell counts as covered with at least this many corner points
FAIL_CELL_FRACTION = 0.6   # fewer covered cells than this: fail
WARN_CELL_FRACTION = 0.85
FAIL_BORDER_FRACTION = 0.5  # of the 16 border cells
WARN_BORDER_FRACTION = 0.75
FAIL_HULL_FRACTION = 0.5   # convex hull of all corner points / image area
WARN_HULL_FRACTION = 0.75
TILT_MIN_DEG = 15.0        # views with the board tilted at least this much help the focal length
FAIL_TILTED_VIEWS = 3
WARN_TILTED_VIEWS = 10
BLUR_RELATIVE = 0.55       # keep frames at least this fraction as sharp as the median (good tilted frames reach ~0.6)
SHARP_REF = {"aprilgrid": 9.0, "charuco": 14.0}  # gradient-energy/variance of an in-focus frame
SHARP_WARN_FRACTION = 0.6  # median sharpness below 0.6x reference: recording is soft overall
SHARP_FAIL_FRACTION = 0.45
MIN_MOTION_PX = 6.0        # a frame must differ from the last accepted one by this many pixels
MAX_VIEWS = 150            # calibrate with at most this many views (evenly thinned)
OUTLIER_FACTOR = 2.5       # drop views whose error exceeds 2.5x the median view error (once)
MAX_OUTLIER_FRACTION = 0.2
FOCAL_RATIO = (0.3, 3.0)   # same range tools/check_calibration.py enforces
FPS_WARN = 15.0

VERDICT_PASS = "PASS"
VERDICT_WARN = "PASS_WITH_WARNINGS"
VERDICT_FAIL = "FAIL"


class InputError(Exception):
    """Something the user must fix in the input."""


# ---------------------------------------------------------------------------- frame input

def iter_frames(source, stride=1):
    """Yield (name, gray uint8) for every frame in a dataset dir, image dir, or video file."""
    if os.path.isdir(source):
        folder = os.path.join(source, "images")
        if not os.path.isdir(folder):
            folder = source
        files = []
        for pattern in ("*.pgm", "*.png", "*.jpg", "*.jpeg", "*.bmp"):
            files.extend(glob.glob(os.path.join(folder, pattern)))
        files.sort()
        if not files:
            raise InputError(f"no .pgm/.png/.jpg images found in {folder}")
        for i, path in enumerate(files):
            if i % stride:
                continue
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                raise InputError(f"cannot read image {path}")
            yield os.path.basename(path), img
    elif os.path.isfile(source):
        cap = cv2.VideoCapture(source)
        if not cap.isOpened():
            raise InputError(f"cannot open video {source}")
        i = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if i % stride == 0:
                yield f"frame{i:06d}", cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            i += 1
        cap.release()
        if i == 0:
            raise InputError(f"video {source} has no readable frames")
    else:
        raise InputError(f"input not found: {source}")


def sharpness(gray, pts=None):
    """Gradient energy divided by intensity variance, over the board's bounding box.

    Dividing by the variance removes dependence on contrast; the value falls as edges blur.
    Measured on synthetic renders: ~9-11 in focus for the AprilGrid, ~6 at Gaussian sigma 2 px.
    """
    g = gray.astype(np.float32)
    if pts is not None and len(pts) > 0:
        h, w = g.shape
        x0, y0 = np.maximum(pts.min(axis=0).astype(int) - 10, 0)
        x1, y1 = np.minimum(pts.max(axis=0).astype(int) + 10, [w, h])
        if x1 - x0 > 8 and y1 - y0 > 8:
            g = g[y0:y1, x0:x1]
    gx = cv2.Sobel(g, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(g, cv2.CV_32F, 0, 1, ksize=3)
    var = float(g.var())
    if var < 1e-3:
        return 0.0
    return float(((gx ** 2 + gy ** 2).mean()) / var)


class Obs:
    def __init__(self, index, name, ids, img_pts, obj_pts, sharp):
        self.index, self.name = index, name
        self.ids, self.img_pts, self.obj_pts, self.sharp = ids, img_pts, obj_pts, sharp


# ---------------------------------------------------------------------------- calibration

def project(model, obj, rvec, tvec, K, D):
    obj = np.asarray(obj, np.float64).reshape(-1, 1, 3)
    if model == "equidistant":
        pts, _ = cv2.fisheye.projectPoints(obj, np.asarray(rvec, np.float64).reshape(3, 1),
                                           np.asarray(tvec, np.float64).reshape(3, 1), K,
                                           np.asarray(D, np.float64).reshape(4, 1))
    else:
        pts, _ = cv2.projectPoints(obj, np.asarray(rvec, np.float64).reshape(3, 1),
                                   np.asarray(tvec, np.float64).reshape(3, 1), K, D)
    return pts.reshape(-1, 2)


def calibrate(model, views, size):
    """Run the OpenCV calibration. Returns dict(K, D, rvecs, tvecs, per_view, rms, std_intr)."""
    obj = [v.obj_pts.astype(np.float64) for v in views]
    img = [v.img_pts.astype(np.float64) for v in views]
    if model == "equidistant":
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW)
        try:
            _, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
                [o.reshape(1, -1, 3) for o in obj], [i.reshape(1, -1, 2) for i in img], size, K, D,
                flags=flags, criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-9))
        except cv2.error as err:
            raise CalibrationFailed(f"fisheye calibration did not converge ({_first_line(err)}); "
                                    "usually the views are too similar or the lens is not "
                                    "fisheye. Try --model radtan.")
        std = None
    else:
        flags = cv2.CALIB_FIX_K3
        try:
            _, K, D, rvecs, tvecs, std_i, _, _ = cv2.calibrateCameraExtended(
                [o.astype(np.float32) for o in obj], [i.astype(np.float32) for i in img], size,
                None, None, flags=flags,
                criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 200, 1e-10))
        except cv2.error as err:
            raise CalibrationFailed(f"calibration failed ({_first_line(err)})")
        D = D.reshape(-1)[:4]
        std = std_i.reshape(-1)
    per_view, sq_sum, n = [], 0.0, 0
    for o, i, r, t in zip(obj, img, rvecs, tvecs):
        e = project(model, o, r, t, K, D) - i
        d2 = (e ** 2).sum(axis=1)
        per_view.append(float(np.sqrt(d2.mean())))
        sq_sum += float(d2.sum())
        n += len(d2)
    return {"K": K, "D": np.asarray(D, np.float64).reshape(-1), "rvecs": rvecs, "tvecs": tvecs,
            "per_view": np.array(per_view), "rms": float(np.sqrt(sq_sum / n)), "std": std}


class CalibrationFailed(Exception):
    pass


def _first_line(err):
    text = str(err).strip().splitlines()
    return text[-1] if text else "unknown OpenCV error"


def holdout_error(model, views, K, D):
    """RMS (2-D distance) of views NOT used in the fit, posed with solvePnP on the fitted model."""
    sq, n = 0.0, 0
    for v in views:
        pts = v.img_pts.reshape(-1, 1, 2)
        if model == "equidistant":
            norm = cv2.fisheye.undistortPoints(pts, K, D.reshape(4, 1)).reshape(-1, 2)
            ok, rvec, tvec = cv2.solvePnP(v.obj_pts, norm, np.eye(3), None,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
        else:
            ok, rvec, tvec = cv2.solvePnP(v.obj_pts, v.img_pts, K, D,
                                          flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        e = project(model, v.obj_pts, rvec, tvec, K, D) - v.img_pts
        sq += float((e ** 2).sum())
        n += len(e)
    return float(np.sqrt(sq / n)) if n else None


# ---------------------------------------------------------------------------- judging

def coverage_stats(views, size):
    w, h = size
    pts = np.concatenate([v.img_pts for v in views], axis=0)
    gx = np.clip((pts[:, 0] / w * GRID).astype(int), 0, GRID - 1)
    gy = np.clip((pts[:, 1] / h * GRID).astype(int), 0, GRID - 1)
    counts = np.zeros((GRID, GRID), int)
    np.add.at(counts, (gy, gx), 1)
    covered = counts >= CELL_MIN_POINTS
    border = np.zeros((GRID, GRID), bool)
    border[0, :] = border[-1, :] = border[:, 0] = border[:, -1] = True
    corners = [(0, 0), (0, GRID - 1), (GRID - 1, 0), (GRID - 1, GRID - 1)]
    hull = cv2.convexHull(pts.astype(np.float32))
    hull_frac = float(cv2.contourArea(hull) / (w * h))
    return {
        "cells_covered": int(covered.sum()), "cells_total": GRID * GRID,
        "cell_fraction": float(covered.sum() / (GRID * GRID)),
        "border_covered": int((covered & border).sum()), "border_total": int(border.sum()),
        "border_fraction": float((covered & border).sum() / border.sum()),
        "empty_corner_cells": int(sum(1 for c in corners if not covered[c])),
        "hull_fraction": hull_frac, "counts": counts,
    }


def tilt_angles(rvecs):
    out = []
    for r in rvecs:
        R, _ = cv2.Rodrigues(np.asarray(r, np.float64).reshape(3, 1))
        out.append(float(np.degrees(np.arccos(np.clip(abs(R[2, 2]), 0, 1)))))
    return np.array(out)


def format_counts(counts):
    lines = []
    for row in counts:
        lines.append("    " + " ".join(f"{int(c):5d}" for c in row))
    return "\n".join(lines)


# ---------------------------------------------------------------------------- main pipeline

def run(source, target_path, model="radtan", stride=1, min_motion=MIN_MOTION_PX,
        max_views=MAX_VIEWS, log=print):
    """Calibrate and judge. Returns a result dict; never raises on bad data, only on bad input."""
    target = board.load_target(target_path)
    warnings, failures = [], []
    result = {"source": os.path.abspath(source), "target": os.path.abspath(target_path),
              "model": model, "warnings": warnings, "failures": failures}

    # capture settings and timestamps, when the dataset has them
    if os.path.isdir(source):
        meta = dataset.read_meta(source)
        w_, f_ = dataset.check_meta(meta)
        warnings.extend(w_)
        failures.extend(f_)
        ts_path = os.path.join(source, "cam_timestamps.csv")
        if os.path.isfile(ts_path):
            _check_timestamps(ts_path, warnings, result)
        else:
            warnings.append("no cam_timestamps.csv: frame rate and dropped frames cannot be checked")
    else:
        warnings.append("input is a video or bare image folder, not a Beacon dataset: capture "
                        "settings (manual exposure, fixed focus, OIS off) cannot be verified. "
                        "A phone-camera video usually has auto exposure/focus and gives a "
                        "calibration that is invalid for the runtime capture.")

    # 1. detect
    obs_all, undetected_sharp, size, n_frames = [], [], None, 0
    for idx, (name, gray) in enumerate(iter_frames(source, stride)):
        n_frames += 1
        if size is None:
            size = (gray.shape[1], gray.shape[0])
        elif (gray.shape[1], gray.shape[0]) != size:
            raise InputError(f"frame {name} is {gray.shape[1]}x{gray.shape[0]} but earlier frames "
                             f"were {size[0]}x{size[1]}: mixed resolutions cannot be calibrated "
                             "together")
        ids, ip, op = target.detect(gray)
        if len(ids) >= target.min_points:
            obs_all.append(Obs(idx, name, ids, ip, op, sharpness(gray, ip)))
        else:
            undetected_sharp.append(sharpness(gray))
    if n_frames == 0:
        raise InputError("no frames read")
    result["image_size"] = list(size)
    n_det = len(obs_all)
    log(f"frames read: {n_frames}   board detected in: {n_det}")
    if n_det == 0:
        failures.append("the board was not detected in any frame (wrong --target file, board out "
                        "of view, too blurred, or too small in the image)")
        return _finish(result, target, None, n_frames, 0, 0, 0, 0, None, None, None, None, size)
    if n_det < 0.2 * n_frames:
        warnings.append(f"board detected in only {n_det} of {n_frames} frames; the rest were "
                        "dropped (board out of view, blurred, or too oblique)")

    # 2. blur rejection
    sharp_vals = np.array([o.sharp for o in obs_all])
    med_sharp = float(np.median(sharp_vals))
    ref = SHARP_REF[target.kind]
    keep = [o for o in obs_all if o.sharp >= BLUR_RELATIVE * med_sharp]
    n_blur = n_det - len(keep)
    if undetected_sharp:
        n_soft_undetected = int(sum(1 for s in undetected_sharp if s < BLUR_RELATIVE * med_sharp))
        if n_soft_undetected > 0.3 * len(undetected_sharp) and len(undetected_sharp) >= 5:
            warnings.append(f"{n_soft_undetected} of the {len(undetected_sharp)} frames where the "
                            "board was not found also look soft: blur is probably why detection "
                            "failed. Move more slowly or shorten the exposure.")
    if med_sharp < SHARP_FAIL_FRACTION * ref:
        failures.append(f"the recording is blurred overall (median sharpness {med_sharp:.1f}, "
                        f"in-focus reference ~{ref:.0f}): focus is wrong or exposure is too long")
    elif med_sharp < SHARP_WARN_FRACTION * ref:
        warnings.append(f"the recording is soft overall (median sharpness {med_sharp:.1f}, "
                        f"in-focus reference ~{ref:.0f}): check focus distance and exposure")
    log(f"blur: median sharpness {med_sharp:.2f}; rejected {n_blur} frame(s) below "
        f"{BLUR_RELATIVE * med_sharp:.2f}")

    # 3. near-duplicate rejection (a board that sits still must not count as many views)
    accepted, last = [], None
    n_dup = 0
    for o in keep:
        if last is not None:
            common, ia, ib = np.intersect1d(o.ids, last.ids, return_indices=True)
            if len(common) >= 8:
                motion = float(np.linalg.norm(o.img_pts[ia] - last.img_pts[ib], axis=1).mean())
                if motion < min_motion:
                    n_dup += 1
                    continue
        accepted.append(o)
        last = o
    n_thinned = 0
    if len(accepted) > max_views:
        sel = np.linspace(0, len(accepted) - 1, max_views).round().astype(int)
        n_thinned = len(accepted) - max_views
        accepted = [accepted[i] for i in sel]
    log(f"views: {len(accepted)} accepted ({n_dup} near-duplicates dropped, {n_thinned} thinned)")

    result["frames"] = {"read": n_frames, "detected": n_det, "rejected_blur": n_blur,
                        "rejected_duplicate": n_dup, "thinned": n_thinned,
                        "accepted": len(accepted)}
    if len(accepted) < FAIL_VIEWS:
        failures.append(f"only {len(accepted)} usable views (need at least {FAIL_VIEWS}, "
                        f"{WARN_VIEWS}+ preferred): record longer and keep moving the board")
        return _finish(result, target, None, n_frames, n_det, n_blur, n_dup, len(accepted),
                       None, accepted, None, None, size)
    if len(accepted) < WARN_VIEWS:
        warnings.append(f"only {len(accepted)} usable views; {WARN_VIEWS}+ preferred")

    # 4. coverage
    cov = coverage_stats(accepted, size)
    _judge_coverage(cov, failures, warnings)

    # 5. calibrate (+ one outlier pass), holdout
    holdout, train = [], list(accepted)
    if len(accepted) >= 30:
        holdout = accepted[4::5]
        train = [v for i, v in enumerate(accepted) if i % 5 != 4]
    try:
        fit = calibrate(model, train, size)
        hold_rms = holdout_error(model, holdout, fit["K"], fit["D"]) if holdout else None
        final_views = list(accepted)
        fit_all = calibrate(model, final_views, size)
        med = float(np.median(fit_all["per_view"]))
        bad = [i for i, e in enumerate(fit_all["per_view"]) if e > OUTLIER_FACTOR * max(med, 0.05)]
        n_out = 0
        if bad and len(bad) <= MAX_OUTLIER_FRACTION * len(final_views):
            final_views = [v for i, v in enumerate(final_views) if i not in set(bad)]
            n_out = len(bad)
            fit_all = calibrate(model, final_views, size)
            log(f"dropped {n_out} outlier view(s) with error > {OUTLIER_FACTOR}x the median")
        elif bad:
            warnings.append(f"{len(bad)} views have unusually large error but too many to drop "
                            "safely: board detection is unreliable in this recording")
    except CalibrationFailed as err:
        failures.append(str(err))
        return _finish(result, target, None, n_frames, n_det, n_blur, n_dup, len(accepted),
                       cov, accepted, None, None, size)
    result["frames"]["outlier_views_dropped"] = n_out
    result["frames"]["accepted"] = len(final_views)
    K, D = fit_all["K"], fit_all["D"]
    rms = fit_all["rms"]
    result["rms_px"] = rms
    result["holdout_rms_px"] = hold_rms
    result["worst_view_px"] = float(fit_all["per_view"].max())
    log(f"reprojection RMS (2-D distance): {rms:.3f} px   held-out RMS: "
        f"{'n/a' if hold_rms is None else f'{hold_rms:.3f} px'}")

    if not np.isfinite(rms):
        failures.append("reprojection error is not finite")
    elif rms > MAX_RMS_PX:
        failures.append(f"reprojection error {rms:.3f} px is above the {MAX_RMS_PX} px limit: "
                        "likely causes are the wrong lens model (try --model equidistant for "
                        "wide-angle), a warped or non-flat board, blur, or wrong target dimensions")
    if hold_rms is not None and hold_rms > MAX_RMS_PX and hold_rms > HOLDOUT_FACTOR * fit["rms"]:
        failures.append(f"held-out views reproject at {hold_rms:.3f} px against {fit['rms']:.3f} px "
                        "on the fitted views: the model is over-fitting this recording")
    elif hold_rms is not None and hold_rms > HOLDOUT_FACTOR * fit["rms"] and hold_rms > 0.2:
        warnings.append(f"held-out error {hold_rms:.3f} px is well above the fit error "
                        f"{fit['rms']:.3f} px: possible over-fitting")

    # 6. tilt diversity
    tilts = tilt_angles(fit_all["rvecs"])
    n_tilted = int((tilts >= TILT_MIN_DEG).sum())
    result["tilt_deg"] = {"max": float(tilts.max()), "views_over_%d" % TILT_MIN_DEG: n_tilted}
    if n_tilted < FAIL_TILTED_VIEWS:
        failures.append(f"only {n_tilted} view(s) with the board tilted {TILT_MIN_DEG:.0f} deg or "
                        "more: a flat, face-on board cannot separate focal length from distance")
    elif n_tilted < WARN_TILTED_VIEWS:
        warnings.append(f"only {n_tilted} tilted views (>= {TILT_MIN_DEG:.0f} deg); "
                        f"{WARN_TILTED_VIEWS}+ preferred")

    # 7. parameter sanity
    fx, fy, cx, cy = float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])
    w, h = size
    if not (FOCAL_RATIO[0] * w <= fx <= FOCAL_RATIO[1] * w and
            FOCAL_RATIO[0] * w <= fy <= FOCAL_RATIO[1] * w):
        failures.append(f"focal length fx={fx:.1f}, fy={fy:.1f} is outside {FOCAL_RATIO[0]}-"
                        f"{FOCAL_RATIO[1]} x image width ({w}): the fit is not physical")
    if not (0 < cx < w and 0 < cy < h):
        failures.append(f"principal point ({cx:.1f}, {cy:.1f}) lies outside the image")
    elif abs(cx - w / 2) > 0.15 * w or abs(cy - h / 2) > 0.15 * h:
        warnings.append(f"principal point ({cx:.1f}, {cy:.1f}) is more than 15 % of the image "
                        "away from the centre; phone cameras are normally within a few percent")
    if abs(fx - fy) / max(fx, fy) > 0.02:
        warnings.append(f"fx and fy differ by {100 * abs(fx - fy) / max(fx, fy):.1f} %; phone "
                        "sensors have square pixels, so this points to a bad fit")
    if fit_all["std"] is not None:
        std = fit_all["std"]
        result["intrinsics_std_px"] = [float(std[0]), float(std[1]), float(std[2]), float(std[3])]
        if std[0] > 0.01 * fx or std[2] > 3.0 or std[3] > 3.0:
            warnings.append(f"intrinsics are poorly constrained (std fx={std[0]:.1f}, "
                            f"cx={std[2]:.1f}, cy={std[3]:.1f} px): add views with more tilt, "
                            "more distance variation and more image coverage")
    if model == "radtan" and abs(D[0]) > 1.0:
        warnings.append(f"k1={D[0]:.2f} is very large for a radtan model: is this a fisheye lens? "
                        "Try --model equidistant.")

    return _finish(result, target, fit_all, n_frames, n_det, n_blur, n_dup, len(final_views), cov,
                   final_views, tilts, K, size, D=D)


def _check_timestamps(path, warnings, result):
    try:
        ts = dataset.read_cam_timestamps(path)
    except dataset.DatasetError as err:
        warnings.append(f"cam_timestamps.csv unreadable: {err}")
        return
    t = ts["t_ns"].astype(np.float64)
    if len(t) < 2:
        return
    dt = np.diff(t) * 1e-9
    if np.any(dt <= 0):
        warnings.append("cam_timestamps.csv has non-increasing timestamps")
        return
    fps = 1.0 / float(np.median(dt))
    gaps = int((dt > 2.5 * np.median(dt)).sum())
    result["camera_fps"] = fps
    if fps < FPS_WARN:
        warnings.append(f"camera rate is {fps:.1f} fps; blur and tracking are worse below "
                        f"{FPS_WARN:.0f} fps")
    if gaps:
        warnings.append(f"{gaps} gap(s) in the camera timestamps longer than 2.5 frame periods "
                        "(dropped frames)")


def _judge_coverage(cov, failures, warnings):
    if cov["cell_fraction"] < FAIL_CELL_FRACTION:
        failures.append(f"board corners cover only {cov['cells_covered']} of {cov['cells_total']} "
                        "image regions: move the board into every part of the image")
    elif cov["cell_fraction"] < WARN_CELL_FRACTION:
        warnings.append(f"board corners cover {cov['cells_covered']} of {cov['cells_total']} "
                        "image regions; aim for nearly all")
    if cov["border_fraction"] < FAIL_BORDER_FRACTION:
        failures.append(f"the image edges and corners are poorly covered "
                        f"({cov['border_covered']} of {cov['border_total']} border regions): "
                        "distortion is worst there and the fit extrapolates")
    elif cov["border_fraction"] < WARN_BORDER_FRACTION:
        warnings.append(f"only {cov['border_covered']} of {cov['border_total']} border regions "
                        "covered; push the board into the edges")
    if cov["empty_corner_cells"] >= 2:
        failures.append(f"{cov['empty_corner_cells']} of the 4 image corners have no board points")
    elif cov["empty_corner_cells"] == 1:
        warnings.append("one image corner has no board points")
    if cov["hull_fraction"] < FAIL_HULL_FRACTION:
        failures.append(f"corner points span only {100 * cov['hull_fraction']:.0f} % of the image "
                        "area (convex hull)")
    elif cov["hull_fraction"] < WARN_HULL_FRACTION:
        warnings.append(f"corner points span {100 * cov['hull_fraction']:.0f} % of the image "
                        "area; aim for 75 % or more")


def _finish(result, target, fit, n_frames, n_det, n_blur, n_dup, n_acc, cov, views, tilts, K,
            size, D=None):
    result.setdefault("frames", {"read": n_frames, "detected": n_det, "rejected_blur": n_blur,
                                 "rejected_duplicate": n_dup, "accepted": n_acc})
    if cov is not None:
        result["coverage"] = {k: v for k, v in cov.items() if k != "counts"}
        result["coverage_counts"] = cov["counts"].tolist()
    if not target.size_verified:
        result["warnings"].append(
            "target file says printed_size_verified: false. Intrinsics do not depend on the "
            "board scale, but any later metric use of this board does; measure the print and "
            "update the target file.")
    if K is not None:
        result["intrinsics"] = [float(K[0, 0]), float(K[1, 1]), float(K[0, 2]), float(K[1, 2])]
        result["distortion_coeffs"] = [float(x) for x in D]
    if result["failures"]:
        result["verdict"] = VERDICT_FAIL
    elif result["warnings"]:
        result["verdict"] = VERDICT_WARN
    else:
        result["verdict"] = VERDICT_PASS
    result["trusted"] = result["verdict"] != VERDICT_FAIL
    return result


# ---------------------------------------------------------------------------- output

def print_report(result):
    print()
    print("=" * 70)
    if "intrinsics" in result:
        fx, fy, cx, cy = result["intrinsics"]
        print(f"model        : {'pinhole-equidistant' if result['model'] == 'equidistant' else 'pinhole-radtan'}"
              f"   image {result['image_size'][0]}x{result['image_size'][1]}")
        print(f"intrinsics   : fx={fx:.2f} fy={fy:.2f} cx={cx:.2f} cy={cy:.2f}")
        print(f"distortion   : {[round(x, 6) for x in result['distortion_coeffs']]}")
    if "rms_px" in result:
        print(f"reprojection : {result['rms_px']:.3f} px RMS (limit {MAX_RMS_PX}),"
              f" worst view {result['worst_view_px']:.3f} px")
    cov = result.get("coverage")
    if cov:
        print(f"coverage     : {cov['cells_covered']}/{cov['cells_total']} regions, "
              f"{cov['border_covered']}/{cov['border_total']} border regions, "
              f"{100 * cov['hull_fraction']:.0f} % of image area, "
              f"{cov['empty_corner_cells']} empty image corner(s)")
        print("  corner points per region (rows top to bottom):")
        print(format_counts(np.array(result["coverage_counts"])))
    if "tilt_deg" in result:
        print(f"board tilt   : max {result['tilt_deg']['max']:.0f} deg")
    fr = result["frames"]
    print(f"frames       : {fr.get('read')} read, {fr.get('detected')} with board, "
          f"{fr.get('rejected_blur')} blurred, {fr.get('rejected_duplicate')} duplicate, "
          f"{fr.get('accepted')} used")
    for w in result["warnings"]:
        print(f"WARNING: {w}")
    for f in result["failures"]:
        print(f"FAIL: {f}")
    print(f"VERDICT: {result['verdict']}" + ("   (do NOT use these numbers)"
                                             if result["verdict"] == VERDICT_FAIL else ""))
    print("=" * 70)


def write_yaml(result, path, source_label=""):
    """Write the intrinsics as a Kalibr-style cam0 block plus a calibration_report block."""
    banner = ""
    if not result["trusted"]:
        banner = ("# UNTRUSTED: calibrate_intrinsics judged this calibration FAIL. The numbers\n"
                  "# below were written only because --write-even-if-failed was given. Do not\n"
                  "# use them on a robot. Failures:\n" +
                  "".join(f"#   - {f}\n" for f in result["failures"]))
    model = result["model"]
    doc = {
        "cam0": {
            "camera_model": "pinhole",
            "distortion_model": model,
            "intrinsics": [round(x, 4) for x in result["intrinsics"]],
            "distortion_coeffs": [round(x, 8) for x in result["distortion_coeffs"]],
            "resolution": result["image_size"],
        },
        "calibration_report": {
            "verdict": result["verdict"],
            "trusted": result["trusted"],
            "rms_reprojection_px": round(result.get("rms_px", float("nan")), 4),
            "holdout_rms_px": (None if result.get("holdout_rms_px") is None
                               else round(result["holdout_rms_px"], 4)),
            "frames": result["frames"],
            "coverage": {k: (round(v, 3) if isinstance(v, float) else v)
                         for k, v in result.get("coverage", {}).items()},
            "warnings": result["warnings"],
            "failures": result["failures"],
            "camera_fps": result.get("camera_fps"),
            "target": os.path.basename(result["target"]),
            "opencv": cv2.__version__,
            "date": datetime.date.today().isoformat(),
        },
    }
    text = yaml.safe_dump(doc, sort_keys=False, default_flow_style=None)
    head = "# Camera intrinsics written by calibration/tools/calib_intrinsics.py\n" + banner
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(head + text)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("source", help="dataset directory (images/*.pgm), image folder, or video file")
    ap.add_argument("--target", required=True, help="target YAML, e.g. targets/aprilgrid_6x6.yaml")
    ap.add_argument("--out", help="write the intrinsics YAML here")
    ap.add_argument("--model", choices=["radtan", "equidistant"], default="radtan",
                    help="pinhole-radtan (default) or equidistant fisheye")
    ap.add_argument("--stride", type=int, default=1, help="use every Nth frame (speed)")
    ap.add_argument("--min-motion-px", type=float, default=MIN_MOTION_PX)
    ap.add_argument("--max-views", type=int, default=MAX_VIEWS)
    ap.add_argument("--strict", action="store_true", help="exit 1 on warnings too")
    ap.add_argument("--write-even-if-failed", action="store_true",
                    help="write the YAML for a FAILed calibration (marked UNTRUSTED)")
    args = ap.parse_args(argv)

    try:
        result = run(args.source, args.target, args.model, args.stride, args.min_motion_px,
                     args.max_views)
    except (InputError, board.TargetError, dataset.DatasetError) as err:
        print(f"Error: {err}", file=sys.stderr)
        return 2
    print_report(result)
    if args.out:
        if "intrinsics" not in result:
            print("No intrinsics were computed, so no file was written.", file=sys.stderr)
        elif result["verdict"] == VERDICT_FAIL and not args.write_even_if_failed:
            print(f"Verdict is FAIL: {args.out} was NOT written "
                  "(use --write-even-if-failed to keep an UNTRUSTED copy).")
        else:
            write_yaml(result, args.out)
            print(f"wrote {args.out}")
    if result["verdict"] == VERDICT_FAIL:
        return 1
    if result["verdict"] == VERDICT_WARN and args.strict:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
