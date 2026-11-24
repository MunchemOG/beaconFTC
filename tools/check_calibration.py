#!/usr/bin/env python3
"""Sanity-check OpenVINS/Kalibr calibration YAML files.

Usage:
    python3 tools/check_calibration.py calibration/phones/galaxy_s22/kalibr_imucam_chain.yaml
    python3 tools/check_calibration.py camera.yaml imu.yaml

A file is a camera chain if it has a top-level key cam0, or an IMU file if it has imu0.
Prints one PASS or FAIL line per check.
Exit status: 0 if every check passed, 1 if any check failed,
2 if a file is missing, unreadable, not YAML, or is neither kind of file.
"""
import argparse
import math
import os
import sys

import numpy as np
import yaml

FOCAL_MIN_RATIO = 0.3   # focal length must be between 0.3 and 3 times the image width
FOCAL_MAX_RATIO = 3.0
TIMESHIFT_MAX_S = 0.05  # |timeshift_cam_imu| must be under 50 ms
ORTHO_TOL = 1e-3        # R^T R must match the identity within this
DET_TOL = 1e-3          # determinant must be +1 within this
BOTTOM_ROW_TOL = 1e-6
UPDATE_RATE_MIN = 100   # Hz
NOISE_KEYS = ["accelerometer_noise_density", "accelerometer_random_walk",
              "gyroscope_noise_density", "gyroscope_random_walk"]


class InputError(Exception):
    """A problem with an input file that the user can fix."""


def load_yaml(path):
    """Read the file, drop an OpenVINS '%YAML:1.0' directive line, and parse it."""
    if not os.path.isfile(path):
        raise InputError(f"file not found: {path}")
    try:
        with open(path, encoding="utf-8-sig") as f:
            text = f.read()
    except UnicodeDecodeError:
        raise InputError(f"{path} is not a text file")
    except OSError as err:
        raise InputError(f"cannot read {path}: {err.strerror}")
    if not text.strip():
        raise InputError(f"{path} is empty")
    if text.startswith("%YAML"):
        # PyYAML rejects the OpenVINS/OpenCV directive; the rest of the file is plain YAML.
        text = text.split("\n", 1)[1] if "\n" in text else ""
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as err:
        first = str(err).strip().splitlines()[0] if str(err).strip() else "unknown error"
        raise InputError(f"{path} is not valid YAML ({first})")
    if not isinstance(data, dict):
        raise InputError(f"{path} does not hold key: value pairs at the top level")
    return data


def kind_of(data, path):
    has_cam = "cam0" in data
    has_imu = "imu0" in data
    if has_cam and has_imu:
        raise InputError(f"{path} has both cam0 and imu0; put them in separate files")
    if has_cam:
        return "camera"
    if has_imu:
        return "imu"
    raise InputError(f"{path} is neither a camera file (no cam0 key) nor an IMU file (no imu0 key)")


def as_number(value):
    """Return a finite float, or None for anything else (bools, strings, lists, NaN)."""
    if isinstance(value, bool):
        return None
    if isinstance(value, str):
        try:
            value = float(value)
        except ValueError:
            return None
    if not isinstance(value, (int, float)):
        return None
    if not math.isfinite(value):
        return None
    return float(value)


def as_numbers(value, count):
    """Return a list of `count` finite floats, or None."""
    if not isinstance(value, list) or len(value) != count:
        return None
    numbers = [as_number(v) for v in value]
    if any(n is None for n in numbers):
        return None
    return numbers


def as_matrix4(value):
    """Return a 4x4 numpy array of finite floats, or None."""
    if not isinstance(value, list) or len(value) != 4:
        return None
    rows = [as_numbers(row, 4) for row in value]
    if any(row is None for row in rows):
        return None
    return np.array(rows, dtype=float)


def check(name, ok, detail, failed):
    """Print one PASS/FAIL line; record the failure."""
    print(f"{'PASS' if ok else 'FAIL'}  {name}: {detail}")
    if not ok:
        failed.append(name)


def check_transform(label, value, failed):
    matrix = as_matrix4(value)
    if matrix is None:
        check(f"{label} is a 4x4 matrix", False, "missing, or not 16 finite numbers", failed)
        return
    rot = matrix[:3, :3]
    bottom = matrix[3, :]
    bottom_err = float(np.max(np.abs(bottom - np.array([0.0, 0.0, 0.0, 1.0]))))
    check(f"{label} bottom row", bottom_err <= BOTTOM_ROW_TOL,
          f"{bottom.tolist()} (must be 0 0 0 1)", failed)
    ortho_err = float(np.max(np.abs(rot.T @ rot - np.eye(3))))
    check(f"{label} rotation orthonormal", ortho_err <= ORTHO_TOL,
          f"max |R^T R - I| = {ortho_err:.2e} (must be <= {ORTHO_TOL:g})", failed)
    det = float(np.linalg.det(rot))
    check(f"{label} rotation determinant", abs(det - 1.0) <= DET_TOL,
          f"{det:.6f} (must be +1 within {DET_TOL:g})", failed)


def check_camera(data, failed):
    cam = data["cam0"]
    if not isinstance(cam, dict):
        check("cam0 is a mapping", False, "cam0 must hold key: value pairs", failed)
        return

    resolution = as_numbers(cam.get("resolution"), 2)
    width = height = None
    if resolution is None or min(resolution) <= 0:
        check("resolution", False, f"{cam.get('resolution')!r} (must be [width, height])", failed)
    else:
        width, height = resolution
        check("resolution", True, f"{width:g} x {height:g}", failed)

    intrinsics = as_numbers(cam.get("intrinsics"), 4)
    if intrinsics is None:
        check("intrinsics", False, f"{cam.get('intrinsics')!r} (must be [fx, fy, cx, cy])", failed)
    elif width is None:
        check("intrinsics", True, f"{intrinsics}", failed)
    else:
        fx, fy, cx, cy = intrinsics
        lo, hi = FOCAL_MIN_RATIO * width, FOCAL_MAX_RATIO * width
        for name, focal in (("focal fx", fx), ("focal fy", fy)):
            check(name, lo <= focal <= hi,
                  f"{focal:g} (must be between {lo:g} and {hi:g}: "
                  f"{FOCAL_MIN_RATIO:g} to {FOCAL_MAX_RATIO:g} x width {width:g})", failed)
        check("principal point", 0 <= cx <= width and 0 <= cy <= height,
              f"cx={cx:g}, cy={cy:g} (must lie inside {width:g} x {height:g})", failed)

    distortion = cam.get("distortion_coeffs")
    ok = isinstance(distortion, list) and len(distortion) > 0 and as_numbers(
        distortion, len(distortion)) is not None
    check("distortion_coeffs", ok, f"{distortion!r}" if not ok else
          f"{len(distortion)} values", failed)

    found = [key for key in ("T_imu_cam", "T_cam_imu") if key in cam]
    if not found:
        check("T_imu_cam or T_cam_imu", False, "neither key is present", failed)
    for key in found:
        check_transform(key, cam[key], failed)

    shift = as_number(cam.get("timeshift_cam_imu"))
    if shift is None:
        check("timeshift_cam_imu", False, f"{cam.get('timeshift_cam_imu')!r} (must be a number)", failed)
    else:
        check("timeshift_cam_imu", abs(shift) < TIMESHIFT_MAX_S,
              f"{shift:g} s (|value| must be under {TIMESHIFT_MAX_S:g} s)", failed)


def check_imu(data, failed):
    imu = data["imu0"]
    if not isinstance(imu, dict):
        check("imu0 is a mapping", False, "imu0 must hold key: value pairs", failed)
        return
    for key in NOISE_KEYS:
        value = as_number(imu.get(key))
        if value is None:
            check(key, False, f"{imu.get(key)!r} (must be a number)", failed)
        else:
            check(key, value > 0, f"{value:g} (must be positive)", failed)
    rate = as_number(imu.get("update_rate"))
    if rate is None:
        check("update_rate", False, f"{imu.get('update_rate')!r} (must be a number, Hz)", failed)
    else:
        check("update_rate", rate >= UPDATE_RATE_MIN,
              f"{rate:g} Hz (must be at least {UPDATE_RATE_MIN})", failed)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Sanity-check OpenVINS/Kalibr calibration YAML files.")
    parser.add_argument("files", nargs="+", metavar="file.yaml", help="camera chain or IMU YAML")
    args = parser.parse_args(argv)

    any_failed = False
    any_input_error = False
    for path in args.files:
        try:
            data = load_yaml(path)
            kind = kind_of(data, path)
        except InputError as err:
            print(f"Error: {err}", file=sys.stderr)
            any_input_error = True
            continue
        print(f"== {path} ({kind} file)")
        failed = []
        if kind == "camera":
            check_camera(data, failed)
        else:
            check_imu(data, failed)
        if failed:
            any_failed = True
            print(f"RESULT: FAIL ({len(failed)} check(s) failed)")
        else:
            print("RESULT: PASS")

    if any_input_error:
        return 2
    return 1 if any_failed else 0


if __name__ == "__main__":
    sys.exit(main())
