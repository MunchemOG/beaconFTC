"""Reading Beacon recorded datasets (the layout is defined in calibration/README.md).

dataset directory
  meta.yaml            capture settings (see check_meta)
  cam_timestamps.csv   frame_id,t_ns,exposure_ns,iso
  imu.csv              t_ns,gx,gy,gz,ax,ay,az,t_acc_ns
  images/000000.pgm    8-bit binary PGM (P5), sensor orientation, not rotated
"""
import csv
import os

import numpy as np
import yaml

CAM_COLUMNS = ["frame_id", "t_ns", "exposure_ns", "iso"]
IMU_COLUMNS = ["t_ns", "gx", "gy", "gz", "ax", "ay", "az", "t_acc_ns"]


class DatasetError(Exception):
    """A dataset file that is missing or malformed."""


def read_meta(directory):
    """Return the meta.yaml dict, or None if there is no meta.yaml."""
    path = os.path.join(directory, "meta.yaml")
    if not os.path.isfile(path):
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as err:
        raise DatasetError(f"cannot read {path}: {err}")
    if not isinstance(data, dict):
        raise DatasetError(f"{path} does not hold key: value pairs")
    return data


def check_meta(meta):
    """Judge capture settings. Returns (warnings, failures) as lists of strings.

    A calibration made with auto exposure, auto focus or optical image stabilisation is not
    valid for the runtime configuration (spec 05), so those are failures.
    """
    warnings, failures = [], []
    if meta is None:
        warnings.append("no meta.yaml in the dataset: cannot confirm manual exposure, fixed focus "
                        "and OIS off, so the capture settings are UNVERIFIED")
        return warnings, failures
    expect = {"ae_mode": "off", "af_mode": "off", "ois": "off"}
    for key, want in expect.items():
        got = str(meta.get(key, "")).lower()
        if got == "":
            warnings.append(f"meta.yaml has no '{key}': capture setting unverified")
        elif got != want:
            failures.append(f"meta.yaml says {key}={got}; the calibration must be recorded with "
                            f"{key}={want} (same settings as the runtime capture)")
    src = str(meta.get("timestamp_source", "")).upper()
    if src == "":
        warnings.append("meta.yaml has no 'timestamp_source': cannot confirm camera and IMU share "
                        "one clock")
    elif src != "REALTIME":
        failures.append(f"meta.yaml says timestamp_source={src}; camera and IMU clocks are not "
                        "known to match (need REALTIME)")
    return warnings, failures


def read_cam_timestamps(path):
    """Return dict with arrays frame_id (int), t_ns (int64). Raises DatasetError."""
    rows = _read_csv(path, CAM_COLUMNS[:2], "cam_timestamps.csv")
    frame_id = np.array([int(float(r[0])) for r in rows], dtype=np.int64)
    t_ns = np.array([int(float(r[1])) for r in rows], dtype=np.int64)
    return {"frame_id": frame_id, "t_ns": t_ns}


def read_imu(path):
    """Read imu.csv. Returns dict of float64 arrays t_ns (int64), gyro (N,3), acc (N,3), t_acc_ns.

    Also accepts a 7-column EuRoC-style file (t_ns,gx,gy,gz,ax,ay,az) with or without a header;
    t_acc_ns is then None.
    """
    if not os.path.isfile(path):
        raise DatasetError(f"IMU file not found: {path}")
    try:
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.reader(f)
            first = next(reader, None)
            if first is None:
                raise DatasetError(f"{path} is empty")
            body = []
            if _is_number(first[0].lstrip("#").strip()):
                body.append(first)
            body.extend(r for r in reader if r and not r[0].startswith("#"))
    except OSError as err:
        raise DatasetError(f"cannot read {path}: {err.strerror}")
    if not body:
        raise DatasetError(f"{path} has a header but no samples")
    width = len(body[0])
    if width not in (7, 8):
        raise DatasetError(f"{path}: expected 7 or 8 columns "
                           f"({','.join(IMU_COLUMNS)}), found {width}")
    try:
        arr = np.array([[float(v) for v in r[:width]] for r in body if len(r) >= width],
                       dtype=np.float64)
    except ValueError as err:
        raise DatasetError(f"{path}: non-numeric value ({err})")
    if len(arr) < len(body):
        raise DatasetError(f"{path}: {len(body) - len(arr)} row(s) have too few columns")
    if not np.all(np.isfinite(arr)):
        raise DatasetError(f"{path}: contains NaN or infinite values")
    t = np.array([int(float(r[0])) for r in body], dtype=np.int64)
    t_acc = np.array([int(float(r[7])) for r in body], dtype=np.int64) if width == 8 else None
    return {"t_ns": t, "gyro": arr[:, 1:4], "acc": arr[:, 4:7], "t_acc_ns": t_acc}


def _is_number(text):
    try:
        float(text)
        return True
    except ValueError:
        return False


def _read_csv(path, required, label):
    if not os.path.isfile(path):
        raise DatasetError(f"{label} not found: {path}")
    with open(path, newline="", encoding="utf-8") as f:
        rows = [r for r in csv.reader(f) if r and not r[0].startswith("#")]
    if rows and not _is_number(rows[0][0]):
        rows = rows[1:]  # header
    if not rows:
        raise DatasetError(f"{label} has no rows")
    for r in rows:
        if len(r) < len(required):
            raise DatasetError(f"{label}: row {r} has fewer than {len(required)} columns")
        try:
            [float(v) for v in r[:len(required)]]
        except ValueError:
            raise DatasetError(f"{label}: non-numeric value in row {r}")
    return rows
