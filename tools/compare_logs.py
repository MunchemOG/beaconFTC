#!/usr/bin/env python3
"""Compare a Beacon pose log with a reference odometry log recorded on the same clock.

Usage:
    python3 tools/compare_logs.py beacon.csv pinpoint.csv
    python3 tools/compare_logs.py beacon.csv pinpoint.csv --out errors.png

Beacon log: header t_ns,x,y,z,qx,qy,qz,qw,status (metres; heading = yaw from quaternion).
Reference:  header t_ns,x,y,heading (metres; radians).
The reference is interpolated onto the Beacon timestamps inside the overlap only.
Only x and y are compared (the field-plane position); z is ignored.
Exit status: 0 on success, 2 if an input file is missing or malformed.
"""
import argparse
import csv
import math
import os
import sys

import matplotlib

matplotlib.use("Agg")  # never open a window; we only write PNG files
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

BEACON_HEADER = ["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "status"]
REF_HEADER = ["t_ns", "x", "y", "heading"]


class InputError(Exception):
    """A problem with an input file that the user can fix."""


def read_table(path, header, label):
    """Read a CSV with the exact header given; return (rows as float array, line numbers)."""
    if not os.path.isfile(path):
        raise InputError(f"{label} file not found: {path}")
    try:
        with open(path, newline="", encoding="utf-8-sig") as f:
            reader = csv.reader(f)
            head = None
            rows = []
            line_nums = []
            for cells in reader:
                if not any(cell.strip() for cell in cells):
                    continue  # skip blank lines
                if head is None:
                    head = [cell.strip() for cell in cells]
                    if head != header:
                        raise InputError(
                            f"{label} file {path} has the wrong header. Expected "
                            f"{','.join(header)}; found {','.join(head)}")
                    continue
                line = reader.line_num
                if len(cells) != len(header):
                    raise InputError(
                        f"{label} file {path} line {line} has {len(cells)} columns; "
                        f"expected {len(header)}")
                values = []
                for name, cell in zip(header, cells):
                    try:
                        value = float(cell)
                    except ValueError:
                        raise InputError(
                            f"{label} file {path} line {line}: '{cell.strip()}' in column "
                            f"'{name}' is not a number")
                    if not math.isfinite(value):
                        raise InputError(
                            f"{label} file {path} line {line}: column '{name}' is not a finite number")
                    values.append(value)
                rows.append(values)
                line_nums.append(line)
    except UnicodeDecodeError:
        raise InputError(f"{label} file {path} is not a text CSV file")
    except OSError as err:
        raise InputError(f"cannot read {label} file {path}: {err.strerror}")
    except csv.Error as err:
        raise InputError(f"{label} file {path} is not valid CSV: {err}")

    if head is None:
        raise InputError(f"{label} file {path} is empty (no header line)")
    arr = np.array(rows, dtype=float).reshape(len(rows), len(header))
    return arr, line_nums


def check_times(arr, line_nums, path, label):
    """Timestamps must strictly increase for interpolation to make sense."""
    for i in range(1, len(arr)):
        if arr[i, 0] <= arr[i - 1, 0]:
            raise InputError(
                f"{label} file {path} line {line_nums[i]}: t_ns must increase on every row")


def load_beacon(path):
    arr, lines = read_table(path, BEACON_HEADER, "Beacon log")
    if len(arr) < 2:
        raise InputError(f"Beacon log {path} needs at least 2 data rows; found {len(arr)}")
    check_times(arr, lines, path, "Beacon log")
    norm = np.sqrt(arr[:, 4] ** 2 + arr[:, 5] ** 2 + arr[:, 6] ** 2 + arr[:, 7] ** 2)
    bad = np.flatnonzero(norm < 1e-12)
    if len(bad) > 0:
        raise InputError(
            f"Beacon log {path} line {lines[bad[0]]} has a zero-length quaternion")
    return arr


def load_reference(path):
    arr, lines = read_table(path, REF_HEADER, "reference")
    if len(arr) < 2:
        raise InputError(f"reference log {path} needs at least 2 data rows; found {len(arr)}")
    check_times(arr, lines, path, "reference")
    return arr


def wrap_angle(angle):
    """Wrap radians into [-pi, pi), so 179 deg minus -179 deg gives 2 deg, not 358 deg."""
    return np.mod(angle + math.pi, 2.0 * math.pi) - math.pi


def yaw_from_quaternion(qx, qy, qz, qw):
    """Rotation about z (radians) from a quaternion given as x, y, z, w (any scale)."""
    norm = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return np.arctan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


def align(beacon, ref):
    """Return (t_ns, pos_err_m, head_err_rad) for Beacon rows that fall inside the reference span."""
    t_ref = ref[:, 0]
    mask = (beacon[:, 0] >= t_ref[0]) & (beacon[:, 0] <= t_ref[-1])
    if not np.any(mask):
        raise InputError(
            "the Beacon log and the reference do not overlap in time; "
            "check that both were recorded on the same clock")
    t = beacon[mask, 0]
    ref_x = np.interp(t, t_ref, ref[:, 1])
    ref_y = np.interp(t, t_ref, ref[:, 2])
    # Interpolate heading as a unit vector so the 180 degree wrap cannot pull the result the long way round.
    ref_c = np.interp(t, t_ref, np.cos(ref[:, 3]))
    ref_s = np.interp(t, t_ref, np.sin(ref[:, 3]))
    ref_heading = np.arctan2(ref_s, ref_c)
    beacon_heading = yaw_from_quaternion(beacon[mask, 4], beacon[mask, 5],
                                         beacon[mask, 6], beacon[mask, 7])
    pos_err = np.hypot(beacon[mask, 1] - ref_x, beacon[mask, 2] - ref_y)
    head_err = wrap_angle(beacon_heading - ref_heading)
    return t, pos_err, head_err


def summarise(pos_err, head_err):
    return {
        "mean_pos_m": float(np.mean(pos_err)),
        "rms_pos_m": float(np.sqrt(np.mean(pos_err ** 2))),
        "max_pos_m": float(np.max(pos_err)),
        "max_head_deg": float(np.degrees(np.max(np.abs(head_err)))),
    }


def draw(t, pos_err, head_err, out_path, title):
    time_s = (t - t[0]) / 1e9
    fig, (ax_pos, ax_head) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    ax_pos.plot(time_s, pos_err, color="#1d3557")
    ax_pos.set_ylabel("position error (m)")
    ax_pos.set_ylim(bottom=0.0)  # an error can't be negative; also avoids a misleading zoomed axis
    ax_pos.ticklabel_format(axis="y", useOffset=False, style="plain")
    ax_pos.grid(True, alpha=0.3)
    ax_head.plot(time_s, np.degrees(head_err), color="#e76f51")
    ax_head.set_ylabel("heading error (deg)")
    ax_head.set_xlabel("time since first matched sample (s)")
    ax_head.grid(True, alpha=0.3)
    ax_pos.set_title(title)
    fig.tight_layout()
    try:
        fig.savefig(out_path, dpi=120)
    except OSError as err:
        raise InputError(f"cannot write plot {out_path}: {err.strerror}")
    finally:
        plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Compare a Beacon log with a reference odometry log.")
    parser.add_argument("beacon", help="Beacon pose log CSV")
    parser.add_argument("reference", help="reference CSV (t_ns,x,y,heading)")
    parser.add_argument("--out", help="output PNG (default: beacon name with _compare.png)")
    args = parser.parse_args(argv)
    try:
        beacon = load_beacon(args.beacon)
        ref = load_reference(args.reference)
        t, pos_err, head_err = align(beacon, ref)
        out_path = args.out or os.path.splitext(args.beacon)[0] + "_compare.png"
        folder = os.path.dirname(os.path.abspath(out_path))
        if not os.path.isdir(folder):
            raise InputError(f"output folder does not exist: {folder}")
        draw(t, pos_err, head_err, out_path, os.path.basename(args.beacon))
    except InputError as err:
        print(f"Error: {err}", file=sys.stderr)
        return 2
    stats = summarise(pos_err, head_err)
    print(f"Matched samples: {len(t)} over {(t[-1] - t[0]) / 1e9:.3f} s of overlap")
    print(f"Position error (m): mean {stats['mean_pos_m']:.4f}  rms {stats['rms_pos_m']:.4f}  "
          f"max {stats['max_pos_m']:.4f}")
    print(f"Heading error (deg): max {stats['max_head_deg']:.3f}")
    print(f"Saved plot: {os.path.abspath(out_path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
