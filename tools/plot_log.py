#!/usr/bin/env python3
"""Plot a Beacon pose log as a top-down path and print a short summary.

Usage:
    python3 tools/plot_log.py run1.csv
    python3 tools/plot_log.py run1.csv --tags run1_tags.csv --out run1_plot.png

The pose log has header t_ns,x,y,z,qx,qy,qz,qw,status (metres, nanoseconds).
The optional tag-fix file has header t_ns,x,y,heading (field frame).
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

LOG_HEADER = ["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "status"]
TAG_HEADER = ["t_ns", "x", "y", "heading"]
STATUS_NAMES = {0: "invalid", 1: "initializing", 2: "tracking", 3: "lost"}
STATUS_COLOURS = {0: "#9e9e9e", 1: "#f0a202", 2: "#2a9d8f", 3: "#d62828"}


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
    """Timestamps must strictly increase so that durations and plots make sense."""
    for i in range(1, len(arr)):
        if arr[i, 0] <= arr[i - 1, 0]:
            raise InputError(
                f"{label} file {path} line {line_nums[i]}: t_ns must increase on every row "
                f"(got {arr[i, 0]:.0f} after {arr[i - 1, 0]:.0f})")


def load_log(path):
    arr, lines = read_table(path, LOG_HEADER, "pose log")
    if len(arr) < 2:
        raise InputError(f"pose log {path} needs at least 2 data rows; found {len(arr)}")
    check_times(arr, lines, path, "pose log")
    for i, status in enumerate(arr[:, 8]):
        if status not in (0, 1, 2, 3):
            raise InputError(
                f"pose log {path} line {lines[i]}: status must be 0, 1, 2 or 3; got {status:g}")
    return arr


def load_tags(path):
    arr, lines = read_table(path, TAG_HEADER, "tag-fix")
    check_times(arr, lines, path, "tag-fix")
    return arr  # zero rows is allowed: it just means no fixes


def check_output(out_path):
    folder = os.path.dirname(os.path.abspath(out_path))
    if not os.path.isdir(folder):
        raise InputError(f"output folder does not exist: {folder}")


def summarise(arr):
    """Return the summary lines for the pose log."""
    x, y, t = arr[:, 1], arr[:, 2], arr[:, 0]
    path_len = float(np.hypot(np.diff(x), np.diff(y)).sum())
    duration = (t[-1] - t[0]) / 1e9
    lines = [
        f"Rows: {len(arr)}",
        f"Duration: {duration:.3f} s",
        f"Path length: {path_len:.3f} m",
        "Status shares:",
    ]
    status = arr[:, 8].astype(int)
    for code in sorted(STATUS_NAMES):
        count = int(np.sum(status == code))
        pct = 100.0 * count / len(arr)
        lines.append(f"  {code} {STATUS_NAMES[code]:<13} {pct:5.1f} %  ({count} rows)")
    return lines


def draw(arr, tags, out_path, title):
    """Draw the top-down path coloured by status, with start, end and tag fixes."""
    x, y, status = arr[:, 1], arr[:, 2], arr[:, 8].astype(int)
    fig, ax = plt.subplots(figsize=(7, 7))
    ax.plot(x, y, color="#cccccc", linewidth=1, zorder=1)
    for code in sorted(STATUS_COLOURS):
        mask = status == code
        if mask.any():
            ax.scatter(x[mask], y[mask], s=10, color=STATUS_COLOURS[code], zorder=2,
                       label=f"status {code} ({STATUS_NAMES[code]})")
    ax.scatter([x[0]], [y[0]], marker="o", s=140, facecolors="none", edgecolors="black",
               linewidths=2, zorder=3, label="start")
    ax.scatter([x[-1]], [y[-1]], marker="s", s=80, color="black", zorder=3, label="end")
    if tags is not None and len(tags) > 0:
        ax.scatter(tags[:, 1], tags[:, 2], marker="x", s=60, color="#1d3557", linewidths=2,
                   zorder=4, label=f"tag fixes ({len(tags)})")
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    fig.tight_layout()
    try:
        fig.savefig(out_path, dpi=120)
    except OSError as err:
        raise InputError(f"cannot write plot {out_path}: {err.strerror}")
    finally:
        plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Plot a Beacon pose log (top-down path).")
    parser.add_argument("log", help="pose log CSV (t_ns,x,y,z,qx,qy,qz,qw,status)")
    parser.add_argument("--tags", help="tag-fix CSV (t_ns,x,y,heading) to overlay")
    parser.add_argument("--out", help="output PNG (default: the log name with .png)")
    args = parser.parse_args(argv)
    try:
        arr = load_log(args.log)
        tags = load_tags(args.tags) if args.tags else None
        out_path = args.out or os.path.splitext(args.log)[0] + ".png"
        check_output(out_path)
        draw(arr, tags, out_path, os.path.basename(args.log))
    except InputError as err:
        print(f"Error: {err}", file=sys.stderr)
        return 2
    for line in summarise(arr):
        print(line)
    if tags is not None:
        print(f"Tag fixes: {len(tags)}")
    print(f"Saved plot: {os.path.abspath(out_path)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
