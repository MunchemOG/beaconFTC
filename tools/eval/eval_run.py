#!/usr/bin/env python3
"""Score a Beacon pose estimate against a run's truth (synthetic) or tape-mark checks (field).

Usage:
    tools/.venv/bin/python tools/eval/eval_run.py --run RUN --est fused.csv
    tools/.venv/bin/python tools/eval/eval_run.py --run RUN --est fused.csv --gate stage1
    tools/.venv/bin/python tools/eval/eval_run.py --run RUN --est fused.csv --plot errors.png

RUN is a dataset directory (plan I2). The reference is RUN/truth.csv when present (synthetic
runs), otherwise RUN/marks.csv (tape-mark checks on the practice field). --est is a CSV with
header t_ns,x,y,heading in the field frame on the same clock (for example fused.csv from the
replay fusion). Time t = 0 is the t_ns in RUN/start.csv, or the first reference sample.

- truth.csv (dense): the truth is interpolated at each estimate time inside the overlap.
- marks.csv (sparse): the estimate is interpolated at each mark time; every mark must lie
  inside the estimate's time span.
Headings are unwrapped before interpolating; heading error is the wrapped difference.

Prints: samples compared, max and RMS position error (and when the max happens), the position
error at t = 150 s, and the max heading error. Gates:
    stage1: position error < 0.05 m at every compared time up to t = 150 s (with marks.csv: at
            every mark), and the comparison reaches t = 150 s (a full match);
    stage2: position error <= 0.02 m and heading error <= 1 deg at every compared time.
Exit status: 0 (gate passed or no gate); 1 gate failed; 2 bad input.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

HEADER = ["t_ns", "x", "y", "heading"]
STAGE1_M, STAGE1_T = 0.05, 150.0
STAGE2_M, STAGE2_DEG = 0.02, 1.0


class InputError(Exception):
    pass


def read_poses(path, label):
    if not os.path.isfile(path):
        raise InputError("%s file not found: %s" % (label, path))
    rows = []
    with open(path, newline="", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        head = None
        for cells in reader:
            if not any(c.strip() for c in cells):
                continue
            if head is None:
                head = [c.strip() for c in cells]
                if head != HEADER:
                    raise InputError("%s %s has the wrong header. Expected %s; found %s"
                                     % (label, path, ",".join(HEADER), ",".join(head)))
                continue
            if len(cells) != 4:
                raise InputError("%s %s line %d has %d columns; expected 4"
                                 % (label, path, reader.line_num, len(cells)))
            try:
                t = int(cells[0])
                vals = [float(c) for c in cells[1:]]
            except ValueError:
                raise InputError("%s %s line %d is not numbers: %s"
                                 % (label, path, reader.line_num, ",".join(cells))) from None
            if not all(math.isfinite(v) for v in vals):
                raise InputError("%s %s line %d: values must be finite (found %s)"
                                 % (label, path, reader.line_num, ",".join(cells)))
            rows.append([t] + vals)
    if head is None or not rows:
        raise InputError("%s %s has no data rows" % (label, path))
    a = np.array(rows, dtype=np.float64)
    t_ns = np.array([r[0] for r in rows], dtype=np.int64)
    if np.any(np.diff(t_ns) <= 0):
        raise InputError("%s %s: t_ns must increase strictly" % (label, path))
    return t_ns, a[:, 1], a[:, 2], a[:, 3]


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def evaluate(run, est_path):
    truth = os.path.join(run, "truth.csv")
    marks = os.path.join(run, "marks.csv")
    if not os.path.isdir(run):
        raise InputError("run directory not found: %s" % run)
    if os.path.isfile(truth):
        ref_name, ref_path, dense = "truth.csv", truth, True
    elif os.path.isfile(marks):
        ref_name, ref_path, dense = "marks.csv", marks, False
    else:
        raise InputError("%s has neither truth.csv nor marks.csv to compare against" % run)
    rt, rx, ry, rh = read_poses(ref_path, "reference")
    et, ex, ey, eh = read_poses(est_path, "estimate")
    start = os.path.join(run, "start.csv")
    t0 = read_poses(start, "start")[0][0] if os.path.isfile(start) else rt[0]

    if dense:
        inside = (et >= rt[0]) & (et <= rt[-1])
        if inside.sum() == 0:
            raise InputError("the estimate's times do not overlap the truth's times")
        t = et[inside]
        tx = np.interp(t, rt, rx)
        ty = np.interp(t, rt, ry)
        th = np.interp(t, rt, np.unwrap(rh))
        px, py, ph = ex[inside], ey[inside], eh[inside]
    else:
        outside = (rt < et[0]) | (rt > et[-1])
        if outside.any():
            raise InputError("%d tape marks lie outside the estimate's time span" % outside.sum())
        t = rt
        tx, ty, th = rx, ry, rh
        px = np.interp(t, et, ex)
        py = np.interp(t, et, ey)
        ph = np.interp(t, et, np.unwrap(eh))
    pos_err = np.hypot(px - tx, py - ty)
    head_err = np.abs(wrap(ph - th))
    ts = (t - t0) / 1e9
    est_end_s = (min(et[-1], rt[-1]) - t0) / 1e9 if dense else (et[-1] - t0) / 1e9
    return ref_name, ts, pos_err, head_err, est_end_s


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--run", required=True, help="dataset directory (truth.csv or marks.csv)")
    p.add_argument("--est", required=True, help="estimate CSV t_ns,x,y,heading (field frame)")
    p.add_argument("--gate", choices=("stage1", "stage2"), help="pass/fail gate to apply")
    p.add_argument("--plot", help="write a PNG of the errors against time")
    args = p.parse_args(argv)
    try:
        ref_name, ts, pos, head, est_end = evaluate(os.path.abspath(args.run), args.est)
    except InputError as e:
        print("Error: " + str(e), file=sys.stderr)
        return 2

    i = int(np.argmax(pos))
    print("Reference: %s" % ref_name)
    print("Compared: %d samples, t = %.2f .. %.2f s" % (len(ts), ts[0], ts[-1]))
    print("Max position error: %.4f m at t = %.2f s" % (pos[i], ts[i]))
    print("RMS position error: %.4f m" % math.sqrt(float(np.mean(pos ** 2))))
    if ts[0] <= STAGE1_T <= ts[-1]:
        print("Error at t = 150 s: %.4f m" % float(np.interp(STAGE1_T, ts, pos)))
    else:
        print("Error at t = 150 s: n/a (compared span does not include t = 150 s)")
    j = int(np.argmax(head))
    print("Max heading error: %.2f deg at t = %.2f s" % (math.degrees(head[j]), ts[j]))

    code = 0
    if args.gate == "stage1":
        # Truth: every sample up to t = 150 s. Tape marks: every mark, including the check
        # made after the run ends (the robot parks after the match).
        m = ts <= STAGE1_T + 1e-9 if ref_name == "truth.csv" else np.ones(len(ts), bool)
        worst = float(pos[m].max()) if m.any() else float("nan")
        if est_end < STAGE1_T - 0.1:
            print("Gate stage1: FAIL (comparison ends at t = %.1f s; a full match needs 150 s)"
                  % est_end)
            code = 1
        elif not worst < STAGE1_M:
            print("Gate stage1: FAIL (max position error %.4f m up to t = 150 s; limit < %.2f m)"
                  % (worst, STAGE1_M))
            code = 1
        else:
            print("Gate stage1: PASS (max %.4f m up to t = 150 s; limit < %.2f m)" % (worst, STAGE1_M))
    elif args.gate == "stage2":
        wp, wh = float(pos.max()), math.degrees(float(head.max()))
        if wp <= STAGE2_M + 1e-12 and wh <= STAGE2_DEG + 1e-9:
            print("Gate stage2: PASS (max %.4f m, %.2f deg; limits %.2f m, %.1f deg)"
                  % (wp, wh, STAGE2_M, STAGE2_DEG))
        else:
            print("Gate stage2: FAIL (max %.4f m, %.2f deg; limits %.2f m, %.1f deg)"
                  % (wp, wh, STAGE2_M, STAGE2_DEG))
            code = 1
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
        ax[0].plot(ts, pos * 100, ".-" if len(ts) < 50 else "-")
        ax[0].axhline(STAGE1_M * 100, color="r", ls="--", lw=0.8)
        ax[0].set_ylabel("position error (cm)")
        ax[1].plot(ts, np.degrees(head), ".-" if len(ts) < 50 else "-")
        ax[1].set_ylabel("heading error (deg)")
        ax[1].set_xlabel("t (s)")
        fig.tight_layout()
        fig.savefig(args.plot, dpi=110)
        print("Wrote %s" % args.plot)
    return code


if __name__ == "__main__":
    sys.exit(main())
