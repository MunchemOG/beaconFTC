"""Tests for tools/eval/eval_run.py (accuracy gates against truth.csv or marks.csv)."""
import math

import numpy as np

from conftest import run_tool, write_csv

T0 = 10_000_000_000
HDR = ["t_ns", "x", "y", "heading"]


def truth_rows(seconds=160.0, hz=200.0):
    """A smooth path that turns through +/- pi so heading wraps."""
    t = np.arange(0.0, seconds, 1.0 / hz)
    x = 1.2 * np.cos(0.05 * t)
    y = 1.2 * np.sin(0.05 * t)
    h = (0.05 * t + math.pi / 2 + math.pi) % (2 * math.pi) - math.pi
    return t, x, y, h


def make_run(tmp_path, name="truth.csv", seconds=160.0):
    run = tmp_path / "run"
    run.mkdir()
    t, x, y, h = truth_rows(seconds)
    write_csv(run / name, HDR, [[T0 + int(round(a * 1e9)), b, c, d] for a, b, c, d in zip(t, x, y, h)])
    write_csv(run / "start.csv", HDR, [[T0, x[0], y[0], h[0]]])
    return run


def est_file(tmp_path, f, hz=50.0, seconds=160.0, name="fused.csv"):
    t = np.arange(0.0, seconds, 1.0 / hz)
    rows = []
    for a in t:
        x, y, h = f(a)
        rows.append([T0 + int(round(a * 1e9)), x, y, (h + math.pi) % (2 * math.pi) - math.pi])
    return write_csv(tmp_path / name, HDR, rows)


def exact(a):
    return 1.2 * math.cos(0.05 * a), 1.2 * math.sin(0.05 * a), 0.05 * a + math.pi / 2


def value(out, label):
    for line in out.splitlines():
        if line.lower().startswith(label):
            return line
    raise AssertionError("no line starting %r in:\n%s" % (label, out))


def test_perfect_estimate_passes(tmp_path):
    run = make_run(tmp_path)
    est = est_file(tmp_path, exact)
    for gate in ("stage1", "stage2"):
        r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", gate], tmp_path)
        assert r.returncode == 0, r.stdout + r.stderr
        assert "PASS" in r.stdout
    assert "0.0000 m" in value(r.stdout, "max position error")


def test_5cm_offset_fails_stage1(tmp_path):
    run = make_run(tmp_path)
    est = est_file(tmp_path, lambda a: (exact(a)[0] + 0.05, exact(a)[1], exact(a)[2]))
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage1"], tmp_path)
    assert r.returncode == 1, r.stdout + r.stderr
    assert "FAIL" in r.stdout
    assert "0.0500 m" in value(r.stdout, "max position error")
    # just under 5 cm passes Stage 1 but not Stage 2
    est = est_file(tmp_path, lambda a: (exact(a)[0] + 0.049, exact(a)[1], exact(a)[2]), name="e2.csv")
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage1"], tmp_path)
    assert r.returncode == 0, r.stdout
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage2"], tmp_path)
    assert r.returncode == 1


def test_stage1_only_counts_the_first_150_s(tmp_path):
    run = make_run(tmp_path, seconds=200.0)
    est = est_file(tmp_path, lambda a: (exact(a)[0] + (0.2 if a > 151 else 0.0),) + exact(a)[1:],
                   seconds=200.0)
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage1"], tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage2"], tmp_path)
    assert r.returncode == 1


def test_stage1_needs_a_full_match(tmp_path):
    run = make_run(tmp_path, seconds=100.0)
    est = est_file(tmp_path, exact, seconds=100.0)
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage1"], tmp_path)
    assert r.returncode == 1 and "150" in r.stdout


def test_heading_gate_stage2(tmp_path):
    run = make_run(tmp_path)
    est = est_file(tmp_path, lambda a: exact(a)[:2] + (exact(a)[2] + math.radians(1.5),))
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage2"], tmp_path)
    assert r.returncode == 1
    assert "1.50 deg" in value(r.stdout, "max heading error")
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage1"], tmp_path)
    assert r.returncode == 0


def test_interpolates_truth_at_est_times(tmp_path):
    """Estimates between truth samples (and across the +/-pi wrap) are compared with the
    interpolated truth, not the nearest sample."""
    run = make_run(tmp_path)
    t = np.arange(0.0013, 159.0, 0.0371)          # never on the 5 ms truth grid
    rows = []
    for a in t:
        x, y, h = exact(a)
        rows.append([T0 + int(round(a * 1e9)), x, y, (h + math.pi) % (2 * math.pi) - math.pi])
    est = write_csv(tmp_path / "odd.csv", HDR, rows)
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est], tmp_path)
    assert r.returncode == 0, r.stderr
    max_err = float(value(r.stdout, "max position error").split()[3])
    # linear interpolation of a 1.2 m circle at 5 ms steps: chord error ~ 1e-9 m;
    # nearest-sample would be up to v * 2.5 ms = 0.15 mm
    assert max_err < 2e-6, max_err
    assert float(value(r.stdout, "max heading error").split()[3]) < 1e-3


def test_error_at_150s_reported(tmp_path):
    run = make_run(tmp_path)
    est = est_file(tmp_path, lambda a: (exact(a)[0] + 0.001 * a,) + exact(a)[1:])
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est], tmp_path)
    line = value(r.stdout, "error at t = 150 s")
    assert abs(float(line.split()[6]) - 0.150) < 1e-3, line


def test_marks_instead_of_truth(tmp_path):
    run = tmp_path / "run"
    run.mkdir()
    marks = []
    for a, dx in ((10.0, 0.0), (60.0, 0.03), (149.0, 0.01)):
        x, y, h = exact(a)
        marks.append([T0 + int(a * 1e9), x - dx, y, (h + math.pi) % (2 * math.pi) - math.pi])
    write_csv(run / "marks.csv", HDR, marks)
    write_csv(run / "start.csv", HDR, [[T0] + list(exact(0.0))])
    est = est_file(tmp_path, exact)
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", est, "--gate", "stage1"], tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "marks.csv" in r.stdout and "3 " in value(r.stdout, "compared")
    assert "0.0300 m" in value(r.stdout, "max position error")


def test_bad_input_reports_cleanly(tmp_path):
    run = make_run(tmp_path)
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", str(tmp_path / "none.csv")],
                 tmp_path)
    assert r.returncode == 2 and "Error:" in r.stderr and "Traceback" not in r.stderr
    bad = write_csv(tmp_path / "bad.csv", ["t", "x"], [[1, 2]])
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", bad], tmp_path)
    assert r.returncode == 2 and "header" in r.stderr
    nan = write_csv(tmp_path / "nan.csv", HDR, [[T0, "nan", 0, 0], [T0 + 1, 0, 0, 0]])
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", nan], tmp_path)
    assert r.returncode == 2 and "finite" in r.stderr
    empty = tmp_path / "empty_run"
    empty.mkdir()
    r = run_tool("eval/eval_run.py", ["--run", str(empty), "--est", bad], tmp_path)
    assert r.returncode == 2 and "truth.csv" in r.stderr and "marks.csv" in r.stderr


def test_odom_only_aligns_to_start(tmp_path):
    """odom_only.py maps Pinpoint-world odometry into the field through start.csv."""
    run = tmp_path / "run"
    run.mkdir()
    t = np.arange(0.0, 5.0, 0.005)
    # truth: start at (1, -1) facing +y, drive forward 1 m/s
    truth = [[T0 + int(round(a * 1e9)), 1.0, -1.0 + a, math.pi / 2] for a in t]
    odom = [[T0 + int(round(a * 1e9)), a, 0.0, 0.0] for a in t]   # Pinpoint world: +x forward
    write_csv(run / "truth.csv", HDR, truth)
    write_csv(run / "odom.csv", HDR, odom)
    write_csv(run / "start.csv", HDR, [truth[0]])
    out = tmp_path / "oo.csv"
    r = run_tool("eval/odom_only.py", ["--run", str(run), "--out", str(out)], tmp_path)
    assert r.returncode == 0, r.stderr
    r = run_tool("eval/eval_run.py", ["--run", str(run), "--est", str(out)], tmp_path)
    assert "0.0000 m" in value(r.stdout, "max position error")
    assert float(value(r.stdout, "max heading error").split()[3]) < 1e-3
    r = run_tool("eval/odom_only.py", ["--run", str(tmp_path), "--out", str(out)], tmp_path)
    assert r.returncode == 2 and "Error:" in r.stderr
