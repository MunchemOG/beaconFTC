import math

import pytest

import compare_logs as cl
from conftest import run_tool, square_points, write_csv, yaw_quaternion

LOG_HEADER = ["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "status"]
REF_HEADER = ["t_ns", "x", "y", "heading"]


def beacon_rows(points, yaw=0.0):
    qx, qy, qz, qw = yaw_quaternion(yaw)
    return [[i * 100_000_000, x, y, 0.0, qx, qy, qz, qw, 2] for i, (x, y) in enumerate(points)]


def ref_rows(points, heading=0.0, dx=0.0, dy=0.0, every=1):
    return [[i * 100_000_000, x + dx, y + dy, heading]
            for i, (x, y) in enumerate(points) if i % every == 0]


def test_square_shifted_by_2cm_gives_2cm_error(tmp_path):
    beacon = write_csv(tmp_path / "beacon.csv", LOG_HEADER, beacon_rows(square_points()))
    ref = write_csv(tmp_path / "ref.csv", REF_HEADER, ref_rows(square_points(), dx=0.02))
    result = run_tool("compare_logs.py", [beacon, ref, "--out", str(tmp_path / "e.png")], tmp_path)
    assert result.returncode == 0, result.stderr
    assert "Position error (m): mean 0.0200  rms 0.0200  max 0.0200" in result.stdout
    assert "Heading error (deg): max 0.000" in result.stdout
    assert (tmp_path / "e.png").read_bytes()[:4] == b"\x89PNG"


def test_stats_are_exact_with_the_library_functions(tmp_path):
    beacon = cl.load_beacon(write_csv(tmp_path / "b.csv", LOG_HEADER, beacon_rows(square_points())))
    ref = cl.load_reference(write_csv(tmp_path / "r.csv", REF_HEADER,
                                      ref_rows(square_points(), dx=0.02, dy=-0.02)))
    t, pos_err, head_err = cl.align(beacon, ref)
    stats = cl.summarise(pos_err, head_err)
    expected = math.hypot(0.02, 0.02)
    assert stats["mean_pos_m"] == pytest.approx(expected, abs=1e-12)
    assert stats["rms_pos_m"] == pytest.approx(expected, abs=1e-12)
    assert stats["max_pos_m"] == pytest.approx(expected, abs=1e-12)
    assert stats["max_head_deg"] == pytest.approx(0.0, abs=1e-9)


def test_reference_at_half_rate_is_interpolated_exactly(tmp_path):
    # Straight segments are exact under linear interpolation, so the 2 cm error must survive.
    beacon = write_csv(tmp_path / "b.csv", LOG_HEADER, beacon_rows(square_points()))
    ref = write_csv(tmp_path / "r.csv", REF_HEADER, ref_rows(square_points(), dx=0.02, every=2))
    result = run_tool("compare_logs.py", [beacon, ref, "--out", str(tmp_path / "e.png")], tmp_path)
    assert result.returncode == 0, result.stderr
    assert "Position error (m): mean 0.0200  rms 0.0200  max 0.0200" in result.stdout


def test_heading_wrap_179_vs_minus_179_is_2_degrees(tmp_path):
    points = square_points()
    beacon = cl.load_beacon(write_csv(tmp_path / "b.csv", LOG_HEADER,
                                      beacon_rows(points, yaw=math.radians(179.0))))
    ref = cl.load_reference(write_csv(tmp_path / "r.csv", REF_HEADER,
                                      ref_rows(points, heading=math.radians(-179.0))))
    _, pos_err, head_err = cl.align(beacon, ref)
    assert float(max(abs(head_err))) == pytest.approx(math.radians(2.0), abs=1e-9)
    assert cl.summarise(pos_err, head_err)["max_head_deg"] == pytest.approx(2.0, abs=1e-6)


def test_wrap_angle_unit_cases():
    assert float(cl.wrap_angle(math.radians(358.0))) == pytest.approx(math.radians(-2.0), abs=1e-12)
    assert float(cl.wrap_angle(math.radians(-358.0))) == pytest.approx(math.radians(2.0), abs=1e-12)
    assert float(cl.wrap_angle(math.radians(10.0))) == pytest.approx(math.radians(10.0), abs=1e-12)


def test_yaw_from_quaternion_matches_construction():
    for deg in (-170.0, -30.0, 0.0, 45.0, 90.0, 179.0):
        qx, qy, qz, qw = yaw_quaternion(math.radians(deg))
        assert float(cl.yaw_from_quaternion(qx, qy, qz, qw)) == pytest.approx(math.radians(deg), abs=1e-9)


def test_only_overlap_is_used(tmp_path):
    points = square_points()
    beacon = cl.load_beacon(write_csv(tmp_path / "b.csv", LOG_HEADER, beacon_rows(points)))
    # Reference covers only the first 1 s (rows 0..10 at 0.1 s spacing).
    ref = cl.load_reference(write_csv(tmp_path / "r.csv", REF_HEADER,
                                      ref_rows(points, dx=0.02)[:11]))
    t, pos_err, _ = cl.align(beacon, ref)
    assert len(t) == 11
    assert t.max() <= ref[-1, 0]
    assert float(pos_err.max()) == pytest.approx(0.02, abs=1e-9)


def test_no_overlap_exits_2(tmp_path):
    beacon = write_csv(tmp_path / "b.csv", LOG_HEADER, beacon_rows(square_points()))
    ref = write_csv(tmp_path / "r.csv", REF_HEADER,
                    [[10_000_000_000 + i * 100_000_000, 0.0, 0.0, 0.0] for i in range(3)])
    result = run_tool("compare_logs.py", [beacon, ref], tmp_path)
    assert result.returncode == 2
    assert "do not overlap" in result.stderr
    assert "Traceback" not in result.stderr


def _bad_case(tmp_path, kind):
    beacon_ok = write_csv(tmp_path / "b.csv", LOG_HEADER, beacon_rows(square_points()))
    ref_ok = write_csv(tmp_path / "r.csv", REF_HEADER, ref_rows(square_points()))
    if kind == "missing_ref":
        return [beacon_ok, str(tmp_path / "nope.csv")], "not found"
    if kind == "empty_ref":
        (tmp_path / "e.csv").write_text("")
        return [beacon_ok, str(tmp_path / "e.csv")], "is empty"
    if kind == "wrong_ref_header":
        bad = write_csv(tmp_path / "bh.csv", ["t", "x", "y", "yaw"], [[1, 2, 3, 4]])
        return [beacon_ok, bad], "wrong header"
    if kind == "wrong_beacon_header":
        bad = write_csv(tmp_path / "bb.csv", ["t_ns", "x", "y", "heading"], [[1, 2, 3, 4]])
        return [bad, ref_ok], "wrong header"
    if kind == "non_numeric_ref":
        rows = ref_rows(square_points())
        rows[3][2] = "oops"
        bad = write_csv(tmp_path / "nn.csv", REF_HEADER, rows)
        return [beacon_ok, bad], "line 5"
    if kind == "one_row_ref":
        one = write_csv(tmp_path / "one.csv", REF_HEADER, ref_rows(square_points())[:1])
        return [beacon_ok, one], "at least 2"
    if kind == "zero_quaternion":
        rows = beacon_rows(square_points())
        rows[2][4:8] = [0.0, 0.0, 0.0, 0.0]
        bad = write_csv(tmp_path / "zq.csv", LOG_HEADER, rows)
        return [bad, ref_ok], "zero-length quaternion"
    raise AssertionError(kind)


@pytest.mark.parametrize("kind", ["missing_ref", "empty_ref", "wrong_ref_header", "wrong_beacon_header",
                                  "non_numeric_ref", "one_row_ref", "zero_quaternion"])
def test_bad_input_exits_2_without_traceback(tmp_path, kind):
    args, expected = _bad_case(tmp_path, kind)
    result = run_tool("compare_logs.py", args, tmp_path)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "Traceback" not in result.stderr
    assert result.stderr.startswith("Error: ")
    assert expected in result.stderr
