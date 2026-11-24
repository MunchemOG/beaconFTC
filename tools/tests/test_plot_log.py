import os

import pytest

from conftest import run_tool, write_csv

LOG_HEADER = ["t_ns", "x", "y", "z", "qx", "qy", "qz", "qw", "status"]
TAG_HEADER = ["t_ns", "x", "y", "heading"]


def test_square_summary_and_png(tmp_path, square_log):
    out = tmp_path / "square_plot.png"
    result = run_tool("plot_log.py", [square_log, "--out", str(out)], tmp_path)
    assert result.returncode == 0, result.stderr
    assert "Rows: 41" in result.stdout
    assert "Duration: 4.000 s" in result.stdout
    assert "Path length: 4.000 m" in result.stdout
    assert "tracking" in result.stdout and "100.0 %" in result.stdout
    assert out.read_bytes()[:4] == b"\x89PNG"


def test_default_output_is_next_to_log(tmp_path, square_log):
    result = run_tool("plot_log.py", [square_log], tmp_path)
    assert result.returncode == 0, result.stderr
    assert os.path.isfile(os.path.join(tmp_path, "square.png"))


def test_status_shares_are_reported(tmp_path):
    rows = [[i * 100_000_000, i * 0.01, 0.0, 0.0, 0, 0, 0, 1, 1 if i < 10 else 3] for i in range(40)]
    log = write_csv(tmp_path / "status.csv", LOG_HEADER, rows)
    result = run_tool("plot_log.py", [log], tmp_path)
    assert result.returncode == 0, result.stderr
    assert "1 initializing   25.0 %  (10 rows)" in result.stdout
    assert "3 lost           75.0 %  (30 rows)" in result.stdout


def test_tags_overlay(tmp_path, square_log):
    tags = write_csv(tmp_path / "tags.csv", TAG_HEADER,
                     [[0, 0.0, 0.0, 0.0], [2_000_000_000, 1.0, 0.5, 1.57], [4_000_000_000, 0.0, 1.0, 3.14]])
    out = tmp_path / "with_tags.png"
    result = run_tool("plot_log.py", [square_log, "--tags", tags, "--out", str(out)], tmp_path)
    assert result.returncode == 0, result.stderr
    assert "Tag fixes: 3" in result.stdout
    assert out.read_bytes()[:4] == b"\x89PNG"


def test_empty_tag_file_with_header_only_is_allowed(tmp_path, square_log):
    tags = write_csv(tmp_path / "tags.csv", TAG_HEADER, [])
    result = run_tool("plot_log.py", [square_log, "--tags", tags, "--out", str(tmp_path / "a.png")], tmp_path)
    assert result.returncode == 0, result.stderr
    assert "Tag fixes: 0" in result.stdout


def _bad_case(tmp_path, kind):
    """Return (args, expected substring) for one kind of bad input."""
    good_rows = [[i * 100_000_000, i * 0.01, 0.0, 0.0, 0, 0, 0, 1, 2] for i in range(5)]
    if kind == "missing":
        return [str(tmp_path / "nope.csv")], "not found"
    if kind == "empty":
        (tmp_path / "empty.csv").write_text("")
        return [str(tmp_path / "empty.csv")], "is empty"
    if kind == "wrong_header":
        log = write_csv(tmp_path / "h.csv", ["time", "x", "y"], [[1, 2, 3]])
        return [log], "wrong header"
    if kind == "non_numeric":
        rows = [list(map(str, r)) for r in good_rows]
        rows[2][1] = "abc"
        log = write_csv(tmp_path / "n.csv", LOG_HEADER, rows)
        return [log], "line 4"  # header is line 1, row index 2 is line 4
    if kind == "bad_status":
        rows = [list(r) for r in good_rows]
        rows[1][8] = 7
        log = write_csv(tmp_path / "s.csv", LOG_HEADER, rows)
        return [log], "status must be 0, 1, 2 or 3"
    if kind == "one_row":
        log = write_csv(tmp_path / "one.csv", LOG_HEADER, good_rows[:1])
        return [log], "at least 2"
    if kind == "non_increasing_time":
        rows = [list(r) for r in good_rows]
        rows[3][0] = rows[2][0]
        log = write_csv(tmp_path / "t.csv", LOG_HEADER, rows)
        return [log], "t_ns must increase"
    if kind == "bad_tags_header":
        log = write_csv(tmp_path / "ok.csv", LOG_HEADER, good_rows)
        tags = write_csv(tmp_path / "tags.csv", ["t_ns", "x", "y"], [[0, 1, 2]])
        return [log, "--tags", tags], "wrong header"
    if kind == "output_folder_missing":
        log = write_csv(tmp_path / "ok.csv", LOG_HEADER, good_rows)
        return [log, "--out", str(tmp_path / "no_such_dir" / "x.png")], "output folder does not exist"
    raise AssertionError(kind)


@pytest.mark.parametrize("kind", ["missing", "empty", "wrong_header", "non_numeric", "bad_status",
                                  "one_row", "non_increasing_time", "bad_tags_header",
                                  "output_folder_missing"])
def test_bad_input_exits_2_without_traceback(tmp_path, kind):
    args, expected = _bad_case(tmp_path, kind)
    result = run_tool("plot_log.py", args, tmp_path)
    assert result.returncode == 2, result.stdout + result.stderr
    assert "Traceback" not in result.stderr
    assert result.stderr.startswith("Error: ")
    assert expected in result.stderr
