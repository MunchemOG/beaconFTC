"""Tests for the BIOBUZZ field model (beacon-field/biobuzz-2026.json, plan I1) and its loader."""
import copy
import json
import math
import os
import sys

import numpy as np
import pytest

from conftest import TOOLS_DIR

sys.path.insert(0, os.path.join(TOOLS_DIR, "sim"))

import fieldmodel  # noqa: E402

REPO = os.path.abspath(os.path.join(TOOLS_DIR, ".."))
FIELD_PATH = os.path.join(REPO, "beacon-field", "biobuzz-2026.json")
IN = 0.0254


@pytest.fixture(scope="module")
def field():
    return fieldmodel.load_field(FIELD_PATH)


@pytest.fixture
def raw():
    with open(FIELD_PATH) as f:
        return json.load(f)


def write(tmp_path, obj):
    p = tmp_path / "f.json"
    p.write_text(json.dumps(obj))
    return str(p)


def test_field_file_validates(field):
    assert field.validate() == []
    assert not field.is_placeholder
    assert "field-cad-step" in field.source


def test_perimeter_closed(field):
    walls = field.walls
    assert len(walls) == 4
    for a, b in zip(walls, walls[1:] + walls[:1]):
        assert np.allclose(a.p1, b.p0, atol=1e-3), (a.name, b.name)
    half = field.floor.half_size_m
    xs = [abs(c) for w in walls for c in (*w.p0, *w.p1)]
    assert np.allclose(xs, half, atol=1e-3), "walls must form a square of the floor's half-size"
    # Official CAD inner face: 1.7929 m (141.2 in). The plan's nominal 1.8288 m (72 in) is the
    # Competition Manual's "approximately 144 in"; the CAD differs by 3.6 cm (DECISIONS.md).
    assert abs(half - 1.7929) < 0.002
    assert abs(half - 1.8288) < 0.04
    # Counter-clockwise: interior on the left of every wall direction.
    for w in walls:
        d = np.subtract(w.p1, w.p0)
        mid = (np.asarray(w.p0) + np.asarray(w.p1)) / 2
        left = np.array([-d[1], d[0]])
        assert np.dot(left, -mid) > 0, w.name
    assert {w.name for w in walls} == {"audience", "blue", "scoring", "red"}
    red = [w for w in walls if w.name == "red"][0]
    assert red.p0[1] < 0 and red.p1[1] < 0, "FTC frame: red wall is at y = -half"
    for w in walls:
        assert 0.25 < w.height_m < 0.35 and 0 < w.rail_height_m < w.height_m


def test_hive_faces_planar(field):
    assert len(field.hive_faces) > 20
    for face in field.hive_faces:
        v = np.asarray(face.vertices)
        assert v.shape[0] >= 3 and v.shape[1] == 3
        c = v.mean(axis=0)
        _, s, vt = np.linalg.svd(v - c, full_matrices=False)
        dist = np.abs((v - c) @ vt[2])
        assert dist.max() < 0.001, (face.name, dist.max())


def test_validate_rejects_missing_hive(tmp_path, raw):
    del raw["hive"]
    errors = fieldmodel.load_field(write(tmp_path, raw)).validate()
    assert any("hive" in e for e in errors), errors


def test_validate_rejects_empty_hive_faces(tmp_path, raw):
    raw["hive"]["faces"] = []
    errors = fieldmodel.load_field(write(tmp_path, raw)).validate()
    assert any("hive" in e and "face" in e for e in errors), errors


def test_tile_grid_covers_floor(field):
    fl = field.floor
    n = round(2 * fl.half_size_m / fl.tile_pitch_m)
    assert n == 6
    seams = field.tile_seams()
    assert len(seams) == 5 and np.allclose(seams, -np.asarray(seams[::-1]), atol=1e-6)
    # 6 x 6 tiles cover the floor to within a few mm (outer tiles are trimmed to fit)
    assert abs(6 * fl.tile_pitch_m - 2 * fl.half_size_m) < 0.01
    assert fl.tiles == (6, 6)


def test_validate_rejects_nonplanar_and_nonfinite(tmp_path, raw):
    bad = copy.deepcopy(raw)
    bad["hive"]["faces"][0]["vertices"][0][2] += 0.05
    errors = fieldmodel.load_field(write(tmp_path, bad)).validate()
    assert any("planar" in e for e in errors), errors
    bad = copy.deepcopy(raw)
    bad["walls"][0]["heightM"] = float("nan")
    p = tmp_path / "nan.json"
    p.write_text(json.dumps(bad))  # json writes NaN
    errors = fieldmodel.load_field(str(p)).validate()
    assert any("finite" in e for e in errors), errors


def test_validate_rejects_open_perimeter(tmp_path, raw):
    raw["walls"][1]["p0"] = [1.0, 1.7929]
    errors = fieldmodel.load_field(write(tmp_path, raw)).validate()
    assert any("closed" in e for e in errors), errors


def test_load_reports_bad_json(tmp_path):
    p = tmp_path / "bad.json"
    p.write_text("{not json")
    with pytest.raises(fieldmodel.FieldError):
        fieldmodel.load_field(str(p))


# ----------------------------------------------------------------- cross-checks with FIRST docs


def test_hive_matches_competition_manual(field):
    """Manual 9.6 / Field Acceptance Checklist numbers, from the CAD-derived model."""
    hive = [f for f in field.hive_faces if f.part.startswith("hive_")]
    frame = [f for f in field.hive_faces if f.part == "frame"]
    z_all = np.concatenate([np.asarray(f.vertices)[:, 2] for f in hive])
    assert abs(z_all.min() - 30.6 * IN) < 1.0 * IN          # bottom of HIVE above TILES
    up = field.hive_edges_named("red_audience_opening")       # red audience CELL is up at start
    z_open = np.asarray(up).reshape(-1, 3)[:, 2]
    assert abs(z_open.min() - 53.5 * IN) < 0.5 * IN          # checklist 1.13: 53.5 +/- 0.50 in
    assert abs(z_open.max() - 65.6 * IN) < 1.0 * IN          # top of HIVE opening
    feet = np.concatenate([np.asarray(f.vertices) for f in frame if "foot_bar" in f.name])
    assert abs((feet[:, 1].max() - feet[:, 1].min()) - 49.46 * IN) < 1.0 * IN   # frame width
    assert abs((feet[:, 0].max() - feet[:, 0].min()) - 38.95 * IN) < 1.0 * IN   # frame depth
    piv = {p.part: p for p in field.pivots}
    assert abs(piv["hive_red"].point[2] - 43.95 * IN) < 1e-3
    assert abs(abs(piv["hive_red"].point[1] - piv["hive_blue"].point[1]) - 25.5 * IN) < 1.0 * IN
    assert piv["hive_red"].point[1] < 0 < piv["hive_blue"].point[1]


def test_tipping_flips_cells(field):
    """At start red's audience CELL is up; tipped, its scoring CELL is up at the same height."""
    tipped = field.hive_at({"hive_red": -math.radians(30)})
    up = np.asarray(tipped.hive_edges_named("red_scoring_opening")).reshape(-1, 3)
    assert abs(up[:, 2].min() - 53.5 * IN) < 0.5 * IN
    down = np.asarray(tipped.hive_edges_named("red_audience_opening")).reshape(-1, 3)
    assert down[:, 2].max() < 45 * IN
    # the frame does not move, blue does not move
    same = {f.name: f for f in field.hive_faces}
    for f in tipped.hive_faces:
        if f.part != "hive_red":
            assert np.allclose(f.vertices, same[f.name].vertices)
    # faces stay planar after the rotation
    for f in tipped.hive_faces:
        v = np.asarray(f.vertices)
        c = v.mean(axis=0)
        _, _, vt = np.linalg.svd(v - c, full_matrices=False)
        assert np.abs((v - c) @ vt[2]).max() < 0.001


def test_textured_faces_are_opaque_prints(field):
    textured = [f for f in field.hive_faces if f.textured]
    assert textured, "stickers / printed panels must be marked textured"
    assert all(f.material in ("sticker", "printed") for f in textured)
    assert all(not f.textured for f in field.hive_faces if f.material == "clear")


def test_placeholder_flag(tmp_path, raw):
    raw["source"] = "PLACEHOLDER - not from drawings"
    f = fieldmodel.load_field(write(tmp_path, raw))
    assert f.is_placeholder
