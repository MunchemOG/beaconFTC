"""calib_intrinsics.py on synthetic renders with a known camera.

These tests prove the math and the plumbing. They do NOT prove the tool works on a real phone
recording: real lenses, rolling shutter, lighting and board flatness are not simulated.

Stated tolerances (what the tool must recover from synthetic data):
  focal length fx, fy : within 1.0 %
  principal point     : within 2.0 px
  distortion k1       : within 0.02 (radtan) / 0.02 (equidistant)
  reprojection RMS    : below the spec's 0.3 px
"""
import os

import numpy as np
import pytest

import board
import calib_intrinsics as ci
import synth

TARGETS = os.path.join(os.path.dirname(__file__), "..", "targets")
APRIL = os.path.join(TARGETS, "aprilgrid_6x6.yaml")
CHARUCO = os.path.join(TARGETS, "charuco_7x5.yaml")

PINHOLE = dict(fx=520.0, fy=521.0, cx=321.0, cy=238.0, dist=[-0.12, 0.05, 0.001, -0.0005])
FISHEYE = dict(fx=300.0, fy=301.0, cx=319.0, cy=241.0, dist=[0.05, -0.01, 0.002, -0.0005])


def make_camera(spec, model):
    return synth.SyntheticCamera(640, 480, spec["fx"], spec["fy"], spec["cx"], spec["cy"],
                                 spec["dist"], model=model)


def render_set(target_path, cam, n, seed, side=40, blur_sigma=0.6, noise=1.5, **pose_kw):
    target = board.load_target(target_path)
    tex, ppm, org = target.render(side)
    rng = np.random.default_rng(seed)
    frames = []
    for _ in range(n):
        R, t = synth.random_pose(rng, target, cam, **pose_kw)
        frames.append(cam.render(target, tex, ppm, org, R, t, rng=rng, blur_sigma=blur_sigma,
                                 noise=noise))
    return frames


def dataset(tmp_path_factory, name, frames, meta=synth.GOOD_META):
    d = str(tmp_path_factory.mktemp(name))
    synth.write_dataset(d, frames, meta=meta)
    return d


@pytest.fixture(scope="module")
def pinhole_cam():
    return make_camera(PINHOLE, "radtan")


@pytest.fixture(scope="module")
def good_april(tmp_path_factory, pinhole_cam):
    frames = render_set(APRIL, pinhole_cam, 90, seed=11, tilt_deg=45)
    return dataset(tmp_path_factory, "good_april", frames)


def check_pinhole(result):
    fx, fy, cx, cy = result["intrinsics"]
    assert abs(fx - PINHOLE["fx"]) / PINHOLE["fx"] < 0.01
    assert abs(fy - PINHOLE["fy"]) / PINHOLE["fy"] < 0.01
    assert abs(cx - PINHOLE["cx"]) < 2.0
    assert abs(cy - PINHOLE["cy"]) < 2.0
    assert abs(result["distortion_coeffs"][0] - PINHOLE["dist"][0]) < 0.02
    assert result["rms_px"] < ci.MAX_RMS_PX


def test_detection_matches_projection_within_subpixel(pinhole_cam):
    """The detector's corners agree with the true projection (no half-pixel offset)."""
    target = board.load_target(APRIL)
    tex, ppm, org = target.render(40)
    rng = np.random.default_rng(2)
    errs = []
    for _ in range(12):
        R, t = synth.random_pose(rng, target, pinhole_cam, tilt_deg=35)
        img = pinhole_cam.render(target, tex, ppm, org, R, t, rng=rng)
        ids, ip, op = target.detect(img)
        errs.append(ip - pinhole_cam.project(op, R, t))
    e = np.concatenate(errs)
    assert np.abs(e.mean(axis=0)).max() < 0.03          # no systematic offset
    assert np.sqrt((e ** 2).sum(axis=1).mean()) < 0.3   # 2-D RMS


def test_charuco_detection_has_no_half_pixel_offset(pinhole_cam):
    target = board.load_target(CHARUCO)
    tex, ppm, org = target.render(60)
    rng = np.random.default_rng(2)
    errs = []
    for _ in range(12):
        R, t = synth.random_pose(rng, target, pinhole_cam, tilt_deg=35)
        img = pinhole_cam.render(target, tex, ppm, org, R, t, rng=rng)
        ids, ip, op = target.detect(img)
        errs.append(ip - pinhole_cam.project(op, R, t))
    e = np.concatenate(errs)
    assert np.abs(e.mean(axis=0)).max() < 0.1          # OpenCV's own bug is 0.5
    assert np.sqrt((e ** 2).sum(axis=1).mean()) < 0.3


def test_recovers_pinhole_radtan_aprilgrid(good_april):
    result = ci.run(good_april, APRIL, log=lambda *_: None)
    check_pinhole(result)
    assert result["verdict"] in (ci.VERDICT_PASS, ci.VERDICT_WARN)
    assert result["trusted"]
    assert result["coverage"]["cells_covered"] >= 23
    assert result["failures"] == []


def test_recovers_pinhole_radtan_charuco(tmp_path_factory, pinhole_cam):
    frames = render_set(CHARUCO, pinhole_cam, 90, seed=12, side=60, tilt_deg=45)
    d = dataset(tmp_path_factory, "good_charuco", frames)
    result = ci.run(d, CHARUCO, log=lambda *_: None)
    check_pinhole(result)
    assert result["trusted"]


def test_recovers_equidistant_fisheye(tmp_path_factory):
    cam = make_camera(FISHEYE, "equidistant")
    # fisheye warps the tag quads, which raises the corner-detection noise floor: render a
    # crisper picture (blur 0.4 px, noise 1.0) so the 0.3 px limit is a fair test of the fit
    frames = render_set(APRIL, cam, 90, seed=13, tilt_deg=40, dist_range=(0.15, 0.3),
                        blur_sigma=0.4, noise=1.0)
    d = dataset(tmp_path_factory, "fisheye", frames)
    result = ci.run(d, APRIL, model="equidistant", log=lambda *_: None)
    fx, fy, cx, cy = result["intrinsics"]
    assert abs(fx - FISHEYE["fx"]) / FISHEYE["fx"] < 0.01
    assert abs(fy - FISHEYE["fy"]) / FISHEYE["fy"] < 0.01
    assert abs(cx - FISHEYE["cx"]) < 2.0 and abs(cy - FISHEYE["cy"]) < 2.0
    assert abs(result["distortion_coeffs"][0] - FISHEYE["dist"][0]) < 0.02
    assert result["rms_px"] < ci.MAX_RMS_PX
    assert result["trusted"]


def test_wrong_lens_model_is_rejected(tmp_path_factory):
    """A fisheye recording fitted with the pinhole-radtan model must not pass."""
    cam = make_camera(dict(FISHEYE, dist=[0.15, -0.06, 0.02, -0.004]), "equidistant")
    frames = render_set(APRIL, cam, 90, seed=14, tilt_deg=40, dist_range=(0.2, 0.4))
    d = dataset(tmp_path_factory, "fisheye_wrong", frames)
    result = ci.run(d, APRIL, model="radtan", log=lambda *_: None)
    assert result["verdict"] == ci.VERDICT_FAIL
    assert any("reprojection error" in f or "fit is not physical" in f for f in result["failures"])


# ------------------------------------------------------------------ deliberately bad inputs

def test_blurred_frames_are_rejected_and_result_stays_good(tmp_path_factory, pinhole_cam):
    sharp = render_set(APRIL, pinhole_cam, 80, seed=21, tilt_deg=45)
    blurred_gauss = render_set(APRIL, pinhole_cam, 8, seed=22, tilt_deg=30)
    target = board.load_target(APRIL)
    tex, ppm, org = target.render(40)
    rng = np.random.default_rng(23)
    blurred_motion = []
    for _ in range(8):
        R, t = synth.random_pose(rng, target, pinhole_cam, tilt_deg=30)
        blurred_motion.append(pinhole_cam.render(target, tex, ppm, org, R, t, rng=rng,
                                                 motion_len=11))
    import cv2
    blurred_gauss = [cv2.GaussianBlur(f, (0, 0), 2.2) for f in blurred_gauss]
    d = dataset(tmp_path_factory, "blur_mix", sharp + blurred_gauss + blurred_motion)
    result = ci.run(d, APRIL, log=lambda *_: None)
    fr = result["frames"]
    assert fr["rejected_blur"] + (fr["read"] - fr["detected"]) >= 12   # most of the 16 bad ones
    check_pinhole(result)


def test_all_frames_blurred_is_not_trusted(tmp_path_factory, pinhole_cam):
    import cv2
    frames = render_set(APRIL, pinhole_cam, 60, seed=24, tilt_deg=45)
    frames = [cv2.GaussianBlur(f, (0, 0), 2.4) for f in frames]
    d = dataset(tmp_path_factory, "all_blur", frames)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert result["verdict"] == ci.VERDICT_FAIL
    assert not result["trusted"]
    assert any("blur" in f.lower() or "not detected" in f or "usable views" in f
               for f in result["failures"])


def test_board_only_in_image_centre_fails_on_coverage(tmp_path_factory, pinhole_cam):
    frames = render_set(APRIL, pinhole_cam, 80, seed=31, tilt_deg=40, dist_range=(0.9, 1.1),
                        spread=0.15)
    d = dataset(tmp_path_factory, "centre_only", frames)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert result["verdict"] == ci.VERDICT_FAIL
    joined = " ".join(result["failures"])
    assert "image regions" in joined or "image area" in joined or "border" in joined
    assert result["coverage"]["border_fraction"] < ci.FAIL_BORDER_FRACTION


def test_too_few_views_fails(tmp_path_factory, pinhole_cam):
    frames = render_set(APRIL, pinhole_cam, 12, seed=41, tilt_deg=45)
    d = dataset(tmp_path_factory, "few", frames)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert result["verdict"] == ci.VERDICT_FAIL
    assert any("usable views" in f for f in result["failures"])


def test_stationary_board_counts_as_one_view(tmp_path_factory, pinhole_cam):
    one = render_set(APRIL, pinhole_cam, 1, seed=51)[0]
    d = dataset(tmp_path_factory, "static", [one] * 60)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert result["frames"]["accepted"] <= 1
    assert result["verdict"] == ci.VERDICT_FAIL


def test_face_on_board_only_fails_on_tilt(tmp_path_factory, pinhole_cam):
    frames = render_set(APRIL, pinhole_cam, 80, seed=61, tilt_deg=3.0)
    d = dataset(tmp_path_factory, "flat", frames)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert any("tilted" in f for f in result["failures"])


def test_no_board_in_images_fails(tmp_path_factory):
    rng = np.random.default_rng(0)
    frames = [rng.integers(0, 255, (480, 640), dtype=np.uint8) for _ in range(10)]
    d = dataset(tmp_path_factory, "noboard", frames)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert result["verdict"] == ci.VERDICT_FAIL
    assert "intrinsics" not in result


def test_auto_exposure_in_meta_fails(tmp_path_factory, pinhole_cam):
    frames = render_set(APRIL, pinhole_cam, 40, seed=71, tilt_deg=45)
    meta = dict(synth.GOOD_META, ae_mode="on", ois="on")
    d = dataset(tmp_path_factory, "ae_on", frames, meta=meta)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert result["verdict"] == ci.VERDICT_FAIL
    assert any("ae_mode" in f for f in result["failures"])
    assert any("ois" in f for f in result["failures"])


def test_missing_meta_warns(tmp_path_factory, pinhole_cam):
    frames = render_set(APRIL, pinhole_cam, 40, seed=72, tilt_deg=45)
    d = dataset(tmp_path_factory, "nometa", frames, meta=None)
    result = ci.run(d, APRIL, log=lambda *_: None)
    assert any("UNVERIFIED" in w for w in result["warnings"])
    assert result["verdict"] != ci.VERDICT_PASS


# ------------------------------------------------------------------ command line

def test_cli_good_dataset_writes_yaml(good_april, tmp_path, capsys):
    out = tmp_path / "intr.yaml"
    code = ci.main([good_april, "--target", APRIL, "--out", str(out)])
    assert code == 0
    assert out.exists()
    import yaml
    doc = yaml.safe_load(out.read_text())
    assert doc["calibration_report"]["trusted"] is True
    assert doc["cam0"]["distortion_model"] == "radtan"
    assert len(doc["cam0"]["intrinsics"]) == 4


def test_cli_strict_turns_warnings_into_failure(good_april):
    # printed_size_verified is false in the shipped target, which is a warning
    assert ci.main([good_april, "--target", APRIL, "--strict"]) == 1


def test_cli_failed_calibration_writes_nothing_unless_forced(tmp_path_factory, pinhole_cam, tmp_path):
    frames = render_set(APRIL, pinhole_cam, 80, seed=31, tilt_deg=40, dist_range=(0.9, 1.1),
                        spread=0.15)
    d = dataset(tmp_path_factory, "centre_only_cli", frames)
    out = tmp_path / "bad.yaml"
    assert ci.main([d, "--target", APRIL, "--out", str(out)]) == 1
    assert not out.exists()
    assert ci.main([d, "--target", APRIL, "--out", str(out), "--write-even-if-failed"]) == 1
    text = out.read_text()
    assert "UNTRUSTED" in text and "trusted: false" in text


def test_cli_missing_input_is_exit_2(tmp_path):
    assert ci.main([str(tmp_path / "nope"), "--target", APRIL]) == 2
    assert ci.main([str(tmp_path), "--target", str(tmp_path / "nope.yaml")]) == 2
