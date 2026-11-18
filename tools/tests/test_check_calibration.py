import math

import pytest

from conftest import run_tool

GOOD_CAM = """%YAML:1.0
---
cam0:
  T_imu_cam:
    - [{c}, {ns}, 0.0, 0.012]
    - [{s}, {c}, 0.0, -0.034]
    - [0.0, 0.0, 1.0, 0.005]
    - [0.0, 0.0, 0.0, 1.0]
  cam_overlaps: []
  camera_model: pinhole
  distortion_coeffs: [-0.2811, 0.0734, 0.00021, -0.00013]
  distortion_model: radtan
  intrinsics: [{fx}, {fy}, {cx}, {cy}]
  resolution: [{w}, {h}]
  rostopic: /cam0/image_raw
  timeshift_cam_imu: {shift}
"""

GOOD_IMU = """%YAML:1.0
---
imu0:
  accelerometer_noise_density: 0.0021
  accelerometer_random_walk: 0.00018
  gyroscope_noise_density: 0.00016
  gyroscope_random_walk: 0.000022
  model: calibrated
  rostopic: /imu0
  time_offset: 0.0
  update_rate: 200.0
"""


def cam_text(**overrides):
    c, s = math.cos(math.radians(30)), math.sin(math.radians(30))
    values = dict(c=c, s=s, ns=-s, fx=458.6, fy=457.9, cx=320.2, cy=238.4, w=640, h=480, shift=-0.0021)
    values.update(overrides)
    return GOOD_CAM.format(**values)


def write(tmp_path, name, text):
    path = tmp_path / name
    path.write_text(text)
    return str(path)


def test_good_camera_file_passes(tmp_path):
    result = run_tool("check_calibration.py", [write(tmp_path, "cam.yaml", cam_text())], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "(camera file)" in result.stdout
    assert "FAIL" not in result.stdout
    assert "RESULT: PASS" in result.stdout
    assert result.stdout.count("PASS  ") == 9


def test_good_camera_with_T_cam_imu_passes(tmp_path):
    text = cam_text().replace("T_imu_cam:", "T_cam_imu:")
    result = run_tool("check_calibration.py", [write(tmp_path, "cam.yaml", text)], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "T_cam_imu bottom row" in result.stdout


def test_good_imu_file_passes(tmp_path):
    result = run_tool("check_calibration.py", [write(tmp_path, "imu.yaml", GOOD_IMU)], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "(imu file)" in result.stdout
    assert "RESULT: PASS" in result.stdout
    assert "FAIL" not in result.stdout


# One file per kind of failure: each must exit 1 and name the failing check.
CAMERA_FAILURES = {
    "focal_too_large": (cam_text(fx=3 * 640 + 1), "focal fx"),
    "focal_too_small": (cam_text(fy=0.2 * 640), "focal fy"),
    "principal_point_outside": (cam_text(cx=700.0), "principal point"),
    "rotation_not_orthonormal": (cam_text(c=math.cos(math.radians(30)) * 1.01,
                                          s=math.sin(math.radians(30)) * 1.01,
                                          ns=-math.sin(math.radians(30)) * 1.01), "rotation orthonormal"),
    "reflection_det_minus_1": (cam_text().replace("[0.0, 0.0, 1.0, 0.005]", "[0.0, 0.0, -1.0, 0.005]"),
                               "rotation determinant"),
    "bottom_row_wrong": (cam_text().replace("[0.0, 0.0, 0.0, 1.0]", "[0.0, 0.0, 0.0, 2.0]"),
                         "T_imu_cam bottom row"),
    "timeshift_too_big": (cam_text(shift=0.1), "timeshift_cam_imu"),
}

IMU_FAILURES = {
    "negative_noise": (GOOD_IMU.replace("0.0021", "-0.0021"), "accelerometer_noise_density"),
    "zero_random_walk": (GOOD_IMU.replace("0.000022", "0.0"), "gyroscope_random_walk"),
    "update_rate_too_low": (GOOD_IMU.replace("200.0", "50.0"), "update_rate"),
}


@pytest.mark.parametrize("kind", sorted(CAMERA_FAILURES))
def test_camera_failure_exits_1_and_names_check(tmp_path, kind):
    text, check_name = CAMERA_FAILURES[kind]
    result = run_tool("check_calibration.py", [write(tmp_path, "cam.yaml", text)], tmp_path)
    assert result.returncode == 1, result.stdout + result.stderr
    fail_lines = [line for line in result.stdout.splitlines() if line.startswith("FAIL")]
    assert any(check_name in line for line in fail_lines), result.stdout
    assert "RESULT: FAIL" in result.stdout
    assert "Traceback" not in result.stderr


@pytest.mark.parametrize("kind", sorted(IMU_FAILURES))
def test_imu_failure_exits_1_and_names_check(tmp_path, kind):
    text, check_name = IMU_FAILURES[kind]
    result = run_tool("check_calibration.py", [write(tmp_path, "imu.yaml", text)], tmp_path)
    assert result.returncode == 1, result.stdout + result.stderr
    fail_lines = [line for line in result.stdout.splitlines() if line.startswith("FAIL")]
    assert any(check_name in line for line in fail_lines), result.stdout


def test_failing_and_passing_files_together_exit_1(tmp_path):
    good = write(tmp_path, "imu.yaml", GOOD_IMU)
    bad = write(tmp_path, "cam.yaml", cam_text(shift=0.2))
    result = run_tool("check_calibration.py", [good, bad], tmp_path)
    assert result.returncode == 1
    assert "(imu file)" in result.stdout and "(camera file)" in result.stdout


def test_missing_file_exits_2(tmp_path):
    result = run_tool("check_calibration.py", [str(tmp_path / "nope.yaml")], tmp_path)
    assert result.returncode == 2
    assert "file not found" in result.stderr
    assert "Traceback" not in result.stderr


def test_empty_file_exits_2(tmp_path):
    result = run_tool("check_calibration.py", [write(tmp_path, "e.yaml", "")], tmp_path)
    assert result.returncode == 2
    assert "is empty" in result.stderr


def test_invalid_yaml_exits_2(tmp_path):
    result = run_tool("check_calibration.py", [write(tmp_path, "bad.yaml", "cam0: [1, 2\n")], tmp_path)
    assert result.returncode == 2
    assert "not valid YAML" in result.stderr
    assert "Traceback" not in result.stderr


def test_neither_cam0_nor_imu0_exits_2(tmp_path):
    result = run_tool("check_calibration.py", [write(tmp_path, "x.yaml", "foo: 1\n")], tmp_path)
    assert result.returncode == 2
    assert "neither a camera file" in result.stderr


def test_non_mapping_top_level_exits_2(tmp_path):
    result = run_tool("check_calibration.py", [write(tmp_path, "l.yaml", "- 1\n- 2\n")], tmp_path)
    assert result.returncode == 2
    assert "key: value pairs" in result.stderr


def test_missing_intrinsics_is_a_fail_not_a_crash(tmp_path):
    text = "\n".join(line for line in cam_text().splitlines() if not line.strip().startswith("intrinsics"))
    result = run_tool("check_calibration.py", [write(tmp_path, "cam.yaml", text + "\n")], tmp_path)
    assert result.returncode == 1
    assert "FAIL  intrinsics" in result.stdout
    assert "Traceback" not in result.stderr
