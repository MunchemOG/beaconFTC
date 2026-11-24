"""Tests for the Mac-side Limelight tools (tools/limelight). No device: a local fake MJPEG
server stands in for the Limelight's stream on port 5800."""
import csv
import json
import math
import os
import socket
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np
import pytest

from conftest import TOOLS_DIR, run_tool

sys.path.insert(0, os.path.join(TOOLS_DIR, "sim"))
sys.path.insert(0, os.path.join(TOOLS_DIR, "limelight"))

from camera import Camera, rot_x, rot_y, rot_z  # noqa: E402
from render import Surface, checkerboard_surface, render_pose  # noqa: E402

BOUNDARY = "--boundarydonotcross"


def make_jpeg(i, width=640, height=480):
    """A colour test frame whose brightness depends on the frame index."""
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:, :, 0] = (i * 7) % 256
    img[:, :, 1] = 100
    img[:, :, 2] = 50
    cv2.putText(img, str(i), (40, 200), cv2.FONT_HERSHEY_SIMPLEX, 4, (255, 255, 255), 8)
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 80])
    assert ok
    return buf.tobytes()


class FakeLimelight:
    """Serves '/' as an MJPEG stream, '/page' as HTML, '/stall' as a stream that never sends."""

    def __init__(self, frames=60, fps=60.0, with_length=True):
        self.frames = frames
        self.fps = fps
        self.with_length = with_length
        outer = self

        class Handler(BaseHTTPRequestHandler):
            def log_message(self, *args):
                pass

            def do_GET(self):
                if self.path == "/page":
                    body = b"<html><body>Limelight web UI</body></html>"
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(body)))
                    self.end_headers()
                    self.wfile.write(body)
                    return
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=" + BOUNDARY)
                self.end_headers()
                if self.path == "/stall":
                    time.sleep(5)
                    return
                try:
                    for i in range(outer.frames):
                        jpg = make_jpeg(i)
                        part = "--" + BOUNDARY + "\r\nContent-Type: image/jpeg\r\n"
                        if outer.with_length:
                            part += "Content-Length: %d\r\n" % len(jpg)
                        self.wfile.write(part.encode() + b"\r\n" + jpg + b"\r\n")
                        self.wfile.flush()
                        time.sleep(1.0 / outer.fps)
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        self.server.daemon_threads = True
        self.port = self.server.server_address[1]
        self.thread = threading.Thread(target=self.server.serve_forever, daemon=True)

    def url(self, path="/"):
        return "http://127.0.0.1:%d%s" % (self.port, path)

    def __enter__(self):
        self.thread.start()
        return self

    def __exit__(self, *exc):
        self.server.shutdown()
        self.server.server_close()


def closed_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def read_frames_csv(path):
    with open(path, newline="") as f:
        rows = list(csv.reader(f))
    return rows[0], [(int(a), int(b)) for a, b in rows[1:]]


def assert_clean_error(result, code=2):
    assert result.returncode == code, result.stdout + result.stderr
    assert "Traceback" not in result.stderr + result.stdout
    assert "Error:" in result.stderr


# --------------------------------------------------------------------------- ll_record


@pytest.mark.parametrize("with_length", [True, False])
def test_record_from_fake_mjpeg(tmp_path, with_length):
    out = tmp_path / "run1"
    with FakeLimelight(frames=200, fps=60, with_length=with_length) as ll:
        r = run_tool("limelight/ll_record.py",
                     ["--url", ll.url(), "--out", str(out), "--max-frames", "30"], tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    header, rows = read_frames_csv(out / "frames.csv")
    assert header == ["idx", "t_ns"]
    assert [i for i, _ in rows] == list(range(30))
    times = [t for _, t in rows]
    assert all(b > a for a, b in zip(times, times[1:])), "t_ns must increase strictly"
    for i in range(30):
        img = cv2.imread(str(out / "frames" / ("%06d.png" % i)), cv2.IMREAD_UNCHANGED)
        assert img is not None and img.dtype == np.uint8 and img.shape == (480, 640)
    # A Mac recording has no odometry and no truth.
    assert not (out / "odom.csv").exists() and not (out / "truth.csv").exists()
    assert "30 frames" in r.stdout
    # Without --camera, uncalibrated datasheet intrinsics sized to the frames (replayable now).
    cam = json.loads((out / "camera.json").read_text())
    assert (cam["width"], cam["height"]) == (640, 480)
    assert abs(cam["fx"] - 320.0 / math.tan(math.radians(54.5 / 2))) < 1e-6
    assert "UNCALIBRATED" in r.stdout


def test_record_copies_camera_and_field_when_given(tmp_path):
    cam = tmp_path / "camera.json"
    Camera.from_hfov().save(cam)
    fld = tmp_path / "field.json"
    fld.write_text('{"schema": 1}\n')
    out = tmp_path / "run2"
    with FakeLimelight(frames=50) as ll:
        r = run_tool("limelight/ll_record.py",
                     ["--url", ll.url(), "--out", str(out), "--max-frames", "5",
                      "--camera", str(cam), "--field", str(fld)], tmp_path)
    assert r.returncode == 0, r.stderr
    assert json.loads((out / "camera.json").read_text())["width"] == 640
    assert (out / "field.json").read_text() == '{"schema": 1}\n'


def test_record_refuses_non_empty_out(tmp_path):
    out = tmp_path / "run3"
    out.mkdir()
    (out / "frames.csv").write_text("idx,t_ns\n")
    r = run_tool("limelight/ll_record.py",
                 ["--url", "http://127.0.0.1:%d/" % closed_port(), "--out", str(out)], tmp_path)
    assert_clean_error(r)
    assert "already" in r.stderr


def test_reports_unreachable(tmp_path):
    r = run_tool("limelight/ll_record.py",
                 ["--url", "http://127.0.0.1:%d/" % closed_port(), "--out", str(tmp_path / "x")],
                 tmp_path)
    assert_clean_error(r)
    assert "Could not connect" in r.stderr
    assert not (tmp_path / "x" / "frames.csv").exists()


def test_reports_unknown_host(tmp_path):
    r = run_tool("limelight/ll_record.py",
                 ["--url", "http://no-such-limelight.invalid:5800/", "--out", str(tmp_path / "x")],
                 tmp_path)
    assert_clean_error(r)
    assert "no-such-limelight.invalid" in r.stderr
    assert "README" in r.stderr


def test_reports_web_page_instead_of_stream(tmp_path):
    with FakeLimelight() as ll:
        r = run_tool("limelight/ll_record.py",
                     ["--url", ll.url("/page"), "--out", str(tmp_path / "x")], tmp_path)
    assert_clean_error(r)
    assert "5800" in r.stderr and "web page" in r.stderr


def test_reports_stalled_stream(tmp_path):
    with FakeLimelight() as ll:
        t0 = time.monotonic()
        r = run_tool("limelight/ll_record.py",
                     ["--url", ll.url("/stall"), "--out", str(tmp_path / "x"), "--timeout", "1"],
                     tmp_path)
        assert time.monotonic() - t0 < 10
    assert_clean_error(r)
    assert "no frame" in r.stderr.lower()


def test_lost_stream_keeps_frames(tmp_path):
    out = tmp_path / "run4"
    with FakeLimelight(frames=12, fps=60) as ll:
        r = run_tool("limelight/ll_record.py",
                     ["--url", ll.url(), "--out", str(out), "--timeout", "1"], tmp_path)
    assert r.returncode == 1, r.stdout + r.stderr
    assert "Traceback" not in r.stderr
    _, rows = read_frames_csv(out / "frames.csv")
    assert len(rows) == 12


# --------------------------------------------------------------------------- mjpeg reader


def test_reader_parses_parts_and_timestamps():
    import mjpeg
    with FakeLimelight(frames=10, fps=100) as ll:
        stream = mjpeg.MjpegStream(ll.url(), timeout_s=2)
        got = []
        with pytest.raises(mjpeg.StreamEnded, match="closed after 10 frames"):
            for t_ns, jpg in stream.frames():
                got.append((t_ns, jpg))
        stream.close()
    assert len(got) == 10
    assert got[3][1] == make_jpeg(3)
    assert all(b[0] > a[0] for a, b in zip(got, got[1:]))


# --------------------------------------------------------------------------- ll_view


def test_view_headless_reports_stats(tmp_path):
    with FakeLimelight(frames=500, fps=50) as ll:
        r = run_tool("limelight/ll_view.py",
                     ["--url", ll.url(), "--no-window", "--duration", "2"], tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    assert "640x480" in r.stdout
    assert "fps" in r.stdout and "jitter" in r.stdout and "KB" in r.stdout


def test_view_reports_unreachable(tmp_path):
    r = run_tool("limelight/ll_view.py",
                 ["--url", "http://127.0.0.1:%d/" % closed_port(), "--no-window"], tmp_path)
    assert_clean_error(r)


# --------------------------------------------------------------------------- snapscript


def test_passthrough_snapscript_returns_unmodified_frame():
    import passthrough_snapscript as snap
    img = np.random.default_rng(1).integers(0, 255, (480, 640, 3), dtype=np.uint8)
    before = img.copy()
    contour, out, llpython = snap.runPipeline(img, [0.0] * 8)
    assert out is img and np.array_equal(out, before)
    assert len(llpython) == 8
    assert np.asarray(contour).size == 0


# --------------------------------------------------------------------------- calibration


def board_views(count, rng):
    """Board poses (R_cam_board, t_cam_board) that keep a 9x6, 25 mm board in view."""
    views = []
    while len(views) < count:
        r = rot_z(rng.uniform(-0.3, 0.3)) @ rot_y(rng.uniform(-0.5, 0.5)) @ rot_x(
            rng.uniform(-0.5, 0.5))
        centre = np.array([rng.uniform(-0.06, 0.06), rng.uniform(-0.04, 0.04),
                           rng.uniform(0.38, 0.6)])
        t = centre - r @ np.array([0.1, 0.0625, 0.0])  # board centre -> board origin
        views.append((r, t))
    return views


def write_board_run(run, camera, views, square_m=0.025):
    shade, poly = checkerboard_surface(9, 6, square_m)
    (run / "frames").mkdir(parents=True)
    rows = []
    for i, (r, t) in enumerate(views):
        # Render in the camera frame: the board plane is placed by (r, t); camera at origin.
        surf = Surface(t, r[:, 0], r[:, 1], poly, shade)
        img = render_pose([surf], camera, np.eye(3), np.zeros(3), supersample=3,
                          background=128.0)
        img = np.clip(np.rint(img), 0, 255).astype(np.uint8)
        cv2.imwrite(str(run / "frames" / ("%06d.png" % i)), img)
        rows.append((i, 1_000_000_000 + i * 33_333_333))
    with open(run / "frames.csv", "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["idx", "t_ns"])
        w.writerows(rows)


def test_calibrate_on_rendered_board(tmp_path):
    truth = Camera(640, 480, 530.0, 531.0, 322.0, 236.0, (-0.12, 0.05, 0.0, 0.0, 0.0))
    run = tmp_path / "board"
    write_board_run(run, truth, board_views(16, np.random.default_rng(7)))
    r = run_tool("limelight/ll_calibrate.py",
                 ["--run", str(run), "--board", "9x6", "--square-mm", "25"], tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    cam = json.loads((run / "camera.json").read_text())
    assert set(["width", "height", "fx", "fy", "cx", "cy", "dist", "mount",
                "rollingShutterReadoutS"]) <= set(cam)
    assert abs(cam["fx"] - truth.fx) / truth.fx < 0.01, cam
    assert abs(cam["fy"] - truth.fy) / truth.fy < 0.01, cam
    assert abs(cam["cx"] - truth.cx) < 3 and abs(cam["cy"] - truth.cy) < 3
    assert abs(cam["dist"][0] - truth.dist[0]) < 0.03
    assert "RMS" in r.stdout


def test_calibrate_keeps_existing_mount(tmp_path):
    truth = Camera.from_hfov()
    run = tmp_path / "board"
    write_board_run(run, truth, board_views(10, np.random.default_rng(3)))
    old = Camera.from_hfov().to_json()
    old["mount"] = {"x": 0.1, "y": 0.0, "z": 0.3, "roll": 0.0, "pitch": 0.1, "yaw": 0.0}
    old["rollingShutterReadoutS"] = 0.02
    (run / "camera.json").write_text(json.dumps(old))
    r = run_tool("limelight/ll_calibrate.py", ["--run", str(run)], tmp_path)
    assert r.returncode == 0, r.stdout + r.stderr
    cam = json.loads((run / "camera.json").read_text())
    assert cam["mount"]["z"] == 0.3 and cam["rollingShutterReadoutS"] == 0.02


def test_calibrate_reports_no_board(tmp_path):
    run = tmp_path / "empty"
    (run / "frames").mkdir(parents=True)
    with open(run / "frames.csv", "w") as f:
        f.write("idx,t_ns\n")
        for i in range(3):
            cv2.imwrite(str(run / "frames" / ("%06d.png" % i)), np.full((480, 640), 128, np.uint8))
            f.write("%d,%d\n" % (i, i + 1))
    r = run_tool("limelight/ll_calibrate.py", ["--run", str(run)], tmp_path)
    assert_clean_error(r)
    assert "checkerboard" in r.stderr.lower()


def test_calibrate_reports_missing_run(tmp_path):
    r = run_tool("limelight/ll_calibrate.py", ["--run", str(tmp_path / "nope")], tmp_path)
    assert_clean_error(r)
