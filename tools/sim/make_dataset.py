#!/usr/bin/env python3
"""Make a synthetic Beacon dataset (plan I2) from a field model.

Usage:
    tools/.venv/bin/python tools/sim/make_dataset.py --field beacon-field/biobuzz-2026.json \
        --traj match --seed 1 --out /tmp/beacon-runs/match1
    tools/.venv/bin/python tools/sim/make_dataset.py --traj spin --duration 20 --fps 15 \
        --cam-height 0.25 --cam-pitch 8 --rs-readout-ms 10 --exposure-ms 8 --blur-samples 4 \
        --out /tmp/beacon-runs/spin1

Writes into --out (which must not already hold a dataset):
    camera.json   intrinsics, distortion, mount (T_robot_cam), rollingShutterReadoutS
    field.json    copy of --field
    frames.csv    idx,t_ns (frame time = centre row, middle of the exposure) + frames/<idx:06d>.png
    odom.csv      t_ns,x,y,heading  Pinpoint-like odometry, its own world frame, 200 Hz
    start.csv     t_ns,x,y,heading  start pose "typed by the team" (truth + the profile's or --start-error-* error)
    truth.csv     t_ns,x,y,heading  true robot pose in the field frame, same times as odom.csv
    hive.csv      t_ns,hive_red,hive_blue  true hive tilts (rad), same times (simulator only)
    meta.json     every parameter, odometry scale errors, slips, collisions, timings
Headings are wrapped to (-pi, pi]. t_ns starts at 10 s (an arbitrary elapsedRealtimeNanos).

Rendering runs on --workers processes. A 150 s match at 30 fps (4500 frames, 640x480) takes a few
minutes on 8 cores and about 1 GB of PNGs; a 20 min session at 30 fps is 36000 frames (~8 GB),
so consider --fps 10 for sessions.

Exit status: 0 OK; 2 bad arguments, invalid field model or output directory already used.
"""
import argparse
import json
import math
import multiprocessing as mp
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import fieldmodel  # noqa: E402
import odomsim  # noqa: E402
import trajectories  # noqa: E402
from camera import Camera, Mount, DEFAULT_HFOV_DEG  # noqa: E402

T0_NS = 10_000_000_000
GENERATOR_VERSION = 1
REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
DEFAULT_FIELD = os.path.join(REPO, "beacon-field", "biobuzz-2026.json")


def fail(msg):
    print("Error: " + msg, file=sys.stderr)
    sys.exit(2)


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--field", default=DEFAULT_FIELD, help="field model JSON (default BIOBUZZ)")
    p.add_argument("--traj", required=True, choices=trajectories.KINDS)
    p.add_argument("--duration", type=float, default=None,
                   help="seconds (default: match 150, session 1200, spin 60, blind 45, clutter 150)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--out", required=True)
    cam = p.add_argument_group("camera (defaults: Limelight 3A datasheet, unconfirmed)")
    cam.add_argument("--width", type=int, default=640)
    cam.add_argument("--height", type=int, default=480)
    cam.add_argument("--hfov", type=float, default=DEFAULT_HFOV_DEG, help="degrees")
    cam.add_argument("--k1", type=float, default=0.0, help="radial distortion k1")
    cam.add_argument("--k2", type=float, default=0.0, help="radial distortion k2")
    cam.add_argument("--fps", type=float, default=30.0)
    cam.add_argument("--cam-height", type=float, default=0.30, help="lens height above tiles, m")
    cam.add_argument("--cam-pitch", type=float, default=5.0, help="degrees, + looks down")
    cam.add_argument("--cam-x", type=float, default=0.15, help="forward of the robot centre, m")
    cam.add_argument("--cam-y", type=float, default=0.0, help="left of the robot centre, m")
    cam.add_argument("--cam-yaw", type=float, default=0.0, help="degrees, + looks left")
    cam.add_argument("--cam-roll", type=float, default=0.0, help="degrees")
    cam.add_argument("--rs-readout-ms", type=float, default=0.0,
                     help="rolling-shutter readout time (0 = global shutter)")
    cam.add_argument("--exposure-ms", type=float, default=0.0, help="exposure for motion blur")
    cam.add_argument("--blur-samples", type=int, default=1,
                     help="sub-frames averaged over the exposure (1 = no blur)")
    cam.add_argument("--noise", type=float, default=2.0, help="sensor noise sigma, grey levels")
    cam.add_argument("--jpeg-quality", type=int, default=0,
                     help="pass frames through JPEG at this quality (0 = off), like MJPEG")
    cam.add_argument("--supersample", type=int, default=1, help="k x k samples per pixel")
    run = p.add_argument_group("run")
    run.add_argument("--odom-hz", type=float, default=200.0)
    run.add_argument("--odom-profile", choices=sorted(odomsim.PROFILES), default="nominal",
                     help="nominal: +/-0.5 %% scale, 0.5 deg at 60 s, 1-3 cm slips, exact start; "
                          "hard: +/-1.5 %% scale, 2 deg at 60 s, 4-8 collisions per 150 s with "
                          "2-5 cm slip and up to 2 deg heading jump, start error 2 cm / 1 deg")
    run.add_argument("--start-error-cm", type=float, default=None,
                     help="position error of the typed start pose (default: from the profile)")
    run.add_argument("--start-error-deg", type=float, default=None,
                     help="heading error of the typed start pose (default: from the profile)")
    run.add_argument("--hive-tips", choices=("auto", "none", "random"), default="auto",
                     help="hive flips during the run (auto: random for match/session/clutter)")
    run.add_argument("--no-venue", action="store_true", help="black background instead of a venue")
    run.add_argument("--no-render", action="store_true",
                     help="write every file except the frame PNGs (for odometry variants of an "
                          "existing run, see make_odom_variant.py)")
    run.add_argument("--workers", type=int, default=max(1, min(8, (os.cpu_count() or 2) - 1)))
    return p.parse_args(argv)


# ----------------------------------------------------------------------------- hive tilts


class HiveTimeline:
    """Hive tilts over time: each flip is a 0.4 s minimum-jerk swing to the other state."""

    def __init__(self, field, duration, rng, random_flips, start_after):
        self.parts = {pv.part: pv for pv in field.pivots}
        self.events = {}
        for part, pv in self.parts.items():
            times = []
            if random_flips:
                t = start_after + rng.exponential(25.0)
                while t < duration - 1:
                    times.append(t)
                    t += 2.0 + rng.exponential(25.0)
            self.events[part] = times

    def tilts(self, t):
        out = {}
        for part, pv in self.parts.items():
            tilt = pv.start_tilt_rad
            for te in self.events[part]:
                if t <= te:
                    break
                other = -tilt
                f = min(1.0, (t - te) / 0.4)
                prof = 10 * f ** 3 - 15 * f ** 4 + 6 * f ** 5
                tilt = tilt + (other - tilt) * prof
                if f < 1.0:
                    break
            out[part] = tilt
        return out


# ----------------------------------------------------------------------------- workers

_W = {}


def _init_worker(state):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import scene
    field = fieldmodel.load_field(state["field_path"])
    _W.update(state)
    _W["field"] = field
    _W["scene"] = scene.Scene(field, seed=state["seed"], venue=state["venue"])


def _render_one(k):
    import cv2
    import render
    st = _W
    traj, cam, hive = st["traj"], st["camera"], st["hive"]
    t = k / st["fps"]
    boxes = [(b, b.pose) for b in traj.static_boxes]
    cx, cy, ch = traj.pose_at(np.array([t]))
    for m in traj.movers:
        mx, my, mh = m.pose_at(np.array([t]))
        # never draw a box around the camera itself
        if math.hypot(float(mx[0] - cx[0]), float(my[0] - cy[0])) > 0.3:
            boxes.append((m.box, (float(mx[0]), float(my[0]), float(mh[0]))))
    surfaces = st["scene"].surfaces(hive.tilts(t), boxes)
    rng = np.random.default_rng([st["seed"], k])
    img = render.render_frame(surfaces, cam, traj.pose_at, t, supersample=st["supersample"],
                              exposure_s=st["exposure_s"], blur_samples=st["blur_samples"],
                              noise_sigma=st["noise"], rng=rng)
    if st["jpeg_quality"] > 0:
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, st["jpeg_quality"]])
        img = cv2.imdecode(buf, cv2.IMREAD_GRAYSCALE)
    cv2.imwrite(os.path.join(st["out"], "frames", "%06d.png" % k), img,
                [cv2.IMWRITE_PNG_COMPRESSION, 3])
    return k


# ----------------------------------------------------------------------------- main


def wrap(a):
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def typed_start_pose(x, y, h, seed, err_cm, err_deg):
    """The start pose the team "typed": truth plus an error of err_cm in a seeded direction and
    err_deg with a seeded sign. The direction and sign depend only on the seed, not on the size,
    so changing the size later (rewrite_start.py) keeps the same direction."""
    srng = np.random.default_rng(seed + 57)
    ang = srng.uniform(-math.pi, math.pi)
    sign = srng.choice([-1.0, 1.0])
    e = err_cm / 100.0
    return (x + e * math.cos(ang), y + e * math.sin(ang), float(wrap(h + sign * math.radians(err_deg))))


def write_pose_csv(path, header, t_ns, cols):
    with open(path, "w") as f:
        f.write(header + "\n")
        for row in zip(t_ns, *cols):
            f.write("%d,%s\n" % (row[0], ",".join("%.6f" % v for v in row[1:])))


def main(argv=None):
    args = parse_args(argv)
    out = os.path.abspath(args.out)
    if os.path.exists(os.path.join(out, "frames.csv")):
        fail("%s already holds a dataset; choose a new --out." % out)
    if args.fps <= 0 or args.width < 16 or args.height < 16 or args.odom_hz < 1:
        fail("fps, image size and odom rate must be positive (image at least 16x16).")
    try:
        field = fieldmodel.load_field(args.field)
    except fieldmodel.FieldError as e:
        fail(str(e))
    problems = field.validate()
    if problems:
        fail("field model %s is not valid: %s" % (args.field, "; ".join(problems[:5])))
    if field.is_placeholder:
        print("Warning: %s is a PLACEHOLDER field model, not official geometry." % args.field)

    t_wall = time.time()
    traj = trajectories.make_trajectory(args.traj, field, args.duration, args.seed)
    duration = traj.duration_s
    rng = np.random.default_rng(args.seed + 31)
    flips = args.hive_tips == "random" or (args.hive_tips == "auto" and
                                           args.traj in ("match", "session", "clutter"))
    hive = HiveTimeline(field, duration, rng, flips, traj.still_until_s + 5.0)
    profile = odomsim.PROFILES[args.odom_profile]
    params = odomsim.OdomParams(**dict(vars(profile), rate_hz=args.odom_hz))
    collisions = list(traj.collisions)
    per150 = odomsim.PROFILE_COLLISIONS_PER_150S[args.odom_profile]
    if per150 and duration > traj.still_until_s + 6:
        crng = np.random.default_rng(args.seed + 97)
        want = int(round(crng.integers(per150[0], per150[1] + 1) * duration / 150.0))
        extra = max(0, want - len(collisions))
        collisions = sorted(collisions + crng.uniform(traj.still_until_s + 5, duration - 1,
                                                      extra).tolist())
    odom = odomsim.simulate(traj.t, traj.x, traj.y, traj.h, seed=args.seed + 1,
                            params=params, collisions=collisions)
    err_cm, err_deg = odomsim.PROFILE_START_ERROR[args.odom_profile]
    if args.start_error_cm is not None:
        err_cm = args.start_error_cm
    if args.start_error_deg is not None:
        err_deg = args.start_error_deg

    mount = Mount(args.cam_x, args.cam_y, args.cam_height, math.radians(args.cam_roll),
                  math.radians(args.cam_pitch), math.radians(args.cam_yaw))
    cam = Camera.from_hfov(args.width, args.height, args.hfov, (args.k1, args.k2, 0, 0, 0),
                           args.rs_readout_ms / 1000.0, mount)
    margin = cam.rs_readout_s / 2 + args.exposure_ms / 2000.0
    n_frames = int(math.floor((duration - margin) * args.fps)) + 1

    os.makedirs(os.path.join(out, "frames"), exist_ok=True)
    cam.save(os.path.join(out, "camera.json"))
    shutil.copyfile(args.field, os.path.join(out, "field.json"))

    t_ns = T0_NS + np.round(odom.t * 1e9).astype(np.int64)
    tx, ty, th = traj.pose_at(odom.t)
    write_pose_csv(os.path.join(out, "truth.csv"), "t_ns,x,y,heading", t_ns, (tx, ty, wrap(th)))
    write_pose_csv(os.path.join(out, "odom.csv"), "t_ns,x,y,heading", t_ns,
                   (odom.x, odom.y, wrap(odom.h)))
    tilts = [hive.tilts(t) for t in odom.t]
    parts = sorted(hive.parts)
    with open(os.path.join(out, "hive.csv"), "w") as f:
        f.write("t_ns," + ",".join(parts) + "\n")
        for tn, tl in zip(t_ns, tilts):
            f.write("%d,%s\n" % (tn, ",".join("%.6f" % tl[p] for p in parts)))
    start = typed_start_pose(traj.x[0], traj.y[0], traj.h[0], args.seed, err_cm, err_deg)
    write_pose_csv(os.path.join(out, "start.csv"), "t_ns,x,y,heading", [T0_NS], [[v] for v in start])

    frame_t = np.arange(n_frames) / args.fps
    with open(os.path.join(out, "frames.csv"), "w") as f:
        f.write("idx,t_ns\n")
        for k, t in enumerate(frame_t):
            f.write("%d,%d\n" % (k, T0_NS + int(round(t * 1e9))))

    state = {"field_path": os.path.abspath(args.field), "seed": args.seed, "venue": not args.no_venue,
             "traj": traj, "camera": cam, "hive": hive, "fps": args.fps, "out": out,
             "supersample": args.supersample, "exposure_s": args.exposure_ms / 1000.0,
             "blur_samples": args.blur_samples, "noise": args.noise,
             "jpeg_quality": args.jpeg_quality}
    print("%s: %d frames (%.1f s at %.1f fps, %dx%d), odometry %d samples, %d workers"
          % (args.traj, n_frames, duration, args.fps, args.width, args.height, len(odom.t),
             args.workers), flush=True)
    t_render = time.time()
    if args.no_render:
        pass
    elif args.workers <= 1:
        _init_worker(state)
        for k in range(n_frames):
            _render_one(k)
            if k and k % 200 == 0:
                print("  %d/%d frames" % (k, n_frames), flush=True)
    else:
        ctx = mp.get_context("spawn")
        with ctx.Pool(args.workers, initializer=_init_worker, initargs=(state,)) as pool:
            done = 0
            for _ in pool.imap_unordered(_render_one, range(n_frames), chunksize=8):
                done += 1
                if done % 500 == 0:
                    print("  %d/%d frames" % (done, n_frames), flush=True)
    render_s = time.time() - t_render

    v, w = traj.speeds()
    meta = {"generatorVersion": GENERATOR_VERSION, "args": vars(args), "durationS": duration,
            "frames": n_frames, "odomSamples": len(odom.t), "t0Ns": T0_NS,
            "odomScale": [odom.scale_x, odom.scale_y],
            "odomProfile": args.odom_profile, "odomParams": vars(params),
            "startErrorCmDeg": [err_cm, err_deg],
            "slips": [{"tS": s[0], "dxRobotM": s[1], "dyRobotM": s[2], "headingJumpRad": s[3]}
                      for s in odom.slips],
            "collisionsS": collisions, "stillUntilS": traj.still_until_s,
            "blindWindowS": traj.blind_window_s, "hiveFlipsS": hive.events,
            "maxSpeedMps": float(v.max()), "maxTurnRadps": float(np.abs(w).max()),
            "startPoseTruth": [float(traj.x[0]), float(traj.y[0]), float(wrap(traj.h[0]))],
            "renderSeconds": round(render_s, 1), "totalSeconds": round(time.time() - t_wall, 1),
            "frameTimeConvention": "t_ns = centre image row, middle of the exposure"}
    with open(os.path.join(out, "meta.json"), "w") as f:
        json.dump(meta, f, indent=1)
        f.write("\n")
    print("Wrote %s (render %.1f s, %.1f ms/frame per worker)"
          % (out, render_s, 1000 * render_s * max(args.workers, 1) / max(n_frames, 1)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
