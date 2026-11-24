#!/usr/bin/env python3
"""Record the Limelight camera stream on a Mac into a Beacon dataset directory (plan I2).

Usage:
    tools/.venv/bin/python tools/limelight/ll_record.py --out runs/desk1
    tools/.venv/bin/python tools/limelight/ll_record.py --url http://limelight.local:5800 \
        --out runs/hall1 --duration 60 --camera runs/board1/camera.json \
        --field beacon-field/biobuzz-2026.json

Writes DIR/frames.csv (idx,t_ns) and DIR/frames/<idx:06d>.png (8-bit gray), plus DIR/camera.json
so the run can be replayed by the native bench (beacon_replay --mode explore) straight away:
- with --camera FILE: a copy of FILE (a calibration from ll_calibrate.py);
- without it: UNCALIBRATED datasheet values (H FOV 54.5 deg, centred, no distortion, mount 0),
  sized to the frames received. Replace it with a calibrated one for anything that needs accuracy.
--field FILE copies a field model in as DIR/field.json (field mode needs it; explore does not).
No odom.csv: a Mac has no Pinpoint.

t_ns is the ARRIVAL time of each frame on this Mac's monotonic clock, in nanoseconds; it is not
the camera's capture time (see mjpeg.py).

Stop with Ctrl-C (or --duration / --max-frames). Exit status: 0 recorded; 1 the stream was lost
part-way (the frames already recorded are kept); 2 could not start (Limelight unreachable,
wrong URL, output directory already used).
"""
import argparse
import os
import shutil
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "sim"))

import mjpeg  # noqa: E402


def fail(msg, code=2):
    print("Error: " + msg, file=sys.stderr)
    sys.exit(code)


def parse_args(argv):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--url", default=mjpeg.DEFAULT_URL,
                   help="camera stream URL (default %(default)s)")
    p.add_argument("--out", required=True, help="new directory for the recording")
    p.add_argument("--duration", type=float, default=0.0,
                   help="stop after this many seconds (default: until Ctrl-C)")
    p.add_argument("--max-frames", type=int, default=0, help="stop after this many frames")
    p.add_argument("--timeout", type=float, default=3.0,
                   help="give up when no frame arrives for this many seconds (default 3)")
    p.add_argument("--camera", help="camera.json to copy into the recording (from ll_calibrate); "
                   "default: write uncalibrated Limelight 3A datasheet values")
    p.add_argument("--field", help="field model JSON to copy into the recording as field.json")
    return p.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    import cv2  # after argument parsing so --help works without OpenCV
    import numpy as np

    out = os.path.abspath(args.out)
    if os.path.exists(os.path.join(out, "frames.csv")):
        fail("%s already has a recording (frames.csv). Choose a new --out directory." % out)
    for label, path in (("--camera", args.camera), ("--field", args.field)):
        if path and not os.path.isfile(path):
            fail("%s file not found: %s" % (label, path))

    try:
        stream = mjpeg.MjpegStream(args.url, timeout_s=args.timeout)
    except mjpeg.StreamError as e:
        fail(str(e))

    os.makedirs(os.path.join(out, "frames"), exist_ok=True)
    if args.camera:
        shutil.copyfile(args.camera, os.path.join(out, "camera.json"))
    if args.field:
        shutil.copyfile(args.field, os.path.join(out, "field.json"))

    reader = mjpeg.StreamReader(stream).start()
    csv_file = open(os.path.join(out, "frames.csv"), "w", newline="")
    csv_file.write("idx,t_ns\n")
    print("Recording %s -> %s  (Ctrl-C to stop)" % (args.url, out), flush=True)

    count = 0
    undecodable = 0
    jpeg_bytes = 0
    size = None
    first_t = last_t = None
    t_start = time.monotonic()
    lost = None
    try:
        while True:
            if args.max_frames and count >= args.max_frames:
                break
            if args.duration and time.monotonic() - t_start >= args.duration:
                break
            item = reader.get(timeout_s=0.5)
            if item is False:
                continue
            if item is None:
                lost = reader.error
                break
            t_ns, jpg = item
            img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
            if img is None:
                undecodable += 1
                continue
            if last_t is not None and t_ns <= last_t:
                t_ns = last_t + 1  # keep t_ns strictly increasing (I2)
            cv2.imwrite(os.path.join(out, "frames", "%06d.png" % count), img,
                        [cv2.IMWRITE_PNG_COMPRESSION, 1])
            csv_file.write("%d,%d\n" % (count, t_ns))
            if count % 30 == 0:
                csv_file.flush()
                if count:
                    print("  %d frames" % count, flush=True)
            first_t = t_ns if first_t is None else first_t
            last_t = t_ns
            size = (img.shape[1], img.shape[0])
            jpeg_bytes += len(jpg)
            count += 1
    except KeyboardInterrupt:
        print("Stopped by Ctrl-C.")
    finally:
        reader.stop()
        csv_file.close()

    span = (last_t - first_t) / 1e9 if count > 1 else 0.0
    fps = (count - 1) / span if span > 0 else 0.0
    print("Recorded %d frames in %.2f s (%.1f fps), %s, mean JPEG %.1f KB, dropped %d, "
          "undecodable %d -> %s"
          % (count, span, fps, "%dx%d" % size if size else "no image",
             jpeg_bytes / max(count, 1) / 1024.0, reader.dropped, undecodable, out))
    print("t_ns = arrival time on this Mac's monotonic clock (not the capture time).")
    if not args.camera and size is not None:
        from camera import Camera  # tools/sim/camera.py
        Camera.from_hfov(size[0], size[1]).save(os.path.join(out, "camera.json"))
        print("camera.json: UNCALIBRATED datasheet values (H FOV 54.5 deg). For accurate work, "
              "calibrate with ll_calibrate.py and record with --camera.")
    if lost is not None:
        if count == 0:
            fail(str(lost))
        fail("%s The %d frames recorded before that are kept in %s." % (lost, count, out), 1)
    if count == 0:
        fail("No frames were recorded.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
