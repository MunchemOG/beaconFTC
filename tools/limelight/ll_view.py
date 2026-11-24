#!/usr/bin/env python3
"""Show the Limelight camera stream on a Mac with live stream statistics.

Usage:
    tools/.venv/bin/python tools/limelight/ll_view.py
    tools/.venv/bin/python tools/limelight/ll_view.py --url http://172.29.0.1:5800
    tools/.venv/bin/python tools/limelight/ll_view.py --no-window --duration 10

The window shows the image with: frame size, measured frames per second, JPEG size, mean
time between frames and its jitter (standard deviation) and maximum, over the last 2 seconds.
Keys: q or Esc quits, s saves the current frame as a PNG in the current directory.
--no-window prints the same numbers once a second instead (no window opens).

This is a quick stream check. To run Beacon itself on the live stream, see README.md
("Run Beacon live on the Mac").

Exit status: 0 normal; 1 the stream was lost part-way; 2 could not connect.
"""
import argparse
import collections
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import mjpeg  # noqa: E402

WINDOW_S = 2.0


class Stats:
    """Frame rate, JPEG size and inter-frame timing over a sliding window."""

    def __init__(self, window_s=WINDOW_S):
        self.window_ns = int(window_s * 1e9)
        self.items = collections.deque()  # (t_ns, jpeg_bytes)
        self.total = 0
        self.lock = threading.Lock()  # add() runs on the reader thread

    def add(self, t_ns, jpg):
        with self.lock:
            self.items.append((t_ns, len(jpg)))
            self.total += 1
            while self.items and t_ns - self.items[0][0] > self.window_ns:
                self.items.popleft()

    def summary(self):
        with self.lock:
            items = list(self.items)
        if len(items) < 2:
            return None
        ts = [t for t, _ in items]
        dts = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
        mean = sum(dts) / len(dts)
        var = sum((d - mean) ** 2 for d in dts) / len(dts)
        fps = (len(ts) - 1) / ((ts[-1] - ts[0]) / 1e9) if ts[-1] > ts[0] else 0.0
        kb = sum(n for _, n in items) / len(items) / 1024.0
        return {"fps": fps, "kb": kb, "interval_ms": mean, "jitter_ms": var ** 0.5,
                "max_ms": max(dts)}


def format_line(size, s):
    if s is None:
        return "%s  waiting for frames..." % size
    return ("%s  %.1f fps  JPEG %.1f KB  interval %.1f ms  jitter %.1f ms (max %.1f ms)"
            % (size, s["fps"], s["kb"], s["interval_ms"], s["jitter_ms"], s["max_ms"]))


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument("--url", default=mjpeg.DEFAULT_URL,
                   help="camera stream URL (default %(default)s)")
    p.add_argument("--no-window", action="store_true", help="print statistics only")
    p.add_argument("--duration", type=float, default=0.0, help="stop after this many seconds")
    p.add_argument("--timeout", type=float, default=3.0,
                   help="give up when no frame arrives for this many seconds (default 3)")
    args = p.parse_args(argv)

    import cv2
    import numpy as np

    try:
        stream = mjpeg.MjpegStream(args.url, timeout_s=args.timeout)
    except mjpeg.StreamError as e:
        print("Error: " + str(e), file=sys.stderr)
        return 2

    stats = Stats()
    # Statistics are taken on the reader thread, so a slow window does not distort them.
    reader = mjpeg.StreamReader(stream, max_queue=4, on_frame=stats.add).start()
    size = "?x?"
    t_start = time.monotonic()
    last_print = t_start
    window = "Limelight stream (q to quit, s to save)"
    latest = None
    saved = 0
    code = 0
    try:
        while True:
            now = time.monotonic()
            if args.duration and now - t_start >= args.duration:
                break
            item = reader.get(timeout_s=0.05)
            if item is None:
                err = reader.error or mjpeg.StreamError("The stream ended.")
                print("Error: " + str(err), file=sys.stderr)
                code = 1 if stats.total else 2
                break
            if item is not False:
                t_ns, jpg = item
                if not args.no_window or size == "?x?":
                    img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if img is not None:
                        size = "%dx%d" % (img.shape[1], img.shape[0])
                        latest = img
            line = format_line(size, stats.summary())
            if args.no_window:
                if now - last_print >= 1.0:
                    print(line, flush=True)
                    last_print = now
                continue
            if latest is not None:
                shown = latest.copy()
                cv2.rectangle(shown, (0, 0), (shown.shape[1], 22), (0, 0, 0), -1)
                cv2.putText(shown, line, (4, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.42,
                            (255, 255, 255), 1, cv2.LINE_AA)
                cv2.imshow(window, shown)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s") and latest is not None:
                name = "limelight_%d.png" % saved
                cv2.imwrite(name, latest)
                saved += 1
                print("Saved " + os.path.abspath(name))
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()
        if not args.no_window:
            cv2.destroyAllWindows()
    print("Final: " + format_line(size, stats.summary()) + "  (%d frames, dropped for display %d)"
          % (stats.total, reader.dropped))
    return code


if __name__ == "__main__":
    sys.exit(main())
