"""Read the Limelight's MJPEG camera stream over HTTP, with error messages a student can act on.

The Limelight 3A serves its camera stream as MJPEG ("multipart/x-mixed-replace", one JPEG per
part) on port 5800 and its web page on port 5801 (Limelight docs, 2026-11-17).

Timestamps are this computer's monotonic clock (time.monotonic_ns), taken when a frame's JPEG
bytes start to arrive. That is ARRIVAL time, not the camera's capture time: the difference
(exposure, encoding, USB network) is unknown and is one of the things milestone 0 measures.
"""
import http.client
import queue
import socket
import threading
import time
import urllib.parse

DEFAULT_URL = "http://limelight.local:5800/"
README_HINT = "See tools/limelight/README.md, section 'If it does not connect'."


class StreamError(Exception):
    """A problem the user can fix (unplugged, wrong URL, ...). str() is the message to print."""


class StreamEnded(StreamError):
    """The server closed the stream."""


def normalise_url(url):
    """Accept 'limelight.local', 'limelight.local:5800' or a full http:// URL."""
    url = url.strip()
    if "://" not in url:
        url = "http://" + url
    parts = urllib.parse.urlsplit(url)
    if parts.scheme != "http":
        raise StreamError("Only http:// URLs are supported (got %r)." % url)
    if not parts.hostname:
        raise StreamError("The URL %r has no host name." % url)
    port = parts.port or 5800
    path = parts.path or "/"
    if parts.query:
        path += "?" + parts.query
    return parts.hostname, port, path


class MjpegStream:
    def __init__(self, url, timeout_s=3.0):
        self.host, self.port, self.path = normalise_url(url)
        self.where = "%s:%d" % (self.host, self.port)
        self.timeout_s = float(timeout_s)
        self.conn = None
        self.resp = None
        self.boundary = None
        self.single_jpeg = False
        self._connect()

    # ------------------------------------------------------------------ connect

    def _connect(self):
        try:
            # Resolve first so a name problem gets its own message.
            socket.getaddrinfo(self.host, self.port, type=socket.SOCK_STREAM)
        except socket.gaierror:
            raise StreamError(
                "Could not find '%s' on the network. Is the Limelight plugged into this Mac by "
                "USB-C, and has its green light come on (15-20 s after plugging in)? "
                "If it has, try the address http://172.29.0.1:5800 instead. %s"
                % (self.host, README_HINT)) from None
        try:
            self.conn = http.client.HTTPConnection(self.host, self.port, timeout=self.timeout_s)
            self.conn.request("GET", self.path, headers={"Accept": "*/*"})
            self.resp = self.conn.getresponse()
        except ConnectionRefusedError:
            raise StreamError(
                "Could not connect to %s: connection refused. Something answers at that address "
                "but nothing is listening on port %d. The camera stream is on port 5800 and the "
                "web page on port 5801. If the Limelight has just been plugged in, wait 20 s and "
                "try again. %s" % (self.where, self.port, README_HINT)) from None
        except (socket.timeout, TimeoutError):
            raise StreamError(
                "Could not connect to %s: no answer within %.0f s. %s"
                % (self.where, self.timeout_s, README_HINT)) from None
        except (OSError, http.client.HTTPException) as e:
            reason = getattr(e, "strerror", None) or str(e) or type(e).__name__
            raise StreamError("Could not connect to %s: %s. %s"
                              % (self.where, reason, README_HINT)) from None
        if self.resp.status != 200:
            raise StreamError("%s answered HTTP %d %s for %s; expected the camera stream. "
                              "Check the URL (for example http://limelight.local:5800/)."
                              % (self.where, self.resp.status, self.resp.reason, self.path))
        ctype = (self.resp.getheader("Content-Type") or "").lower()
        if ctype.startswith("multipart/"):
            self.boundary = _boundary_from(self.resp.getheader("Content-Type"))
        elif ctype.startswith("image/jpeg"):
            self.single_jpeg = True
        elif ctype.startswith("text/html"):
            raise StreamError(
                "%s%s returned a web page, not a camera stream. The web page is on port 5801; "
                "the camera stream is on port 5800 (for example http://%s:5800/)."
                % (self.where, self.path, self.host))
        else:
            raise StreamError("%s%s returned content of type %r, not an MJPEG camera stream."
                              % (self.where, self.path, ctype or "unknown"))

    def close(self):
        try:
            if self.conn is not None:
                self.conn.close()
        except OSError:
            pass

    # ------------------------------------------------------------------ frames

    def frames(self):
        """Yield (t_ns, jpeg_bytes) until the stream ends; raises StreamError on trouble."""
        count = 0
        try:
            if self.single_jpeg:
                t_ns = time.monotonic_ns()
                yield t_ns, self.resp.read()
                return
            fp = self.resp
            pending_boundary = False
            while True:
                if not pending_boundary:
                    self._skip_to_boundary(fp, count)
                headers = self._read_part_headers(fp, count)
                t_ns = time.monotonic_ns()
                length = headers.get("content-length")
                if length is not None:
                    data = _read_exact(fp, int(length))
                    if data is None:
                        raise StreamEnded("The stream from %s closed after %d frames."
                                          % (self.where, count))
                    pending_boundary = False
                else:
                    data = self._read_until_boundary(fp, count)
                    pending_boundary = True
                if data[:2] != b"\xff\xd8":
                    continue  # not a JPEG part; skip it
                count += 1
                yield t_ns, data
        except (socket.timeout, TimeoutError):
            raise StreamError("No frame from %s for %.1f s (after %d frames). The Limelight may "
                              "have been unplugged or restarted." % (self.where, self.timeout_s,
                                                                      count)) from None
        except (ConnectionError, http.client.HTTPException, OSError) as e:
            raise StreamError("Lost the stream from %s after %d frames: %s"
                              % (self.where, count, e)) from None

    def _is_boundary(self, line):
        # Servers differ on whether the header's boundary already includes the leading "--",
        # so compare without leading dashes.
        s = line.strip()
        return s.startswith(b"--") and s.lstrip(b"-").startswith(self.boundary)

    def _skip_to_boundary(self, fp, count):
        while True:
            line = fp.readline(65536)
            if not line:
                raise StreamEnded("The stream from %s closed after %d frames."
                                  % (self.where, count))
            if self._is_boundary(line):
                return

    def _read_part_headers(self, fp, count):
        headers = {}
        while True:
            line = fp.readline(65536)
            if not line:
                raise StreamEnded("The stream from %s closed after %d frames."
                                  % (self.where, count))
            line = line.strip()
            if not line:
                return headers
            if b":" in line:
                k, v = line.split(b":", 1)
                headers[k.strip().lower().decode("latin-1")] = v.strip().decode("latin-1")

    def _read_until_boundary(self, fp, count):
        chunks = []
        while True:
            line = fp.readline(1 << 20)
            if not line:
                raise StreamEnded("The stream from %s closed after %d frames."
                                  % (self.where, count))
            if self._is_boundary(line):
                break
            chunks.append(line)
        data = b"".join(chunks)
        if data.endswith(b"\r\n"):
            data = data[:-2]
        elif data.endswith(b"\n"):
            data = data[:-1]
        return data


def _boundary_from(content_type):
    for item in content_type.split(";")[1:]:
        k, _, v = item.strip().partition("=")
        if k.strip().lower() == "boundary":
            v = v.strip().strip('"').lstrip("-")
            if v:
                return v.encode("latin-1")
    raise StreamError("The stream's Content-Type has no boundary: %r" % content_type)


def _read_exact(fp, n):
    data = fp.read(n)
    if len(data) < n:
        return None
    return data


class StreamReader:
    """Reads the stream on its own thread so frame timestamps do not wait for disk or screen.

    Frames go into a bounded queue; when the consumer falls behind, the newest frames are
    dropped and counted (dropped), never buffered without limit.
    """

    def __init__(self, stream, max_queue=512, on_frame=None):
        self.stream = stream
        self.on_frame = on_frame  # called on the reader thread for every frame received
        self.queue = queue.Queue(maxsize=max_queue)
        self.dropped = 0
        self.received = 0
        self.error = None
        self.done = threading.Event()
        self._stop = threading.Event()
        self.thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def stop(self):
        self._stop.set()
        self.stream.close()

    def _run(self):
        try:
            for t_ns, jpg in self.stream.frames():
                if self._stop.is_set():
                    break
                self.received += 1
                if self.on_frame is not None:
                    self.on_frame(t_ns, jpg)
                try:
                    self.queue.put_nowait((t_ns, jpg))
                except queue.Full:
                    self.dropped += 1
        except StreamError as e:
            if not self._stop.is_set():
                self.error = e
        except Exception as e:  # never die silently on the reader thread
            if not self._stop.is_set():
                self.error = StreamError("Unexpected problem reading the stream: %r" % (e,))
        finally:
            self.done.set()

    def get(self, timeout_s):
        """Next (t_ns, jpeg) or None when the reader has finished and the queue is empty."""
        while True:
            try:
                return self.queue.get(timeout=min(timeout_s, 0.1))
            except queue.Empty:
                if self.done.is_set() and self.queue.empty():
                    return None
                timeout_s -= 0.1
                if timeout_s <= 0:
                    return False  # nothing yet, still running
