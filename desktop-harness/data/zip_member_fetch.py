#!/usr/bin/env python3
"""Download one member of a remote zip file using HTTP Range requests.

EuRoC is now hosted on the ETH Research Collection as one zip per environment
(machine_hall.zip is 12.7 GB, vicon_room1.zip 6.0 GB). Each contains the per-sequence
ASL zip we actually want (MH_01_easy.zip is about 1.5 GB). The server honours Range
requests, so we read the outer zip's central directory over HTTP, locate the member,
and download only its bytes. The CRC-32 from the central directory is checked.

Standard library only, so it runs before the venv exists.

Usage: zip_member_fetch.py <url> <member path inside zip> <output file>
"""
import os
import sys
import time
import urllib.error
import urllib.request
import zipfile
import zlib

USER_AGENT = "beacon-desktop-harness/1.0"
CHUNK = 32 * 1024 * 1024  # bytes per read when re-hashing a partial file
READ = 4 * 1024 * 1024    # bytes per socket read while streaming the member
RETRIES = 8


def backoff(attempt, err):
    """Seconds to wait before retry. The Research Collection answers 429 when it thinks it is
    being scraped; that needs minutes, not seconds."""
    if isinstance(err, urllib.error.HTTPError) and err.code == 429:
        return min(600, 60 * (attempt + 1))
    return min(120, 5 * (attempt + 1))


def open_range(url, start, end_inclusive=None):
    """Open a Range request and return the response object (caller reads/closes it)."""
    rng = "bytes=%d-" % start if end_inclusive is None else "bytes=%d-%d" % (start, end_inclusive)
    req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT, "Range": rng})
    resp = urllib.request.urlopen(req, timeout=120)
    if resp.status != 206:
        resp.close()
        raise IOError("server ignored Range request (HTTP %d)" % resp.status)
    return resp


def http_range(url, start, end_inclusive):
    """Return bytes [start, end_inclusive] of url, retrying transient failures."""
    last_err = None
    for attempt in range(RETRIES):
        try:
            with open_range(url, start, end_inclusive) as resp:
                data = resp.read()
            want = end_inclusive - start + 1
            if len(data) != want:
                raise IOError("short read: got %d of %d bytes" % (len(data), want))
            return data
        except Exception as e:  # network errors, 429, short reads
            last_err = e
            wait = backoff(attempt, e)
            sys.stderr.write("  range %d-%d failed (%s); retry in %ds\n" % (start, end_inclusive, e, wait))
            time.sleep(wait)
    raise IOError("giving up on range %d-%d: %s" % (start, end_inclusive, last_err))


def remote_size(url):
    last_err = None
    for attempt in range(RETRIES):
        try:
            with open_range(url, 0, 0) as resp:
                cr = resp.headers.get("Content-Range")  # "bytes 0-0/12683729426"
                resp.read()
            if cr is None:
                raise IOError("no Content-Range header; server does not support Range")
            return int(cr.split("/")[-1])
        except Exception as e:
            last_err = e
            wait = backoff(attempt, e)
            sys.stderr.write("  size probe failed (%s); retry in %ds\n" % (e, wait))
            time.sleep(wait)
    raise IOError("cannot probe remote size: %s" % last_err)


class HttpRangeFile:
    """Minimal read-only seekable file over HTTP, enough for zipfile to read the
    central directory. Reads are served from a small block cache."""

    BLOCK = 1024 * 1024

    def __init__(self, url):
        self.url = url
        self.size = remote_size(url)
        self.pos = 0
        self.cache = {}

    def seekable(self):
        return True

    def readable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=0):
        if whence == 0:
            self.pos = offset
        elif whence == 1:
            self.pos += offset
        else:
            self.pos = self.size + offset
        return self.pos

    def _block(self, idx):
        if idx not in self.cache:
            start = idx * self.BLOCK
            end = min(self.size, start + self.BLOCK) - 1
            self.cache[idx] = http_range(self.url, start, end)
        return self.cache[idx]

    def read(self, n=-1):
        if n is None or n < 0:
            n = self.size - self.pos
        n = max(0, min(n, self.size - self.pos))
        out = bytearray()
        while n > 0:
            idx, off = divmod(self.pos, self.BLOCK)
            blk = self._block(idx)
            take = min(n, len(blk) - off)
            out += blk[off:off + take]
            self.pos += take
            n -= take
        return bytes(out)

    def close(self):
        pass


def main():
    if len(sys.argv) != 4:
        sys.stderr.write(__doc__)
        return 2
    url, member, out_path = sys.argv[1], sys.argv[2], sys.argv[3]

    f = HttpRangeFile(url)
    print("remote zip: %d bytes" % f.size)
    zf = zipfile.ZipFile(f)
    info = zf.getinfo(member)
    print("member %s: compressed %d bytes, uncompressed %d bytes, method %d, crc %08x"
          % (member, info.compress_size, info.file_size, info.compress_type, info.CRC))
    if info.compress_type not in (zipfile.ZIP_STORED, zipfile.ZIP_DEFLATED):
        raise SystemExit("unsupported compression method %d" % info.compress_type)

    # Local file header: 30 fixed bytes, then file name and extra field.
    lh = http_range(url, info.header_offset, info.header_offset + 29)
    if lh[:4] != b"PK\x03\x04":
        raise SystemExit("bad local header signature at %d" % info.header_offset)
    name_len = int.from_bytes(lh[26:28], "little")
    extra_len = int.from_bytes(lh[28:30], "little")
    data_start = info.header_offset + 30 + name_len + extra_len
    data_end = data_start + info.compress_size  # exclusive

    tmp = out_path + ".part"
    # Resume support only for stored members (byte offsets map 1:1 to output).
    done = 0
    if info.compress_type == zipfile.ZIP_STORED and os.path.exists(tmp):
        done = os.path.getsize(tmp)
    crc = 0
    if done:
        with open(tmp, "rb") as g:
            while True:
                b = g.read(CHUNK)
                if not b:
                    break
                crc = zlib.crc32(b, crc)
        print("resuming at %d bytes" % done)
    inflater = zlib.decompressobj(-15) if info.compress_type == zipfile.ZIP_DEFLATED else None

    # One streaming request for the whole member (the server rate-limits many small requests);
    # on a dropped connection, reopen at the current offset.
    t0 = time.time()
    pos = data_start + done
    attempt = 0
    with open(tmp, "ab" if done else "wb") as g:
        while pos < data_end:
            try:
                with open_range(url, pos, data_end - 1) as resp:
                    while pos < data_end:
                        raw = resp.read(min(READ, data_end - pos))
                        if not raw:
                            raise IOError("connection closed at offset %d" % pos)
                        data = inflater.decompress(raw) if inflater else raw
                        g.write(data)
                        crc = zlib.crc32(data, crc)
                        pos += len(raw)
                        got = pos - data_start
                        rate = (got - done) / max(1e-3, time.time() - t0) / 1e6
                        sys.stdout.write("\r  %7.1f / %7.1f MB  (%.1f MB/s)   "
                                         % (got / 1e6, info.compress_size / 1e6, rate))
                        sys.stdout.flush()
                        attempt = 0
            except Exception as e:
                # A deflated member resumes too: the inflater keeps its state in memory and we
                # reopen exactly at the next unread compressed byte.
                if attempt >= RETRIES:
                    raise
                wait = backoff(attempt, e)
                attempt += 1
                sys.stderr.write("\n  stream failed at %d (%s); retry in %ds\n" % (pos, e, wait))
                g.flush()
                time.sleep(wait)
        if inflater:
            tail = inflater.flush()
            g.write(tail)
            crc = zlib.crc32(tail, crc)
    print()
    if (crc & 0xFFFFFFFF) != info.CRC:
        raise SystemExit("CRC mismatch: got %08x, expected %08x (kept %s)" % (crc & 0xFFFFFFFF, info.CRC, tmp))
    os.replace(tmp, out_path)
    print("CRC OK, wrote %s" % out_path)
    return 0


if __name__ == "__main__":
    sys.exit(main())
