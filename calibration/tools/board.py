"""Calibration targets: load a target YAML, detect it, and render it.

Two target types are supported:

  aprilgrid  Kalibr-layout AprilGrid built from OpenCV's DICT_APRILTAG_36h11 markers. OpenCV has
             no AprilGrid board class, so this module supplies the geometry itself: four corners
             per tag, tag id = row * cols + col with row 0 at the BOTTOM of the printed grid
             (the Kalibr numbering).
  charuco    OpenCV's CharucoBoard (chessboard corners located between markers).

Board frame (both types): origin at the top-left outer corner of the grid, x to the right,
y DOWN the printed face, z into the paper, metres. The viewer looks at the face from the -z side.
All lengths are in metres. A point is identified by an integer id so that per-frame detections
can be compared.
"""
import os

import cv2
import numpy as np
import yaml

DICTIONARIES = {
    "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11,
    "DICT_5X5_250": cv2.aruco.DICT_5X5_250,
    "DICT_4X4_50": cv2.aruco.DICT_4X4_50,
    "DICT_6X6_250": cv2.aruco.DICT_6X6_250,
}
# Cells across one marker including its black border: 36h11 has a 6x6 payload, 5x5 has 5x5, etc.
MARKER_CELLS = {
    "DICT_APRILTAG_36h11": 8,
    "DICT_5X5_250": 7,
    "DICT_4X4_50": 6,
    "DICT_6X6_250": 8,
}


class TargetError(Exception):
    """A target YAML that cannot be used."""


class Target:
    """A planar calibration target. Build one with load_target()."""

    def __init__(self, kind, spec, path=None):
        self.kind = kind
        self.spec = spec
        self.path = path
        self.dict_name = spec["dictionary"]
        self.dictionary = cv2.aruco.getPredefinedDictionary(DICTIONARIES[self.dict_name])
        self.size_verified = bool(spec.get("printed_size_verified", False))
        params = cv2.aruco.DetectorParameters()
        # Corner refinement is done in refine_* below. OpenCV 4.11's built-in refinement was
        # measured on synthetic renders to leave ~0.5 px error (aruco SUBPIX) or a 0.5 px offset
        # (CORNER_REFINE_APRILTAG and ChArUco corners), so the corners are refined here instead.
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
        self._params = params
        if kind == "aprilgrid":
            self.cols = int(spec["tagCols"])
            self.rows = int(spec["tagRows"])
            self.tag_size = float(spec["tagSize"])
            self.tag_spacing = float(spec["tagSpacing"])
            self.pitch = self.tag_size * (1.0 + self.tag_spacing)
            self.width = self.cols * self.tag_size + (self.cols - 1) * self.tag_size * self.tag_spacing
            self.height = self.rows * self.tag_size + (self.rows - 1) * self.tag_size * self.tag_spacing
            self.min_points = 16  # at least four whole tags
            self.mark_size = self.tag_size
            self._detector = cv2.aruco.ArucoDetector(self.dictionary, params)
            self.num_points = 4 * self.rows * self.cols
        else:
            self.sx = int(spec["squaresX"])
            self.sy = int(spec["squaresY"])
            self.square = float(spec["squareLength"])
            self.marker = float(spec["markerLength"])
            self.width = self.sx * self.square
            self.height = self.sy * self.square
            self.min_points = 8
            self.mark_size = self.square
            self._board = cv2.aruco.CharucoBoard((self.sx, self.sy), self.square, self.marker,
                                                  self.dictionary)
            self._detector = cv2.aruco.CharucoDetector(self._board, cv2.aruco.CharucoParameters(),
                                                       params)
            self.num_points = (self.sx - 1) * (self.sy - 1)

    # ---------------------------------------------------------------- geometry

    def tag_origin(self, tag_id):
        """(x, y) of the top-left corner of AprilGrid tag `tag_id`, board frame."""
        krow, col = divmod(int(tag_id), self.cols)
        print_row = self.rows - 1 - krow  # Kalibr row 0 is the bottom row
        return col * self.pitch, print_row * self.pitch

    def object_points(self, ids):
        """Board-frame 3-D points for point ids (N,) -> (N, 3) float64."""
        ids = np.asarray(ids, dtype=np.int64).ravel()
        out = np.zeros((len(ids), 3), dtype=np.float64)
        if self.kind == "aprilgrid":
            for i, pid in enumerate(ids):
                x0, y0 = self.tag_origin(pid // 4)
                corner = pid % 4  # OpenCV order: TL, TR, BR, BL
                dx = self.tag_size if corner in (1, 2) else 0.0
                dy = self.tag_size if corner in (2, 3) else 0.0
                out[i] = (x0 + dx, y0 + dy, 0.0)
        else:
            allc = self._board.getChessboardCorners().astype(np.float64)
            out[:] = allc[ids]
        return out

    # ---------------------------------------------------------------- detection

    def detect(self, gray):
        """Detect the target in a grayscale uint8 image.

        Returns (ids, img_pts, obj_pts): ids (N,) int, img_pts (N,2) float64 pixels,
        obj_pts (N,3) float64 board metres. Empty arrays when nothing usable was found.
        """
        if self.kind == "aprilgrid":
            corners, mids, _ = self._detector.detectMarkers(gray)
            if mids is None or len(mids) == 0:
                return _empty()
            ids, pts = [], []
            limit = self.rows * self.cols
            for c, m in zip(corners, mids.ravel()):
                if m >= limit:
                    continue  # a stray marker that is not part of this grid
                for k in range(4):
                    ids.append(int(m) * 4 + k)
                    pts.append(c.reshape(4, 2)[k])
            if not ids:
                return _empty()
            ids = np.array(ids, dtype=np.int64)
            pts = self._refine_marker_corners(gray, np.array(pts, dtype=np.float64))
            return ids, pts, self.object_points(ids)
        cc, ci, _, _ = self._detector.detectBoard(gray)
        if ci is None or len(ci) == 0:
            return _empty()
        ids = ci.ravel().astype(np.int64)
        pts = self._refine_charuco(gray, cc.reshape(-1, 2).astype(np.float64))
        return ids, pts, self.object_points(ids)

    def _refine_marker_corners(self, gray, pts):
        """cornerSubPix on each tag's four corners, window sized from that tag's pixel size."""
        out = pts.copy()
        cells = MARKER_CELLS[self.dict_name]
        for i in range(0, len(pts), 4):
            quad = pts[i:i + 4]
            side = np.mean([np.linalg.norm(quad[k] - quad[(k + 1) % 4]) for k in range(4)])
            half = max(1, int(round(0.7 * side / cells)))
            out[i:i + 4] = _subpix(gray, quad, half)
        return out

    def _refine_charuco(self, gray, pts):
        """cornerSubPix on ChArUco corners. Start points may be ~0.5 px off; the window absorbs it."""
        if len(pts) >= 2:
            d = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
            d[d == 0] = np.inf
            square_px = float(np.median(d.min(axis=1)))
        else:
            square_px = 20.0
        half = int(np.clip(round(0.25 * square_px), 2, 10))
        refined = _subpix(gray, pts, half)
        moved = np.linalg.norm(refined - pts, axis=1)
        refined[moved > 1.5] = pts[moved > 1.5]  # a refinement that ran away is not trusted
        return refined

    # ---------------------------------------------------------------- rendering

    def render(self, side_px, margin_cells=None):
        """Render the flat board as a uint8 image.

        side_px sets the resolution: one marker (AprilGrid) or one square (ChArUco) is
        `side_px` pixels, rounded to a size that keeps every edge on a whole pixel.
        Returns (image, px_per_m, origin_px). A board point (x, y) metres sits at
        pixel (origin_px[0] + x * px_per_m, origin_px[1] + y * px_per_m) in continuous
        coordinates where pixel i covers [i, i+1). A white quiet-zone margin surrounds the grid.
        """
        if self.kind == "aprilgrid":
            return self._render_aprilgrid(side_px)
        return self._render_charuco(side_px)

    def _render_aprilgrid(self, side_px):
        side = self.exact_side_px(side_px)
        pitch_px = int(round(side * (1.0 + self.tag_spacing)))
        margin = int(round(0.5 * pitch_px))
        w = (self.cols - 1) * pitch_px + side + 2 * margin
        h = (self.rows - 1) * pitch_px + side + 2 * margin
        img = np.full((h, w), 255, dtype=np.uint8)
        for krow in range(self.rows):
            for col in range(self.cols):
                tag_id = krow * self.cols + col
                marker = cv2.aruco.generateImageMarker(self.dictionary, tag_id, side)
                print_row = self.rows - 1 - krow
                x0 = margin + col * pitch_px
                y0 = margin + print_row * pitch_px
                img[y0:y0 + side, x0:x0 + side] = marker
        return img, side / self.tag_size, (float(margin), float(margin))

    def exact_side_px(self, side_px):
        """Smallest tag size >= side_px whose pixel grid has whole-pixel cells and spacing."""
        cells = MARKER_CELLS[self.dict_name]
        side = int(np.ceil(side_px / cells)) * cells
        for _ in range(400):
            spacing_px = side * self.tag_spacing
            if abs(spacing_px - round(spacing_px)) < 1e-9:
                return side
            side += cells
        return int(np.ceil(side_px / cells)) * cells

    def _render_charuco(self, side_px):
        cells = MARKER_CELLS[self.dict_name]
        ratio = self.marker / self.square
        side = None
        for cand in range(max(int(side_px), cells), int(side_px) + 600):
            marker_px = cand * ratio
            if abs(marker_px / cells - round(marker_px / cells)) < 1e-9:
                side = cand  # marker is a whole number of cells, so no resampling blur
                break
        if side is None:
            side = int(side_px)
        margin = int(round(0.5 * side))
        flat = self._board.generateImage((self.sx * side, self.sy * side), marginSize=0,
                                         borderBits=1)
        img = np.full((flat.shape[0] + 2 * margin, flat.shape[1] + 2 * margin), 255, np.uint8)
        img[margin:margin + flat.shape[0], margin:margin + flat.shape[1]] = flat
        return img, side / self.square, (float(margin), float(margin))


def _subpix(gray, pts, half):
    arr = np.ascontiguousarray(pts.reshape(-1, 1, 2), dtype=np.float32)
    h, w = gray.shape[:2]
    inside = ((arr[:, 0, 0] > half + 1) & (arr[:, 0, 0] < w - half - 2) &
              (arr[:, 0, 1] > half + 1) & (arr[:, 0, 1] < h - half - 2))
    if inside.any():
        sub = np.ascontiguousarray(arr[inside])
        cv2.cornerSubPix(gray, sub, (half, half), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, 1e-3))
        arr[inside] = sub
    return arr.reshape(-1, 2).astype(np.float64)


def _empty():
    return (np.zeros(0, np.int64), np.zeros((0, 2)), np.zeros((0, 3)))


def load_target(path):
    """Read a target YAML (Kalibr-style keys) into a Target. Raises TargetError."""
    if not os.path.isfile(path):
        raise TargetError(f"target file not found: {path}")
    try:
        with open(path, encoding="utf-8") as f:
            spec = yaml.safe_load(f)
    except (OSError, yaml.YAMLError) as err:
        raise TargetError(f"cannot read target {path}: {err}")
    if not isinstance(spec, dict):
        raise TargetError(f"{path} does not hold key: value pairs")
    kind = str(spec.get("target_type", "")).lower()
    if kind == "aprilgrid":
        need = ["tagCols", "tagRows", "tagSize", "tagSpacing"]
        spec.setdefault("dictionary", "DICT_APRILTAG_36h11")
    elif kind == "charuco":
        need = ["squaresX", "squaresY", "squareLength", "markerLength", "dictionary"]
    else:
        raise TargetError(f"{path}: target_type must be 'aprilgrid' or 'charuco', got {kind!r}")
    missing = [k for k in need if k not in spec]
    if missing:
        raise TargetError(f"{path}: missing key(s) {', '.join(missing)}")
    if spec["dictionary"] not in DICTIONARIES:
        raise TargetError(f"{path}: unknown dictionary {spec['dictionary']!r}; "
                          f"known: {', '.join(DICTIONARIES)}")
    for k in need:
        if k == "dictionary":
            continue
        v = spec[k]
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not np.isfinite(v) or v <= 0:
            raise TargetError(f"{path}: {k} must be a positive number, got {v!r}")
    if kind == "aprilgrid" and spec["dictionary"] != "DICT_APRILTAG_36h11":
        raise TargetError(f"{path}: an aprilgrid target must use DICT_APRILTAG_36h11")
    if kind == "charuco" and spec["markerLength"] >= spec["squareLength"]:
        raise TargetError(f"{path}: markerLength must be smaller than squareLength")
    return Target(kind, spec, path)
