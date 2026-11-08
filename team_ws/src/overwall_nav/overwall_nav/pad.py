"""Find the landing mat in the colour image (no ROS; adapted from goalnav's pad_detect.py).

The mat's colour model is a chromaticity histogram (colour independent of brightness) of a
picture cut from a photo of the real mat (``pad_reference.npy``, a copy of goalnav/assets/), with
the orange floor tarp's colour cut out. Each frame:

1. pixels whose chromaticity is common on the mat, OR in the mat's blue-cyan hue band
   (``PadFinder.mask``): under the hall's lights the reference histogram alone marked 9 % of the
   real mat (photos/New_Ladder_1.jpeg) and the flights saw no pad at all; the band marks ~85 %;
   then thin parts (a 3x3 opening: ladder rails and rungs a pixel or two wide) are removed;
2. 4-connected blobs of them (the white arcs and grey H are holes);
3. each blob pixel's ray cast onto the floor, in the camera's level frame (F forward, L left),
   for a camera ONE metre above the floor, weighted by the floor area it covers;
4. its shape: the mat is a SOLID, SQUARE patch (its H and arcs are holes inside it). Something as
   blue but thin and open -- a ladder's rails and rungs -- fills little of its own convex hull
   (solidity) and is long and narrow. pick_pad refuses both.

Floor positions scale with the camera's height, so ``candidates`` needs no height: the mission,
which knows it, scales them and checks the size (``pick_pad``). A mono image finds nothing: in
brightness alone the cyan mat is within ~10 % of the orange tarp.
"""
from collections import deque
from dataclasses import dataclass
import math
from pathlib import Path

import numpy as np

from overwall_nav.stereo import mount_matrix

BINS = 24
REFERENCE = Path(__file__).resolve().parent / 'pad_reference.npy'
FLOOR_RGB = (226, 92, 44)          # the course's orange tarp (goalnav, from photos)


@dataclass
class PadConfig:
    mat_size: float = 0.85         # m, the mat's side (team estimate)
    min_side: float = 0.45         # a blob cut off by the frame still counts if this wide ...
    max_side: float = 1.25
    min_pixels: int = 25           # ... but its centre is used only if it is >= 0.8 mat_size both ways
    min_solidity: float = 0.55     # blob pixels / their convex hull's area (mat ~0.8, ladder ~0.3)
    min_squareness: float = 0.6    # short side / long side on the floor (mat ~1 at any rotation)
    threshold: float = 0.35        # of the colour model's peak
    max_width: int = 160           # work on an image this wide at most (speed)
    hue_deg: tuple = (160.0, 260.0)  # blue-cyan band (the mat ~196 deg; the orange tarp ~15 deg)
    min_saturation: float = 0.24
    min_value: float = 0.24


def chromaticity(rgb):
    rgb = np.asarray(rgb, float)
    total = rgb.sum(axis=-1, keepdims=True) + 1.0
    return rgb[..., :2] / total, total[..., 0]


def _bins(rg):
    return np.clip((rg * BINS / 0.8).astype(int), 0, BINS - 1)


class PadFinder:
    def __init__(self, reference=None, config=None):
        self.cfg = config or PadConfig()
        self.reference = np.load(REFERENCE) if reference is None else np.asarray(reference)
        rg, brightness = chromaticity(self.reference.reshape(-1, 3))
        b = _bins(rg[brightness > 60])                       # dark marks carry no colour
        hist = np.zeros((BINS, BINS))
        np.add.at(hist, (b[:, 0], b[:, 1]), 1.0)
        hist = _blur(hist)
        hist /= hist.max()
        fb = _bins(chromaticity(np.array(FLOOR_RGB, float)[None])[0])[0]
        hist[max(fb[0] - 2, 0):fb[0] + 3, max(fb[1] - 2, 0):fb[1] + 3] = 0.0   # never the tarp
        self.model = hist

    def mask(self, image):
        rg, brightness = chromaticity(image)
        b = _bins(rg)
        model = (self.model[b[..., 0], b[..., 1]] > self.cfg.threshold) & (brightness > 60)
        h, sat, val = hsv(image)
        lo, hi = self.cfg.hue_deg
        band = (h >= lo) & (h <= hi) & (sat >= self.cfg.min_saturation) & (val >= self.cfg.min_value)
        return model | band

    def candidates(self, image, K, pitch_deg, roll_deg=0.0, undistort=None):
        """Blobs that lie on the floor: [{'centre': [f, l], 'sides': [short, long], 'touches', 'pixels'}],
        floor metres per metre of camera height, in the camera's level frame. ``image`` RGB (H, W, 3);
        ``K`` its 3x3 intrinsics; ``undistort``: optional f(pixels Nx2) -> undistorted pixels."""
        if np.ndim(image) != 3:
            return []
        step = max(1, int(math.ceil(image.shape[1] / self.cfg.max_width)))
        small = np.asarray(image)[::step, ::step]
        mask = opening(self.mask(small))
        K = np.asarray(K, float)
        fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
        to_level = mount_matrix(pitch_deg, roll_deg)
        found = []
        for pixels in _blobs(mask, self.cfg.min_pixels):
            uv = np.column_stack([(pixels[:, 1] + 0.5) * step, (pixels[:, 0] + 0.5) * step])
            if undistort is not None:
                uv = undistort(uv)
            rays = np.column_stack([(uv[:, 0] - cx) / fx, (uv[:, 1] - cy) / fy, np.ones(len(uv))]) @ to_level.T
            down = rays[:, 2] < -1e-3
            if down.mean() < 0.95:
                continue                                     # not on the floor
            t = 1.0 / -rays[down, 2]
            points = rays[down, :2] * t[:, None]
            area = t ** 2 / np.abs(rays[down, 2])            # near pixels cover less floor
            centre = np.average(points, axis=0, weights=area)
            spread = np.sqrt(12.0 * np.clip(np.linalg.eigvalsh(np.cov((points - centre).T, aweights=area)), 0, None))
            touches = bool(pixels[:, 0].min() == 0 or pixels[:, 1].min() == 0
                           or pixels[:, 0].max() == mask.shape[0] - 1 or pixels[:, 1].max() == mask.shape[1] - 1)
            found.append({'centre': [float(centre[0]), float(centre[1])],
                          'sides': [float(spread.min()), float(spread.max())],
                          'touches': touches, 'pixels': int(len(pixels)),
                          'solidity': round(solidity(pixels), 3)})
        return found


def pick_pad(candidates, cam_height, config=None):
    """(forward, left) metres from the camera to the mat's centre, or None: the largest candidate
    that is mat-sized at this camera height and, if cut off by the frame, still nearly whole."""
    cfg = config or PadConfig()
    best = None
    for c in candidates or ():
        short, long_ = (float(s) * cam_height for s in c['sides'])
        if long_ > cfg.max_side or short < (cfg.min_side if c.get('touches') else 0.6 * cfg.mat_size):
            continue
        if c.get('touches') and short < 0.8 * cfg.mat_size:
            continue                                         # cut off: its centre is not the mat's
        if c.get('solidity', 1.0) < cfg.min_solidity:
            continue                                         # open, like a ladder: not the solid mat
        if not c.get('touches') and short < cfg.min_squareness * long_:
            continue                                         # long and narrow: not the square mat
        if best is None or c.get('pixels', 0) > best.get('pixels', 0):
            best = c
    if best is None:
        return None
    return (float(best['centre'][0]) * cam_height, float(best['centre'][1]) * cam_height)


def hsv(image):
    """Hue (deg), saturation and value (0..1) of an RGB image."""
    rgb = np.asarray(image, float) / 255.0
    mx, mn = rgb.max(axis=-1), rgb.min(axis=-1)
    d = mx - mn
    r, g, b = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    with np.errstate(divide='ignore', invalid='ignore'):
        h = np.where(mx == r, (g - b) / d % 6, np.where(mx == g, (b - r) / d + 2, (r - g) / d + 4)) * 60.0
        sat = np.where(mx > 0, d / mx, 0.0)
    return np.nan_to_num(h), sat, mx


def opening(mask):
    """3x3 erosion then dilation: drops parts 1-2 px wide (ladder rails, rungs, noise), keeps the mat."""
    def shifts(m):
        p = np.pad(m, 1)
        return [p[1 + a:p.shape[0] - 1 + a, 1 + b:p.shape[1] - 1 + b] for a in (-1, 0, 1) for b in (-1, 0, 1)]
    eroded = np.logical_and.reduce(shifts(mask))
    return np.logical_or.reduce(shifts(eroded))


def solidity(pixels):
    """Pixel count / area of the convex hull of the pixels' squares (1 = convex and filled)."""
    i, j = pixels[:, 0].astype(float), pixels[:, 1].astype(float)
    corners = np.unique(np.concatenate([np.column_stack([i + a, j + b]) for a in (0, 1) for b in (0, 1)]), axis=0)
    hull = _hull(corners)
    x, y = hull[:, 0], hull[:, 1]
    area = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
    return float(len(pixels) / area) if area > 0 else 1.0


def _hull(points):
    """Convex hull (Andrew's monotone chain) of sorted unique 2-D points, counter-clockwise."""
    pts = points[np.lexsort((points[:, 1], points[:, 0]))]

    def half(seq):
        out = []
        for p in seq:
            while len(out) >= 2 and ((out[-1][0] - out[-2][0]) * (p[1] - out[-2][1])
                                     - (out[-1][1] - out[-2][1]) * (p[0] - out[-2][0])) <= 0:
                out.pop()
            out.append(p)
        return out
    lower, upper = half(pts), half(pts[::-1])
    return np.array(lower[:-1] + upper[:-1])


def _blur(h):
    out = h.copy()
    for axis in (0, 1):
        out = out + 0.5 * (np.roll(out, 1, axis) + np.roll(out, -1, axis))
    return out


def _blobs(mask, min_pixels):
    """4-connected components of a small boolean image, largest first."""
    seen = np.zeros_like(mask)
    found = []
    h, w = mask.shape
    for start in zip(*np.nonzero(mask)):
        if seen[start]:
            continue
        queue, pixels = deque([start]), []
        seen[start] = True
        while queue:
            i, j = queue.popleft()
            pixels.append((i, j))
            for a, b in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
                if 0 <= a < h and 0 <= b < w and mask[a, b] and not seen[a, b]:
                    seen[a, b] = True
                    queue.append((a, b))
        if len(pixels) >= min_pixels:
            found.append(np.array(pixels))
    return sorted(found, key=len, reverse=True)
