"""Find the landing target in the RGB image, from a reference picture of it.

The reference (``assets/pad_reference.npy``, cut and rectified from a photo
of the real mat by ``tools/make_pad_reference.py``) gives the mat's colour
model: a histogram of its pixels' chromaticity (colour independent of
brightness, so exposure changes and shadow do not matter). Each frame:

1. back-project the model onto the image: pixels whose chromaticity is
   common on the mat and rare on a reference of the floor tarp;
2. group them into connected blobs (the white arcs and grey H are holes);
3. cast each blob pixel's ray onto the floor plane with the camera pose,
   weighting each by the floor area it covers;
4. keep a blob that lies on the floor with a mat-sized footprint (people in
   blue beyond the net do not project to a flat ~0.8 m square on the floor)
   and report its centre in the odometry frame.

Pure NumPy; no OpenCV needed.
"""
from collections import deque
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from drone_ref.goalnav import resource
BINS = 24


def chromaticity(rgb):
    rgb = np.asarray(rgb, float)
    total = rgb.sum(axis=-1, keepdims=True) + 1.
    return rgb[..., :2] / total, total[..., 0]


def _bins(rg):
    return np.clip((rg * BINS / 0.8).astype(int), 0, BINS - 1)       # r, g < 0.8 always


@dataclass
class PadConfig:
    mat_size: float = 0.85        # m, nominal side (the team's estimate)
    min_side: float = 0.45        # partially seen mats still count
    max_side: float = 1.25
    min_pixels: int = 25
    threshold: float = 0.35       # of the model's peak


class PadDetector:
    def __init__(self, reference=None, floor_reference=(226, 92, 44), config=None):
        self.cfg = config or PadConfig()
        ref = np.load(resource('assets/pad_reference.npy')) if reference is None else np.asarray(reference)
        rg, brightness = chromaticity(ref.reshape(-1, 3))
        keep = brightness > 60                                  # dark marks carry no colour
        hist = np.zeros((BINS, BINS))
        b = _bins(rg[keep])
        np.add.at(hist, (b[:, 0], b[:, 1]), 1.)
        hist = _blur(hist)
        hist /= hist.max()
        floor_rg, _ = chromaticity(np.array(floor_reference, float)[None])
        fb = _bins(floor_rg)[0]
        hist[max(fb[0] - 2, 0):fb[0] + 3, max(fb[1] - 2, 0):fb[1] + 3] = 0.   # never the floor tarp
        self.model = hist
        self.last = None

    def mask(self, image):
        rg, brightness = chromaticity(image)
        b = _bins(rg)
        score = self.model[b[..., 0], b[..., 1]]
        return (score > self.cfg.threshold) & (brightness > 60)

    def detect(self, image, fx, fy, cx, cy, camera_origin, camera_rotation, floor_z):
        """Pad centre (x, y) in the odometry frame, or None.

        ``camera_rotation`` maps optical (x right, y down, z forward) to odom."""
        cfg = self.cfg
        mask = self.mask(image)
        best = None
        self.last = {'mask': mask, 'blobs': [], 'pick': None}     # For replay and debugging only.
        for pixels in _blobs(mask, cfg.min_pixels):
            blob = {'box': [int(pixels[:, 1].min()), int(pixels[:, 0].min()),
                            int(pixels[:, 1].max()) + 1, int(pixels[:, 0].max()) + 1], 'why': 'off the floor'}
            self.last['blobs'].append(blob)
            v, u = pixels[:, 0] + 0.5, pixels[:, 1] + 0.5
            rays = np.column_stack([(u - cx) / fx, (v - cy) / fy, np.ones(len(u))]) @ camera_rotation.T
            down = rays[:, 2] < -1e-3
            if down.mean() < 0.95:
                continue                                       # Not on the floor.
            t = (floor_z - camera_origin[2]) / rays[down, 2]
            points = camera_origin[:2] + rays[down, :2] * t[:, None]
            # Weight each pixel by the floor it covers (t^2 / |ray z|): a plain
            # mean favours the near side of the mat, which takes more pixels.
            area = t ** 2 / np.abs(rays[down, 2])
            centre = np.average(points, axis=0, weights=area)
            spread = np.sqrt(12. * np.linalg.eigvalsh(np.cov((points - centre).T, aweights=area)))
            touches = (pixels[:, 0].min() == 0 or pixels[:, 1].min() == 0
                       or pixels[:, 0].max() == mask.shape[0] - 1 or pixels[:, 1].max() == mask.shape[1] - 1)
            blob['why'] = 'wrong size'
            if spread.max() > cfg.max_side or spread.min() < (cfg.min_side if touches else 0.6 * cfg.mat_size):
                continue
            blob['why'] = 'cut off by the frame'
            if touches and spread.min() < 0.8 * cfg.mat_size:
                continue                                       # Cut off by the frame: centre unreliable.
            blob['why'] = 'smaller'
            if best is None or len(pixels) > best[0]:
                best = (len(pixels), centre, blob, pixels)
        if best is None:
            return None
        best[2]['why'] = 'pad'
        self.last['pick'] = best[3]
        return best[1]


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
