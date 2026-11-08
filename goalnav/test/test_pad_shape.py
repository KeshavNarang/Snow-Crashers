"""pad_detect must not take a blue ladder for the mat (team report, 2026-11-07: past the wall a
ladder as blue as the mat lies next to it, and a goalnav flight landed on it).

Rendered colour views from skybeta-03's colour camera (640x400, factory K: fx 489.6; 27 deg down
mount), the mat textured with assets/pad_reference.npy (0.85 m), on the course's orange tarp. The
ladder is the worst case: the mat's own cyan, two 4 cm rails and 3 cm rungs every 0.25 m, a
0.5 x 1.1 m section -- inside the old 0.45-1.25 m size gate.
"""
import math
import sys
import unittest
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from drone_ref.goalnav import resource                    # noqa: E402
from drone_ref.goalnav.pad_detect import PadDetector      # noqa: E402

FX, FY, CX, CY, W, H = 489.6, 489.3, 320.0, 200.0, 640, 400
PITCH, MAT, CAM_H = math.radians(27.0), 0.85, 0.8
# optical (x right, y down, z forward) -> level (F, L, U), camera pitched down
ROT = np.stack([[0.0, -1.0, 0.0], [-math.sin(PITCH), 0.0, -math.cos(PITCH)],
                [math.cos(PITCH), 0.0, -math.sin(PITCH)]], axis=1)
CYAN = (31.0, 152.0, 172.0)


def _local(f, l, cf, cl, yaw):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    return c * (f - cf) + s * (l - cl), -s * (f - cf) + c * (l - cl)


def render(mat=None, ladders=()):
    u, v = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u)], axis=-1) @ ROT.T
    down = rays[..., 2] < -1e-6
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(down, CAM_H / -rays[..., 2], np.nan)
    f, l = rays[..., 0] * t, rays[..., 1] * t
    shade = np.nan_to_num(0.85 + 0.12 * np.sin(5.1 * f + 1.3 * l) * np.sin(3.7 * l - 0.6 * f + 1.0), nan=1.0)
    img = np.array([226.0, 92.0, 44.0]) * shade[..., None]
    if mat is not None:
        a, b = _local(f, l, *mat)
        inside = down & (np.abs(a) < MAT / 2) & (np.abs(b) < MAT / 2)
        ref = np.load(resource('assets/pad_reference.npy')).astype(float)
        n = ref.shape[0]
        i = np.clip(((a[inside] / MAT + 0.5) * n).astype(int), 0, n - 1)
        j = np.clip(((b[inside] / MAT + 0.5) * n).astype(int), 0, n - 1)
        img[inside] = ref[i, j] * shade[inside][:, None]
    for lf, ll, lyaw, width, length in ladders:
        a, b = _local(f, l, lf, ll, lyaw)
        inside = down & (np.abs(a) < length / 2) & (np.abs(b) < width / 2)
        blue = inside & ((np.abs(b) > width / 2 - 0.04) | (np.abs(((a + length / 2) % 0.25) - 0.125) < 0.015))
        img[blue] = np.array(CYAN) * shade[blue][:, None]
    img[~down] = (120.0, 120.0, 125.0)
    img += np.random.default_rng(0).normal(0.0, 4.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def detect(**scene):
    return PadDetector().detect(render(**scene), FX, FY, CX, CY, np.array([0.0, 0.0, CAM_H]), ROT, 0.0)


class PadShapeTest(unittest.TestCase):
    def test_blue_ladder_is_not_the_pad(self):
        self.assertIsNone(detect(ladders=[(1.3, 0.0, 20.0, 0.5, 1.1)]))

    def test_pad_beside_the_ladder_is_found(self):
        got = detect(mat=(1.5, -0.35, 10.0), ladders=[(1.3, 0.75, 75.0, 0.5, 1.1)])   # 15 cm clear
        self.assertIsNotNone(got)
        self.assertAlmostEqual(float(got[0]), 1.5, delta=0.08)
        self.assertAlmostEqual(float(got[1]), -0.35, delta=0.08)

    def test_mat_alone_still_found(self):
        got = detect(mat=(1.3, 0.3, 45.0))
        self.assertIsNotNone(got)
        self.assertAlmostEqual(float(got[0]), 1.3, delta=0.08)


if __name__ == '__main__':
    unittest.main()
