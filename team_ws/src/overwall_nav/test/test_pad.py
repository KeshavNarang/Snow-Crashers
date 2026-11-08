"""pad.py on rendered colour views of the real mat, from skybeta-03's colour camera.

Camera: the OAK-D-S2 colour camera (IMX378) at 640x400, K from its factory calibration
(agents/logs/colourtest-20261107-184424/calibration.json: fx 1468.8 at 1920 wide, so 489.6 at
640), mounted with the mono pair at 27 deg down. Its vertical field is only +-22 deg, so it sees
the floor from 5 to 49 deg below level: nothing nearer than ~0.87 x its height.

Scene: the mat (overwall_nav/pad_reference.npy, goalnav/assets/ copied, cut from a photo of the real mat,
0.85 m a side) on an orange tarp of the course's colour (226, 92, 44, as goalnav measured) with
smooth brightness wrinkles; a grey hall above the horizon.
"""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from overwall_nav.pad import PadFinder, pick_pad    # noqa: E402
from overwall_nav.stereo import mount_matrix        # noqa: E402

K = np.array([[489.6, 0.0, 320.0], [0.0, 489.3, 200.0], [0.0, 0.0, 1.0]])
SIZE = (640, 400)
PITCH = 27.0
MAT = 0.85


LADDER_RGB = (31.0, 152.0, 172.0)      # the mat's own cyan (mean of the reference): the worst case


def _local(f, l, centre_f, centre_l, yaw_deg):
    c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    return c * (f - centre_f) + s * (l - centre_l), -s * (f - centre_f) + c * (l - centre_l)


def render(cam_height, mat=None, seed=0, ladders=()):
    """Colour image (H, W, 3) uint8. mat: (forward, left, yaw_deg) of its centre from the camera.
    ladders: (forward, left, yaw_deg, width, length) lying flat: two 4 cm rails, 3 cm rungs every 0.25 m."""
    w, h = SIZE
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    rays = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u)], axis=-1)
    level = rays @ mount_matrix(PITCH).T                          # (F, L, U)
    down = level[..., 2] < -1e-6
    with np.errstate(divide='ignore', invalid='ignore'):
        t = np.where(down, cam_height / -level[..., 2], np.nan)
    f, l = level[..., 0] * t, level[..., 1] * t
    rng = np.random.default_rng(seed)
    # Tarp: wrinkles as smooth brightness bands on the floor, plus pixel noise.
    shade = 0.85 + 0.12 * np.sin(5.1 * f + 1.3 * l) * np.sin(3.7 * l - 0.6 * f + 1.0)
    img = np.array([226.0, 92.0, 44.0]) * np.nan_to_num(shade, nan=1.0)[..., None]
    if mat is not None:
        mf, ml, yaw = mat
        c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
        a = c * (f - mf) + s * (l - ml)                               # mat axes
        b = -s * (f - mf) + c * (l - ml)
        inside = down & (np.abs(a) < MAT / 2) & (np.abs(b) < MAT / 2)
        ref = PadFinder().reference
        n = ref.shape[0]
        i = np.clip(((a[inside] / MAT + 0.5) * n).astype(int), 0, n - 1)
        j = np.clip(((b[inside] / MAT + 0.5) * n).astype(int), 0, n - 1)
        img[inside] = ref[i, j] * np.nan_to_num(shade, nan=1.0)[inside][:, None]
    for lf, ll, lyaw, width, length in ladders:
        a, b = _local(f, l, lf, ll, lyaw)                         # a along the ladder, b across
        inside = down & (np.abs(a) < length / 2) & (np.abs(b) < width / 2)
        rail = np.abs(b) > width / 2 - 0.04
        rung = np.abs(((a + length / 2) % 0.25) - 0.125) < 0.015
        blue = inside & (rail | rung)
        img[blue] = np.array(LADDER_RGB) * np.nan_to_num(shade, nan=1.0)[blue][:, None]
    img[~down] = (120.0, 120.0, 125.0)
    img += rng.normal(0.0, 4.0, img.shape)
    return np.clip(img, 0, 255).astype(np.uint8)


def find(cam_height, mat=None, ladders=()):
    candidates = PadFinder().candidates(render(cam_height, mat, ladders=ladders), K, PITCH)
    return pick_pad(candidates, cam_height)


class PadTest(unittest.TestCase):
    def test_finds_the_mat_from_the_look_height(self):
        # Past the wall the mission drops the camera to 0.8 m and looks: the mat 1.3 m ahead,
        # 0.3 m to the left, turned 20 deg (it may lie at any rotation).
        got = find(0.8, (1.3, 0.3, 20.0))
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got[0], 1.3, delta=0.08)
        self.assertAlmostEqual(got[1], 0.3, delta=0.08)

    def test_mat_cut_off_by_the_frame_gives_no_centre(self):
        # Crossing the test wall the camera is ~2.0 m up (1.6 m wall + 0.4 m margin) and the mat
        # 1.9 m ahead: its near half is below the image (floor seen from ~1.75 m). A centre from
        # the visible half would be ~0.2 m long: the mission must not land on that.
        self.assertIsNone(find(2.0, (1.9, 0.0, 0.0)))

    def test_blue_ladder_is_not_the_pad(self):
        # Team report, 2026-11-07: past the wall a ladder as blue as the mat sits next to it and a
        # flight landed on it. A 0.5 x 1.1 m section fits the size gate (0.45-1.25 m); it is not a
        # solid square, so it must be refused.
        self.assertIsNone(find(0.8, None, ladders=[(1.3, 0.0, 20.0, 0.5, 1.1)]))

    def test_pad_beside_the_ladder_is_still_found(self):
        got = find(0.8, (1.5, -0.35, 10.0), ladders=[(1.3, 0.75, 75.0, 0.5, 1.1)])   # 15 cm clear of the mat
        self.assertIsNotNone(got)
        self.assertAlmostEqual(got[0], 1.5, delta=0.08)
        self.assertAlmostEqual(got[1], -0.35, delta=0.08)

    def test_bare_tarp_is_not_a_pad(self):
        self.assertIsNone(find(0.8, None))


if __name__ == '__main__':
    unittest.main()
