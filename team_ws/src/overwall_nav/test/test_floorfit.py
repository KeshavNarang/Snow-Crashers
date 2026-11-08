"""fit_floor must return the FLOOR, not whichever plane has the most inliers.

Found on skybeta-03 (agents/logs/2026-11-07-depth.md): on a glossy table only ~28 % of pixels match,
so the table and a box face ahead tie in inliers and consecutive frames flip between them
(pitch 28-37 deg / height 0.10 m = correct; pitch 68-91 deg / height 0.3 m = the box). The wrong fit
becomes the mission's ground height and biases obstacle and wall-top readings by ~0.18 m.
"""
from pathlib import Path
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, fit_floor, mount_matrix   # noqa: E402
import synthetic                                                                          # noqa: E402

PITCH = 27.0
SEEDS = range(40)             # one fit per "frame": the node reseeds nothing, noise differs per frame


def cloud(floor_n=300, box_n=330, height=0.10, box_dist=0.30, pitch=PITCH, seed=0):
    """Camera-frame points: a floor `height` below the camera and a vertical box face `box_dist`
    ahead (level frame F forward, L left, U up), 3 mm noise. Counts make the box the bigger plane."""
    rng = np.random.default_rng(seed)
    f = rng.uniform(0.30, 2.5, floor_n)
    floor = np.stack([f, rng.uniform(-0.5, 0.5, floor_n), np.full(floor_n, -height)], 1)
    box = np.stack([np.full(box_n, box_dist), rng.uniform(-0.15, 0.15, box_n),
                    rng.uniform(-height + 0.01, 0.25, box_n)], 1)
    pts = np.concatenate([floor, box][: 2 if box_n else 1])
    pts = pts + rng.normal(0, 0.003, pts.shape)
    return pts @ mount_matrix(pitch)          # level -> camera optical frame


def is_floor(fit, pitch=PITCH, height=0.10):
    """The repo's own floor tolerance (test_stereo: 0.03 + 0.03 * height), pitch within 4 deg."""
    return (fit is not None and abs(fit['pitch_deg'] - pitch) < 4
            and abs(fit['height_m'] - height) < 0.03 + 0.03 * height and abs(fit['roll_deg']) < 6)


class FloorVersusBox(unittest.TestCase):
    def test_table_and_box_never_flip_to_the_box(self):
        wrong = [s for s in SEEDS if not is_floor(fit_floor(cloud(seed=s), seed=s, pitch_deg=PITCH))]
        self.assertEqual(wrong, [], f'{len(wrong)}/{len(SEEDS)} frames did not fit the floor')

    def test_the_old_behaviour_is_the_bug(self):
        """Without the orientation prior the box face wins some frames: this is what was found."""
        wrong = [s for s in SEEDS if not is_floor(fit_floor(cloud(seed=s), seed=s))]
        self.assertGreater(len(wrong), 0, 'the scene no longer reproduces the field bug; fix the eval')

    def test_floor_alone_still_fits(self):
        for height in (0.10, 0.5, 0.85):
            fit = fit_floor(cloud(box_n=0, height=height), pitch_deg=PITCH)
            self.assertTrue(is_floor(fit, height=height), (height, fit))

    def test_a_wall_alone_is_not_a_floor(self):
        wall = cloud(floor_n=0, box_n=400)[:400]
        pts = np.concatenate([wall])
        self.assertIsNone(fit_floor(pts, pitch_deg=PITCH))

    def test_mount_off_by_ten_degrees_still_fits(self):
        for mount in (17.0, 37.0):
            fit = fit_floor(cloud(box_n=0, pitch=mount), pitch_deg=PITCH)
            self.assertIsNotNone(fit, mount)
            self.assertAlmostEqual(fit['pitch_deg'], mount, delta=3)

    def test_a_tilted_camera_is_not_hidden_by_the_prior(self):
        """sensor_check exists to catch a wrongly mounted camera: 60 deg is reported as no fit,
        not silently bent to look like 27."""
        fit = fit_floor(cloud(box_n=0, pitch=60.0), pitch_deg=PITCH)
        self.assertTrue(fit is None or abs(fit['pitch_deg'] - 60.0) < 3, fit)

    def test_no_prior_behaves_as_before(self):
        fit = fit_floor(cloud(box_n=0))
        self.assertTrue(is_floor(fit), fit)


class EndToEnd(unittest.TestCase):
    """Exact synthetic stereo pairs through the real StereoRig.points, then fit_floor.

    Two geometries. The realistic one (a wall 1.5 m ahead, camera 0.85 m up) keeps the strict bar.
    The hard one is the field case: camera 0.10 m above a table, a tall obstacle 0.45 m ahead. There
    the floor is only visible between 0.3 and 0.45 m plus a few far points and the near-field stereo
    noise (MAD ~3 cm) is wider than the 2 cm RANSAC tolerance, so ~3 cm / ~6 deg is the best this data
    gives; the bar is "the floor, not the wall" with margin over that accuracy (the old code returned the
    wall: 112 deg, 0.45 m)."""

    @classmethod
    def setUpClass(cls):
        r, t = extrinsics_from_tf(synthetic.T_LR, synthetic.Q_LR)
        cls.rig = StereoRig(synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, r, t)

    def points(self, **scene):
        left, right = synthetic.pair(**scene)
        return self.rig.points(left, right)[0]

    def test_realistic_geometry_is_accurate_in_every_frame(self):
        pts = self.points(cam_height=0.85, wall_distance=1.5, wall_top=1.0)
        for s in range(20):
            fit = fit_floor(pts, seed=s, pitch_deg=PITCH)
            self.assertTrue(is_floor(fit, height=0.85), (s, fit))

    def test_hard_near_wall_geometry_returns_the_floor_not_the_wall(self):
        pts = self.points(cam_height=0.10, wall_distance=0.45, wall_top=0.35)
        for s in range(20):
            fit = fit_floor(pts, seed=s, pitch_deg=PITCH)
            self.assertIsNotNone(fit, s)
            # data-limited (worst seen: 5.2 cm, 9.8 deg); the old failure was 0.35 m and 85 deg
            self.assertLess(abs(fit['pitch_deg'] - PITCH), 15, (s, fit))
            self.assertLess(abs(fit['height_m'] - 0.10), 0.08, (s, fit))

    def test_the_same_hard_scene_without_the_prior_returns_the_wall(self):
        pts = self.points(cam_height=0.10, wall_distance=0.45, wall_top=0.35)
        fit = fit_floor(pts, seed=0)
        self.assertGreater(abs(fit['pitch_deg'] - PITCH), 45, fit)      # the bug, end to end


if __name__ == '__main__':
    unittest.main()
