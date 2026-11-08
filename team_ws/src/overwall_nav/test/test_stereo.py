"""stereo.py + perception.py on exact synthetic stereo pairs (test/synthetic.py): skybeta-03's real
calibration, the camera pitched 27 deg down, a textured floor and wall."""
from pathlib import Path
import math
import sys
import unittest

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
sys.path.insert(0, str(HERE))
from overwall_nav.perception import analyze, bin_points                           # noqa: E402
from overwall_nav.stereo import StereoRig, extrinsics_from_tf, fit_floor, level   # noqa: E402
import synthetic                                                                   # noqa: E402

PITCH = 27.0
CORRIDOR = 0.4


def make_rig():
    r, t = extrinsics_from_tf(synthetic.T_LR, synthetic.Q_LR)
    return StereoRig(synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, r, t)


RIG = make_rig()


def corridor_points(rig, left, right, pitch=PITCH):
    """What stereo_depth publishes: the binned cells of the drone-wide corridor."""
    pts, valid = rig.points(left, right)
    return bin_points(level(pts, pitch), CORRIDOR), valid, pts


class FloorTest(unittest.TestCase):
    def test_floor_fit_measures_the_mount_pitch_and_height(self):
        for pitch, height in ((27.0, 0.14), (27.0, 0.85), (20.0, 0.5)):
            left, right = synthetic.pair(pitch_deg=pitch, cam_height=height)
            pts, valid = RIG.points(left, right)
            floor = fit_floor(pts)
            self.assertIsNotNone(floor, (pitch, height, valid))
            self.assertAlmostEqual(floor['pitch_deg'], pitch, delta=1.5, msg=(pitch, height, floor))
            self.assertAlmostEqual(floor['height_m'], height, delta=0.03 + 0.03 * height, msg=(pitch, height, floor))
            self.assertLess(abs(floor['roll_deg']), 1.5, floor)

    def test_on_the_ground_the_floor_is_all_it_sees(self):
        left, right = synthetic.pair(cam_height=0.14)
        flu, valid, _ = corridor_points(RIG, left, right)
        a = analyze(flu, 0.14, RIG.top_elevation_deg(PITCH))
        self.assertIsNone(a['front'], a)                       # the floor is not an obstacle
        self.assertGreater(valid, 0.3)

    def test_camera_sees_nothing_above_its_own_height(self):
        self.assertLess(RIG.top_elevation_deg(PITCH), 0.0)
        self.assertGreater(RIG.top_elevation_deg(PITCH), -3.0)


class WallTest(unittest.TestCase):
    def test_wall_distance_is_horizontal_not_along_the_tilted_axis(self):
        for d in (0.5, 0.8, 1.5, 2.5):
            left, right = synthetic.pair(cam_height=0.85, wall_distance=d, wall_top=1.0)
            flu, valid, _ = corridor_points(RIG, left, right)
            a = analyze(flu, 0.85, RIG.top_elevation_deg(PITCH))
            self.assertIsNotNone(a['front'], (d, a, valid))
            self.assertAlmostEqual(a['front'], d, delta=0.05 * d + 0.02, msg=(d, a))

    def test_wall_top_hidden_below_and_measured_above(self):
        left, right = synthetic.pair(cam_height=0.85, wall_distance=0.5, wall_top=1.0)
        flu, _, _ = corridor_points(RIG, left, right)
        below = analyze(flu, 0.85, RIG.top_elevation_deg(PITCH), wall_distance=0.5)
        self.assertFalse(below['top_visible'], below)
        # 5 cm cells: the top reads up to one cell high (the safe direction: climb a bit more),
        # so it is "visible" once the camera is ~0.15 m above it.
        for over in (0.2, 0.3, 0.45):
            h = 1.0 + over
            left, right = synthetic.pair(cam_height=h, wall_distance=0.5, wall_top=1.0)
            flu, _, _ = corridor_points(RIG, left, right)
            above = analyze(flu, h, RIG.top_elevation_deg(PITCH), wall_distance=0.5)
            self.assertTrue(above['top_visible'], (over, above))
            self.assertGreaterEqual(h + above['top_u'], 0.98, (over, above))     # never low
            self.assertLessEqual(h + above['top_u'], 1.07, (over, above))

    def test_blank_wall_reads_unknown_not_clear(self):
        left, right = synthetic.pair(cam_height=0.85, wall_distance=0.5, wall_textured=False)
        flu, _, _ = corridor_points(RIG, left, right)
        a = analyze(flu, 0.85, RIG.top_elevation_deg(PITCH))
        self.assertIsNone(a['front'], a)

    def test_the_mount_pitch_matters(self):
        # Treat the 27 deg camera as level and the floor ahead tilts up into a phantom wall.
        left, right = synthetic.pair(cam_height=0.85)
        right_pitch = analyze(corridor_points(RIG, left, right)[0], 0.85, RIG.top_elevation_deg(PITCH))
        level_guess = analyze(corridor_points(RIG, left, right, pitch=0.0)[0], 0.85, RIG.top_elevation_deg(0.0))
        self.assertIsNone(right_pitch['front'], right_pitch)
        self.assertIsNotNone(level_guess['front'], level_guess)
        self.assertLess(level_guess['front'], 2.0, level_guess)

    def test_no_phantom_wall_in_flight(self):
        for h in (0.5, 0.85, 1.4):
            left, right = synthetic.pair(cam_height=h)
            a = analyze(corridor_points(RIG, left, right)[0], h, RIG.top_elevation_deg(PITCH))
            self.assertIsNone(a['front'], (h, a))

    def test_rectifying_with_the_measured_rotation_matches_more(self):
        left, right = synthetic.pair(cam_height=0.85, wall_distance=0.7, wall_top=1.0)
        flat = StereoRig(synthetic.K_LEFT, synthetic.K_RIGHT, synthetic.SIZE, np.eye(3),
                         np.array([[-0.075], [0.0], [0.0]]))
        self.assertGreater(RIG.points(left, right)[1], flat.points(left, right)[1])

    def test_range_limits(self):
        self.assertLess(RIG.min_range, 0.3)
        self.assertAlmostEqual(RIG.baseline, 0.075, delta=0.002)


if __name__ == '__main__':
    unittest.main()
