"""The simulator's RGBSTEREO depth and colour look like skybeta-03's, as measured on 2026-11-08.

Read-only capture on skybeta-03 (2026-11-08, ``/goalnav/stereo/points``, ``/oak/rgb/image_raw``; analysis
in agents/logs/2026-11-08-sim-stereo-drone.md). Depth comes from drone_ref/stereo_node.py running
overwall_nav.stereo.StereoRig (scale 0.4, 96 disparities, step 2, rows 0-90 %):

- field of view -22.8..+37.5 deg across, -26.3..+20.6 deg down (the left 96 of 410 columns cannot be
  matched; the bottom 10 % of rows is never searched); the simulator gave +-38.75 / +-26.6;
- disparity noise ~0.3 px at fB = 19.87 px m (sigma_z ~ 0.015 z^2), 3.5 % of points > 15 cm off a plane;
- gaps follow texture (a dark floor gave no depth at all), ~39 % of the matchable grid valid, ~6700 points
  a frame; the simulator dropped 50 % of points at random and gave ~2000;
- 3.77 Hz, frame interval median 267 ms, p90 383, max 567; age median 0.111 s, p90 0.28, max 0.52;
- colour 640x400 at 10 Hz, 46 ms old, on its own clock; principal point (315.1, 196.6);
  in the dark room its mean was 14/255.
"""
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from localsim import goalrun, jetson_stereo, messy_course    # noqa: E402
from localsim.messy_rgb import FloorPainter, render as render_rgb   # noqa: E402
from localsim.messy_world import MessyWorld                  # noqa: E402

# Poses over the course (position, yaw): the launch, the maze, the window room, the pad.
POSES = [((-2.5, 0.9, 1.1), math.pi / 2), ((-1.3, 1.6, 1.15), math.pi / 2),
         ((0.5, -2.4, 1.18), math.radians(160)), ((-1.6, -2.0, 1.2), math.pi)]


def _yaw(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def _decode(message):
    return np.frombuffer(bytes(message.data), dtype=np.float32).reshape(-1, 3)


class SimStereoLikeDrone(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.tmp = tempfile.TemporaryDirectory()
        course = messy_course.generate(3, 0.7)
        obj, meta_path = messy_course.save(course, Path(cls.tmp.name) / 'course')
        cls.meta = json.loads(meta_path.read_text())
        cls.world = MessyWorld(obj, cls.meta, seed=3, cache_dir=ROOT / 'localsim/.cache')
        cls.camera = goalrun.left_camera()

    @classmethod
    def tearDownClass(cls):
        cls.tmp.cleanup()

    def _frames(self, **options):
        rng = np.random.default_rng(5)
        for position, yaw in POSES:
            message = goalrun.stereo_points(self.world, self.camera, np.array(position), _yaw(yaw), 0, rng, **options)
            yield position, yaw, _decode(message)

    def test_depth_covers_only_the_region_stereo_can_match(self):
        """Measured: -22.8..+37.5 deg across, -26.3..+20.6 deg down (sim was +-38.75 / +-26.6)."""
        across, down = [], []
        for _, _, p in self._frames():
            across += list(np.degrees(np.arctan2(p[:, 0], p[:, 2])))
            down += list(np.degrees(np.arctan2(p[:, 1], p[:, 2])))
        self.assertGreater(min(across), -22.8 - 1.5)
        self.assertLess(max(across), 37.5 + 1.5)
        self.assertGreater(min(down), -26.3 - 1.5)
        self.assertLess(max(down), 20.6 + 1.5)
        # The whole matchable region is still used.
        self.assertLess(min(across), -21.)
        self.assertGreater(max(across), 35.5)

    def test_disparity_noise_and_outliers_on_a_flat_wall(self):
        """Measured on a wall 1.0-2.0 m away: sigma_z ~ 0.015 z^2 (0.3 px at fB 19.87), 3.5 % > 15 cm."""
        z = np.full(20000, 1.3)
        got = jetson_stereo.disparity_noise(z, np.random.default_rng(2))
        err = got - z
        sigma = 1.4826 * np.median(np.abs(err - np.median(err)))
        self.assertAlmostEqual(sigma, 0.015 * 1.3 ** 2, delta=0.25 * 0.015 * 1.3 ** 2)
        self.assertTrue(0.015 <= np.mean(np.abs(err) > 0.15) <= 0.05, np.mean(np.abs(err) > 0.15))

    def test_density_and_gaps_follow_surfaces_not_random_points(self):
        """Measured: ~39 % of the matchable grid valid, ~6700 points a frame; gaps are whole low-texture
        surfaces (the sim's 50 % iid drop left a neighbour's validity uninformative)."""
        cells = len(jetson_stereo.grid()[0])
        valid, joint, pairs = [], 0, 0
        for _, _, p in self._frames():
            valid.append(len(p) / cells)
            mask = jetson_stereo.valid_mask(p)
            both = mask[:, 1:] & mask[:, :-1]
            joint += both.sum()
            pairs += mask[:, :-1].sum()
        overall = np.mean(valid)
        self.assertTrue(0.3 <= overall <= 0.5, valid)
        self.assertGreater(max(valid) * cells, 5000)
        # P(valid | left neighbour valid) well above P(valid): holes come in surfaces.
        self.assertGreater(joint / pairs, overall + 0.25)

    def test_dark_floor_gives_no_depth(self):
        """Measured: over a dark floor the drone's stereo returned no floor points at all."""
        def floor_points(**options):
            n = 0
            for position, yaw, p in self._frames(**options):
                body = p.astype(float) @ self.camera.body_from_optical().T + np.asarray(self.camera.offset)
                n += np.count_nonzero((body @ _yaw(yaw).T + position)[:, 2] < 0.05)
            return n
        default = floor_points()
        self.assertGreater(default, 500)                 # the default floor does return depth
        # What is left under 5 cm are wall feet (3-5 cm up) spread by the depth noise, not floor.
        self.assertLess(floor_points(dark_floor=True), 0.1 * default)

    def test_depth_timing_jitters_like_the_drone(self):
        """Measured 3.77 Hz: interval median 0.267 s, p90 0.383, max 0.567; age median 0.111, p90 0.28, max 0.52."""
        args = goalrun.build_parser().parse_args([])
        depth, _ = goalrun.camera_schedule(args, 400., np.random.default_rng(1))
        interval, age = np.diff(depth[:, 0]), depth[:, 1] - depth[:, 0]
        self.assertAlmostEqual(np.mean(interval), 1 / 3.77, delta=0.01)
        self.assertAlmostEqual(np.median(interval), 0.267, delta=0.03)
        self.assertAlmostEqual(np.percentile(interval, 90), 0.383, delta=0.04)
        self.assertTrue(0.45 < interval.max() <= 0.6, interval.max())
        self.assertAlmostEqual(np.median(age), 0.111, delta=0.02)
        self.assertAlmostEqual(np.percentile(age, 90), 0.28, delta=0.04)
        self.assertTrue(0.4 < age.max() <= 0.53, age.max())
        self.assertTrue(np.all(np.diff(depth[:, 1]) > 0))     # published in order, as on the drone
        # --camera-hz / --camera-latency still set the mean.
        args = goalrun.build_parser().parse_args(['--camera-hz', '2', '--camera-latency', '0.3'])
        depth, _ = goalrun.camera_schedule(args, 400., np.random.default_rng(1))
        self.assertAlmostEqual(np.mean(np.diff(depth[:, 0])), 0.5, delta=0.02)
        self.assertAlmostEqual(np.mean(depth[:, 1] - depth[:, 0]), 0.3, delta=0.02)

    def test_colour_runs_on_its_own_10_hz_clock(self):
        """Measured: colour 10 Hz, 46 ms old, independent of the depth frames."""
        args = goalrun.build_parser().parse_args([])
        depth, colour = goalrun.camera_schedule(args, 60., np.random.default_rng(1))
        self.assertTrue(np.allclose(np.diff(colour[:, 0]), 0.1))
        self.assertTrue(np.allclose(colour[:, 1] - colour[:, 0], 0.046, atol=0.005))
        between = np.diff(np.searchsorted(colour[:, 0], depth[:, 0]))
        self.assertGreater(len(set(between)), 1)        # not one colour frame per depth frame

    def test_colour_principal_point_is_the_drones_and_the_render_agrees(self):
        """Measured CameraInfo: cx 315.1, cy 196.6 at 640x400 (the sim used the exact centre)."""
        cam = goalrun.colour_camera()
        _, (_, info) = goalrun.image_messages(cam, np.zeros((cam.height, cam.width), np.uint8), 0, 'a', 'b', 'mono8')
        fx, cx, cy = info.k[0], info.k[2], info.k[5]
        self.assertAlmostEqual(cx, (315.1 + 0.5) * cam.width / 640 - 0.5, places=2)
        self.assertAlmostEqual(cy, (196.6 + 0.5) * cam.height / 400 - 0.5, places=2)
        # The rendered pixel (i, j) looks along ((i - cx) / fx, (j - cy) / fx, 1), as the CameraInfo says.
        from localsim.messy_rgb import pixel_rays
        rays = pixel_rays(cam).reshape(cam.height, cam.width, 3)
        np.testing.assert_allclose(rays[0, 0, :2] / rays[0, 0, 2], [(0 - cx) / fx, (0 - cy) / fx], atol=1e-9)

    def test_dark_exposure_option_and_unchanged_default(self):
        """Measured in the dark room: colour mean 14/255. The default exposure stays as it was."""
        cam = goalrun.colour_camera()
        painter = FloorPainter(self.meta)
        position, yaw = POSES[1]
        default = render_rgb(self.world, cam, painter, np.array(position), _yaw(yaw), np.random.default_rng(0))
        dark = render_rgb(self.world, goalrun.colour_camera(exposure=goalrun.DARK_EXPOSURE), painter,
                          np.array(position), _yaw(yaw), np.random.default_rng(0))
        self.assertGreater(default.mean(), 60)
        self.assertAlmostEqual(dark.mean(), 14, delta=6)


if __name__ == '__main__':
    unittest.main()
