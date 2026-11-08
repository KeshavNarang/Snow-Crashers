"""The safe profile flies slowly enough for the depth and vision skybeta-03 actually delivers.

Observation (skybeta-03, 2026-11-08 07:2x, goalnav on main, props off): Jetson depth 3.75-4.0 Hz,
vision age median 63-133 ms with the slowest tenth 280-324 ms. The user: we cannot bank on
low-latency depth, so fly slowly; the course allows 3 min and sim runs took ~70 s; do not move
anywhere the drone is not confident of.
"""
from pathlib import Path
import unittest

import yaml

PROFILES = Path(__file__).resolve().parents[1] / 'config' / 'goalnav_profiles.yaml'
DEPTH_INTERVAL = 1 / 3.75          # s between depth frames, measured
VISION_AGE = 0.324                 # s, slowest tenth measured


class SlowEnoughForTheSensors(unittest.TestCase):
    def setUp(self):
        self.safe = yaml.safe_load(PROFILES.read_text())['safe']

    def test_reaction_covers_a_late_depth_frame_and_late_vision(self):
        self.assertGreaterEqual(self.safe['reaction_s'], DEPTH_INTERVAL + VISION_AGE + 0.1)

    def test_no_faster_than_a_quarter_metre_a_second(self):
        self.assertLessEqual(self.safe['max_speed'], 0.25)
        self.assertTrue(all(v <= 0.25 for v in self.safe['kind_speed'].values()))

    def test_the_return_home_is_slow_too(self):
        # MissionConfig's return_speed default is 1.2 m/s: the same late depth and vision on the way back.
        self.assertLessEqual(self.safe.get('return_speed', 1.2), 0.4)
