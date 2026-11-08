"""goalnav_node takes camera images only when it uses them (rgbstereo mode).

Observation (skybeta-03, 2026-11-08 01:25-01:31, goalnav running): vision reached PX4 236-400 ms
old and PX4 fused only 60 % of a 30 s sample, against 62-105 ms and fused with the same camera
without goalnav. goalnav subscribed to the full-size mono left image (1024x640, 30 Hz) for a floor
tape no goal uses any more, and to colour (640x400, ~9 Hz) the whole flight. rclpy decodes every
message before the callback can drop it; stereo_node's own note: a second Python subscriber to the
30 Hz images loads the cores cuVSLAM runs on. The pad still needs colour near the end.
"""
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT.parent / 'team_ws' / 'src' / 'overwall_nav'))

from localsim import fakeros as m                                   # noqa: E402

MONO, RGB = '/oak/left/image_mono', '/oak/rgb/image_raw'
RGBSTEREO = {'depth_topic': '', 'points_topic': '/goalnav/stereo/points', 'image_topic': '',
             'pad_image_topic': RGB, 'pad_info_topic': '/oak/rgb/camera_info'}


class ImagesOnlyWhenUsed(unittest.TestCase):
    def setUp(self):
        self.bus = m.Bus({'drone_ref_goalnav': dict(RGBSTEREO, goals_file=str(ROOT / 'config/goals.yaml'))})
        self.ctx = m.installed(self.bus, ROOT)
        self.ctx.__enter__()
        sys.modules.pop('drone_ref.goalnav_node', None)    # bind to this test's fake rclpy and bus
        from drone_ref import goalnav_node
        self.node = goalnav_node.GoalNavNode()

    def tearDown(self):
        self.ctx.__exit__(None, None, None)

    def subscribed(self, topic):
        return bool(self.bus.subscribers.get(topic))

    def test_no_full_size_mono_and_no_colour_before_the_pad(self):
        self.assertFalse(self.subscribed(MONO), 'goalnav takes the 30 Hz mono image it never uses')
        self.assertFalse(self.subscribed(RGB), 'colour decoded the whole flight, not only near the pad')

    def test_colour_while_the_pad_is_near_then_released(self):
        self.node._wants_pad = lambda: True
        self.node._update_image_subscriptions()
        self.assertTrue(self.subscribed(RGB), 'no colour near the pad: the mat cannot be found')
        self.node._wants_pad = lambda: False
        self.node._update_image_subscriptions()
        self.assertFalse(self.subscribed(RGB))


if __name__ == '__main__':
    unittest.main()
