"""The simulator feeds goalnav the way the drone does with the team camera config (RGBSTEREO).

On skybeta-03 the team camera config (team_ws/camera/oakd_vslam.yaml) runs RGBSTEREO: colour + the
mono pair, no on-camera depth (on-camera depth cost PX4 its VIO fusion; team_ws/preflight.sh). The
launch (goalnav.launch.py CAMERA_MODES) then starts stereo_node, measured on skybeta-03 on 2026-11-08 at
3.77 Hz and 0.159 s old on average (agents/logs/2026-11-08-sim-stereo-drone.md; earlier ~4.5 Hz,
0.13 s in agents/logs/2026-11-07-depth-bottleneck.md), and finds the pad in colour. Planner results
from the simulator only carry over if it hands goalnav the same topics at that rate.
"""
import ast
from pathlib import Path
import sys
import unittest

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

from localsim import goalrun  # noqa: E402

LAUNCH = HERE.parent / 'launch' / 'goalnav.launch.py'


def _launch_modes():
    tree = ast.parse(LAUNCH.read_text())
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == 'CAMERA_MODES' for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError('goalnav.launch.py has no CAMERA_MODES')


class SimCameraLikeDrone(unittest.TestCase):
    def test_default_is_rgbstereo_at_the_drones_stereo_rate(self):
        args = goalrun.build_parser().parse_args([])
        self.assertEqual(args.camera, 'rgbstereo')
        hz, latency = goalrun.camera_timing(args)
        self.assertAlmostEqual(hz, 3.77)
        self.assertAlmostEqual(latency, 0.159)

    def test_goalnav_gets_the_launch_files_topics_in_each_mode(self):
        launch = _launch_modes()
        for mode in ('rgbstereo', 'rgbd'):
            self.assertEqual(goalrun.SIM_CAMERAS[mode]['goalnav'], launch[mode]['goalnav'], mode)

    def test_explicit_rate_still_wins(self):
        args = goalrun.build_parser().parse_args(['--camera', 'rgbd', '--camera-hz', '7'])
        self.assertEqual(goalrun.camera_timing(args), (7.0, 0.1))


if __name__ == '__main__':
    unittest.main()
