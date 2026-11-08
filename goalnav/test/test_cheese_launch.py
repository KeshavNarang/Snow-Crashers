"""The cheese launch flies what experiments/cheese/run.sh simulated.

Observation: on skybeta-03 goalnav's node defaults to the mono left image for the pad
(goalnav_node.PARAMS image_topic), while the simulator's --colour switches it to
/oak/rgb/image_raw. A drone launch that differs from the simulated settings would fly
an untested configuration and, on mono, never see the mat (2026-11-07 validation).
Static checks: the launch file and run.sh are read as source, not run.
"""
import ast
from pathlib import Path
import re
import unittest

import yaml

ROOT = Path(__file__).resolve().parents[1]
CHEESE = ROOT / 'launch' / 'cheese.launch.py'
RUN = ROOT / 'experiments' / 'cheese' / 'run.sh'


def _assigned(path, name):
    for node in ast.parse(path.read_text()).body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    raise AssertionError(f'{path.name} has no {name}')


class CheeseLaunchMatchesSim(unittest.TestCase):
    def setUp(self):
        self.assertTrue(CHEESE.exists(), 'no launch/cheese.launch.py: the cheese run cannot be flown as simulated')
        self.args = _assigned(CHEESE, 'ARGS')
        self.run = RUN.read_text()

    def test_colour_camera_for_the_pad(self):
        self.assertIn('--colour', self.run)
        self.assertEqual(self.args['image_topic'], '/oak/rgb/image_raw')
        self.assertEqual(self.args['image_info_topic'], '/oak/rgb/camera_info')

    def test_same_goals_profile_and_tuning(self):
        tunes = dict(t.split('=', 1) for t in re.findall(r'--tune (\S+)', self.run))
        self.assertEqual(yaml.safe_load(self.args['tuning']), {k: yaml.safe_load(v) for k, v in tunes.items()})
        goals = re.search(r'goals_file=\$PWD/(\S+?)"', self.run).group(1)
        self.assertEqual(self.args['goals_file'], goals)
        self.assertEqual(self.args.get('profile', 'safe'), 'safe')
        self.assertNotIn('--profile', self.run)


if __name__ == '__main__':
    unittest.main()
