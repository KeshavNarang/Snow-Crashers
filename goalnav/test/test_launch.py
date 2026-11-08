"""goalnav.launch.py is entered the way overwall_nav's overwall.launch.py (the launch that flew) is.

Static checks (no ROS needed): the launch file is read as source, not imported.
"""
import ast
from pathlib import Path
import unittest

HERE = Path(__file__).resolve().parent
GOALNAV = HERE.parent / 'launch' / 'goalnav.launch.py'
OVERWALL = HERE.parents[1] / 'team_ws' / 'src' / 'overwall_nav' / 'launch' / 'overwall.launch.py'


def _tree(path):
    return ast.parse(path.read_text()), path.read_text()


def _assigned(tree, name):
    for node in tree.body:
        if isinstance(node, ast.Assign) and any(getattr(t, 'id', None) == name for t in node.targets):
            return ast.literal_eval(node.value)
    return None


def _node_calls(tree):
    """{executable: Node(...) call} for every launch_ros Node in the file."""
    calls = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Call) and getattr(node.func, 'id', None) == 'Node':
            kw = {k.arg: k.value for k in node.keywords}
            if isinstance(kw.get('executable'), ast.Constant):
                calls[kw['executable'].value] = node
    return calls


def _keyword_source(call, name, source):
    for k in call.keywords:
        if k.arg == name:
            return ast.get_source_segment(source, k.value)
    return None


@unittest.skipUnless(OVERWALL.exists(), 'overwall_nav not next to goalnav (on the drone the packages sit apart)')
class LaunchLikeOverwall(unittest.TestCase):
    def setUp(self):
        self.goal, self.goal_src = _tree(GOALNAV)
        self.wall, self.wall_src = _tree(OVERWALL)

    def test_same_flight_interface_arguments(self):
        wall = _assigned(self.wall, 'INTERFACE')
        self.assertIsNotNone(wall, 'overwall.launch.py lost its INTERFACE table: update this test')
        self.assertEqual(_assigned(self.goal, 'INTERFACE'), wall)

    def test_flight_interface_started_the_same_way(self):
        wall = _node_calls(self.wall)['flight_interface']
        goal = _node_calls(self.goal).get('flight_interface')
        self.assertIsNotNone(goal, 'goalnav.launch.py does not start flight_interface')
        for key in ('package', 'name', 'parameters'):
            self.assertEqual(_keyword_source(goal, key, self.goal_src), _keyword_source(wall, key, self.wall_src), key)
        self.assertNotIn('condition', [k.arg for k in goal.keywords], 'always started, as in overwall')

    def test_depth_from_the_mono_pair_on_the_jetson(self):
        """skybeta-03, 2026-11-07 23:49: with on-camera depth (rgbd) vision reached PX4 665 ms old
        (timestamp - timestamp_sample), estimator cs_ev_pos false, xy_valid false: no position.
        The RGBSTEREO image fuses, so depth comes from goalnav's stereo_node on the Jetson."""
        calls = _node_calls(self.goal)
        self.assertIn('goalnav_node', calls)
        self.assertIn('stereo_node', calls)
        self.assertNotIn('stereo_depth', calls)       # not overwall's CPU stereo as well
        defaults = {name: default for name, default, _ in _assigned(self.goal, 'GOALNAV')}
        self.assertEqual(defaults.get('points_topic'), '/goalnav/stereo/points')
        self.assertEqual(defaults.get('depth_topic'), '')    # the image has no on-camera depth

    def test_entry_is_generate_launch_description(self):
        names = [n.name for n in self.goal.body if isinstance(n, ast.FunctionDef)]
        self.assertIn('generate_launch_description', names)


if __name__ == '__main__':
    unittest.main()
