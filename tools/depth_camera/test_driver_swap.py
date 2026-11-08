"""Evals for driver_swap.py (no ROS: a fake runner stands in for the ros2 CLI):

    python3 -m unittest -v test_driver_swap

The swap replaces only the OAK driver component inside the stack's own oak_container, so no process exits and
the slam pane, cuVSLAM and the PX4 agent keep running (a dead slam pane makes the stack service restart the
agent, which can wedge the FC link). These tests pin what the tool must and must not do.
"""
import os
import tempfile
import unittest

import driver_params as dp
import driver_swap as ds


def read(path):
    with open(path) as f:
        return f.read()

FULL_LIST = """\
/oak_container
  1  /oak
  2  /oak_state_publisher
/vslam_container
  1  /left_format_converter
  2  /right_format_converter
  3  /visual_slam_node
"""

DUMP_STEREO = """\
/oak:
  ros__parameters:
    driver:
      i_enable_ir: false
    imu:
      i_acc_freq: 200
    left:
      i_fps: 30.0
      i_height: 640
      i_publish_topic: true
      i_width: 1024
    pipeline_gen:
      i_pipeline_type: stereo
    right:
      i_fps: 30.0
      i_height: 640
      i_publish_topic: true
      i_width: 1024
"""
DUMP_DEPTH = DUMP_STEREO.replace('i_pipeline_type: stereo', 'i_pipeline_type: depth')


class FakeRunner:
    """Records every command; answers the read-only ones from canned text."""

    def __init__(self, dump=DUMP_STEREO, components=FULL_LIST, healthy=True, healthy_after_restore=True):
        self.cmds, self.dump, self.components = [], dump, components
        self.healthy, self.healthy_after_restore = healthy, healthy_after_restore
        self.loaded = []

    def run(self, cmd):
        self.cmds.append(cmd)
        joined = ' '.join(cmd)
        if joined.startswith('ros2 component list'):
            return self.components
        if joined.startswith('ros2 param dump'):
            return self.dump
        if joined.startswith('ros2 launch'):
            self.loaded.append([a for a in cmd if a.startswith('params_file:=')][0].split(':=', 1)[1])
        return ''

    def health(self, want_depth):
        if len(self.loaded) >= 2:                          # second load = the automatic restore
            return {'ok': self.healthy_after_restore, 'why': 'fake'}
        return {'ok': self.healthy, 'why': 'fake'}

    def mutating(self):
        return [c for c in self.cmds if c[:3] in (['ros2', 'component', 'unload'],) or c[:2] == ['ros2', 'launch']]


class Parse(unittest.TestCase):
    def test_full_listing(self):
        comps = ds.parse_component_list(FULL_LIST)
        self.assertEqual(comps['/oak_container'], [(1, '/oak'), (2, '/oak_state_publisher')])
        self.assertEqual(comps['/vslam_container'][2], (3, '/visual_slam_node'))

    def test_single_container_listing(self):
        comps = ds.parse_component_list('1  /oak\n2  /oak_state_publisher\n', container='/oak_container')
        self.assertEqual(comps['/oak_container'][0], (1, '/oak'))

    def test_find_driver(self):
        comps = ds.parse_component_list(FULL_LIST)
        self.assertEqual(ds.find_component(comps, '/oak_container', '/oak'), 1)
        with self.assertRaises(LookupError):
            ds.find_component(comps, '/oak_container', '/nope')
        with self.assertRaises(LookupError):
            ds.find_component(comps, '/missing_container', '/oak')


class Swap(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.state = self.tmp.name

    def tearDown(self):
        self.tmp.cleanup()

    def swap(self, runner, overlay=None, mode='candidate', dry_run=False):
        return ds.Swapper(runner, self.state, launch_file='/x/load_driver.launch.py').swap(
            mode, overlay if overlay is not None else dp.depth_overlay(), dry_run=dry_run)

    def test_candidate_swap_unloads_by_id_then_loads_the_candidate(self):
        r = FakeRunner()
        res = self.swap(r)
        self.assertTrue(res['ok'], res)
        self.assertIn(['ros2', 'component', 'unload', '/oak_container', '1'], r.cmds)
        unload_i = r.cmds.index(['ros2', 'component', 'unload', '/oak_container', '1'])
        launch_i = next(i for i, c in enumerate(r.cmds) if c[:2] == ['ros2', 'launch'])
        self.assertLess(unload_i, launch_i)
        cand = dp.parse_dump(read(r.loaded[0]))
        self.assertEqual(cand['pipeline_gen.i_pipeline_type'], 'depth')
        self.assertEqual(cand['left.i_width'], 1024)

    def test_original_is_saved_once_and_never_overwritten(self):
        r = FakeRunner()
        self.swap(r)
        orig = read(os.path.join(self.state, 'original_oak_params.yaml'))
        r2 = FakeRunner(dump=DUMP_DEPTH)                  # the candidate is now the running config
        res2 = self.swap(r2, overlay=dp.depth_overlay(preset='ROBOTICS'))   # trying another candidate is allowed
        self.assertTrue(res2['ok'], res2)
        self.assertEqual(read(os.path.join(self.state, 'original_oak_params.yaml')), orig)
        self.assertEqual(dp.parse_dump(orig)['pipeline_gen.i_pipeline_type'], 'stereo')
        # and the second candidate is built on the ORIGINAL, not on the first candidate
        self.assertEqual(dp.parse_dump(read(r2.loaded[0]))['stereo.i_depth_preset'], 'ROBOTICS')

    def test_a_later_stereo_config_does_not_replace_the_first_original(self):
        r = FakeRunner()
        self.swap(r)
        changed = DUMP_STEREO.replace('i_acc_freq: 200', 'i_acc_freq: 100')
        self.swap(FakeRunner(dump=changed))
        saved = dp.parse_dump(read(os.path.join(self.state, 'original_oak_params.yaml')))
        self.assertEqual(saved['imu.i_acc_freq'], 200)

    def test_refuses_to_take_a_non_stereo_config_as_the_original(self):
        r = FakeRunner(dump=DUMP_DEPTH)                   # nothing saved yet and the running config is not stock
        res = self.swap(r)
        self.assertFalse(res['ok'])
        self.assertEqual(r.mutating(), [])
        self.assertIn('original', res['why'])

    def test_invalid_candidate_touches_nothing(self):
        r = FakeRunner()
        bad = {**dp.depth_overlay(), 'left.i_width': 1280}
        res = self.swap(r, overlay=bad)
        self.assertFalse(res['ok'])
        self.assertEqual(r.mutating(), [])
        self.assertTrue(any('left.i_width' in p for p in res['problems']))

    def test_dry_run_changes_nothing(self):
        r = FakeRunner()
        res = self.swap(r, dry_run=True)
        self.assertTrue(res['ok'])
        self.assertEqual(r.mutating(), [])
        self.assertTrue(res['plan'])                        # but it says what it would do

    def test_unhealthy_candidate_is_reverted_automatically(self):
        r = FakeRunner(healthy=False)
        res = self.swap(r)
        self.assertFalse(res['ok'])
        self.assertTrue(res['reverted'])
        self.assertEqual(len(r.loaded), 2)
        restored = dp.parse_dump(read(r.loaded[1]))
        self.assertEqual(restored['pipeline_gen.i_pipeline_type'], 'stereo')
        self.assertEqual(sum(1 for c in r.cmds if c[:3] == ['ros2', 'component', 'unload']), 2)

    def test_failed_revert_is_reported_loudly(self):
        r = FakeRunner(healthy=False, healthy_after_restore=False)
        res = self.swap(r)
        self.assertFalse(res['ok'])
        self.assertFalse(res['restore_healthy'])
        self.assertIn('MANUAL', res['why'])

    def test_restore_uses_the_saved_original(self):
        r = FakeRunner()
        self.swap(r)
        r2 = FakeRunner(dump=DUMP_DEPTH)
        res = self.swap(r2, mode='restore')
        self.assertTrue(res['ok'], res)
        self.assertEqual(dp.parse_dump(read(r2.loaded[0]))['pipeline_gen.i_pipeline_type'], 'stereo')

    def test_restore_without_a_saved_original_refuses(self):
        r = FakeRunner(dump=DUMP_DEPTH)
        res = self.swap(r, mode='restore')
        self.assertFalse(res['ok'])
        self.assertEqual(r.mutating(), [])

    def test_missing_driver_component_touches_nothing(self):
        r = FakeRunner(components='/vslam_container\n  1  /visual_slam_node\n')
        res = self.swap(r)
        self.assertFalse(res['ok'])
        self.assertEqual(r.mutating(), [])


if __name__ == '__main__':
    unittest.main()
