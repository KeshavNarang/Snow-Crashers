"""manual.launch.py: record a hand-flown flight, and run NOTHING that can fly the drone.

    python3 -m pytest team_ws/src/manual_flight/test        (needs ROS 2 launch / launch_ros)

The danger it guards against: overwall.launch.py and goalnav.launch.py start the organizers'
flight_interface with auto_offboard_on_arm, so a pilot who arms and switches to Position hands the
drone to the mission. A manual flight must have no flight_interface and no mission node at all.
"""
import importlib.util
import json
import os
import tempfile
import unittest
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

LAUNCH = Path(__file__).resolve().parents[1] / 'launch' / 'manual.launch.py'


def load():
    spec = importlib.util.spec_from_file_location('manual_launch', LAUNCH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def expand(module, **args):
    """What ros2 launch would start (the logger's actions), with the defaults plus ``args``."""
    context = LaunchContext()
    values = {name: default for name, default, _ in module.ARGS}
    values.update(args)
    context.launch_configurations.update(values)
    return context, module._logger(context)


def recorder(expanded):
    context, actions = expanded
    found = [a for a in actions if isinstance(a, ExecuteProcess)]
    assert len(found) == 1, found
    return [perform_substitutions(context, part) for part in found[0].cmd]


class ManualLaunchTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.module = load()

    def tearDown(self):
        self.tmp.cleanup()

    def actions(self, **args):
        args.setdefault('log_dir', self.tmp.name)
        return expand(self.module, **args)

    def test_nothing_that_can_fly_the_drone(self):
        entities = self.module.generate_launch_description().entities
        self.assertEqual([e for e in entities if not isinstance(e, (DeclareLaunchArgument, OpaqueFunction))], [])
        actions = self.actions()
        self.assertEqual([a for a in actions[1] if isinstance(a, Node)], [], 'a manual flight runs no ROS node')
        text = LAUNCH.read_text()
        code = '\n'.join(line for line in text.splitlines() if not line.lstrip().startswith('#'))
        self.assertNotIn("package='skybeta_flight_interface'", code)
        self.assertNotIn('/fmu/in/', ' '.join(recorder(actions)).replace('/fmu/in/vehicle_visual_odometry', ''))

    def test_the_old_missions_do_take_over_on_position(self):
        # So the test above is guarding a real difference: the flying launches run flight_interface
        # with auto_offboard_on_arm true.
        root = Path(__file__).resolve().parents[4]
        for path in ('team_ws/src/overwall_nav/launch/overwall.launch.py', 'goalnav/launch/goalnav.launch.py'):
            text = (root / path).read_text()
            self.assertIn("executable='flight_interface'", text, path)
            self.assertIn("('auto_offboard_on_arm', 'true'", text, path)

    def test_records_the_pilots_inputs_the_drones_outputs_and_vio(self):
        cmd = recorder(self.actions())
        self.assertEqual(cmd[:4], ['ros2', 'bag', 'record', '-s'])
        for topic in ('/fmu/out/manual_control_setpoint',       # the pilot's sticks, as PX4 sees them
                      '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude',
                      '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_land_detected',
                      '/fmu/out/estimator_status_flags', '/fmu/out/battery_status',
                      '/fmu/in/vehicle_visual_odometry',          # what the organizers' bridge feeds PX4
                      '/visual_slam/tracking/odometry', '/oak/imu/data',
                      '/oak/left/image_mono', '/oak/right/image_mono', '/oak/rgb/image_raw',
                      '/tf_static'):
            self.assertIn(topic, cmd, topic)

    def test_never_asks_for_compressed_images(self):
        # A subscriber on .../compressed makes the camera driver compress every frame: CPU taken from cuVSLAM.
        cmd = recorder(self.actions())
        self.assertFalse([t for t in cmd if 'compressed' in t or 'theora' in t])

    def test_video_off(self):
        cmd = recorder(self.actions(record_video='false'))
        self.assertFalse([t for t in cmd if 'image' in t and 'camera_info' not in t], cmd)
        self.assertIn('/fmu/out/manual_control_setpoint', cmd)

    def test_bag_survives_a_power_off(self):
        cmd = recorder(self.actions())
        self.assertEqual(cmd[cmd.index('--max-cache-size') + 1], '0')       # straight to disk

    def test_one_log_folder_per_power_up_where_tools_fly_logs_finds_it(self):
        cmd = recorder(self.actions())
        bag = cmd[cmd.index('-o') + 1]
        run = Path(bag).parent
        self.assertEqual(Path(bag).name, 'bag')
        self.assertEqual(run.parent, Path(self.tmp.name))                  # <log_dir>/<date-time>/bag
        self.assertNotEqual(run.name, 'overwall')
        info = json.loads((run / 'run.json').read_text())
        self.assertEqual(info['mission'], 'manual_flight')
        self.assertTrue((run / 'ros').is_dir())

    def test_default_log_dir_is_the_one_tools_fly_logs_reads(self):
        defaults = {name: default for name, default, _ in self.module.ARGS}
        self.assertEqual(defaults['log_dir'], '~/team_ws/logs')
        self.assertEqual(os.path.expanduser('~/team_ws/logs'), os.path.join(os.path.expanduser('~'), 'team_ws', 'logs'))

    def test_no_video_launch_is_the_same_recorder_without_images(self):
        text = (LAUNCH.parent / 'manual_novideo.launch.py').read_text()
        self.assertIn("'record_video': 'false'", text)
        self.assertIn("'manual.launch.py'", text)
        self.assertNotIn('flight_interface', text.split('"""')[-1])


if __name__ == '__main__':
    unittest.main()
