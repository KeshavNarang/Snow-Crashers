"""The cheese run on skybeta-03: goalnav.launch.py with the settings experiments/cheese/run.sh simulated.

Climb to 2.25 m, cross the barrier, find the H mat in the colour image, land and disarm on it.
No return. The pilot arms on Altitude, then switches to Position (flight_interface enters
Offboard); goalnav never arms. Colour from the RGBSTEREO image, depth from stereo_node on the Jetson
(goalnav.launch.py defaults: on-camera depth cost PX4 its VIO fusion on skybeta-03).
Pick it with ~/team_ws/run_mission.sh drone_ref cheese.launch.py (tools/fly deploy NAME cheese).
test/test_cheese_launch.py keeps ARGS equal to run.sh.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

ARGS = {
    'goals_file': 'config/cheese_goals.yaml',
    'camera': 'rgbstereo',          # colour pad + stereo_node depth (goalnav.launch.py CAMERA_MODES)
    'profile': 'safe',
    'tuning': '{cruise_height: 2.25, handover_height: 2.15, return_enabled: false, '
              'pad_disarm: true, heading_until: over_barrier}',
}


def generate_launch_description():
    goalnav = os.path.join(get_package_share_directory('drone_ref'), 'launch', 'goalnav.launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(goalnav),
                                                       launch_arguments=ARGS.items())])
