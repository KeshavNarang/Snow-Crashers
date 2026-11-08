"""The cheese run on skybeta-03: goalnav.launch.py with the settings experiments/cheese/run.sh simulated.

Climb to 2.25 m, cross the barrier, find the H mat in the colour image, land and disarm on it.
No return. The pilot arms on Altitude, then switches to Position (flight_interface enters
Offboard); goalnav never arms. Needs the rgbd camera pipeline (team_ws/camera/oakd_vslam.yaml).
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
    'image_topic': '/oak/rgb/image_raw',
    'image_info_topic': '/oak/rgb/camera_info',
    'profile': 'safe',
    'tuning': '{cruise_height: 2.25, handover_height: 2.15, return_enabled: false, '
              'pad_disarm: true, heading_until: over_barrier}',
}


def generate_launch_description():
    goalnav = os.path.join(get_package_share_directory('drone_ref'), 'launch', 'goalnav.launch.py')
    return LaunchDescription([IncludeLaunchDescription(PythonLaunchDescriptionSource(goalnav),
                                                       launch_arguments=ARGS.items())])
