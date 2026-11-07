"""First real flight: the same mission, small. competitor.launch.py with height 0.5 m, forward 0.5 m.

Select it with COMPETITOR_LAUNCH=low_test.launch.py in /etc/skybeta/drone.env.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    full = os.path.join(get_package_share_directory('snowcrashers_nav'), 'launch', 'competitor.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(full),
                                 launch_arguments={'height_m': '0.5', 'forward_m': '0.5'}.items()),
    ])
