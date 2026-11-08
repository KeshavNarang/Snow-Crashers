"""manual.launch.py without the camera images (when the drone is short of disk: ~1 MB/s instead of ~50).

Select it:  ~/team_ws/run_mission.sh manual_flight manual_novideo.launch.py (tools/manual-flight setup
--no-video), then power-cycle. Same recorder, same folder, still NO flight_interface and no mission.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    full = os.path.join(get_package_share_directory('manual_flight'), 'launch', 'manual.launch.py')
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(full),
                                 launch_arguments={'record_video': 'false'}.items()),
    ])
