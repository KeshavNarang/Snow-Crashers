"""Sensor check only: stereo_depth, no flight_interface, no mission. Nothing here can fly the
drone (without flight_interface there is no takeover at all), so it is safe to arm-test with.

    ~/team_ws/run_mission.sh overwall_nav depth_test.launch.py      then power-cycle, or by hand:
    ros2 launch overwall_nav depth_test.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='overwall_nav', executable='stereo_depth', name='stereo_depth',
             output='screen', emulate_tty=True),
    ])
