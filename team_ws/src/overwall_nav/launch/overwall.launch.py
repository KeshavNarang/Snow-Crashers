"""flight_interface (exactly as the organizers' example) + stereo_depth + pad_detect + the mission.

pad_detect needs the colour camera; with the stock camera config it publishes nothing and the
mission lands at land_beyond_wall_m as before.

What the stack runs with COMPETITOR_PKG=overwall_nav COMPETITOR_LAUNCH=overwall.launch.py
(~/team_ws/run_mission.sh overwall_nav overwall.launch.py). The test settings (wall height,
backstop distance, camera angle ...) are in config/overwall.yaml: edit that, then power-cycle.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# The flight interface's arguments: the organizers' example, but the altitude clamp lowered to 2.5 m for the net.
INTERFACE = [
    ('max_altitude_m', '2.5', 'ceiling above the odometry origin, metres (net top 9.5 ft = 2.9 m)'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0', 'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false', 'SIM ONLY. On the drone Offboard follows arm, then Position'),
    ('auto_offboard_on_arm', 'true', 'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry', 'ENU pose the altitude band is measured against'),
]


def generate_launch_description():
    settings = os.path.join(get_package_share_directory('overwall_nav'), 'config', 'overwall.yaml')
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in INTERFACE]
    flight_interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in INTERFACE}],
    )
    depth = Node(package='overwall_nav', executable='stereo_depth', name='stereo_depth',
                 output='screen', emulate_tty=True, parameters=[settings])
    pad = Node(package='overwall_nav', executable='pad_detect', name='pad_detect',
               output='screen', emulate_tty=True, parameters=[settings])
    mission = Node(package='overwall_nav', executable='overwall_mission', name='overwall_mission',
                   output='screen', emulate_tty=True, parameters=[settings])
    return LaunchDescription(declared + [flight_interface, depth, pad, mission])
