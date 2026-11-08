"""flight_interface (exactly as the organizers' example) + stereo_depth + pad_detect + the mission.

pad_detect needs the colour camera; with the stock camera config it publishes nothing and the
mission lands at land_beyond_wall_m as before.

What the stack runs with COMPETITOR_PKG=overwall_nav COMPETITOR_LAUNCH=overwall.launch.py
(~/team_ws/run_mission.sh overwall_nav overwall.launch.py). The test settings (wall height,
backstop distance, camera angle ...) are in config/overwall.yaml: edit that, then power-cycle.

Every run is logged to ~/team_ws/logs/overwall/<UTC date-time>/ on the drone (bind-mounted, so it
survives the container and a power-off): every node's log file (ROS_LOG_DIR), a small MCAP bag of the
mission's topics (no images), and pad_detect's 1 Hz colour snapshots. record:=false turns the bag off.
These are for reading after a run, never inputs to a later run (rule 6.4).
"""
import os
import time

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
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


LOG_ROOT = '/root/team_ws/logs/overwall'
# Small topics only (~30 kB/s): what the mission saw, decided and commanded. PX4's view (armed, mode,
# landed, local position) is in /skybeta/state: on skybeta-03 `ros2 bag record` never discovered the
# /fmu/out/* topics (tried with best-effort QoS overrides too), so they are not listed here.
BAG_TOPICS = ['/overwall/front', '/overwall/pad', '/overwall/remote', '/skybeta/state', '/skybeta/setpoint/pose',
              '/skybeta/setpoint/twist', '/visual_slam/tracking/odometry', '/rosout']


def generate_launch_description():
    settings = os.path.join(get_package_share_directory('overwall_nav'), 'config', 'overwall.yaml')
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in INTERFACE]
    declared.append(DeclareLaunchArgument('record', default_value='true', description='record the MCAP bag'))
    run_dir = os.path.join(LOG_ROOT, time.strftime('%Y%m%d-%H%M%S', time.gmtime()))
    os.makedirs(run_dir, exist_ok=True)
    logging = [
        SetEnvironmentVariable('ROS_LOG_DIR', run_dir),            # every node's log file goes here
        # No write cache (default 100 MB in memory): a power-off lost the whole flight's bag.
        ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--max-cache-size', '0',
                            '-o', os.path.join(run_dir, 'bag')] + BAG_TOPICS,
                       output='log', condition=IfCondition(LaunchConfiguration('record'))),
    ]
    flight_interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in INTERFACE}],
    )
    depth = Node(package='overwall_nav', executable='stereo_depth', name='stereo_depth',
                 output='screen', emulate_tty=True, parameters=[settings])
    pad = Node(package='overwall_nav', executable='pad_detect', name='pad_detect',
               output='screen', emulate_tty=True, parameters=[settings])
    remote = Node(package='overwall_nav', executable='remote_link', name='remote_link',
                  output='screen', emulate_tty=True, parameters=[settings])
    mission = Node(package='overwall_nav', executable='overwall_mission', name='overwall_mission',
                   output='screen', emulate_tty=True, parameters=[settings])
    return LaunchDescription(declared + logging + [flight_interface, depth, pad, remote, mission])
