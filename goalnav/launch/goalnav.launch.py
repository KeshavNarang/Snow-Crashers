"""The goal navigator, the stereo node it sees with, skybeta's flight interface, and a logger.

Staged tests: tuning:='{stop_after: corridor_1}'. flight_interface is the organizers'
node, unmodified, started as their competitor example starts it: it enters Offboard
once when the pilot arms and then switches to Position.

Every launch logs to ``log_dir/<date-time>/`` (default ``~/team_ws/logs``: in the stack's
container that is the drone's ~/team_ws, the one folder that survives a stack restart
or a power cycle; never /tmp):

- ``bag/``    rosbag2 (MCAP, split every 2 GB): both mono camera streams (the video), the
              IMU and camera calibration, goalnav's stereo points and image, cuVSLAM, PX4's
              state, /skybeta/state and every setpoint, /goalnav/status and /rosout;
- ``ros/``    ROS_LOG_DIR: every node's log and its screen output;
- ``run.json`` the launch arguments, host and start time.

Pull it to a laptop with ``tools/drone pull team_ws/logs/<date-time> <here>``.
"""
from datetime import datetime
import json
import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

ARGS = {'profile': ('safe', 'risk profile, config/goalnav_profiles.yaml'),
        'tuning': ('', 'YAML overrides, e.g. "{stop_after: first_corridor, max_speed: 0.4}"'),
        'flight_interface': ('true', 'also start skybeta_flight_interface (false if another launch does)'),
        'log_dir': ('~/team_ws/logs', 'one folder per launch is made in here'),
        'record': ('true', 'record the rosbag'),
        'record_video': ('true', 'include both mono camera streams in it (~40 MB/s)')}

VIDEO = ['/oak/left/image_mono', '/oak/right/image_mono']
TOPICS = ['/oak/left/camera_info', '/oak/right/camera_info', '/oak/imu/data',
          '/goalnav/stereo/points', '/goalnav/stereo/left', '/goalnav/stereo/left_info',
          '/visual_slam/tracking/odometry', '/visual_slam/status',
          '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude', '/fmu/out/vehicle_status_v1',
          '/fmu/out/vehicle_land_detected', '/fmu/out/vehicle_control_mode', '/fmu/out/estimator_status_flags',
          '/fmu/out/failsafe_flags', '/fmu/out/battery_status',
          '/skybeta/state', '/skybeta/setpoint/twist', '/skybeta/setpoint/pose',
          '/goalnav/status', '/rosout', '/tf', '/tf_static']


def _run(context):
    get = lambda name: LaunchConfiguration(name).perform(context)        # noqa: E731
    run = os.path.join(os.path.expanduser(get('log_dir')), datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(os.path.join(run, 'ros'), exist_ok=True)
    with open(os.path.join(run, 'run.json'), 'w') as handle:
        json.dump({'started': datetime.now().isoformat(timespec='seconds'), 'host': socket.gethostname(),
                   'args': {name: get(name) for name in ARGS}}, handle, indent=1)
    topics = TOPICS + (VIDEO if get('record_video') == 'true' else [])
    record = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--max-bag-size', str(2 << 30),
                                 '-o', os.path.join(run, 'bag')] + topics,
                            output='both', condition=IfCondition(LaunchConfiguration('record')))
    interface = Node(package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
                     output='both', condition=IfCondition(LaunchConfiguration('flight_interface')),
                     parameters=[{'auto_offboard_on_arm': True}])
    stereo = Node(package='drone_ref', executable='stereo_node', name='goalnav_stereo', output='both')
    node = Node(package='drone_ref', executable='goalnav_node', name='drone_ref_goalnav', output='both',
                parameters=[{'profile': LaunchConfiguration('profile'), 'tuning': LaunchConfiguration('tuning')}])
    # ROS_LOG_DIR first: every process started after it logs into this run's folder.
    return [SetEnvironmentVariable('ROS_LOG_DIR', os.path.join(run, 'ros')), record, interface, stereo, node]


def generate_launch_description():
    declared = [DeclareLaunchArgument(name, default_value=default, description=text)
                for name, (default, text) in ARGS.items()]
    return LaunchDescription(declared + [OpaqueFunction(function=_run)])
