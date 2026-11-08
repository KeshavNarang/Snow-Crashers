"""flight_interface (exactly as overwall_nav's overwall.launch.py) + the goal navigator + a logger.

What the stack runs with COMPETITOR_PKG=drone_ref COMPETITOR_LAUNCH=goalnav.launch.py
(~/team_ws/run_mission.sh drone_ref goalnav.launch.py, then power-cycle). camera:=rgbstereo (default)
matches the team camera config (team_ws/camera/oakd_vslam.yaml: colour + the mono pair, no on-camera
depth): stereo_node makes depth from the pair and the pad is found in colour. camera:=rgbd uses the
OAK-D's own depth image instead (it cost PX4 its VIO fusion on skybeta-03).
Staged tests: tuning:='{stop_after: corridor_1}'. The pilot arms on Altitude, then switches to Position.

Every launch logs to ``log_dir/<date-time>/`` (default ``~/team_ws/logs``, the drone's one folder that
survives a stack restart or power cycle): ``bag/`` (MCAP), ``ros/`` (every node's log), ``run.json``.
Pull it with ``tools/drone pull team_ws/logs/<date-time> <here>``.
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
from launch_ros.parameter_descriptions import ParameterValue

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

# goalnav's own arguments.
GOALNAV = [
    ('profile', 'safe', 'risk profile, config/goalnav_profiles.yaml'),
    ('goals_file', 'config/goals.yaml', 'the goal chain (relative: installed share/drone_ref)'),
    ('tuning', '', 'YAML overrides, e.g. "{stop_after: corridor_1, max_speed: 0.4}"'),
    ('log_dir', '~/team_ws/logs', 'one folder per launch is made in here'),
    ('record', 'true', 'record the rosbag'),
    ('record_colour', 'false', 'include the colour images (pad debugging, ~8 MB/s)'),
    ('record_video', 'false', 'include both mono camera streams in it (~40 MB/s)'),
    ('camera', 'rgbstereo', 'camera pipeline: rgbstereo (team config) or rgbd (on-camera depth)'),
]

# What each camera pipeline gives goalnav (goalnav_node PARAMS) and whether our stereo node runs.
CAMERA_MODES = {
    'rgbstereo': {'stereo_node': True,
                  'goalnav': {'depth_topic': '', 'points_topic': '/goalnav/stereo/points', 'image_topic': '',
                              'pad_image_topic': '/oak/rgb/image_raw', 'pad_info_topic': '/oak/rgb/camera_info'}},
    'rgbd': {'stereo_node': False,
             'goalnav': {'depth_topic': '/oak/stereo/image_raw', 'points_topic': '',
                         'pad_image_topic': '/oak/rgb/image_raw', 'pad_info_topic': '/oak/rgb/camera_info'}},
}

VIDEO = ['/oak/left/image_mono', '/oak/right/image_mono']
COLOUR = ['/oak/rgb/image_raw']                  # record_colour:=true for pad debugging (~8 MB/s)
# stereo_node: depth at most this often. 5-8 Hz measured on skybeta-03; the sim plans with 2.8 Hz;
# fewer frames leave the cores to cuVSLAM (vision was late with goalnav running, 2026-11-08).
STEREO = {'rate_hz': 4.0}
TOPICS = ['/oak/left/camera_info', '/oak/right/camera_info', '/oak/imu/data',
          '/oak/stereo/image_raw', '/oak/stereo/camera_info', '/oak/rgb/camera_info',
          '/visual_slam/tracking/odometry', '/visual_slam/status',
          '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude', '/fmu/out/vehicle_status_v1',
          '/fmu/out/vehicle_land_detected', '/fmu/out/vehicle_control_mode', '/fmu/out/estimator_status_flags',
          '/fmu/out/failsafe_flags', '/fmu/out/battery_status',
          '/skybeta/state', '/skybeta/setpoint/twist', '/skybeta/setpoint/pose',
          '/goalnav/status', '/goalnav/stereo/points', '/rosout', '/tf', '/tf_static']


def _logger(context):
    """This run's log folder, ROS_LOG_DIR into it, and the rosbag recorder."""
    get = lambda name: LaunchConfiguration(name).perform(context)        # noqa: E731
    run = os.path.join(os.path.expanduser(get('log_dir')), datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(os.path.join(run, 'ros'), exist_ok=True)
    with open(os.path.join(run, 'run.json'), 'w') as handle:
        json.dump({'started': datetime.now().isoformat(timespec='seconds'), 'host': socket.gethostname(),
                   'args': {name: get(name) for name, _, _ in INTERFACE + GOALNAV}}, handle, indent=1)
    topics = TOPICS + (VIDEO if get('record_video') == 'true' else []) + (COLOUR if get('record_colour') == 'true' else [])
    record = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--max-bag-size', str(2 << 30),
                                 '-o', os.path.join(run, 'bag')] + topics,
                            output='both', condition=IfCondition(LaunchConfiguration('record')))
    return [SetEnvironmentVariable('ROS_LOG_DIR', os.path.join(run, 'ros')), record]


def generate_launch_description():
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in INTERFACE + GOALNAV]
    flight_interface = Node(
        package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
        output='screen', emulate_tty=True,
        parameters=[{name: LaunchConfiguration(name) for name, _, _ in INTERFACE}],
    )
    # The logger first: ROS_LOG_DIR must be set before the nodes start so they log into this run's folder.
    return LaunchDescription(declared + [OpaqueFunction(function=_logger), flight_interface,
                                         OpaqueFunction(function=_navigation)])


def _navigation(context):
    """goalnav_node fed as the camera pipeline allows, and our stereo node when there is no camera depth."""
    camera = LaunchConfiguration('camera').perform(context)
    if camera not in CAMERA_MODES:
        raise RuntimeError(f"camera:={camera}: use one of {sorted(CAMERA_MODES)}")
    mode = CAMERA_MODES[camera]
    goalnav = Node(package='drone_ref', executable='goalnav_node', name='drone_ref_goalnav',
                   output='screen', emulate_tty=True,
                   # value_type=str: launch would otherwise read tuning's '{...}' as a YAML dict.
                   parameters=[{**{name: ParameterValue(LaunchConfiguration(name), value_type=str)
                                   for name in ('profile', 'tuning', 'goals_file')}, **mode['goalnav']}])
    stereo = Node(package='drone_ref', executable='stereo_node', name='drone_ref_stereo',
                  output='screen', emulate_tty=True, parameters=[STEREO])
    return [stereo, goalnav] if mode['stereo_node'] else [goalnav]
