"""Hand-flown flight, fully recorded. Runs NO flight_interface and NO mission: only a rosbag recorder.

Why it matters: overwall.launch.py and goalnav.launch.py start flight_interface with
auto_offboard_on_arm, so arming and switching to Position hands the drone to the mission. With this
launch selected nothing can take the drone from the pilot. The organizers' stack (agent, cuVSLAM,
the VIO bridge into PX4) still runs as always, so Position mode holds as usual.

Select it:  ~/team_ws/run_mission.sh manual_flight manual.launch.py, then power-cycle
(tools/manual-flight setup does both). Steps for people: MANUAL_FLIGHT.md.

Each power-up records into ``log_dir/<date-time>/`` (default ``~/team_ws/logs``, the folder
``tools/fly logs`` pulls): ``bag/`` (MCAP), ``ros/`` (node logs), ``run.json``. PX4's own flight log
on its SD card (sticks, motor outputs, everything at full rate) is separate: QGC -> Log Download.
"""
from datetime import datetime
import json
import os
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration

ARGS = [
    ('log_dir', '~/team_ws/logs', 'one folder per launch is made in here (tools/fly logs reads it)'),
    ('record_video', 'true', 'include the camera images (~50 MB/s, ~3 GB a minute; false: the rest only)'),
]

# What the pilot did and what the drone did, as PX4 publishes it (names as on skybeta-03, PX4 v1.16;
# a listed topic that is not there is simply not recorded).
PX4 = ['/fmu/out/manual_control_setpoint',            # the pilot's sticks and switches
       '/fmu/out/vehicle_status_v1', '/fmu/out/vehicle_control_mode', '/fmu/out/vehicle_land_detected',
       '/fmu/out/vehicle_local_position', '/fmu/out/vehicle_odometry', '/fmu/out/vehicle_attitude',
       '/fmu/out/vehicle_global_position', '/fmu/out/sensor_combined',
       '/fmu/out/vehicle_thrust_setpoint', '/fmu/out/vehicle_torque_setpoint',
       '/fmu/out/estimator_status_flags', '/fmu/out/failsafe_flags', '/fmu/out/battery_status',
       '/fmu/out/timesync_status']
# What the drone saw: cameras' calibration, IMU, cuVSLAM, what the organizers' bridge sends PX4, frames.
SENSING = ['/oak/left/camera_info', '/oak/right/camera_info', '/oak/rgb/camera_info', '/oak/stereo/camera_info',
           '/oak/imu/data', '/visual_slam/tracking/odometry', '/visual_slam/status',
           '/fmu/in/vehicle_visual_odometry', '/tf', '/tf_static', '/rosout']
# Images, as goalnav.launch.py records them: the mono pair (cuVSLAM's view, ~40 MB/s), colour, and on-camera
# depth when the camera makes it (camera rgbd). Never the .../compressed topics: subscribing to those makes
# the driver compress every frame (CPU cuVSLAM needs). No video: manual_novideo.launch.py.
VIDEO = ['/oak/left/image_mono', '/oak/right/image_mono', '/oak/rgb/image_raw', '/oak/stereo/image_raw']


def _logger(context):
    """This power-up's log folder, ROS_LOG_DIR into it, and the recorder."""
    get = lambda name: LaunchConfiguration(name).perform(context)        # noqa: E731
    run = os.path.join(os.path.expanduser(get('log_dir')), datetime.now().strftime('%Y%m%d-%H%M%S'))
    os.makedirs(os.path.join(run, 'ros'), exist_ok=True)
    with open(os.path.join(run, 'run.json'), 'w') as handle:
        json.dump({'mission': 'manual_flight', 'started': datetime.now().isoformat(timespec='seconds'),
                   'host': socket.gethostname(), 'args': {name: get(name) for name, _, _ in ARGS}},
                  handle, indent=1)
    topics = PX4 + SENSING + (VIDEO if get('record_video') == 'true' else [])
    # --max-cache-size 0: straight to disk (overwall lost a whole bag to a power-off with the 100 MB cache).
    record = ExecuteProcess(cmd=['ros2', 'bag', 'record', '-s', 'mcap', '--max-cache-size', '0',
                                 '--max-bag-size', str(2 << 30), '-o', os.path.join(run, 'bag')] + topics,
                            output='both')
    return [SetEnvironmentVariable('ROS_LOG_DIR', os.path.join(run, 'ros')), record]


def generate_launch_description():
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in ARGS]
    return LaunchDescription(declared + [OpaqueFunction(function=_logger)])
