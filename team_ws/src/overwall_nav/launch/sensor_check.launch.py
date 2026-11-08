"""Pre-flight sensor check: stereo_depth + sensor_check (prints PASS/WARN/FAIL, then everything
stops). Nothing here commands the drone. Drone on the ground, still and level.

    ros2 launch overwall_nav sensor_check.launch.py
    ros2 launch overwall_nav sensor_check.launch.py camera_pitch_deg:=25.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pitch = ParameterValue(LaunchConfiguration('camera_pitch_deg'), value_type=float)
    depth = Node(package='overwall_nav', executable='stereo_depth', name='stereo_depth', output='screen',
                 emulate_tty=True, parameters=[{'camera_pitch_deg': pitch}])
    check = Node(package='overwall_nav', executable='sensor_check', name='sensor_check', output='screen',
                 emulate_tty=True, parameters=[{'expected_pitch_deg': pitch}])
    return LaunchDescription([
        DeclareLaunchArgument('camera_pitch_deg', default_value='27.0', description='camera mount, deg down'),
        depth, check,
        RegisterEventHandler(OnProcessExit(target_action=check, on_exit=[EmitEvent(event=Shutdown())])),
    ])
