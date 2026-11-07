"""The goal navigator. Staged tests: tuning:='{stop_after: corridor_1}'."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument('profile', default_value='safe'),
            DeclareLaunchArgument('tuning', default_value='',
                                  description='YAML overrides, e.g. "{stop_after: first_corridor, max_speed: 0.4}"')]
    node = Node(package='drone_ref', executable='goalnav_node', name='drone_ref_goalnav', output='screen',
                parameters=[{'profile': LaunchConfiguration('profile'), 'tuning': LaunchConfiguration('tuning')}])
    return LaunchDescription(args + [node])
