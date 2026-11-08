"""The goal navigator, and skybeta's flight interface it flies through.

Staged tests: tuning:='{stop_after: corridor_1}'. flight_interface is the
organizers' node, unmodified, started as their competitor example starts it:
it enters Offboard once when the pilot arms and then switches to Position.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [DeclareLaunchArgument('profile', default_value='safe'),
            DeclareLaunchArgument('tuning', default_value='',
                                  description='YAML overrides, e.g. "{stop_after: first_corridor, max_speed: 0.4}"'),
            DeclareLaunchArgument('flight_interface', default_value='true',
                                  description='also start skybeta_flight_interface (false if another launch does)')]
    interface = Node(package='skybeta_flight_interface', executable='flight_interface', name='flight_interface',
                     output='screen', condition=IfCondition(LaunchConfiguration('flight_interface')),
                     parameters=[{'auto_offboard_on_arm': True}])
    node = Node(package='drone_ref', executable='goalnav_node', name='drone_ref_goalnav', output='screen',
                parameters=[{'profile': LaunchConfiguration('profile'), 'tuning': LaunchConfiguration('tuning')}])
    return LaunchDescription(args + [interface, node])
