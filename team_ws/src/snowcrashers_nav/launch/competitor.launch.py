"""flight_interface (exactly as competitor_example/launch/competitor.launch.py) plus our node.

What the stack's `competitor` window runs once /etc/skybeta/drone.env says
COMPETITOR_PKG=snowcrashers_nav and COMPETITOR_LAUNCH=competitor.launch.py.

    ros2 launch snowcrashers_nav competitor.launch.py
    ros2 launch snowcrashers_nav competitor.launch.py height_m:=1.0 forward_m:=1.0
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# The flight interface's arguments, unchanged from the organizers' example.
INTERFACE = [
    ('max_altitude_m', '3.0', 'ceiling above the odometry origin, metres'),
    ('min_altitude_m', '0.3', 'floor above the odometry origin, metres'),
    ('max_speed_mps', '1.5', 'cap on the norm of a twist setpoint, m/s'),
    ('setpoint_rate_hz', '10.0', 'OffboardControlMode heartbeat rate -- PX4 needs > 2 Hz'),
    ('allow_mode_request', 'false', 'SIM ONLY. On the drone Offboard follows arm, then Position'),
    ('auto_offboard_on_arm', 'true', 'enter Offboard once when the pilot arms, then switches to Position'),
    ('odometry_topic', '/visual_slam/tracking/odometry', 'ENU pose the altitude band is measured against'),
]
# Ours.
MISSION = [
    ('height_m', '2.0', 'climb this far above the start, metres'),
    ('forward_m', '2.0', 'then fly this far along the nose direction at the start, metres'),
    ('climb_speed_mps', '0.5', 'climb rate of the target, m/s'),
    ('forward_speed_mps', '0.4', 'cruise speed of the target, m/s'),
    ('descent_slow_mps', '0.4', 'final descent, m/s (>= 0.3, or PX4 never detects the landing)'),
    ('competitor', 'true', 'also start our node (false: the flight interface only)'),
]


def generate_launch_description():
    declared = [DeclareLaunchArgument(n, default_value=d, description=h) for n, d, h in INTERFACE + MISSION]

    flight_interface = Node(
        package='skybeta_flight_interface',
        executable='flight_interface',
        name='flight_interface',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'max_altitude_m': LaunchConfiguration('max_altitude_m'),
            'min_altitude_m': LaunchConfiguration('min_altitude_m'),
            'max_speed_mps': LaunchConfiguration('max_speed_mps'),
            'setpoint_rate_hz': LaunchConfiguration('setpoint_rate_hz'),
            'allow_mode_request': LaunchConfiguration('allow_mode_request'),
            'auto_offboard_on_arm': LaunchConfiguration('auto_offboard_on_arm'),
            'odometry_topic': LaunchConfiguration('odometry_topic'),
        }],
    )

    mission = Node(
        package='snowcrashers_nav',
        executable='up_forward_land',
        name='up_forward_land',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('competitor')),
        parameters=[{
            name: ParameterValue(LaunchConfiguration(name), value_type=float)
            for name, _, _ in MISSION if name != 'competitor'
        }],
    )

    return LaunchDescription(declared + [flight_interface, mission])
