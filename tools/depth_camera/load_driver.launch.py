"""Load the OAK driver component into the stack's RUNNING /oak_container (used by driver_swap.py).

Mirrors the ComposableNode in depthai_ros_driver_v3's driver.launch.py (v3.3.0-humble): same package, plugin,
node name 'oak', empty namespace and the same points remapping. Its parameters come from one file: the running
driver's own dump plus a checked overlay (driver_params.py), which already carries the TF parameters that
driver.launch.py passes in. Loading into the existing container means no process starts or exits, so the stack's
slam pane is untouched. One-shot: the launch exits once the load service call returns.

    ros2 launch load_driver.launch.py params_file:=/root/team_ws/depth_state/candidate_oak_params.yaml
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('params_file', description='parameter file for node /oak'),
        DeclareLaunchArgument('container', default_value='/oak_container'),
        LoadComposableNodes(
            target_container=LaunchConfiguration('container'),
            composable_node_descriptions=[ComposableNode(
                package='depthai_ros_driver_v3',
                plugin='depthai_ros_driver::Driver',
                name='oak',
                namespace='',
                parameters=[LaunchConfiguration('params_file')],
                remappings=[('oak/rgbd/points', 'oak/rgbd/points')],
            )],
        ),
    ])
