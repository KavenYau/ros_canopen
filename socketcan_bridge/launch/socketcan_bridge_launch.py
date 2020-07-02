import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')

    # Getting directories and launch-files
    socketcan_bridge_dir = get_package_share_directory('socketcan_bridge')

    # Declare the launch arguments
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(socketcan_bridge_dir, 'params', 'socketcan_bridge.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Nodes launching commands
    socketcan_bridge_cmd = Node(
            package='socketcan_bridge',
            node_executable='socketcan_bridge_node',
            output='screen',
            parameters=[params_file])

    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)

    # Running SocketCan Bridge
    ld.add_action(socketcan_bridge_cmd)

    return ld
