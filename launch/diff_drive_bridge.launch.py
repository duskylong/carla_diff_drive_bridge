from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('carla_diff_drive_bridge')
    params_file = os.path.join(pkg_dir, 'config', 'bridge_params.yaml')

    return LaunchDescription([
        Node(
            package='carla_diff_drive_bridge',
            executable='bridge_node',
            name='carla_diff_drive_bridge',
            parameters=[params_file],
            output='screen',
        ),
    ])
