import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('gr4_dropzone_mission')
    default_params = os.path.join(pkg_dir, 'config', 'dropzones.yaml')

    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Dropzone mission parameter YAML',
        ),
        Node(
            package='gr4_dropzone_mission',
            executable='dropzone_mission',
            name='dropzone_mission',
            output='screen',
            parameters=[params_file],
        ),
    ])
