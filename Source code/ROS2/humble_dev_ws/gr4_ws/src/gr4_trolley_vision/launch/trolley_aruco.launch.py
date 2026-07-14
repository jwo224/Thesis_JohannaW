import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_config(path):
    with open(path, 'r', encoding='utf-8') as f:
        return yaml.safe_load(f)


def launch_setup(context, *args, **kwargs):
    config_file = LaunchConfiguration('config_file').perform(context)
    launch_cameras = LaunchConfiguration('launch_cameras')
    launch_monitor = LaunchConfiguration('launch_monitor')
    config = load_config(config_file)

    camera_defaults = config.get('camera', {})
    actions = []

    for name in ('front', 'rear', 'left', 'right'):
        camera = config['cameras'][name]

        actions.append(
            Node(
                package='gr4_trolley_vision',
                executable='opencv_camera_node',
                name=f'{name}_camera',
                namespace='',
                condition=IfCondition(launch_cameras),
                output='screen',
                parameters=[{
                    'device': camera['device'],
                    'camera_name': name,
                    'frame_id': camera['frame_id'],
                    'image_topic': camera['image_topic'],
                    'camera_info_topic': camera['camera_info_topic'],
                    'width': int(camera_defaults.get('width', 640)),
                    'height': int(camera_defaults.get('height', 480)),
                    'fps': float(camera_defaults.get('fps', 15.0)),
                    'camera_fx': float(camera_defaults.get('fx', 615.0)),
                    'camera_fy': float(camera_defaults.get('fy', 615.0)),
                    'camera_cx': float(camera_defaults.get('cx', 320.0)),
                    'camera_cy': float(camera_defaults.get('cy', 240.0)),
                }],
            )
        )

        actions.append(
            Node(
                package='aruco_pose_estimation',
                executable='aruco_node.py',
                name=f'{name}_trolley_aruco_node',
                output='screen',
                parameters=[{
                    'marker_size': float(camera['marker_size']),
                    'aruco_dictionary_id': 'DICT_4X4_50',
                    'image_topic': camera['image_topic'],
                    'use_depth_input': False,
                    'camera_info_topic': camera['camera_info_topic'],
                    'camera_frame': camera['frame_id'],
                    'detected_markers_topic': camera['aruco_topic'],
                    'markers_visualization_topic': camera['aruco_poses_topic'],
                    'output_image_topic': camera['aruco_image_topic'],
                }],
            )
        )

    actions.append(
        Node(
            package='gr4_trolley_vision',
            executable='aruco_detection_monitor',
            name='aruco_detection_monitor',
            condition=IfCondition(launch_monitor),
            output='screen',
        )
    )
    return actions


def generate_launch_description():
    pkg_share = get_package_share_directory('gr4_trolley_vision')
    default_config = os.path.join(pkg_share, 'config', 'trolley_vision.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=default_config,
            description='Real robot trolley ArUco camera configuration.',
        ),
        DeclareLaunchArgument(
            'launch_cameras',
            default_value='true',
            description='Start OpenCV /dev/video camera publishers. Set false if another camera driver is already publishing.',
        ),
        DeclareLaunchArgument(
            'launch_monitor',
            default_value='true',
            description='Print detected marker IDs for a quick test.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
