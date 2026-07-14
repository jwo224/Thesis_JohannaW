import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def aruco_node(name, image_topic, depth_topic, info_topic, camera_frame, marker_topic, pose_topic, output_topic):
    return Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name=name,
        parameters=[{
            'use_sim_time': True,
            'marker_size': 0.12,
            'aruco_dictionary_id': 'DICT_4X4_50',
            'image_topic': image_topic,
            'use_depth_input': False,
            'depth_image_topic': depth_topic,
            'camera_info_topic': info_topic,
            'camera_frame': camera_frame,
            'detected_markers_topic': marker_topic,
            'markers_visualization_topic': pose_topic,
            'output_image_topic': output_topic,
        }],
        output='screen',
    )


def generate_launch_description():
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    models_dir = os.path.join(pkg_share, 'models')

    lateral_offsets = LaunchConfiguration('lateral_offsets')
    yaw_offsets_deg = LaunchConfiguration('yaw_offsets_deg')
    result_dir = LaunchConfiguration('result_dir')
    csv_filename = LaunchConfiguration('csv_filename')

    xacro_file = os.path.join(pkg_share, 'description', 'mecanum_bot.urdf.xacro')
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': True,
        }],
    )

    world_file = os.path.join(pkg_share, 'worlds', 'nav_test_room.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items(),
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_mecanum_bot',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mecanum_bot',
            '-x', '2.5',
            '-y', '0.85',
            '-z', '0.02',
            '-Y', '0.0',
        ],
        output='screen',
    )

    spawn_charging_zone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_charging_zone',
        arguments=[
            '-entity', 'charging_zone',
            '-file', os.path.join(models_dir, 'ChargingZone', 'model.sdf'),
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    spawn_drop_zone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drop_zone',
        arguments=[
            '-entity', 'drop_zone',
            '-file', os.path.join(models_dir, 'DropZone', 'model.sdf'),
            '-x', '2.5',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    spawn_orange_drop_zone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_orange_drop_zone',
        arguments=[
            '-entity', 'orange_drop_zone',
            '-file', os.path.join(models_dir, 'OrangeDropZone', 'model.sdf'),
            '-x', '5.2',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    spawn_trolley = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_trolley',
        arguments=[
            '-entity', 'Trolley',
            '-file', os.path.join(models_dir, 'Trolley', 'model.sdf'),
            '-x', '2.5',
            '-y', '0.0',
            '-z', '0.0',
            '-Y', '0.0',
        ],
        output='screen',
    )

    front_aruco = aruco_node(
        'front_trolley_aruco_node',
        '/camera_front/image_raw',
        '/camera_front/depth/image_raw',
        '/camera_front/camera_info',
        'camera_front_optical_link',
        '/front/aruco_markers',
        '/front/aruco_poses',
        '/front/aruco_image',
    )
    rear_aruco = aruco_node(
        'rear_trolley_aruco_node',
        '/camera_rear/image_raw',
        '/camera_rear/depth/image_raw',
        '/camera_rear/camera_info',
        'camera_rear_optical_link',
        '/rear/aruco_markers',
        '/rear/aruco_poses',
        '/rear/aruco_image',
    )
    left_aruco = aruco_node(
        'left_trolley_aruco_node',
        '/camera_left/image_raw',
        '/camera_left/depth/image_raw',
        '/camera_left/camera_info',
        'camera_left_optical_link',
        '/left/aruco_markers',
        '/left/aruco_poses',
        '/left/aruco_image',
    )
    right_aruco = aruco_node(
        'right_trolley_aruco_node',
        '/camera_right/image_raw',
        '/camera_right/depth/image_raw',
        '/camera_right/camera_info',
        'camera_right_optical_link',
        '/right/aruco_markers',
        '/right/aruco_poses',
        '/right/aruco_image',
    )

    test_runner = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share, 'scripts', 'aruco_docking_test_runner.py'),
            '--ros-args',
            '-p', 'use_sim_time:=True',
            '-p', ['lateral_offsets:=[', lateral_offsets, ']'],
            '-p', ['yaw_offsets_deg:=[', yaw_offsets_deg, ']'],
            '-p', ['result_dir:=', result_dir],
            '-p', ['csv_filename:=', csv_filename],
            '-p', 'robot_entity:=mecanum_bot',
            '-p', 'trolley_entity:=Trolley',
            '-p', 'trolley_x:=2.5',
            '-p', 'trolley_y:=0.0',
            '-p', 'trolley_yaw_deg:=0.0',
            '-p', 'robot_start_x:=2.5',
            '-p', 'robot_start_y:=0.85',
            '-p', 'robot_start_yaw_deg:=0.0',
            '-p', 'entity_wait_timeout:=20.0',
            '-p', 'entity_service_timeout:=3.0',
            '-p', 'reset_retries:=5',
            '-p', 'min_aruco_markers:=3',
            '-p', 'entry_offset_x:=0.015',
            '-p', 'side_align_x_tolerance:=0.020',
            '-p', 'side_align_yaw_tolerance_deg:=2.0',
            '-p', 'skip_side_align_if_already_close:=True',
            '-p', 'fine_position_tolerance:=0.010',
            '-p', 'fine_yaw_tolerance_deg:=1.0',
            '-p', 'x_control_sign:=1.0',
            '-p', 'y_control_sign:=1.0',
            '-p', 'yaw_control_sign:=1.0',
            '-p', 'straight_under_stop_y:=0.16',
            '-p', 'fine_max_vx:=0.035',
            '-p', 'fine_max_vy:=0.035',
            '-p', 'fine_max_wz:=0.10',
            '-p', 'straight_under_speed:=0.16',
            '-p', 'side_align_timeout:=30.0',
            '-p', 'straight_under_timeout:=20.0',
            '-p', 'final_align_timeout:=45.0',
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('lateral_offsets', default_value='0.0,0.05,-0.05,0.10,-0.10'),
        DeclareLaunchArgument('yaw_offsets_deg', default_value='0,3,-3,6,-6'),
        DeclareLaunchArgument(
            'result_dir',
            default_value='/home/group4/Nursing-Home-Robot/aruco_docking_test_results',
        ),
        DeclareLaunchArgument('csv_filename', default_value='aruco_docking_test.csv'),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[models_dir, ':', EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')],
        ),
        gazebo,
        robot_state_publisher,
        spawn_charging_zone,
        spawn_drop_zone,
        spawn_orange_drop_zone,
        spawn_robot,
        spawn_trolley,
        TimerAction(period=3.0, actions=[front_aruco, rear_aruco, left_aruco, right_aruco]),
        TimerAction(period=6.0, actions=[test_runner]),
    ])
