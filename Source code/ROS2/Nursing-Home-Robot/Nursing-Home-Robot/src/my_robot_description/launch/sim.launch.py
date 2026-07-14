import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PythonExpression

from launch_ros.actions import Node


def generate_launch_description():
    mission_step = LaunchConfiguration('mission_step')

    # -----------------------------
    # Package
    # -----------------------------
    pkg_name = 'my_robot_description'
    pkg_share = get_package_share_directory(pkg_name)
    pkg_prefix = os.path.dirname(os.path.dirname(pkg_share))
    models_dir = os.path.join(pkg_share, 'models')

    # -----------------------------
    # Robot Description (XACRO)
    # -----------------------------
    xacro_file = os.path.join(
        pkg_share,
        'description',
        'mecanum_bot.urdf.xacro'
    )

    robot_description_raw = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description_raw,
                'use_sim_time': True
            }
        ]
    )

    # -----------------------------
    # Gazebo World with State Plugin
    # -----------------------------
    world_file = os.path.join(
        pkg_share,
        'worlds',
        'nav_test_room.world'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            )
        ),
        launch_arguments={
            'world': world_file
        }.items()
    )

    robot_x = PythonExpression([
        "'2.5' if '", mission_step, "' in ['step1', 'step2', 'step3'] else '0.0'"
    ])
    robot_y = PythonExpression([
        "'0.85' if '", mission_step, "' == 'step1' else "
        "('0.0' if '", mission_step, "' in ['step2', 'step3'] else '0.0')"
    ])
    robot_yaw = '0.0'
    launch_aruco = PythonExpression([
        "'", mission_step, "' in ['step0', 'step1', 'step2']"
    ])

    # -----------------------------
    # Spawn Robot in Gazebo
    # -----------------------------
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_mecanum_bot',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mecanum_bot',
            '-x', robot_x,
            '-y', robot_y,
            '-z', '0.02',
            '-Y', robot_yaw
        ],
        output='screen'
    )

    home_x = '0.0'
    home_y = '0.0'
    home_yaw = '0.0'
    drop_zone_x = '2.5'
    drop_zone_y = '0.0'
    drop_zone_yaw = '0.0'
    orange_drop_zone_x = '5.2'
    orange_drop_zone_y = '0.0'
    orange_drop_zone_yaw = '0.0'

    # -----------------------------
    # Spawn robot origin / charging marker in Gazebo
    # -----------------------------
    spawn_charging_zone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_charging_zone',
        arguments=[
            '-entity', 'charging_zone',
            '-file', os.path.join(
                models_dir,
                'ChargingZone',
                'model.sdf'
            ),
            '-x', home_x,
            '-y', home_y,
            '-z', '0.0',
            '-Y', home_yaw
        ],
        output='screen'
    )

    # -----------------------------
    # Spawn blue trolley drop zone in Gazebo
    # Dimensions: 1060 mm x 615 mm.
    # -----------------------------
    spawn_drop_zone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_drop_zone',
        arguments=[
            '-entity', 'drop_zone',
            '-file', os.path.join(
                models_dir,
                'DropZone',
                'model.sdf'
            ),
            '-x', drop_zone_x,
            '-y', drop_zone_y,
            '-z', '0.0',
            '-Y', drop_zone_yaw
        ],
        output='screen'
    )

    # -----------------------------
    # Spawn orange trolley delivery drop zone in Gazebo
    # Dimensions: 1060 mm x 615 mm.
    # -----------------------------
    spawn_orange_drop_zone = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_orange_drop_zone',
        arguments=[
            '-entity', 'orange_drop_zone',
            '-file', os.path.join(
                models_dir,
                'OrangeDropZone',
                'model.sdf'
            ),
            '-x', orange_drop_zone_x,
            '-y', orange_drop_zone_y,
            '-z', '0.0',
            '-Y', orange_drop_zone_yaw
        ],
        output='screen'
    )

    # -----------------------------
    # Spawn Trolley in Gazebo
    # Trolley starts centered on the drop zone.
    # -----------------------------
    spawn_trolley = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_trolley',
        arguments=[
            '-entity', 'Trolley',
            '-file', os.path.join(
                models_dir,
                'Trolley',
                'model.sdf'
            ),
            '-x', drop_zone_x,
            '-y', drop_zone_y,
            '-z', '0.0',
            '-Y', drop_zone_yaw
        ],
        output='screen'
    )

    front_aruco = Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name='front_trolley_aruco_node',
        condition=IfCondition(launch_aruco),
        parameters=[{
            'use_sim_time': True,
            'marker_size': 0.08,
            'aruco_dictionary_id': 'DICT_4X4_50',
            'image_topic': '/camera_front/image_raw',
            'use_depth_input': False,
            'depth_image_topic': '/camera_front/depth/image_raw',
            'camera_info_topic': '/camera_front/camera_info',
            'camera_frame': 'camera_front_optical_link',
            'detected_markers_topic': '/front/aruco_markers',
            'markers_visualization_topic': '/front/aruco_poses',
            'output_image_topic': '/front/aruco_image',
        }],
        output='screen'
    )

    rear_aruco = Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name='rear_trolley_aruco_node',
        condition=IfCondition(launch_aruco),
        parameters=[{
            'use_sim_time': True,
            'marker_size': 0.08,
            'aruco_dictionary_id': 'DICT_4X4_50',
            'image_topic': '/camera_rear/image_raw',
            'use_depth_input': False,
            'depth_image_topic': '/camera_rear/depth/image_raw',
            'camera_info_topic': '/camera_rear/camera_info',
            'camera_frame': 'camera_rear_optical_link',
            'detected_markers_topic': '/rear/aruco_markers',
            'markers_visualization_topic': '/rear/aruco_poses',
            'output_image_topic': '/rear/aruco_image',
        }],
        output='screen'
    )

    left_aruco = Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name='left_trolley_aruco_node',
        condition=IfCondition(launch_aruco),
        parameters=[{
            'use_sim_time': True,
            'marker_size': 0.038,
            'aruco_dictionary_id': 'DICT_4X4_50',
            'image_topic': '/camera_left/image_raw',
            'use_depth_input': False,
            'depth_image_topic': '/camera_left/depth/image_raw',
            'camera_info_topic': '/camera_left/camera_info',
            'camera_frame': 'camera_left_optical_link',
            'detected_markers_topic': '/left/aruco_markers',
            'markers_visualization_topic': '/left/aruco_poses',
            'output_image_topic': '/left/aruco_image',
        }],
        output='screen'
    )

    right_aruco = Node(
        package='aruco_pose_estimation',
        executable='aruco_node.py',
        name='right_trolley_aruco_node',
        condition=IfCondition(launch_aruco),
        parameters=[{
            'use_sim_time': True,
            'marker_size': 0.038,
            'aruco_dictionary_id': 'DICT_4X4_50',
            'image_topic': '/camera_right/image_raw',
            'use_depth_input': False,
            'depth_image_topic': '/camera_right/depth/image_raw',
            'camera_info_topic': '/camera_right/camera_info',
            'camera_frame': 'camera_right_optical_link',
            'detected_markers_topic': '/right/aruco_markers',
            'markers_visualization_topic': '/right/aruco_poses',
            'output_image_topic': '/right/aruco_image',
        }],
        output='screen'
    )

    # -----------------------------
    # Trolley-ready docking controller
    # Listens on /trolley_command for "trolley_ready" or "trolley ready".
    # Uses holonomic planar movement by publishing x, y, and yaw on /cmd_vel.
    # -----------------------------
    trolley_ready_docking_controller = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                pkg_share,
                'scripts',
                'trolley_ready_docking_controller.py'
            ),
            '--ros-args',
            '-p', 'use_sim_time:=True',
            '-p', 'robot_entity:=mecanum_bot',
            '-p', 'trolley_entity:=Trolley',
            '-p', 'target_entity:=drop_zone',
            '-p', 'delivery_entity:=orange_drop_zone',
            '-p', 'home_x:=0.0',
            '-p', 'home_y:=0.0',
            '-p', 'home_yaw:=0.0',
            '-p', 'return_lane_y:=1.25',
            '-p', 'target_offset_x:=0.0',
            '-p', 'target_offset_y:=0.0',
            '-p', 'entry_offset_x:=0.015',
            '-p', 'aruco_timeout:=2.0',
            '-p', 'min_aruco_markers:=1',
            '-p', 'entrance_axis:=local_y',
            '-p', 'approach_side:=+',
            '-p', 'approach_clearance:=1.35',
            '-p', 'delivery_exit_clearance:=0.95',
            '-p', 'look_clearance:=1.25',
            '-p', 'look_pause_time:=1.0',
            '-p', 'staging_tolerance:=0.08',
            '-p', 'coarse_position_tolerance:=0.04',
            '-p', 'coarse_yaw_tolerance_deg:=2.0',
            '-p', 'fine_position_tolerance:=0.010',
            '-p', 'fine_yaw_tolerance_deg:=1.0',
            '-p', 'side_align_x_tolerance:=0.0050',
            '-p', 'side_align_yaw_tolerance_deg:=0.75',
            '-p', 'kyaw:=3.0',
            '-p', 'max_vx:=0.25',
            '-p', 'max_vy:=0.25',
            '-p', 'max_wz:=1.20',
            '-p', 'fine_max_vx:=0.04',
            '-p', 'fine_max_vy:=0.04',
            '-p', 'fine_max_wz:=0.12',
            '-p', 'fine_min_vxy:=0.020',
            '-p', 'side_max_wz:=0.60',
        ],
        output='screen'
    )

    # -----------------------------
    # Spawn Trolley Image Wall
    # Currently not launched.
    # Add spawn_trolley_image to LaunchDescription below if needed.
    # -----------------------------
    spawn_trolley_image = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_trolley_image',
        arguments=[
            '-entity', 'TrolleyImage',
            '-file', os.path.join(
                pkg_share,
                'models',
                'TrolleyImage',
                'model.sdf'
            ),
            '-x', '2.0',
            '-y', '0.0',
            '-z', '0.0'
        ],
        output='screen'
    )

    # -----------------------------
    # YOLO Object Detection Package
    # Currently not launched.
    # Add spawn_yolo to LaunchDescription below if needed.
    # -----------------------------
    spawn_yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('object_detection'),
                'launch',
                'launch_yolov8.launch.py'
            )
        )
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(
                pkg_share,
                'config',
                'mapper_params_online_async.yaml'
            )
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            )
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'autostart': 'true',
            'params_file': os.path.join(
                pkg_share,
                'config',
                'nav2_full_params.yaml'
            )
        }.items()
    )

    nav2_trolley_mission_controller = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                pkg_share,
                'scripts',
                'nav2_trolley_mission_controller.py'
            ),
            '--ros-args',
            '-p', 'use_sim_time:=True',
            '-p', ['mission_step:=', mission_step],
            '-p', 'goal_topic:=/holonomic_goal',
            '-p', 'cancel_topic:=/holonomic_cancel',
            '-p', 'ignore_radius_topic:=/holonomic_lidar_ignore_radius',
            '-p', 'home_x:=0.0',
            '-p', 'home_y:=0.0',
            '-p', 'home_yaw:=0.0',
            '-p', 'pickup_x:=2.5',
            '-p', 'pickup_y:=0.0',
            '-p', 'pickup_yaw:=0.0',
            '-p', 'trolley_model_name:=Trolley',
            '-p', 'trolley_reset_z:=0.0',
            '-p', 'dropoff_x:=5.2',
            '-p', 'dropoff_y:=0.0',
            '-p', 'dropoff_yaw:=0.0',
            '-p', 'approach_clearance:=1.35',
            '-p', 'pickup_aruco_clearance:=0.85',
            '-p', 'exit_clearance:=1.05',
            '-p', 'entry_offset_x:=0.015',
            '-p', 'local_max_vx:=0.08',
            '-p', 'local_max_vy:=0.10',
            '-p', 'local_max_wz:=0.30',
            '-p', 'min_aruco_markers:=2',
            '-p', 'aruco_start_min_markers:=2',
            '-p', 'aruco_start_max_abs_y:=1.10',
            '-p', 'direct_close_speed:=0.10',
            '-p', 'direct_close_timeout:=15.0',
            '-p', 'attached_lidar_ignore_radius:=2.5',
            '-p', 'attached_local_max_vx:=0.22',
            '-p', 'attached_local_max_vy:=0.32',
            '-p', 'attached_local_max_wz:=0.60',
            '-p', 'attached_local_timeout:=45.0',
            '-p', 'attached_exit_position_tolerance:=0.12',
            '-p', 'attached_local_yaw_tolerance_deg:=8.0',
            '-p', 'return_local_max_vx:=0.18',
            '-p', 'return_local_max_vy:=0.18',
            '-p', 'return_local_max_wz:=0.45',
            '-p', 'return_local_timeout:=90.0',
            '-p', 'aruco_align_timeout:=90.0',
            '-p', 'side_align_x_tolerance:=0.008',
            '-p', 'side_align_yaw_tolerance_deg:=1.0',
            '-p', 'fine_position_tolerance:=0.010',
            '-p', 'attach_position_tolerance:=0.025',
            '-p', 'fine_yaw_tolerance_deg:=2.0',
            '-p', 'drive_under_max_vx:=0.05',
            '-p', 'drive_under_max_vy:=0.12',
            '-p', 'drive_under_max_wz:=0.16',
            '-p', 'drive_under_fast_until_y:=0.20',
            '-p', 'straight_under_speed:=0.16',
            '-p', 'straight_under_stop_y:=0.16',
            '-p', 'straight_under_timeout:=20.0',
        ],
        output='screen'
    )

    holonomic_goal_controller = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(
                pkg_share,
                'scripts',
                'holonomic_goal_controller.py'
            ),
            '--ros-args',
            '-p', 'use_sim_time:=True',
            '-p', 'goal_topic:=/holonomic_goal',
            '-p', 'cancel_topic:=/holonomic_cancel',
            '-p', 'ignore_radius_topic:=/holonomic_lidar_ignore_radius',
            '-p', 'max_vx:=0.16',
            '-p', 'max_vy:=0.16',
            '-p', 'max_wz:=0.16',
            '-p', 'xy_tolerance:=0.04',
            '-p', 'avoid_distance:=0.95',
            '-p', 'stop_distance:=0.55',
            '-p', 'obstacle_gain:=0.22',
            '-p', 'attached_ignore_rear:=1.80',
            '-p', 'attached_ignore_front:=0.85',
            '-p', 'attached_ignore_half_width:=0.85',
        ],
        output='screen'
    )

    # -----------------------------
    # Launch Everything
    # -----------------------------
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission_step',
            default_value='step0',
            description=(
                'Test start point: step0=start/home, step1=outside trolley for ArUco '
                'alignment, step2=under trolley before attach, step3=docked/attached pickup.'
            )
        ),
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                models_dir,
                os.pathsep,
                EnvironmentVariable('GAZEBO_MODEL_PATH', default_value='')
            ]
        ),
        SetEnvironmentVariable(
            name='GAZEBO_PLUGIN_PATH',
            value=[
                os.path.join(pkg_prefix, 'lib'),
                os.pathsep,
                EnvironmentVariable('GAZEBO_PLUGIN_PATH', default_value='')
            ]
        ),
        SetEnvironmentVariable(
            name='FASTRTPS_DEFAULT_PROFILES_FILE',
            value=os.path.join(
                pkg_share,
                'config',
                'fastdds_no_shm.xml'
            )
        ),
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        spawn_charging_zone,
        spawn_drop_zone,
        spawn_orange_drop_zone,
        spawn_trolley,
        TimerAction(period=3.0, actions=[slam]),
        TimerAction(period=5.0, actions=[nav2]),
        TimerAction(period=8.0, actions=[nav2_trolley_mission_controller]),
        TimerAction(period=8.0, actions=[holonomic_goal_controller]),

        # Optional:
        front_aruco,
        rear_aruco,
        left_aruco,
        right_aruco,
        # trolley_ready_docking_controller,
        # spawn_trolley_image,
        # spawn_yolo,
    ])
