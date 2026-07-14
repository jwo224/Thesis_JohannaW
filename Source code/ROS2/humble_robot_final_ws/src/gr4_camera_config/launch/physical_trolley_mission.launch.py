import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("gr4_camera_config")
    enable_motion = LaunchConfiguration("enable_motion")
    fine_max_vx = LaunchConfiguration("fine_max_vx")
    fine_max_vy = LaunchConfiguration("fine_max_vy")
    fine_max_wz = LaunchConfiguration("fine_max_wz")
    fine_min_vxy = LaunchConfiguration("fine_min_vxy")
    fine_min_vx = LaunchConfiguration("fine_min_vx")
    fine_min_vy = LaunchConfiguration("fine_min_vy")
    fine_acquire_enable = LaunchConfiguration("fine_acquire_enable")
    fine_acquire_vx = LaunchConfiguration("fine_acquire_vx")
    fine_acquire_vy = LaunchConfiguration("fine_acquire_vy")
    fine_acquire_wz = LaunchConfiguration("fine_acquire_wz")
    fine_acquire_min_markers = LaunchConfiguration("fine_acquire_min_markers")
    fine_acquire_timeout = LaunchConfiguration("fine_acquire_timeout")
    fine_position_tolerance = LaunchConfiguration("fine_position_tolerance")
    fine_yaw_tolerance_deg = LaunchConfiguration("fine_yaw_tolerance_deg")
    min_under_markers = LaunchConfiguration("min_under_markers")
    min_front_under_markers = LaunchConfiguration("min_front_under_markers")
    min_rear_under_markers = LaunchConfiguration("min_rear_under_markers")
    under_marker_ids = LaunchConfiguration("under_marker_ids")
    under_vx_sign = LaunchConfiguration("under_vx_sign")
    under_vy_sign = LaunchConfiguration("under_vy_sign")
    under_wz_sign = LaunchConfiguration("under_wz_sign")

    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_share,
                "launch",
                "physical_camera_tf.launch.py",
            )
        )
    )

    camera_pair_node = Node(
        package="gr4_camera_config",
        executable="physical_four_camera_node",
        name="physical_camera_pair_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "camera_pair": "front_rear",
            "width": 1280,
            "height": 720,
            "fps": 10,
        }],
    )

    front_aruco = Node(
        package="aruco_pose_estimation",
        executable="aruco_node.py",
        name="front_trolley_aruco_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "marker_size": 0.08,
            "aruco_dictionary_id": "DICT_4X4_50",

            "image_topic": "/camera_front/image_raw",
            "use_depth_input": False,
            "depth_image_topic": "/camera_front/depth/image_raw",
            "camera_info_topic": "/camera_front/camera_info",

            "camera_frame": "camera_front_optical_link",

            "detected_markers_topic": "/front/aruco_markers",
            "markers_visualization_topic": "/front/aruco_poses",
            "output_image_topic": "/front/aruco_image",
        }],
    )

    rear_aruco = Node(
        package="aruco_pose_estimation",
        executable="aruco_node.py",
        name="rear_trolley_aruco_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "marker_size": 0.08,
            "aruco_dictionary_id": "DICT_4X4_50",

            "image_topic": "/camera_rear/image_raw",
            "use_depth_input": False,
            "depth_image_topic": "/camera_rear/depth/image_raw",
            "camera_info_topic": "/camera_rear/camera_info",

            "camera_frame": "camera_rear_optical_link",

            "detected_markers_topic": "/rear/aruco_markers",
            "markers_visualization_topic": "/rear/aruco_poses",
            "output_image_topic": "/rear/aruco_image",
        }],
    )

    controller = Node(
        package="gr4_camera_config",
        executable="physical_trolley_alignment_controller",
        name="physical_trolley_alignment_controller",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "enable_motion": enable_motion,

            "cmd_vel_topic": "/cmd_vel",
            "command_topic": "/physical_docking_command",
            "status_topic": "/physical_docking_status",
            "base_frame": "base_link",

            "front_aruco_topic": "/front/aruco_markers",
            "rear_aruco_topic": "/rear/aruco_markers",
            "left_aruco_topic": "/left/aruco_markers",
            "right_aruco_topic": "/right/aruco_markers",

            "aruco_timeout": 1.0,
            "control_rate": 10.0,

            "side_align_timeout": 45.0,
            "drive_under_timeout": 8.0,
            "fine_align_timeout": 45.0,

            "min_side_markers": 2,
            "min_under_markers": min_under_markers,
            "min_front_under_markers": min_front_under_markers,
            "min_rear_under_markers": min_rear_under_markers,
            "under_marker_ids": under_marker_ids,
            "estimate_window_size": 8,
            "estimate_max_std_xy": 0.025,
            "estimate_max_std_yaw_deg": 5.0,
            "estimate_jump_reject_m": 0.08,
            "max_marker_set_change": 2,
            "max_reasonable_error_xy": 0.30,

            "entry_offset_x": 0.015,
            "target_offset_x": 0.1378,
            "target_offset_y": 0.0038,

            "side_align_x_tolerance": 0.020,
            "side_align_yaw_tolerance_deg": 3.0,

            "fine_position_tolerance": fine_position_tolerance,
            "fine_yaw_tolerance_deg": fine_yaw_tolerance_deg,
            "success_hold_time": 0.35,

            "side_max_vx": 0.0,
            "side_max_wz": 0.0,

            "fine_max_vx": fine_max_vx,
            "fine_max_vy": fine_max_vy,
            "fine_max_wz": fine_max_wz,
            "fine_min_vxy": fine_min_vxy,
            "fine_min_vx": fine_min_vx,
            "fine_min_vy": fine_min_vy,
            "fine_acquire_enable": fine_acquire_enable,
            "fine_acquire_vx": fine_acquire_vx,
            "fine_acquire_vy": fine_acquire_vy,
            "fine_acquire_wz": fine_acquire_wz,
            "fine_acquire_min_markers": fine_acquire_min_markers,
            "fine_acquire_timeout": fine_acquire_timeout,

            "aruco_kxy": 0.50,
            "aruco_kyaw": 0.50,

            "straight_under_speed": 0.0,
            "straight_under_stop_y": 0.16,

            "side_vx_sign": 1.0,
            "side_wz_sign": 1.0,

            "under_vx_sign": under_vx_sign,
            "under_vy_sign": under_vy_sign,
            "under_wz_sign": under_wz_sign,

            "straight_under_vy_sign": 1.0,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="Enable nonzero cmd_vel after marker estimate stability gates pass.",
        ),
        DeclareLaunchArgument(
            "fine_max_vx",
            default_value="0.0",
            description="Fine alignment max linear x command. Keep 0 unless testing motion.",
        ),
        DeclareLaunchArgument(
            "fine_max_vy",
            default_value="0.0",
            description="Fine alignment max linear y command. Keep 0 unless testing motion.",
        ),
        DeclareLaunchArgument(
            "fine_max_wz",
            default_value="0.0",
            description="Fine alignment max angular z command. Keep 0 unless testing motion.",
        ),
        DeclareLaunchArgument(
            "fine_min_vxy",
            default_value="0.0",
            description="Fine alignment minimum linear command after deadband. Keep 0 unless testing motion.",
        ),
        DeclareLaunchArgument(
            "fine_min_vx",
            default_value="-1.0",
            description="Fine alignment x minimum command. -1 uses fine_min_vxy.",
        ),
        DeclareLaunchArgument(
            "fine_min_vy",
            default_value="-1.0",
            description="Fine alignment y minimum command. -1 uses fine_min_vxy.",
        ),
        DeclareLaunchArgument(
            "fine_acquire_enable",
            default_value="false",
            description="Allow a slow configured motion to acquire enough under markers.",
        ),
        DeclareLaunchArgument(
            "fine_acquire_vx",
            default_value="0.0",
            description="Acquisition linear x command while waiting for enough markers.",
        ),
        DeclareLaunchArgument(
            "fine_acquire_vy",
            default_value="0.0",
            description="Acquisition linear y command while waiting for enough markers.",
        ),
        DeclareLaunchArgument(
            "fine_acquire_wz",
            default_value="0.0",
            description="Acquisition angular z command while waiting for enough markers.",
        ),
        DeclareLaunchArgument(
            "fine_acquire_min_markers",
            default_value="1",
            description="Minimum visible whitelisted markers before acquisition motion is allowed.",
        ),
        DeclareLaunchArgument(
            "fine_acquire_timeout",
            default_value="5.0",
            description="Maximum continuous acquisition motion duration in seconds.",
        ),
        DeclareLaunchArgument(
            "fine_position_tolerance",
            default_value="0.025",
            description="Fine alignment position tolerance in meters.",
        ),
        DeclareLaunchArgument(
            "fine_yaw_tolerance_deg",
            default_value="4.0",
            description="Fine alignment yaw tolerance in degrees.",
        ),
        DeclareLaunchArgument(
            "min_under_markers",
            default_value="2",
            description="Minimum underside marker count required for fine alignment.",
        ),
        DeclareLaunchArgument(
            "min_front_under_markers",
            default_value="1",
            description="Minimum front-camera underside marker count required for fine alignment.",
        ),
        DeclareLaunchArgument(
            "min_rear_under_markers",
            default_value="1",
            description="Minimum rear-camera underside marker count required for fine alignment.",
        ),
        DeclareLaunchArgument(
            "under_marker_ids",
            default_value="3,5,10,12",
            description="Comma-separated calibrated underside marker IDs allowed for fine alignment.",
        ),
        DeclareLaunchArgument(
            "under_vx_sign",
            default_value="1.0",
            description="Fine alignment x command sign. Flip to -1.0 if robot drives wrong way.",
        ),
        DeclareLaunchArgument(
            "under_vy_sign",
            default_value="-1.0",
            description="Fine alignment y command sign. Flip to -1.0 if robot drives wrong way.",
        ),
        DeclareLaunchArgument(
            "under_wz_sign",
            default_value="1.0",
            description="Fine alignment yaw command sign. Flip to -1.0 if robot rotates wrong way.",
        ),
        camera_tf,
        camera_pair_node,
        front_aruco,
        rear_aruco,
        controller,
    ])
