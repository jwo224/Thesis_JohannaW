import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def aruco_node(
    name,
    marker_size,
    image_topic,
    info_topic,
    camera_frame,
    detected_topic,
    output_topic,
    draw_axes,
):
    return Node(
        package="aruco_pose_estimation",
        executable="aruco_node.py",
        name=name,
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "marker_size": marker_size,
            "aruco_dictionary_id": "DICT_4X4_50",
            "image_topic": image_topic,
            "use_depth_input": False,
            "camera_info_topic": info_topic,
            "camera_frame": camera_frame,
            "detected_markers_topic": detected_topic,
            "output_image_topic": output_topic,
            "draw_axes": draw_axes,
        }],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("gr4_camera_config")

    enable_motion = LaunchConfiguration("enable_motion")
    side_max_vx = LaunchConfiguration("side_max_vx")
    side_min_vx = LaunchConfiguration("side_min_vx")
    side_max_wz = LaunchConfiguration("side_max_wz")
    side_vx_sign = LaunchConfiguration("side_vx_sign")
    side_wz_sign = LaunchConfiguration("side_wz_sign")
    right_side_vx_sign = LaunchConfiguration("right_side_vx_sign")
    right_side_wz_sign = LaunchConfiguration("right_side_wz_sign")
    straight_under_speed = LaunchConfiguration("straight_under_speed")
    straight_under_vy_sign = LaunchConfiguration("straight_under_vy_sign")
    straight_under_stop_y = LaunchConfiguration("straight_under_stop_y")
    straight_under_duration_sec = LaunchConfiguration("straight_under_duration_sec")
    straight_under_distance_m = LaunchConfiguration("straight_under_distance_m")
    straight_under_x_hold_kp = LaunchConfiguration("straight_under_x_hold_kp")
    straight_under_max_vx_correction = LaunchConfiguration(
        "straight_under_max_vx_correction"
    )
    straight_under_vx_bias = LaunchConfiguration("straight_under_vx_bias")
    straight_under_wz_bias = LaunchConfiguration("straight_under_wz_bias")
    drive_under_extra_cmd_vel_topic = LaunchConfiguration(
        "drive_under_extra_cmd_vel_topic"
    )
    drive_under_lidar_ignore_radius = LaunchConfiguration(
        "drive_under_lidar_ignore_radius"
    )
    drive_under_timeout = LaunchConfiguration("drive_under_timeout")
    drive_under_stop_on_under_markers = LaunchConfiguration(
        "drive_under_stop_on_under_markers"
    )
    drive_under_marker_stop_hold_sec = LaunchConfiguration(
        "drive_under_marker_stop_hold_sec"
    )
    drive_under_marker_stop_min_progress = LaunchConfiguration(
        "drive_under_marker_stop_min_progress"
    )
    aruco_timeout = LaunchConfiguration("aruco_timeout")
    draw_aruco_axes = LaunchConfiguration("draw_aruco_axes")
    fine_debug_period_sec = LaunchConfiguration("fine_debug_period_sec")
    target_offset_x = LaunchConfiguration("target_offset_x")
    target_offset_y = LaunchConfiguration("target_offset_y")
    use_orientation_target_offsets = LaunchConfiguration("use_orientation_target_offsets")
    target_offset_x_normal = LaunchConfiguration("target_offset_x_normal")
    target_offset_y_normal = LaunchConfiguration("target_offset_y_normal")
    target_offset_x_flipped = LaunchConfiguration("target_offset_x_flipped")
    target_offset_y_flipped = LaunchConfiguration("target_offset_y_flipped")
    min_under_markers = LaunchConfiguration("min_under_markers")
    min_front_under_markers = LaunchConfiguration("min_front_under_markers")
    min_rear_under_markers = LaunchConfiguration("min_rear_under_markers")
    fine_estimate_window_size = LaunchConfiguration("fine_estimate_window_size")
    fine_require_opposite_y_pair = LaunchConfiguration("fine_require_opposite_y_pair")
    fine_min_pair_local_y_separation = LaunchConfiguration(
        "fine_min_pair_local_y_separation"
    )
    fine_preferred_pair_ids = LaunchConfiguration("fine_preferred_pair_ids")
    fine_require_preferred_pair = LaunchConfiguration("fine_require_preferred_pair")
    fine_use_all_under_markers = LaunchConfiguration("fine_use_all_under_markers")
    fine_max_vx = LaunchConfiguration("fine_max_vx")
    fine_max_vy = LaunchConfiguration("fine_max_vy")
    fine_max_wz = LaunchConfiguration("fine_max_wz")
    fine_min_wz = LaunchConfiguration("fine_min_wz")
    fine_min_vxy = LaunchConfiguration("fine_min_vxy")
    fine_min_vx = LaunchConfiguration("fine_min_vx")
    fine_min_vy = LaunchConfiguration("fine_min_vy")
    fine_acquire_enable = LaunchConfiguration("fine_acquire_enable")
    fine_acquire_front_seen_vx = LaunchConfiguration("fine_acquire_front_seen_vx")
    fine_acquire_rear_seen_vx = LaunchConfiguration("fine_acquire_rear_seen_vx")
    fine_acquire_timeout = LaunchConfiguration("fine_acquire_timeout")
    fine_reacquire_after_pair_loss = LaunchConfiguration("fine_reacquire_after_pair_loss")
    fine_position_tolerance = LaunchConfiguration("fine_position_tolerance")
    fine_position_tolerance_x = LaunchConfiguration("fine_position_tolerance_x")
    fine_position_tolerance_y = LaunchConfiguration("fine_position_tolerance_y")
    fine_target_yaw_deg = LaunchConfiguration("fine_target_yaw_deg")
    fine_target_yaw_deg_normal = LaunchConfiguration("fine_target_yaw_deg_normal")
    fine_target_yaw_deg_flipped = LaunchConfiguration("fine_target_yaw_deg_flipped")
    fine_yaw_tolerance_deg = LaunchConfiguration("fine_yaw_tolerance_deg")
    fine_sequential_alignment = LaunchConfiguration("fine_sequential_alignment")
    fine_control_mode = LaunchConfiguration("fine_control_mode")
    fine_step_settle_sec = LaunchConfiguration("fine_step_settle_sec")
    fine_step_min_pulse_sec = LaunchConfiguration("fine_step_min_pulse_sec")
    fine_step_max_pulse_sec = LaunchConfiguration("fine_step_max_pulse_sec")
    fine_step_error_fraction = LaunchConfiguration("fine_step_error_fraction")
    fine_auto_flip_bad_xy_step = LaunchConfiguration("fine_auto_flip_bad_xy_step")
    fine_bad_step_error_margin = LaunchConfiguration("fine_bad_step_error_margin")
    under_marker_ids = LaunchConfiguration("under_marker_ids")
    side_marker_ids = LaunchConfiguration("side_marker_ids")
    side_target_pair_ids = LaunchConfiguration("side_target_pair_ids")
    side_target_offset_x = LaunchConfiguration("side_target_offset_x")
    side_target_pair_offsets_x = LaunchConfiguration("side_target_pair_offsets_x")
    side_require_target_pair_visible = LaunchConfiguration(
        "side_require_target_pair_visible"
    )
    side_align_x_tolerance = LaunchConfiguration("side_align_x_tolerance")
    side_align_yaw_tolerance_deg = LaunchConfiguration("side_align_yaw_tolerance_deg")
    under_vx_sign = LaunchConfiguration("under_vx_sign")
    under_vy_sign = LaunchConfiguration("under_vy_sign")
    under_wz_sign = LaunchConfiguration("under_wz_sign")

    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "physical_camera_tf.launch.py")
        )
    )

    camera_pair_node = Node(
        package="gr4_camera_config",
        executable="physical_four_camera_node",
        name="physical_camera_pair_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "camera_pair": "left",
            "camera_pair_command_topic": "/physical_camera_pair_command",
            "camera_pair_status_topic": "/physical_camera_pair_status",
            "width": 1280,
            "height": 720,
            "fps": 10,
        }],
    )

    front_aruco = aruco_node(
        "front_trolley_aruco_node",
        0.08,
        "/camera_front/image_raw",
        "/camera_front/camera_info",
        "camera_front_optical_link",
        "/front/aruco_markers",
        "/front/aruco_image",
        draw_aruco_axes,
    )
    rear_aruco = aruco_node(
        "rear_trolley_aruco_node",
        0.08,
        "/camera_rear/image_raw",
        "/camera_rear/camera_info",
        "camera_rear_optical_link",
        "/rear/aruco_markers",
        "/rear/aruco_image",
        draw_aruco_axes,
    )
    left_aruco = aruco_node(
        "left_trolley_aruco_node",
        0.038,
        "/camera_left/image_raw",
        "/camera_left/camera_info",
        "camera_left_optical_link",
        "/left/aruco_markers",
        "/left/aruco_image",
        draw_aruco_axes,
    )
    right_aruco = aruco_node(
        "right_trolley_aruco_node",
        0.038,
        "/camera_right/image_raw",
        "/camera_right/camera_info",
        "camera_right_optical_link",
        "/right/aruco_markers",
        "/right/aruco_image",
        draw_aruco_axes,
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
            "drive_under_extra_cmd_vel_topic": drive_under_extra_cmd_vel_topic,
            "command_topic": "/physical_docking_command",
            "status_topic": "/physical_docking_status",
            "camera_pair_command_topic": "/physical_camera_pair_command",
            "camera_pair_republish_sec": 0.5,
            "lidar_ignore_radius_topic": "/holonomic_lidar_ignore_radius",
            "base_frame": "base_link",
            "odom_frame": "odom",
            "front_aruco_topic": "/front/aruco_markers",
            "rear_aruco_topic": "/rear/aruco_markers",
            "left_aruco_topic": "/left/aruco_markers",
            "right_aruco_topic": "/right/aruco_markers",
            "aruco_timeout": aruco_timeout,
            "control_rate": 10.0,
            "side_align_timeout": 45.0,
            "drive_under_timeout": drive_under_timeout,
            "fine_align_timeout": 45.0,
            "fine_debug_period_sec": fine_debug_period_sec,
            "camera_switch_settle_sec": 1.5,
            "min_side_markers": 2,
            "min_under_markers": min_under_markers,
            "min_front_under_markers": min_front_under_markers,
            "min_rear_under_markers": min_rear_under_markers,
            "under_marker_ids": under_marker_ids,
            "side_marker_ids": side_marker_ids,
            "side_target_pair_ids": side_target_pair_ids,
            "estimate_window_size": 8,
            "fine_estimate_window_size": fine_estimate_window_size,
            "estimate_max_std_xy": 0.025,
            "estimate_max_std_yaw_deg": 5.0,
            "estimate_jump_reject_m": 0.08,
            "max_marker_set_change": 2,
            "max_reasonable_error_xy": 0.30,
            "side_max_reasonable_error_xy": 0.60,
            "entry_offset_x": 0.015,
            "target_offset_x": target_offset_x,
            "target_offset_y": target_offset_y,
            "use_orientation_target_offsets": use_orientation_target_offsets,
            "target_offset_x_normal": target_offset_x_normal,
            "target_offset_y_normal": target_offset_y_normal,
            "target_offset_x_flipped": target_offset_x_flipped,
            "target_offset_y_flipped": target_offset_y_flipped,
            "side_target_offset_x": side_target_offset_x,
            "side_target_pair_offsets_x": side_target_pair_offsets_x,
            "fine_require_opposite_y_pair": fine_require_opposite_y_pair,
            "fine_min_pair_local_y_separation": fine_min_pair_local_y_separation,
            "fine_preferred_pair_ids": fine_preferred_pair_ids,
            "fine_require_preferred_pair": fine_require_preferred_pair,
            "fine_use_all_under_markers": fine_use_all_under_markers,
            "side_require_target_pair_visible": side_require_target_pair_visible,
            "side_align_x_tolerance": side_align_x_tolerance,
            "side_align_yaw_tolerance_deg": side_align_yaw_tolerance_deg,
            "fine_position_tolerance": fine_position_tolerance,
            "fine_position_tolerance_x": fine_position_tolerance_x,
            "fine_position_tolerance_y": fine_position_tolerance_y,
            "fine_target_yaw_deg": fine_target_yaw_deg,
            "fine_target_yaw_deg_normal": fine_target_yaw_deg_normal,
            "fine_target_yaw_deg_flipped": fine_target_yaw_deg_flipped,
            "fine_yaw_tolerance_deg": fine_yaw_tolerance_deg,
            "fine_sequential_alignment": fine_sequential_alignment,
            "fine_control_mode": fine_control_mode,
            "fine_step_settle_sec": fine_step_settle_sec,
            "fine_step_min_pulse_sec": fine_step_min_pulse_sec,
            "fine_step_max_pulse_sec": fine_step_max_pulse_sec,
            "fine_step_error_fraction": fine_step_error_fraction,
            "fine_auto_flip_bad_xy_step": fine_auto_flip_bad_xy_step,
            "fine_bad_step_error_margin": fine_bad_step_error_margin,
            "success_hold_time": 0.35,
            "side_max_vx": side_max_vx,
            "side_min_vx": side_min_vx,
            "side_max_wz": side_max_wz,
            "right_side_vx_sign": right_side_vx_sign,
            "right_side_wz_sign": right_side_wz_sign,
            "fine_max_vx": fine_max_vx,
            "fine_max_vy": fine_max_vy,
            "fine_max_wz": fine_max_wz,
            "fine_min_wz": fine_min_wz,
            "fine_min_vxy": fine_min_vxy,
            "fine_min_vx": fine_min_vx,
            "fine_min_vy": fine_min_vy,
            "fine_acquire_enable": fine_acquire_enable,
            "fine_acquire_front_seen_vx": fine_acquire_front_seen_vx,
            "fine_acquire_rear_seen_vx": fine_acquire_rear_seen_vx,
            "fine_acquire_timeout": fine_acquire_timeout,
            "fine_reacquire_after_pair_loss": fine_reacquire_after_pair_loss,
            "aruco_kxy": 0.50,
            "aruco_kyaw": 0.50,
            "straight_under_speed": straight_under_speed,
            "straight_under_stop_y": straight_under_stop_y,
            "straight_under_duration_sec": straight_under_duration_sec,
            "straight_under_distance_m": straight_under_distance_m,
            "straight_under_x_hold_kp": straight_under_x_hold_kp,
            "straight_under_max_vx_correction": straight_under_max_vx_correction,
            "straight_under_vx_bias": straight_under_vx_bias,
            "straight_under_wz_bias": straight_under_wz_bias,
            "drive_under_lidar_ignore_radius": drive_under_lidar_ignore_radius,
            "drive_under_stop_on_under_markers": drive_under_stop_on_under_markers,
            "drive_under_marker_stop_hold_sec": drive_under_marker_stop_hold_sec,
            "drive_under_marker_stop_min_progress": drive_under_marker_stop_min_progress,
            "side_vx_sign": side_vx_sign,
            "side_wz_sign": side_wz_sign,
            "under_vx_sign": under_vx_sign,
            "under_vy_sign": under_vy_sign,
            "under_wz_sign": under_wz_sign,
            "straight_under_vy_sign": straight_under_vy_sign,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_motion", default_value="false"),
        DeclareLaunchArgument("side_max_vx", default_value="0.025"),
        DeclareLaunchArgument("side_min_vx", default_value="0.012"),
        DeclareLaunchArgument("side_max_wz", default_value="0.060"),
        DeclareLaunchArgument("side_vx_sign", default_value="-1.0"),
        DeclareLaunchArgument("side_wz_sign", default_value="1.0"),
        DeclareLaunchArgument("right_side_vx_sign", default_value="1.0"),
        DeclareLaunchArgument("right_side_wz_sign", default_value="1.0"),
        DeclareLaunchArgument("straight_under_speed", default_value="0.070"),
        DeclareLaunchArgument("straight_under_vy_sign", default_value="1.0"),
        DeclareLaunchArgument("straight_under_stop_y", default_value="0.040"),
        DeclareLaunchArgument("straight_under_duration_sec", default_value="0.0"),
        DeclareLaunchArgument("straight_under_distance_m", default_value="1.30"),
        DeclareLaunchArgument("straight_under_x_hold_kp", default_value="0.30"),
        DeclareLaunchArgument("straight_under_max_vx_correction", default_value="0.025"),
        DeclareLaunchArgument("straight_under_vx_bias", default_value="0.0"),
        DeclareLaunchArgument("straight_under_wz_bias", default_value="0.0"),
        DeclareLaunchArgument("drive_under_extra_cmd_vel_topic", default_value="/cmd_vel_joy"),
        DeclareLaunchArgument("drive_under_lidar_ignore_radius", default_value="1.25"),
        DeclareLaunchArgument("drive_under_timeout", default_value="30.0"),
        DeclareLaunchArgument("drive_under_stop_on_under_markers", default_value="true"),
        DeclareLaunchArgument("drive_under_marker_stop_hold_sec", default_value="0.0"),
        DeclareLaunchArgument("drive_under_marker_stop_min_progress", default_value="0.20"),
        DeclareLaunchArgument("aruco_timeout", default_value="1.5"),
        DeclareLaunchArgument("draw_aruco_axes", default_value="false"),
        DeclareLaunchArgument("fine_debug_period_sec", default_value="1.0"),
        DeclareLaunchArgument("target_offset_x", default_value="0.1378"),
        DeclareLaunchArgument("target_offset_y", default_value="0.0038"),
        DeclareLaunchArgument("use_orientation_target_offsets", default_value="true"),
        DeclareLaunchArgument("target_offset_x_normal", default_value="0.1378"),
        DeclareLaunchArgument("target_offset_y_normal", default_value="0.0038"),
        DeclareLaunchArgument("target_offset_x_flipped", default_value="0.1347"),
        DeclareLaunchArgument("target_offset_y_flipped", default_value="0.0038"),
        DeclareLaunchArgument("min_under_markers", default_value="2"),
        DeclareLaunchArgument("min_front_under_markers", default_value="1"),
        DeclareLaunchArgument("min_rear_under_markers", default_value="1"),
        DeclareLaunchArgument("fine_estimate_window_size", default_value="3"),
        DeclareLaunchArgument("fine_require_opposite_y_pair", default_value="true"),
        DeclareLaunchArgument("fine_min_pair_local_y_separation", default_value="0.08"),
        DeclareLaunchArgument("fine_preferred_pair_ids", default_value="5,10;3,12"),
        DeclareLaunchArgument("fine_require_preferred_pair", default_value="true"),
        DeclareLaunchArgument("fine_use_all_under_markers", default_value="false"),
        DeclareLaunchArgument("fine_max_vx", default_value="0.018"),
        DeclareLaunchArgument("fine_max_vy", default_value="0.018"),
        DeclareLaunchArgument("fine_max_wz", default_value="0.050"),
        DeclareLaunchArgument("fine_min_wz", default_value="0.035"),
        DeclareLaunchArgument("fine_min_vxy", default_value="0.010"),
        DeclareLaunchArgument("fine_min_vx", default_value="-1.0"),
        DeclareLaunchArgument("fine_min_vy", default_value="-1.0"),
        DeclareLaunchArgument("fine_acquire_enable", default_value="true"),
        DeclareLaunchArgument("fine_acquire_front_seen_vx", default_value="0.010"),
        DeclareLaunchArgument("fine_acquire_rear_seen_vx", default_value="-0.010"),
        DeclareLaunchArgument("fine_acquire_timeout", default_value="8.0"),
        DeclareLaunchArgument("fine_reacquire_after_pair_loss", default_value="true"),
        DeclareLaunchArgument("fine_position_tolerance", default_value="0.025"),
        DeclareLaunchArgument("fine_position_tolerance_x", default_value="0.025"),
        DeclareLaunchArgument("fine_position_tolerance_y", default_value="0.010"),
        DeclareLaunchArgument("fine_target_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("fine_target_yaw_deg_normal", default_value="0.0"),
        DeclareLaunchArgument("fine_target_yaw_deg_flipped", default_value="-8.1"),
        DeclareLaunchArgument("fine_yaw_tolerance_deg", default_value="6.0"),
        DeclareLaunchArgument("fine_sequential_alignment", default_value="true"),
        DeclareLaunchArgument("fine_control_mode", default_value="continuous"),
        DeclareLaunchArgument("fine_step_settle_sec", default_value="0.35"),
        DeclareLaunchArgument("fine_step_min_pulse_sec", default_value="0.12"),
        DeclareLaunchArgument("fine_step_max_pulse_sec", default_value="0.35"),
        DeclareLaunchArgument("fine_step_error_fraction", default_value="0.45"),
        DeclareLaunchArgument("fine_auto_flip_bad_xy_step", default_value="true"),
        DeclareLaunchArgument("fine_bad_step_error_margin", default_value="0.002"),
        DeclareLaunchArgument("under_marker_ids", default_value="0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15"),
        DeclareLaunchArgument("side_marker_ids", default_value="16,17,18,19,20,21,22,23,24,25,26,27"),
        DeclareLaunchArgument("side_target_pair_ids", default_value="18,19;24,25"),
        DeclareLaunchArgument("side_target_offset_x", default_value="0.352"),
        DeclareLaunchArgument("side_target_pair_offsets_x", default_value="18,19:0.356;24,25:0.349"),
        DeclareLaunchArgument("side_require_target_pair_visible", default_value="false"),
        DeclareLaunchArgument("side_align_x_tolerance", default_value="0.015"),
        DeclareLaunchArgument("side_align_yaw_tolerance_deg", default_value="2.0"),
        DeclareLaunchArgument("under_vx_sign", default_value="-1.0"),
        DeclareLaunchArgument("under_vy_sign", default_value="-1.0"),
        DeclareLaunchArgument("under_wz_sign", default_value="-1.0"),
        camera_tf,
        camera_pair_node,
        front_aruco,
        rear_aruco,
        left_aruco,
        right_aruco,
        controller,
    ])
