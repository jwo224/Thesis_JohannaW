import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def aruco_node(name, image_topic, info_topic, camera_frame, detected_topic, output_topic):
    return Node(
        package="aruco_pose_estimation",
        executable="aruco_node.py",
        name=name,
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "marker_size": 0.038,
            "aruco_dictionary_id": "DICT_4X4_50",
            "image_topic": image_topic,
            "use_depth_input": False,
            "camera_info_topic": info_topic,
            "camera_frame": camera_frame,
            "detected_markers_topic": detected_topic,
            "output_image_topic": output_topic,
        }],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("gr4_camera_config")

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
            "width": 1280,
            "height": 720,
            "fps": 10,
        }],
    )

    left_aruco = aruco_node(
        "left_trolley_aruco_node",
        "/camera_left/image_raw",
        "/camera_left/camera_info",
        "camera_left_optical_link",
        "/left/aruco_markers",
        "/left/aruco_image",
    )

    controller = Node(
        package="gr4_camera_config",
        executable="physical_trolley_alignment_controller",
        name="physical_trolley_alignment_controller",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "enable_motion": False,
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
            "min_side_markers": 2,
            "estimate_window_size": 8,
            "estimate_max_std_xy": 0.025,
            "estimate_max_std_yaw_deg": 5.0,
            "estimate_jump_reject_m": 0.08,
            "max_marker_set_change": 2,
            "max_reasonable_error_xy": 0.30,
            "side_align_timeout": 45.0,
            "side_align_x_tolerance": 0.020,
            "side_align_yaw_tolerance_deg": 3.0,
            "side_max_vx": 0.0,
            "side_max_wz": 0.0,
            "fine_max_vx": 0.0,
            "fine_max_vy": 0.0,
            "fine_max_wz": 0.0,
            "fine_min_vxy": 0.0,
            "straight_under_speed": 0.0,
        }],
    )

    return LaunchDescription([
        camera_tf,
        camera_pair_node,
        left_aruco,
        controller,
    ])
