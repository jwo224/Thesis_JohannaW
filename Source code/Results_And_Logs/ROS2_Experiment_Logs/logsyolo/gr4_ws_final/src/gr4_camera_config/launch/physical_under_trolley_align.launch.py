from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
        output="screen",
    )

    rear_aruco = Node(
        package="aruco_pose_estimation",
        executable="aruco_node.py",
        name="rear_trolley_aruco_node",
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
        output="screen",
    )

    controller = Node(
        package="gr4_camera_config",
        executable="physical_trolley_alignment_controller",
        name="physical_trolley_alignment_controller",
        output="screen",
        parameters=[{
            "use_sim_time": False,

            "cmd_vel_topic": "/cmd_vel",
            "command_topic": "/physical_docking_command",

            "aruco_timeout": 1.0,
            "under_timeout": 30.0,
            "control_rate": 10.0,

            "under_position_tolerance_x": 0.014,
            "under_position_tolerance_z": 0.040,
            "under_yaw_tolerance": 0.080,

            "min_under_markers": 2,

            "max_vx": 0.035,
            "max_vy": 0.025,
            "max_wz": 0.060,

            "kx": 0.25,
            "kz": 0.25,
            "kyaw": 0.18,

            "under_min_vx": 0.025,
            "under_min_vy": 0.018,
            "under_min_wz": 0.03,

            "under_x_to_vy_sign": -1.0,
            "under_z_to_vx_sign": -1.0,
            "under_yaw_sign": -1.0,
        }],
    )

    return LaunchDescription([
        camera_pair_node,
        front_aruco,
        rear_aruco,
        controller,
    ])