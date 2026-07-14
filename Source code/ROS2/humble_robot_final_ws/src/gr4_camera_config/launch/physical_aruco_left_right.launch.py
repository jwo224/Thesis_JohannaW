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
            "camera_pair": "left_right",
            "width": 640,
            "height": 480,
            "fps": 10,
        }],
    )

    left_aruco = Node(
        package="aruco_pose_estimation",
        executable="aruco_node.py",
        name="left_trolley_aruco_node",
        parameters=[{
            "use_sim_time": False,
            "marker_size": 0.038,
            "aruco_dictionary_id": "DICT_4X4_50",
            "image_topic": "/camera_left/image_raw",
            "use_depth_input": False,
            "depth_image_topic": "/camera_left/depth/image_raw",
            "camera_info_topic": "/camera_left/camera_info",
            "camera_frame": "camera_left_optical_link",
            "detected_markers_topic": "/left/aruco_markers",
            "markers_visualization_topic": "/left/aruco_poses",
            "output_image_topic": "/left/aruco_image",
        }],
        output="screen",
    )

    right_aruco = Node(
        package="aruco_pose_estimation",
        executable="aruco_node.py",
        name="right_trolley_aruco_node",
        parameters=[{
            "use_sim_time": False,
            "marker_size": 0.038,
            "aruco_dictionary_id": "DICT_4X4_50",
            "image_topic": "/camera_right/image_raw",
            "use_depth_input": False,
            "depth_image_topic": "/camera_right/depth/image_raw",
            "camera_info_topic": "/camera_right/camera_info",
            "camera_frame": "camera_right_optical_link",
            "detected_markers_topic": "/right/aruco_markers",
            "markers_visualization_topic": "/right/aruco_poses",
            "output_image_topic": "/right/aruco_image",
        }],
        output="screen",
    )

    return LaunchDescription([
        camera_pair_node,
        left_aruco,
        right_aruco,
    ])
