from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    physical_cameras = Node(
        package="gr4_camera_config",
        executable="physical_four_camera_node",
        name="physical_four_camera_node",
        output="screen",
        parameters=[{
            "use_sim_time": False,
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
        physical_cameras,
        front_aruco,
        rear_aruco,
        left_aruco,
        right_aruco,
    ])
