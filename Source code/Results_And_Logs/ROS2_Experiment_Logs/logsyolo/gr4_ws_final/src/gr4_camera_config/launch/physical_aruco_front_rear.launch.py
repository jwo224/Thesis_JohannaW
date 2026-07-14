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
            "width": 1920,
            "height": 1080,
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

    return LaunchDescription([
        camera_pair_node,
        front_aruco,
        rear_aruco,
    ])
