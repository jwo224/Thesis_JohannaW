from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = LaunchConfiguration("camera")
    camera_topic = LaunchConfiguration("camera_topic")
    model = LaunchConfiguration("model")
    confidence = LaunchConfiguration("confidence")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera",
            default_value="front",
            description="Camera selector: front, rear, left, right, or leave empty if using camera_topic.",
        ),

        DeclareLaunchArgument(
            "camera_topic",
            default_value="",
            description="Explicit ROS image topic. Overrides camera selector if set.",
        ),

        DeclareLaunchArgument(
            "model",
            default_value="emptyTrolley.pt",
            description="YOLO model file name. Example: emptyTrolley.pt, trolleys.pt, yolov8n.pt",
        ),

        DeclareLaunchArgument(
            "confidence",
            default_value="0.25",
            description="YOLO confidence threshold.",
        ),

        Node(
            package="object_detection",
            executable="yolov8_ros2_pt.py",
            name="yolov8_camera_detector",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "camera": camera,
                "camera_topic": camera_topic,
                "model": model,
                "confidence": confidence,
                "publish_annotated_image": True,
            }],
        ),
    ])