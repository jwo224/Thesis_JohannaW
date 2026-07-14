from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    camera = LaunchConfiguration("camera")
    camera_topic = LaunchConfiguration("camera_topic")
    model = LaunchConfiguration("model")
    confidence = LaunchConfiguration("confidence")
    enabled = LaunchConfiguration("enabled")
    command_topic = LaunchConfiguration("command_topic")
    rotate_180 = LaunchConfiguration("rotate_180")
    save_images = LaunchConfiguration("save_images")
    save_only_when_detected = LaunchConfiguration("save_only_when_detected")
    save_every_n_frames = LaunchConfiguration("save_every_n_frames")
    output_dir = LaunchConfiguration("output_dir")

    return LaunchDescription([
        DeclareLaunchArgument(
            "camera",
            default_value="front",
            description="Camera selector: front, rear, left, right.",
        ),
        DeclareLaunchArgument(
            "camera_topic",
            default_value="",
            description="Explicit ROS image topic. Overrides camera selector if set.",
        ),
        DeclareLaunchArgument(
            "model",
            default_value="emptyTrolley.pt",
            description="YOLO model file.",
        ),
        DeclareLaunchArgument(
            "confidence",
            default_value="0.25",
            description="YOLO confidence threshold.",
        ),
        DeclareLaunchArgument(
            "enabled",
            default_value="false",
            description="Whether the detector starts enabled. Dropzone mission enables it after side align.",
        ),
        DeclareLaunchArgument(
            "command_topic",
            default_value="/yolov8_detector_command",
            description="std_msgs/String command topic. Commands: enable, disable.",
        ),
        DeclareLaunchArgument(
            "rotate_180",
            default_value="true",
            description="Rotate camera image 180 degrees before YOLO inference and debug saving.",
        ),
        DeclareLaunchArgument(
            "save_images",
            default_value="true",
            description="Whether to save annotated detection images.",
        ),
        DeclareLaunchArgument(
            "save_only_when_detected",
            default_value="false",
            description="Save images only when at least one object is detected.",
        ),
        DeclareLaunchArgument(
            "save_every_n_frames",
            default_value="10",
            description="Save every N processed camera frames.",
        ),
        DeclareLaunchArgument(
            "output_dir",
            default_value="/home/rocket/gr4_ws/yolo_detection_logs",
            description="Directory for YOLO debug images and CSV logs.",
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
                "enabled": enabled,
                "command_topic": command_topic,
                "rotate_180": rotate_180,
                "publish_annotated_image": True,
                "save_images": save_images,
                "save_only_when_detected": save_only_when_detected,
                "save_every_n_frames": save_every_n_frames,
                "output_dir": output_dir,
            }],
        ),
    ])
