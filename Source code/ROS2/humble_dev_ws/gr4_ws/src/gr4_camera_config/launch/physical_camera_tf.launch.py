from launch import LaunchDescription
from launch_ros.actions import Node


def static_tf(name, parent, child, x, y, z, roll, pitch, yaw):
    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            str(x), str(y), str(z),
            str(roll), str(pitch), str(yaw),
            parent,
            child,
        ],
        output="screen",
    )


def generate_launch_description():
    return LaunchDescription([
        static_tf(
            "front_camera_link_tf",
            "base_link",
            "camera_front_link",
            0.23314, 0.0, 0.06986,
            0.0, -0.436332, 0.0,
        ),
        static_tf(
            "front_camera_optical_tf",
            "camera_front_link",
            "camera_front_optical_link",
            0.0, 0.0, 0.0,
            -1.5708, 0.0, -1.5708,
        ),

        static_tf(
            "rear_camera_link_tf",
            "base_link",
            "camera_rear_link",
            -0.23314, 0.0, 0.06986,
            0.0, -0.436332, 3.14159265,
        ),
        static_tf(
            "rear_camera_optical_tf",
            "camera_rear_link",
            "camera_rear_optical_link",
            0.0, 0.0, 0.0,
            -1.5708, 0.0, -1.5708,
        ),

        static_tf(
            "left_camera_link_tf",
            "base_link",
            "camera_left_link",
            0.0, 0.25591, 0.07175,
            0.0, -0.261799, 1.5708,
        ),
        static_tf(
            "left_camera_optical_tf",
            "camera_left_link",
            "camera_left_optical_link",
            0.0, 0.0, 0.0,
            -1.5708, 0.0, -1.5708,
        ),

        static_tf(
            "right_camera_link_tf",
            "base_link",
            "camera_right_link",
            0.0, -0.25591, 0.07175,
            0.0, -0.261799, -1.5708,
        ),
        static_tf(
            "right_camera_optical_tf",
            "camera_right_link",
            "camera_right_optical_link",
            0.0, 0.0, 0.0,
            -1.5708, 0.0, -1.5708,
        ),
    ])