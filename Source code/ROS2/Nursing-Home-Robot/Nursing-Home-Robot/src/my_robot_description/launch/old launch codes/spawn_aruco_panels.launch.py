from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    panel_sdf = LaunchConfiguration("panel_sdf")

    front_x = LaunchConfiguration("front_x")
    front_y = LaunchConfiguration("front_y")
    front_z = LaunchConfiguration("front_z")
    front_yaw = LaunchConfiguration("front_yaw")

    rear_x = LaunchConfiguration("rear_x")
    rear_y = LaunchConfiguration("rear_y")
    rear_z = LaunchConfiguration("rear_z")
    rear_yaw = LaunchConfiguration("rear_yaw")

    return LaunchDescription([
        DeclareLaunchArgument(
            "panel_sdf",
            default_value="/home/group4/Nursing-Home-Robot/src/gazebo_aruco_test/aruco_marker_panel/model.sdf",
            description="Path to the ArUco marker panel SDF file",
        ),

        # Front panel position
        DeclareLaunchArgument("front_x", default_value="0.4875"),
        DeclareLaunchArgument("front_y", default_value="0.0"),
        DeclareLaunchArgument("front_z", default_value="0.264"),
        DeclareLaunchArgument("front_yaw", default_value="-1.570795"),

        # Rear panel position
        DeclareLaunchArgument("rear_x", default_value="-0.4875"),
        DeclareLaunchArgument("rear_y", default_value="0.0"),
        DeclareLaunchArgument("rear_z", default_value="0.264"),
        DeclareLaunchArgument("rear_yaw", default_value="1.570795"),

        # Spawn front ArUco panel
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_front_aruco_panel",
            output="screen",
            arguments=[
                "-entity", "front_aruco_panel",
                "-file", panel_sdf,
                "-x", front_x,
                "-y", front_y,
                "-z", front_z,
                "-Y", front_yaw,
            ],
        ),

        # Spawn rear ArUco panel
        Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            name="spawn_rear_aruco_panel",
            output="screen",
            arguments=[
                "-entity", "rear_aruco_panel",
                "-file", panel_sdf,
                "-x", rear_x,
                "-y", rear_y,
                "-z", rear_z,
                "-Y", rear_yaw,
            ],
        ),
    ])
