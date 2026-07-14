import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_name = "my_robot_description"
    xacro_subpath = "urdf/mecanum_bot.urdf.xacro"

    # Process xacro -> URDF XML
    xacro_file = os.path.join(get_package_share_directory(pkg_name), xacro_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # Robot State Publisher (publishes TF from URDF + joint states)
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_raw}],
    )

    # RViz2 (optionally load a config if you create one)
    rviz_config = os.path.join(
        get_package_share_directory(pkg_name),
        "rviz",
        "view_urdf.rviz",
    )

    node_rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config] if os.path.exists(rviz_config) else [],
    )

    return LaunchDescription(
        [
            node_robot_state_publisher,
            node_rviz2,
        ]
    )

