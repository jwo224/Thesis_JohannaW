import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("my_robot_description")
    world = os.path.join(pkg_share, "worlds", "empty.world")
    urdf = os.path.join(pkg_share, "urdf", "mecanum_bot.urdf")

    # Start gzserver only (most stable in VirtualBox)
    gzserver = ExecuteProcess(
        cmd=[
            "gzserver",
            "--verbose",
            world,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    # Spawn robot from URDF file
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "mecanum_bot",
            "-file", urdf,
            "-x", "0", "-y", "0", "-z", "0.3",
        ],
    )

    delayed_spawn = TimerAction(period=2.0, actions=[spawn])

    return LaunchDescription([gzserver, delayed_spawn])
