#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    gazebo_ros_dir = get_package_share_directory("gazebo_ros")
    turtlebot3_gazebo_dir = get_package_share_directory("turtlebot3_gazebo")

    turtlebot3_model_path = os.path.join(turtlebot3_gazebo_dir, "models")
    world_path = os.path.join(
        turtlebot3_gazebo_dir,
        "worlds",
        "turtlebot3_world.world",
    )
    robot_sdf = os.path.join(
        turtlebot3_model_path,
        "turtlebot3_waffle",
        "model.sdf",
    )
    map_yaml = os.path.join(
        nav2_bringup_dir,
        "maps",
        "turtlebot3_world.yaml",
    )
    nav2_params = os.path.join(
        nav2_bringup_dir,
        "params",
        "nav2_params.yaml",
    )
    rviz_config = os.path.join(
        nav2_bringup_dir,
        "rviz",
        "nav2_default_view.rviz",
    )

    gazebo_model_path = turtlebot3_model_path

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world": world_path,
            "verbose": "true",
            "factory": "true",
        }.items(),
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_dir, "launch", "gzclient.launch.py")
        )
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                turtlebot3_gazebo_dir,
                "launch",
                "robot_state_publisher.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": "true",
        }.items(),
    )

    spawn_turtlebot = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_turtlebot3_waffle",
                output="screen",
                arguments=[
                    "-entity",
                    "turtlebot3_waffle",
                    "-file",
                    robot_sdf,
                    "-x",
                    "-2.00",
                    "-y",
                    "-0.50",
                    "-z",
                    "0.01",
                    "-timeout",
                    "120",
                ],
            )
        ],
    )

    nav2_bringup = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "namespace": "",
                    "use_namespace": "False",
                    "slam": "False",
                    "use_sim_time": "True",
                    "params_file": nav2_params,
                    "autostart": "True",
                    "use_composition": "True",
                    "use_respawn": "False",
                }.items(),
            )
        ],
    )

    rviz = TimerAction(
        period=14.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, "launch", "rviz_launch.py")
                ),
                launch_arguments={
                    "rviz_config": rviz_config,
                    "namespace": "",
                    "use_namespace": "False",
                }.items(),
            )
        ],
    )

    spawn_zone_marks = TimerAction(
        period=20.0,
        actions=[
            Node(
                package="turtlebot_adapted",
                executable="spawn_zone_marks",
                name="spawn_zone_marks",
                output="screen",
            )
        ],
    )

    trolley_image_detector = TimerAction(
        period=25.0,
        actions=[
            Node(
                package="object_detection",
                executable="trolley_image_detector.py",
                name="trolley_image_detector",
                output="screen",
            )
        ],
    )

    zone_delivery_node = TimerAction(
        period=30.0,
        actions=[
            Node(
                package="turtlebot_adapted",
                executable="zones_and_delivery",
                name="zones_and_delivery",
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable(name="TURTLEBOT3_MODEL", value="waffle"),
            SetEnvironmentVariable(name="GAZEBO_MODEL_PATH", value=gazebo_model_path),
            gazebo_server,
            gazebo_client,
            robot_state_publisher,
            spawn_turtlebot,
            nav2_bringup,
            rviz,
            spawn_zone_marks,
            trolley_image_detector,
            zone_delivery_node,
        ]
    )
