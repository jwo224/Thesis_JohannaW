from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Args (same idea as the tutorial)
    gui_arg = DeclareLaunchArgument(
        name="gui",
        default_value="true",
        description="Start Gazebo GUI"
    )

    urdf_package_arg = DeclareLaunchArgument(
        name="urdf_package",
        default_value="my_robot_description",
        description="Package containing the robot description (URDF)"
    )

    urdf_package_path_arg = DeclareLaunchArgument(
        name="urdf_package_path",
        default_value="urdf/mecanum_bot.urdf",
        description="Path to URDF file inside the package"
    )

    # Start Gazebo (Gazebo Classic) from gazebo_ros
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
            )
        ),
        launch_arguments={
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    # Publish /robot_description using urdf_launch (same pattern as tutorial)
    # This launch file reads the URDF (or xacro) and starts robot_state_publisher.
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("urdf_launch"), "launch", "description.launch.py"]
            )
        ),
        launch_arguments={
            "urdf_package": LaunchConfiguration("urdf_package"),
            "urdf_package_path": LaunchConfiguration("urdf_package_path"),
        }.items(),
    )

    # Spawn robot in Gazebo from /robot_description (factory)
    urdf_spawner_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="urdf_spawner",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robot",
            "-x", "0",
            "-y", "0",
            "-z", "0",
        ],
        output="screen",
    )

    return LaunchDescription([
        gui_arg,
        urdf_package_arg,
        urdf_package_path_arg,
        gazebo_launch,
        description_launch,
        urdf_spawner_node,
    ])
