from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    script_path = "/home/group4/Nursing-Home-Robot/src/my_robot_description/scripts/aruco_limit_tester.py"

    velocity_values = LaunchConfiguration("velocity_values")
    angular_velocity_values = LaunchConfiguration("angular_velocity_values")
    lateral_offsets = LaunchConfiguration("lateral_offsets")
    yaw_offsets_deg = LaunchConfiguration("yaw_offsets_deg")

    trial_timeout = LaunchConfiguration("trial_timeout")
    max_trial_time = LaunchConfiguration("max_trial_time")
    stagnation_timeout = LaunchConfiguration("stagnation_timeout")
    min_progress = LaunchConfiguration("min_progress")

    min_markers_required = LaunchConfiguration("min_markers_required")
    result_dir = LaunchConfiguration("result_dir")

    kx = LaunchConfiguration("kx")
    ky = LaunchConfiguration("ky")
    kyaw = LaunchConfiguration("kyaw")

    max_v_default = "'0.02,0.04,0.06,0.08'"
    max_wz_default = "'0.08,0.12,0.15,0.20'"
    lateral_default = "'0.00,0.02,0.04,0.06,0.08,0.10,0.12,0.14,0.16,0.18,0.20'"
    yaw_default = "'0,2,4,6,8,10,12,14,16,18,20'"

    return LaunchDescription([
        DeclareLaunchArgument(
            "velocity_values",
            default_value=max_v_default,
        ),
        DeclareLaunchArgument(
            "angular_velocity_values",
            default_value=max_wz_default,
        ),
        DeclareLaunchArgument(
            "lateral_offsets",
            default_value=lateral_default,
        ),
        DeclareLaunchArgument(
            "yaw_offsets_deg",
            default_value=yaw_default,
        ),

        # 0.0 means no fixed timeout.
        DeclareLaunchArgument(
            "trial_timeout",
            default_value="0.0",
        ),

        # Absolute safety limit per trial.
        DeclareLaunchArgument(
            "max_trial_time",
            default_value="180.0",
        ),

        # Stop if no meaningful improvement happens for this many seconds.
        DeclareLaunchArgument(
            "stagnation_timeout",
            default_value="25.0",
        ),

        # Minimum improvement in combined error score.
        DeclareLaunchArgument(
            "min_progress",
            default_value="0.002",
        ),

        DeclareLaunchArgument(
            "min_markers_required",
            default_value="2",
        ),

        DeclareLaunchArgument(
            "result_dir",
            default_value="/home/group4/Nursing-Home-Robot/aruco_limit_test_results",
        ),

        DeclareLaunchArgument(
            "kx",
            default_value="0.25",
        ),
        DeclareLaunchArgument(
            "ky",
            default_value="0.50",
        ),
        DeclareLaunchArgument(
            "kyaw",
            default_value="0.90",
        ),

        ExecuteProcess(
            cmd=[
                "python3",
                script_path,
                "--ros-args",

                "-p", ["velocity_values:=", velocity_values],
                "-p", ["angular_velocity_values:=", angular_velocity_values],
                "-p", ["lateral_offsets:=", lateral_offsets],
                "-p", ["yaw_offsets_deg:=", yaw_offsets_deg],

                "-p", ["trial_timeout:=", trial_timeout],
                "-p", ["max_trial_time:=", max_trial_time],
                "-p", ["stagnation_timeout:=", stagnation_timeout],
                "-p", ["min_progress:=", min_progress],

                "-p", ["min_markers_required:=", min_markers_required],
                "-p", ["result_dir:=", result_dir],

                "-p", ["kx:=", kx],
                "-p", ["ky:=", ky],
                "-p", ["kyaw:=", kyaw],
            ],
            output="screen",
        ),
    ])