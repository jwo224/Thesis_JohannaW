import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')

    runs_per_candidate = LaunchConfiguration('runs_per_candidate')
    confirmation_runs = LaunchConfiguration('confirmation_runs')
    mission_step = LaunchConfiguration('mission_step')
    per_run_timeout = LaunchConfiguration('per_run_timeout')
    result_dir = LaunchConfiguration('result_dir')
    speed_scales = LaunchConfiguration('speed_scales')
    pickup_aruco_clearances = LaunchConfiguration('pickup_aruco_clearances')
    attached_lidar_ignore_radii = LaunchConfiguration('attached_lidar_ignore_radii')
    straight_under_stop_ys = LaunchConfiguration('straight_under_stop_ys')

    sweep = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share, 'scripts', 'mission_parameter_sweep.py'),
            '--runs-per-candidate', runs_per_candidate,
            '--confirmation-runs', confirmation_runs,
            '--mission-step', mission_step,
            '--per-run-timeout', per_run_timeout,
            '--result-dir', result_dir,
            '--speed-scales', speed_scales,
            '--pickup-aruco-clearances', pickup_aruco_clearances,
            '--attached-lidar-ignore-radii', attached_lidar_ignore_radii,
            '--straight-under-stop-ys', straight_under_stop_ys,
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('runs_per_candidate', default_value='1'),
        DeclareLaunchArgument('confirmation_runs', default_value='10'),
        DeclareLaunchArgument('mission_step', default_value='step0'),
        DeclareLaunchArgument('per_run_timeout', default_value='420.0'),
        DeclareLaunchArgument(
            'result_dir',
            default_value='/home/group4/Nursing-Home-Robot/mission_parameter_sweep_results',
        ),
        DeclareLaunchArgument('speed_scales', default_value='1.0,1.15,1.30,1.45'),
        DeclareLaunchArgument('pickup_aruco_clearances', default_value='0.85'),
        DeclareLaunchArgument('attached_lidar_ignore_radii', default_value='2.5,2.8,3.1'),
        DeclareLaunchArgument('straight_under_stop_ys', default_value='0.16'),
        sweep,
    ])
