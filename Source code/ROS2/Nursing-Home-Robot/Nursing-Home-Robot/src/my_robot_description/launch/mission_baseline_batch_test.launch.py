import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_description')

    runs = LaunchConfiguration('runs')
    mission_step = LaunchConfiguration('mission_step')
    per_run_timeout = LaunchConfiguration('per_run_timeout')
    result_dir = LaunchConfiguration('result_dir')
    csv_filename = LaunchConfiguration('csv_filename')
    side_align_target_x = LaunchConfiguration('side_align_target_x')
    side_align_target_y = LaunchConfiguration('side_align_target_y')
    side_align_target_yaw_deg = LaunchConfiguration('side_align_target_yaw_deg')
    fine_align_target_x = LaunchConfiguration('fine_align_target_x')
    fine_align_target_y = LaunchConfiguration('fine_align_target_y')
    fine_align_target_yaw_deg = LaunchConfiguration('fine_align_target_yaw_deg')
    aruco_target_x = LaunchConfiguration('aruco_target_x')
    aruco_target_y = LaunchConfiguration('aruco_target_y')
    aruco_target_yaw_deg = LaunchConfiguration('aruco_target_yaw_deg')

    baseline_tester = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share, 'scripts', 'mission_batch_tester.py'),
            '--ros-args',
            '-p', ['runs:=', runs],
            '-p', ['mission_step:=', mission_step],
            '-p', ['per_run_timeout:=', per_run_timeout],
            '-p', ['result_dir:=', result_dir],
            '-p', ['csv_filename:=', csv_filename],
            '-p', 'startup_timeout:=70.0',
            '-p', 'success_hold_time:=1.0',
            '-p', 'dropoff_x:=5.2',
            '-p', 'dropoff_y:=0.0',
            '-p', 'dropoff_tolerance_x:=0.25',
            '-p', 'dropoff_tolerance_y:=0.20',
            '-p', 'charging_x:=0.0',
            '-p', 'charging_y:=0.0',
            '-p', 'charging_tolerance_x:=0.20',
            '-p', 'charging_tolerance_y:=0.20',
            '-p', ['side_align_target_x:=', side_align_target_x],
            '-p', ['side_align_target_y:=', side_align_target_y],
            '-p', ['side_align_target_yaw_deg:=', side_align_target_yaw_deg],
            '-p', ['fine_align_target_x:=', fine_align_target_x],
            '-p', ['fine_align_target_y:=', fine_align_target_y],
            '-p', ['fine_align_target_yaw_deg:=', fine_align_target_yaw_deg],
            '-p', ['aruco_target_x:=', aruco_target_x],
            '-p', ['aruco_target_y:=', aruco_target_y],
            '-p', ['aruco_target_yaw_deg:=', aruco_target_yaw_deg],
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('runs', default_value='50'),
        DeclareLaunchArgument('mission_step', default_value='step0'),
        DeclareLaunchArgument('per_run_timeout', default_value='420.0'),
        DeclareLaunchArgument(
            'result_dir',
            default_value='/home/group4/Nursing-Home-Robot/mission_baseline_results',
        ),
        DeclareLaunchArgument('csv_filename', default_value='baseline_mission_results.csv'),
        DeclareLaunchArgument('side_align_target_x', default_value='0.015'),
        DeclareLaunchArgument('side_align_target_y', default_value='0.0'),
        DeclareLaunchArgument('side_align_target_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('fine_align_target_x', default_value='0.0'),
        DeclareLaunchArgument('fine_align_target_y', default_value='0.0'),
        DeclareLaunchArgument('fine_align_target_yaw_deg', default_value='0.0'),
        DeclareLaunchArgument('aruco_target_x', default_value='0.0'),
        DeclareLaunchArgument('aruco_target_y', default_value='0.0'),
        DeclareLaunchArgument('aruco_target_yaw_deg', default_value='0.0'),
        baseline_tester,
    ])
