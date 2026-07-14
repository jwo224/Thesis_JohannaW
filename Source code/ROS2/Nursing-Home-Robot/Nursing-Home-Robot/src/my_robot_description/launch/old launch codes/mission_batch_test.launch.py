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
    mission_param_overrides = LaunchConfiguration('mission_param_overrides')
    holonomic_param_overrides = LaunchConfiguration('holonomic_param_overrides')

    batch_tester = ExecuteProcess(
        cmd=[
            'python3',
            os.path.join(pkg_share, 'scripts', 'mission_batch_tester.py'),
            '--ros-args',
            '-p', ['runs:=', runs],
            '-p', ['mission_step:=', mission_step],
            '-p', ['per_run_timeout:=', per_run_timeout],
            '-p', ['result_dir:=', result_dir],
            '-p', ['csv_filename:=', csv_filename],
            '-p', ['mission_param_overrides:=', mission_param_overrides],
            '-p', ['holonomic_param_overrides:=', holonomic_param_overrides],
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
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('runs', default_value='50'),
        DeclareLaunchArgument('mission_step', default_value='step0'),
        DeclareLaunchArgument('per_run_timeout', default_value='420.0'),
        DeclareLaunchArgument('mission_param_overrides', default_value=''),
        DeclareLaunchArgument('holonomic_param_overrides', default_value=''),
        DeclareLaunchArgument(
            'result_dir',
            default_value='/home/group4/Nursing-Home-Robot/mission_batch_results',
        ),
        DeclareLaunchArgument('csv_filename', default_value='mission_batch_results.csv'),
        batch_tester,
    ])
