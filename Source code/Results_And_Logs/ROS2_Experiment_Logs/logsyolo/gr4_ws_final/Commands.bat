
Twist MUX:
ros2 run twist_mux twist_mux   --ros-args   --params-file /home/group4/Nursing-Home-Robot/src/my_robot_description/config/twist_mux.yaml   -r cmd_vel_out:=diff_cont/cmd_vel_unstamped



New Teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard   --ros-args -r cmd_vel:=cmd_vel_joy


SLAM:
ros2 launch slam_toolbox online_async_launch.py   slam_params_file:=/home/group4/Nursing-Home-Robot/src/my_robot_description/config/mapper_params_online_async.yaml   use_sim_time:=true

NAV2:
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true

AMCL:
ros2 launch nav2_bringup localization_launch.py map:=Test_room2_save.yaml use_sim_time:=true




run simulation:
ros2 launch my_robot_description sim.launch.py

run simulation from mission test steps:
ros2 launch my_robot_description sim.launch.py mission_step:=step0
ros2 launch my_robot_description sim.launch.py mission_step:=step1
ros2 launch my_robot_description sim.launch.py mission_step:=step2
ros2 launch my_robot_description sim.launch.py mission_step:=step3

baseline mission batch test, using the parameters from sim.launch.py:
ros2 launch my_robot_description mission_baseline_batch_test.launch.py runs:=10

step0 = robot at charger/origin, full mission
step1 = robot outside trolley, start with ArUco alignment and docking
step2 = robot under trolley, start with final ArUco alignment and attach
step3 = robot under trolley/docked pose, attach and drive to orange

cd /home/group4/Nursing-Home-Robot
source install/setup.bash
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: trolley_ready}"


clean rebuild:
pkill -f gzserver
pkill -f gzclient
pkill -f gazebo

cd ~/Nursing-Home-Robot
source /opt/ros/humble/setup.bash
rm -rf build install log --packages-select my_robot_description
colcon build --packages-select my_robot_description
source install/setup.bash
ros2 launch my_robot_description sim.launch.py

cd ~/Nursing-Home-Robot
source /opt/ros/humble/setup.bash
rm -rf build install log
colcon build

source install/setup.bash

ros2 launch my_robot_description sim.launch.py

colcon build
source install/setup.bash

ros2 launch my_robot_description sim.launch.py


Install Controllers: sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-ros2controlcli


Real ROBOT:
SSH to robot. rocket@192.168.8.221 pw rocket

Camera:
export ROS_DOMAIN_ID=138
source ~/gr4_ws/install/setup.bash
ros2 launch gr4_camera_config physical_aruco_front_rear.launch.py

ros2 run gr4_camera_config aruco_web_stream

Drop zones:
cd ~/gr4_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=138
ros2 launch gr4_dropzone_mission dropzone_mission.launch.py

ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'charging'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'laundry'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trash'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trolley'}"

if you want to change the coordinates of the dropzones, edit the dropzones.yaml file in the gr4_dropzone_mission package, then rebuild and relaunch the mission.

Test Object detection:
ros2 launch object_detection launch_yolov8_camera.launch.py camera:=front save_every_n_frames:=1

Calibrate docking position:

export ROS_DOMAIN_ID=138
source ~/gr4_ws/install/setup.bash
ros2 launch gr4_camera_config physical_aruco_front_rear.launch.py


export ROS_DOMAIN_ID=138
source ~/gr4_ws/install/setup.bash
ros2 run gr4_camera_config trolley_reference_logger


align outside of trolley:
export ROS_DOMAIN_ID=138
source ~/gr4_ws/install/setup.bash
ros2 launch gr4_camera_config physical_side_align_left.launch.py

export ROS_DOMAIN_ID=138
source ~/gr4_ws/install/setup.bash
ros2 topic pub /physical_docking_command std_msgs/msg/String "data: 'side_align_left'" --once

ros2 topic pub /physical_docking_command std_msgs/msg/String "data: 'stop'" --once
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once

ros2 topic pub /physical_docking_command std_msgs/msg/String "data: 'under_trolley_align'" --once