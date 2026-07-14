# Mission Baseline Batch Test

This folder is the default output location for the simulation baseline mission
batch test. The test repeatedly launches the Gazebo/Nav2 trolley simulation,
sends `trolley_ready`, waits for the pickup-deliver-return workflow, and writes
one row per run to `baseline_mission_results.csv`.

## Files Involved

- `src/my_robot_description/launch/mission_baseline_batch_test.launch.py`
  starts the batch tester and forwards target values.
- `src/my_robot_description/scripts/mission_batch_tester.py`
  starts each simulation run, watches the logs, reads Gazebo ground truth, and
  writes the CSV.
- `src/my_robot_description/launch/sim.launch.py`
  starts the simulated robot, trolley, Nav2, ArUco nodes, and controllers.
- `src/my_robot_description/scripts/nav2_trolley_mission_controller.py`
  runs the trolley mission and prints the side/fine ArUco completion lines.
- `src/my_robot_description/scripts/holonomic_goal_controller.py`
  handles local holonomic goal motion in the simulation.

## Mission Stages

The CSV separates the two ArUco alignment stages:

- `side_align`: the outside/pre-entry alignment before the robot drives under
  the trolley. The controller aligns only side distance in base-frame x and
  trolley yaw.
- `fine_align`: the under-trolley alignment before attaching. The controller
  aligns x, y, and yaw under the trolley.

This naming replaces the older ambiguous `final_aruco_*` and `attach_actual_*`
wording. Existing old CSV files keep their old headers; new runs use the clearer
stage-prefixed names.

## Run The Baseline Test

Use a clean terminal. Stop old Gazebo processes first if needed:

```bash
ps -C gzserver,gzclient,gazebo -o pid,etime,cmd
pkill -f 'gzserver|gzclient'
```

Build and source:

```bash
cd ~/Nursing-Home-Robot
source /opt/ros/humble/setup.bash
colcon build --packages-select my_robot_description
source install/setup.bash
```

Run a quick smoke test:

```bash
ros2 launch my_robot_description mission_baseline_batch_test.launch.py runs:=1
```

Run the full baseline:

```bash
ros2 launch my_robot_description mission_baseline_batch_test.launch.py runs:=50
```

The default output file is:

```text
~/Nursing-Home-Robot/mission_baseline_results/baseline_mission_results.csv
```

## Target Arguments

The batch tester compares Gazebo ground truth against the same targets that the
mission controller uses for alignment.

Side alignment target:

- `side_align_target_x`: desired trolley center x in the robot base frame during
  side alignment. Default `0.015`, matching `entry_offset_x`.
- `side_align_target_y`: desired trolley center y for side-alignment reporting.
  Default `0.0`. The controller does not actively control side y.
- `side_align_target_yaw_deg`: desired side-alignment yaw error. Default `0.0`.

Fine alignment target:

- `fine_align_target_x`: desired trolley center x in the robot base frame under
  the trolley. Default `0.0`.
- `fine_align_target_y`: desired trolley center y in the robot base frame under
  the trolley. Default `0.0`.
- `fine_align_target_yaw_deg`: desired under-trolley yaw error. Default `0.0`.

Legacy aliases still accepted by `mission_batch_tester.py`:

- `aruco_target_x`
- `aruco_target_y`
- `aruco_target_yaw_deg`

Prefer the `fine_align_target_*` names for new commands.

Example:

```bash
ros2 launch my_robot_description mission_baseline_batch_test.launch.py \
  runs:=10 \
  side_align_target_x:=0.015 \
  fine_align_target_x:=0.0 \
  fine_align_target_y:=0.0 \
  fine_align_target_yaw_deg:=0.0
```

## CSV Column Dictionary

General run columns:

- `run`: run number inside the batch.
- `success`: `True` if the trolley ends in the orange drop-off zone and the
  robot returns to charging after detach.
- `failure_reason`: short failure category if the run fails.
- `failure_detail`: shortened source log line for the failure.
- `duration_sec`: wall-clock runtime of this run.
- `startup_duration_sec`: time until the mission controller reported ready.
- `command_to_success_sec`: time from run start until the success condition held.
- `mission_started_sec`: time when the mission controller started the mission.
- `attach_sec`: time when the attach service succeeded.
- `detach_sec`: time when the detach service succeeded.
- `dropoff_detected_sec`: first time the trolley was detected inside the
  drop-off tolerance box.
- `charging_detected_sec`: first time the robot was detected inside the charging
  tolerance box after detach.
- `mission_param_overrides`: mission controller parameters applied by the tester.
- `holonomic_param_overrides`: holonomic controller parameters applied by the
  tester.
- `last_log_line`: last relevant simulation log line captured by the tester.

Last observed Gazebo poses:

- `last_trolley_x`: last trolley world x from Gazebo.
- `last_trolley_y`: last trolley world y from Gazebo.
- `last_trolley_yaw_deg`: last trolley world yaw from Gazebo.
- `last_robot_x`: last robot world x from Gazebo.
- `last_robot_y`: last robot world y from Gazebo.
- `last_robot_yaw_deg`: last robot world yaw from Gazebo.

Whole-mission final world-frame errors:

- `final_trolley_dropoff_error_x`: final trolley world x minus drop-off x.
- `final_trolley_dropoff_error_y`: final trolley world y minus drop-off y.
- `final_trolley_dropoff_error_dist`: planar distance from trolley to drop-off.
- `final_robot_charging_error_x`: final robot world x minus charging x.
- `final_robot_charging_error_y`: final robot world y minus charging y.
- `final_robot_charging_error_dist`: planar distance from robot to charging.

Side alignment columns:

- `side_align_complete_sec`: time when outside side alignment completed.
- `side_align_aruco_est_error_x`: controller-reported side x error at
  completion.
- `side_align_aruco_est_error_y`: controller-reported side y error. This is
  normally `0.0` because side alignment only controls x and yaw.
- `side_align_aruco_est_yaw_error_deg`: controller-reported side yaw error.
- `side_align_aruco_est_marker_count`: marker count used by the side estimate.
- `side_align_actual_trolley_base_x`: Gazebo trolley center x in the robot base
  frame at side completion.
- `side_align_actual_trolley_base_y`: Gazebo trolley center y in the robot base
  frame at side completion.
- `side_align_actual_trolley_yaw_error_deg`: Gazebo trolley yaw relative to the
  robot at side completion.
- `side_align_actual_aruco_error_x`: Gazebo side x error relative to
  `side_align_target_x`.
- `side_align_actual_aruco_error_y`: Gazebo side y error relative to
  `side_align_target_y`.
- `side_align_actual_aruco_error_dist`: planar Gazebo side error distance.
- `side_align_actual_aruco_yaw_error_deg`: Gazebo side yaw error relative to
  `side_align_target_yaw_deg`.
- `side_align_aruco_vs_actual_error_x`: ArUco side x error minus Gazebo side x
  error.
- `side_align_aruco_vs_actual_error_y`: ArUco side y error minus Gazebo side y
  error.
- `side_align_aruco_vs_actual_yaw_error_deg`: ArUco side yaw error minus Gazebo
  side yaw error.

Fine alignment columns:

- `fine_align_complete_sec`: time when under-trolley fine alignment completed.
- `fine_align_aruco_est_error_x`: controller-reported fine x error at
  completion.
- `fine_align_aruco_est_error_y`: controller-reported fine y error at
  completion.
- `fine_align_aruco_est_yaw_error_deg`: controller-reported fine yaw error.
- `fine_align_aruco_est_marker_count`: marker count used by the fine estimate.
- `fine_align_actual_trolley_base_x`: Gazebo trolley center x in the robot base
  frame at fine completion.
- `fine_align_actual_trolley_base_y`: Gazebo trolley center y in the robot base
  frame at fine completion.
- `fine_align_actual_trolley_yaw_error_deg`: Gazebo trolley yaw relative to the
  robot at fine completion.
- `fine_align_actual_aruco_error_x`: Gazebo fine x error relative to
  `fine_align_target_x`.
- `fine_align_actual_aruco_error_y`: Gazebo fine y error relative to
  `fine_align_target_y`.
- `fine_align_actual_aruco_error_dist`: planar Gazebo fine error distance.
- `fine_align_actual_aruco_yaw_error_deg`: Gazebo fine yaw error relative to
  `fine_align_target_yaw_deg`.
- `fine_align_aruco_vs_actual_error_x`: ArUco fine x error minus Gazebo fine x
  error.
- `fine_align_aruco_vs_actual_error_y`: ArUco fine y error minus Gazebo fine y
  error.
- `fine_align_aruco_vs_actual_yaw_error_deg`: ArUco fine yaw error minus Gazebo
  fine yaw error.

## Interpreting The Stage Values

Use the `*_aruco_est_*` columns to see what the ArUco controller believed.
Use the `*_actual_*` columns to see where Gazebo says the trolley really was.
Use the `*_aruco_vs_actual_*` columns to estimate ArUco perception error.

For side alignment, the most important values are:

- `side_align_aruco_est_error_x`
- `side_align_actual_aruco_error_x`
- `side_align_aruco_vs_actual_error_x`
- `side_align_actual_aruco_yaw_error_deg`

For fine alignment, the most important values are:

- `fine_align_aruco_est_error_x`
- `fine_align_aruco_est_error_y`
- `fine_align_actual_aruco_error_x`
- `fine_align_actual_aruco_error_y`
- `fine_align_aruco_vs_actual_error_x`
- `fine_align_aruco_vs_actual_error_y`

The per-run ROS logs are written under:

```text
/tmp/mission_batch_ros_logs/run_XXX
```
