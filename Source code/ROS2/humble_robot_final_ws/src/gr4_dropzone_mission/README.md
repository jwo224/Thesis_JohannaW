# GR4 Dropzone Mission

This package coordinates the real-robot trolley mission. It publishes the
configured map zones, sends Nav2 goals, remembers the detected trolley type,
and forwards operator-approved alignment commands to `gr4_camera_config`.

The implemented flow is intentionally manual and staged. Every movement stage
starts only after an operator command on `/trolley_command`.

## Important Files

All paths below are relative to `~/gr4_ws` on the robot:

- `src/gr4_dropzone_mission/gr4_dropzone_mission/dropzone_mission.py`
  contains the mission state machine, command handling, YOLO result memory,
  delivery selection, and direct `drive_out` motion.
- `src/gr4_dropzone_mission/config/dropzones.yaml` contains map coordinates,
  approach-pose settings, topic names, detection settings, and drive-out
  velocity/duration.
- `src/gr4_dropzone_mission/launch/dropzone_mission.launch.py` starts the
  mission node with `dropzones.yaml`.
- `src/gr4_camera_config/launch/physical_staged_trolley_mission.launch.py`
  starts the physical cameras, ArUco nodes, and alignment controller.
- `src/gr4_camera_config/gr4_camera_config/physical_trolley_alignment_controller.py`
  implements side alignment, drive-under, and fine alignment.
- `src/object_detection/launch/launch_yolov8_camera.launch.py` starts live
  trolley classification from the left camera.
- `tools/mission_run_logger.py` records commands, statuses, YOLO results,
  timings, and operator marks.

The old, unused autonomous supervisor was deliberately removed. The supported
workflow is the command-by-command sequence documented below.

## Architecture And Topics

| Topic | Type | Direction | Purpose |
| --- | --- | --- | --- |
| `/trolley_command` | `std_msgs/msg/String` | Input | Operator mission commands |
| `/dropzone_mission_status` | `std_msgs/msg/String` | Output | Current phase and next expected command |
| `/goal_pose_raw` | `geometry_msgs/msg/PoseStamped` | Output | Map goal consumed by the Rocket goal-restamp/Nav2 flow |
| `/delivery_zones` | `visualization_msgs/msg/MarkerArray` | Output | RViz zones and approach marker |
| `/physical_docking_command` | `std_msgs/msg/String` | Output | Commands sent to the physical alignment controller |
| `/physical_docking_status` | `std_msgs/msg/String` | Input | Side/under/fine alignment results |
| `/Yolov8_Inference` | `object_detection/msg/Yolov8Inference` | Input | Live trolley classifications |
| `/yolov8_detector_command` | `std_msgs/msg/String` | Output | Enables/disables YOLO inference |
| `/cmd_vel`, `/cmd_vel_joy` | `geometry_msgs/msg/Twist` | Output | Direct drive-out commands |

The dropzone node does not wait for Nav2 action completion. After
`trolley_ready`, `deliver_trolley`, or `charging`, the operator must confirm in
RViz or the Nav2 terminal that the robot reached the goal before sending the
next movement command.

## Configured Map Positions

The final values in `config/dropzones.yaml` are:

| Zone | X | Y | Yaw |
| --- | ---: | ---: | ---: |
| Charging | `1.99777` | `-0.232068` | `0.0` |
| Trolley dropzone | `-1.0` | `0.55` | `0.0` |
| Laundry dropoff | `-3.0` | `0.55` | `0.0` |
| Trash dropoff | `-4.0` | `0.55` | `0.0` |

The trolley approach pose is computed from the trolley dropzone:

```text
approach_x = trolley_dropzone.x + trolley_stage_offset_x
approach_y = trolley_dropzone.y + trolley_stage_offset_y
```

With the final offsets `(0.0, -0.9)`, the approach goal is approximately
`(-1.0, -0.35)`. `trolley_stage_camera: left` rotates the robot so its left
camera faces the trolley zone.

## Build On The Robot

After copying or editing the source files:

```bash
ssh rocket@192.168.8.221
cd ~/gr4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gr4_dropzone_mission gr4_camera_config object_detection
source install/setup.bash
```

Use the same environment in every robot terminal:

```bash
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash
```

## Full Startup

### Terminal 1: Nav2

```bash
ssh rocket@192.168.8.221
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash

ros2 launch articubot_one nav.launch.py \
  map:=/home/rocket/maps/mekagangen.yaml
```

### Terminal 2: Set The Known Initial Pose

Place the robot at the physical reference position, then publish:

```bash
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash

ros2 topic pub --once /initialpose \
  geometry_msgs/msg/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 1.99777, y: -0.232068, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.700552, w: 0.713601}
    },
    covariance: [
      0.01, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.01, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0076
    ]
  }
}"
```

Check RViz before continuing. The robot model and laser scan should agree with
the map.

### Terminal 3: Dropzone Mission

```bash
ssh rocket@192.168.8.221
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash

ros2 launch gr4_dropzone_mission dropzone_mission.launch.py
```

### Terminal 4: Cameras And Physical Alignment

For observation without motion:

```bash
ros2 launch gr4_camera_config physical_staged_trolley_mission.launch.py \
  enable_motion:=false
```

Final tested motion configuration:

```bash
ros2 launch gr4_camera_config physical_staged_trolley_mission.launch.py \
  enable_motion:=true \
  side_target_offset_x:=0.356 \
  side_align_x_tolerance:=0.006 \
  fine_control_mode:=step \
  fine_sequential_alignment:=true \
  fine_use_all_under_markers:=true \
  use_orientation_target_offsets:=true \
  target_offset_x_normal:=0.1325 \
  target_offset_y_normal:=0.0062 \
  fine_target_yaw_deg_normal:=-11.0 \
  target_offset_x_flipped:=0.134 \
  target_offset_y_flipped:=0.0038 \
  fine_target_yaw_deg_flipped:=165.0 \
  fine_position_tolerance_x:=0.004 \
  fine_position_tolerance_y:=0.010 \
  fine_yaw_tolerance_deg:=3.0 \
  fine_max_wz:=0.050 \
  fine_min_wz:=0.035 \
  fine_step_max_pulse_sec:=0.18 \
  fine_step_settle_sec:=0.45 \
  fine_step_error_fraction:=0.35 \
  under_vx_sign:=-1.0
```

Important: the controller supports the calibrated pair-specific setting
`18,19:0.356;24,25:0.349`, but the final
`physical_staged_trolley_mission.launch.py` does not declare/pass the
`side_target_pair_offsets_x` launch argument. Passing it to the current launch
file may fail as an unknown argument. The common `side_target_offset_x:=0.356`
above is the setting currently exposed by the launch file. See the camera
package README before changing this.

### Terminal 5: Live Trolley Detection

The detector starts disabled. The dropzone mission enables it after successful
side alignment and disables it after `check_trolley_type` or `drive_under`.

```bash
ssh rocket@192.168.8.221
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash

ros2 launch object_detection launch_yolov8_camera.launch.py \
  camera:=left \
  model:=yolov8s_no_augmentation.pt \
  confidence:=0.25
```

The launch defaults rotate the image by 180 degrees before inference and save
periodic images, including frames without detections, under:

```text
/home/rocket/gr4_ws/yolo_detection_logs
```

### Optional Terminal 6: Mission Logger

```bash
ssh rocket@192.168.8.221
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash

python3 ~/gr4_ws/tools/mission_run_logger.py
```

The logger automatically starts a run when it sees `trolley_ready`. Add marks:

```bash
ros2 topic pub --once /mission_run_logger_command std_msgs/msg/String \
  "{data: 'mark object_detection_correct'}"

ros2 topic pub --once /mission_run_logger_command std_msgs/msg/String \
  "{data: 'mark docking_successful'}"

ros2 topic pub --once /mission_run_logger_command std_msgs/msg/String \
  "{data: 'mark fine_align_retry_with_light'}"
```

Logger control commands are `new_run`, `mark <label>`, `status`, `end_run`, and
`abort_run`. CSV files are written to `~/gr4_ws/mission_run_logs`.

### Dev PC: Status And RViz

```bash
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash

ros2 topic echo /dropzone_mission_status
```

In RViz use fixed frame `map` and add:

- `Map` on `/map`
- `RobotModel`
- `TF`
- `MarkerArray` on `/delivery_zones`
- the local/global costmaps when debugging Nav2

## Manual Trolley Flow

Wait for the expected status after every command. Do not send the full list at
once.

### 1. Drive To The Trolley Approach Pose

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'trolley_ready'}"
```

Expected status starts with:

```text
approach_goal_sent
```

This clears the previously remembered trolley type and publishes the staged
Nav2 goal. Wait until Nav2 reports `Goal succeeded` and visually inspect the
robot before continuing.

### 2. Side Align With The Trolley

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'side_align'}"
```

Expected progression:

```text
side_aligning via side_align_left
side_align_complete: side_align_left_done | final ...
```

The dropzone node forwards `side_align_left` to
`/physical_docking_command`. The camera controller opens only the left camera
and aligns using side markers `16-27`. A successful final status includes the
visible IDs, selected target pair, measured target X, errors, and tolerances.

After side alignment completes, live YOLO detection is enabled. Incoming YOLO
results are only remembered. They do not move the robot and do not stop
detection until `check_trolley_type` is sent.

If side alignment times out, inspect the camera/markers and send `side_align`
again. The mission permits another side-alignment request from `idle`.

### 3. Check And Remember Trolley Type

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'check_trolley_type'}"
```

Possible results:

- `laundry_trolley`: remembers `laundry_dropoff`.
- `trash_trolley`: remembers `trash_dropoff`.
- `empty_trolley`: disables YOLO and immediately sends the charging Nav2 goal.
- no recent detection: reports `trolley_type_unknown_live_yolo`; wait for a
  detection and send `check_trolley_type` again.

Detection memory is `90` seconds in `dropzones.yaml`. Image-file fallback is
disabled to avoid using stale test images.

For an empty trolley, no `drive_under` command is expected. Watch the Nav2
terminal for the charging result. The generic final message `Goal failed` is
not enough to diagnose planning; inspect the preceding planner lines for
`Starting point in lethal space`, `Goal in lethal space`, `no valid path`, or
`Robot is out of bounds`.

For manual type override:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'laundry_trolley'}"

ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'trash_trolley'}"
```

### 4. Drive Straight Under The Trolley

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'drive_under'}"
```

Expected progression:

```text
drive_under_active via drive_straight_under
under_trolley
```

This disables YOLO and forwards `drive_straight_under`. The physical controller
switches to the front/rear cameras, publishes direct velocity to `/cmd_vel` and
`/cmd_vel_joy`, temporarily increases `/holonomic_lidar_ignore_radius`, and
uses odometry to hold the previously aligned X line.

The default maximum distance is `1.30 m`, but
`drive_under_stop_on_under_markers:=true` allows an earlier stop once a valid
front/rear underside-marker estimate is available after minimum progress.

### 5. Fine Align Under The Trolley

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'fine_align'}"
```

Expected statuses include:

```text
fine_aligning via fine_align_under
fine_align_first_aruko: fine_first_estimate | ...
fine_align_complete: fine_align_under_done | final ...
```

The final tested configuration uses front and rear underside markers, step
control, all valid under-marker observations, orientation-specific targets,
and sequential yaw, Y, then X correction.

If fine alignment times out, improve lighting/marker visibility and send
`fine_align` again. The mission accepts retries from `idle`, `fine_aligning`,
or `fine_align_complete`.

### 6. Deliver The Docked Trolley

After physically confirming that docking is successful:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'deliver_trolley'}"
```

The remembered type selects either `laundry_dropoff` or `trash_dropoff`, and a
Nav2 goal is published. Expected status:

```text
dropoff_goal_sent: remembered ... -> ...
```

If the type was not checked or remembered, delivery is rejected. Use
`check_trolley_type`, `laundry_trolley`, or `trash_trolley` first.

### 7. Release The Trolley

Wait until Nav2 reaches the dropoff, inspect the robot and surroundings, then:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'drive_out'}"
```

The final configuration directly publishes:

```text
vx = 0.0
vy = -0.07
wz = 0.0
duration = 7.0 s
```

Expected status:

```text
drive_out_done
```

This clears the remembered trolley type and returns the mission phase to
`idle`. It does not automatically send the charging goal.

### 8. Return To Charging

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'charging'}"
```

Wait for Nav2 to report success. A new mission begins with `trolley_ready`.

## Status, Stop, And Recovery

Show the current phase and expected next command:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'status'}"
```

Stop direct alignment/drive-out motion and disable YOLO:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String \
  "{data: 'stop'}"
```

Emergency zero Twist:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{}"
ros2 topic pub --once /cmd_vel_joy geometry_msgs/msg/Twist "{}"
```

Important: `stop` tells the physical alignment controller to stop, but it does
not cancel a Nav2 goal that is already active. Cancel that goal in RViz or
through the Nav2 action interface as well.

Clear costmaps when stale obstacles prevent planning:

```bash
ros2 service call /global_costmap/clear_entirely_global_costmap \
  nav2_msgs/srv/ClearEntireCostmap "{}"

ros2 service call /local_costmap/clear_entirely_local_costmap \
  nav2_msgs/srv/ClearEntireCostmap "{}"
```

## Direct Zone Commands

These bypass the staged state machine and publish a map goal:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'charging'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trolley_dropzone'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'laundry_dropoff'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trash_dropoff'}"
```

Aliases include `charge`, `home`, `trolley`, `pickup`, `laundry`, and `trash`.

## Editing And Rebuilding

Map positions and drive-out behavior:

```text
src/gr4_dropzone_mission/config/dropzones.yaml
```

Mission phases, command ordering, detection memory, and delivery logic:

```text
src/gr4_dropzone_mission/gr4_dropzone_mission/dropzone_mission.py
```

After edits:

```bash
cd ~/gr4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gr4_dropzone_mission
source install/setup.bash
```
