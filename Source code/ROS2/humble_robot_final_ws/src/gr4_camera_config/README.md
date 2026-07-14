# GR4 Physical Camera And Trolley Alignment

This package manages the four physical USB cameras and controls ArUco-based
trolley alignment on the real robot.

It provides three movement stages used by `gr4_dropzone_mission`:

1. Side alignment using the left or right camera.
2. Straight drive under the trolley using odometry and front/rear marker
   visibility.
3. Fine alignment under the trolley using front and rear ArUco observations.

## Important Files

Paths are relative to `~/gr4_ws` on the robot:

- `src/gr4_camera_config/launch/physical_staged_trolley_mission.launch.py`
  is the main real-robot launch file.
- `src/gr4_camera_config/gr4_camera_config/physical_trolley_alignment_controller.py`
  contains all side-align, drive-under, fine-align, safety, estimation, and
  status logic.
- `src/gr4_camera_config/gr4_camera_config/physical_four_camera_node.py`
  opens the selected physical camera pair. It supports `left`, `right`,
  `left_right`, and `front_rear`.
- `src/gr4_camera_config/launch/physical_camera_tf.launch.py` defines the
  calibrated static transforms from `base_link` to all cameras.
- `src/gr4_camera_config/config/camera_mapping.yaml` records the physical
  `/dev/v4l/by-path` mapping.
- `src/gr4_camera_config/gr4_camera_config/camera_web_stream.py` serves the
  ArUco debug streams.
- `src/gr4_camera_config/gr4_camera_config/trolley_reference_logger.py` and
  `single_aruco_reference_logger.py` record calibration/reference data.

The mission-level commands and Nav2 flow are documented in:

```text
src/gr4_dropzone_mission/README.md
```

## Camera Layout

`physical_camera_tf.launch.py` defines:

| Camera | Base position | Intended use |
| --- | --- | --- |
| Front | `x=+0.23314`, `y=0.0`, `z=0.06986` | Under-trolley markers |
| Rear | `x=-0.23314`, `y=0.0`, `z=0.06986` | Under-trolley markers |
| Left | `x=0.0`, `y=+0.25591`, `z=0.07175` | Left side alignment and YOLO |
| Right | `x=0.0`, `y=-0.25591`, `z=0.07175` | Optional right side alignment |

Only the required cameras are opened:

- Launch starts with `left`.
- `side_align_left` requests `left`.
- `side_align_right` requests `right`.
- `drive_straight_under` requests `front_rear`.
- `fine_align_under` requests `front_rear`.

Camera commands and status use:

```text
/physical_camera_pair_command
/physical_camera_pair_status
```

## Marker Groups

The built-in physical trolley layout in
`physical_trolley_alignment_controller.py` uses:

- underside markers `0-15`
- side markers `16-27`
- side center pairs `18,19` and `24,25`

Marker sizes in the staged launch are:

- front/rear underside markers: `0.08 m`
- left/right side markers: `0.038 m`
- dictionary: `DICT_4X4_50`

Do not change marker IDs, sizes, or layout coordinates without rechecking the
physical trolley labels and collecting new reference logs.

## Main Launch

Always start with motion disabled when validating cameras, transforms, or a new
calibration:

```bash
export ROS_DOMAIN_ID=138
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash

ros2 launch gr4_camera_config physical_staged_trolley_mission.launch.py \
  enable_motion:=false
```

The controller will still estimate positions and publish statuses, but blocks
nonzero motion.

Final tested motion command:

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

## Direct Controller Commands

The dropzone package normally publishes these commands. For isolated testing:

```bash
ros2 topic pub --once /physical_docking_command std_msgs/msg/String \
  "{data: 'side_align_left'}"

ros2 topic pub --once /physical_docking_command std_msgs/msg/String \
  "{data: 'drive_straight_under'}"

ros2 topic pub --once /physical_docking_command std_msgs/msg/String \
  "{data: 'fine_align_under'}"

ros2 topic pub --once /physical_docking_command std_msgs/msg/String \
  "{data: 'stop'}"

ros2 topic pub --once /physical_docking_command std_msgs/msg/String \
  "{data: 'clear_markers'}"
```

Watch:

```bash
ros2 topic echo /physical_docking_status
```

## Stage 1: Side Alignment

`run_side_align()` estimates trolley pose from side markers seen by the
selected side camera.

For the normal left-camera flow:

- only the left camera is used
- at least two side markers are required
- the closest configured target pair is selected
- X distance and parallel yaw are corrected
- completion requires the configured error tolerances to remain valid for the
  success hold time

The successful status contains:

```text
final camera=left
ids=[...]
target_pair=[18, 19] or [24, 25]
target_x=...
error_x=...
yaw_error=...
```

The final common target is:

```text
side_target_offset_x = 0.356 m
side_align_x_tolerance = 0.006 m
side_align_yaw_tolerance = 2 deg (launch default)
```

### Pair-Specific Side Calibration

The controller already implements:

```text
side_target_pair_offsets_x = 18,19:0.356;24,25:0.349
```

in `physical_trolley_alignment_controller.py`, through
`side_target_offset_for_pair()`. However, the final
`physical_staged_trolley_mission.launch.py` does not currently expose or pass
that parameter. Therefore this command is not valid against the present launch
file:

```text
side_target_pair_offsets_x:='18,19:0.356;24,25:0.349'
```

To enable it in future work, add a `LaunchConfiguration`, pass it in the
controller parameter dictionary, and add a `DeclareLaunchArgument` in
`physical_staged_trolley_mission.launch.py`. Until then,
`side_target_offset_x` applies to both pairs.

## Stage 2: Drive Under

`drive_straight_under` is direct velocity control, not a Nav2 goal.

Important behavior:

- publishes to `/cmd_vel`
- also publishes to `/cmd_vel_joy` for the configured twist-mux override
- sets `/holonomic_lidar_ignore_radius` to `1.25 m` during the movement
- captures the starting pose in `odom`
- drives in robot-local Y at `0.070 m/s`
- corrects local X drift with odometry
- restores the lidar ignore radius to zero when finished or stopped

Default stop conditions:

- maximum odometry distance `1.30 m`, or
- a valid front/rear underside-marker estimate after at least `0.20 m`
  progress because `drive_under_stop_on_under_markers=true`

The marker stop allows fine alignment to begin before the robot drives too far
through the trolley.

## Stage 3: Fine Alignment

Fine alignment estimates the trolley center from front and rear underside
markers. The final tested settings are orientation-aware:

| Target | Normal trolley | Flipped 180 degrees |
| --- | ---: | ---: |
| X | `0.1325 m` | `0.134 m` |
| Y | `0.0062 m` | `0.0038 m` |
| Yaw | `-11.0 deg` | `165.0 deg` |

Orientation is selected from the estimated trolley yaw:

```text
cos(yaw) < 0 -> flipped_180
otherwise    -> normal
```

With `fine_control_mode:=step`, the controller:

1. Measures a stable estimate.
2. Sends one bounded motion pulse.
3. Stops and waits for `fine_step_settle_sec`.
4. Measures again before choosing the next pulse.

With `fine_sequential_alignment:=true`, correction order is:

1. yaw
2. Y
3. X

This avoids trying to correct all axes at once. The final tolerances are:

```text
X:   0.004 m
Y:   0.010 m
yaw: 3.0 deg
```

`fine_use_all_under_markers:=true` lets the estimate use all suitable current
underside observations rather than requiring only the preferred pairs.

If the pair disappears after it has already been seen, step mode holds position
or performs the configured slow marker-reacquisition motion. It does not
continue full-speed blind correction.

## Debug Topics

Camera images:

```text
/camera_front/image_raw
/camera_rear/image_raw
/camera_left/image_raw
/camera_right/image_raw
```

Detected markers:

```text
/front/aruco_markers
/rear/aruco_markers
/left/aruco_markers
/right/aruco_markers
```

Annotated images:

```text
/front/aruco_image
/rear/aruco_image
/left/aruco_image
/right/aruco_image
```

Motion and safety:

```text
/cmd_vel
/cmd_vel_joy
/holonomic_lidar_ignore_radius
/odom
/tf
/tf_static
```

Useful commands:

```bash
ros2 topic echo /physical_camera_pair_status
ros2 topic echo /physical_docking_status
ros2 topic echo /left/aruco_markers
ros2 topic echo /front/aruco_markers
ros2 topic echo /rear/aruco_markers
```

Start the browser debug stream:

```bash
ros2 run gr4_camera_config aruco_web_stream
```

Avoid launching a second camera/ArUco launch file at the same time as
`physical_staged_trolley_mission.launch.py`. The staged launch already starts
the physical camera node and all four ArUco processing nodes. Opening the same
USB camera twice can cause `No frame from ...` errors.

## Camera Mapping And Identification

The expected by-path mapping is stored in:

```text
src/gr4_camera_config/config/camera_mapping.yaml
```

Current reference:

```text
left  -> /dev/video0
right -> /dev/video2
rear  -> /dev/video4
front -> /dev/video6
```

The stable `/dev/v4l/by-path/...` entries are more important than the
`/dev/videoN` numbers, which can change after reboot or reconnect.

Identify cameras:

```bash
ros2 run gr4_camera_config identify_cameras
```

If a camera repeatedly reports no frames:

1. Stop every launch that may own the cameras.
2. Check the by-path devices.
3. Start only the staged launch.
4. Watch `/physical_camera_pair_status`.
5. Verify that the requested camera image topic is publishing.

## Calibration And Reference Logging

The controller contains a physical marker layout. Calibration logs should be
used to validate final offsets rather than modifying marker coordinates from a
single frame.

Available tools:

```bash
ros2 run gr4_camera_config trolley_reference_logger
ros2 run gr4_camera_config single_aruco_reference_logger
```

The reference logger data is used to determine:

- target X/Y under the trolley
- target yaw for normal and flipped orientations
- stable marker pairs
- camera/TF consistency

When collecting a reference:

1. Place the robot manually in the desired physical center.
2. Keep it stationary.
3. Ensure both front and rear cameras see underside markers.
4. Collect multiple samples.
5. Compare mean and standard deviation, not one reading.
6. Record whether the trolley is normal or flipped.

## Safety And Troubleshooting

- Test new parameters with `enable_motion:=false`.
- Keep an operator ready to send `stop`.
- Do not stand in the trolley/robot motion path.
- `stop` publishes zero velocity and restores the lidar ignore radius.
- Fine alignment requires fresh transforms from camera optical frames to
  `base_link`.
- Poor lighting can make estimates disappear or cause a timeout.
- A visible marker image does not guarantee a stable geometry estimate; check
  `/physical_docking_status` and the controller log reasons.
- If velocity becomes too small to move the physical base, adjust the minimum
  velocity parameters carefully. The final fine yaw minimum is `0.035 rad/s`.

## Rebuild

After camera package edits:

```bash
cd ~/gr4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select gr4_camera_config
source install/setup.bash
```
