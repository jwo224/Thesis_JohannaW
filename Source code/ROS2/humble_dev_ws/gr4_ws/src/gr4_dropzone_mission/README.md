# GR4 Dropzone Mission

Publishes four real-map zones to RViz and sends goal poses from simple text commands.

## Run

Start Nav2 on the robot first:

```bash
ros2 launch articubot_one nav.launch.py map:=/home/rocket/maps/viggo.yaml
```

Then run this node from `gr4_ws` on the robot or the dev PC:

```bash
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash
export ROS_DOMAIN_ID=138
ros2 launch gr4_dropzone_mission dropzone_mission.launch.py
```

## RViz

Add a `MarkerArray` display:

```text
Topic: /delivery_zones
```

## Commands

Publish one of these to `/trolley_command`:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'charging'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trolley_dropzone'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'laundry_dropoff'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trash_dropoff'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'cancel'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trolley_ready'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'side_align'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'drive_under'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'fine_align'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'status'}"
```

Aliases also work: `charge`, `home`, `trolley`, `pickup`, `laundry`, and `trash`.

The node publishes each selected zone as a `PoseStamped` on `/goal_pose_raw`, matching the Rocket workspace's goal-restamp flow.
It also publishes a small `TROLLEY APPROACH` rectangle in RViz at the staged approach pose.
Mission progress is published on `/dropzone_mission_status`.

`trolley_ready` starts the physical staged dropzone flow, but every movement after the
approach goal waits for your approval:

1. Publish a staged goal at `trolley_dropzone` plus `trolley_stage_offset_x/y`.
2. Stop the mission flow and wait for `side_align`.
3. `side_align` publishes `side_align_command` to `/physical_docking_command`.
4. When `/physical_docking_status` reports `<side_align_command>_done`, wait for `drive_under`.
5. `drive_under` publishes `final_drive_command`.
6. When `/physical_docking_status` reports `drive_straight_under_done`, wait for `fine_align`.
7. `fine_align` publishes `fine_align_command`.

The staged pose is computed in the map frame:

```text
stage_x = trolley_dropzone.x + trolley_stage_offset_x
stage_y = trolley_dropzone.y + trolley_stage_offset_y
```

The staged yaw is computed from `trolley_stage_camera` so the selected camera looks
toward the trolley dropzone. With `trolley_stage_camera: left`, the left camera points
straight into the trolley zone at the approach pose.

For physical ArUco alignment, run the staged camera/controller launch as well:

```bash
ros2 launch gr4_camera_config physical_staged_trolley_mission.launch.py enable_motion:=true
```

`enable_motion:=false` keeps the controller in debug mode: it logs estimates and
publishes zero `cmd_vel`.

The old image-classification flow is still available through:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'detect_trolley'}"
```

Put the image you want classified at:

```text
src/object_detection/test_images/current_trolley.jpg
```

or change `test_image_path` / `test_image_directory` in `config/dropzones.yaml`.

## Tune Coordinates

Edit:

```text
config/dropzones.yaml
```

Then rebuild and source:

```bash
colcon build --symlink-install --packages-select gr4_dropzone_mission
source install/setup.bash
```
