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
```

Aliases also work: `charge`, `home`, `trolley`, `pickup`, `laundry`, and `trash`.

The node publishes each selected zone as a `PoseStamped` on `/goal_pose_raw`, matching the Rocket workspace's goal-restamp flow.

`trolley_ready` runs the real delivery flow:

1. Publish the `trolley_dropzone` goal.
2. Wait `trolley_drive_wait_sec`.
3. Classify the configured test image using `object_detection/scripts/trolleys.pt`.
4. Publish either `laundry_dropoff` or `trash_dropoff`.

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
