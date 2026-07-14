# turtlebot_adapted

This package is the TurtleBot3/Gazebo simulation version of the nursing-home trolley workflow. It was used to test the delivery-zone navigation flow before running the physical robot with ArUco docking.

The main package path is:

```bash
~/Nursing-Home-Robot/src/turtlebot_adapted
```

## What This Package Starts

The main launch file is:

```bash
launch/turtlebot_zones.launch.py
```

It starts:

- TurtleBot3 Waffle simulation through Nav2's `tb3_simulation_launch.py`.
- Gazebo delivery-zone floor markers from `turtlebot_adapted/spawn_zone_marks.py`.
- The trolley mission controller from `turtlebot_adapted/zones_and_delivery.py`.
- An object-detection node from the `object_detection` package.

The important topics are:

- `/trolley_command`: command input for the mission.
- `/detect_trolley_request`: request sent by the mission when it wants trolley classification.
- `/delivery_zones`: RViz marker array for the charging, pickup, trash, and laundry zones.

## Build

From the workspace root:

```bash
cd ~/Nursing-Home-Robot
source /opt/ros/humble/setup.bash
colcon build --packages-select turtlebot_adapted object_detection
source install/setup.bash
```

If you copied this package into another workspace, use that workspace root instead of `~/Nursing-Home-Robot`.

## Run The Full Simulation

Terminal 1:

```bash
cd ~/Nursing-Home-Robot
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch turtlebot_adapted turtlebot_zones.launch.py
```

The launch file sets `TURTLEBOT3_MODEL=waffle` automatically. If you run TurtleBot pieces manually, set it yourself:

```bash
export TURTLEBOT3_MODEL=waffle
```

Wait around 30 seconds after launch. The launch intentionally delays helper nodes so Gazebo and Nav2 have time to start.

## Run The Trolley Flow

Terminal 2:

```bash
cd ~/Nursing-Home-Robot
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Start the pickup flow:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trolley_ready'}"
```

The robot should drive from the waiting/charging zone to the pickup zone. When it reaches pickup, `zones_and_delivery.py` publishes `true` on `/detect_trolley_request` and waits for a trolley type command.

Send one of these after detection:

```bash
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'laundry_trolley'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'trash_trolley'}"
ros2 topic pub --once /trolley_command std_msgs/msg/String "{data: 'unknown_trolley'}"
```

Behavior:

- `laundry_trolley`: drive to `LAUNDRY_DROP_ZONE`, then return to charging.
- `trash_trolley`: drive to `TRASH_DROP_ZONE`, then return to charging.
- `unknown_trolley`: skip delivery and return to charging.

## Zone Coordinates

The simulation zones are defined directly in:

```bash
turtlebot_adapted/zones_and_delivery.py
```

Current values:

| Zone | x | y | yaw |
| --- | ---: | ---: | ---: |
| `WAITING_CHARGING_ZONE` | -2.0 | -0.5 | 0.0 |
| `PICKUP_ZONE` | 0.5 | 0.0 | 0.0 |
| `TRASH_DROP_ZONE` | 1.8 | 0.8 | 1.57 |
| `LAUNDRY_DROP_ZONE` | 1.8 | -0.8 | -1.57 |

Gazebo visual markers for these zones are spawned by:

```bash
turtlebot_adapted/spawn_zone_marks.py
```

If you change zone positions in `zones_and_delivery.py`, also update `spawn_zone_marks.py` so Gazebo and RViz show the same locations.

## Run Individual Nodes

Mission controller only:

```bash
ros2 run turtlebot_adapted zones_and_delivery
```

Spawn Gazebo zone markers only:

```bash
ros2 run turtlebot_adapted spawn_zone_marks
```

Optional robot visual shell:

```bash
ros2 run turtlebot_adapted robot_shell_spawner --ros-args \
  --params-file ~/Nursing-Home-Robot/src/turtlebot_adapted/config/robot_shell.yaml
```

The visual shell is configured in:

```bash
config/robot_shell.yaml
urdf/nursing_home_shell.xacro
```

## RViz Checks

To see the simulated delivery zones in RViz, add a `MarkerArray` display and set:

```bash
/delivery_zones
```

The markers use `map` as their frame.

## Object Detection Note

The launch file starts the simulation-only detector:

```bash
package="object_detection"
executable="trolley_image_detector.py"
```

This detector does not use a live camera. It waits for `/detect_trolley_request`, picks random images from:

```bash
~/Nursing-Home-Robot/src/object_detection/test_images/trolley_rotated
```

and falls back to:

```bash
~/Nursing-Home-Robot/src/object_detection/test_images/trolleys
```

It then publishes one of these commands on `/trolley_command`:

- `laundry_trolley`
- `trash_trolley`
- `unknown_trolley`

Check that the detector is installed with:

```bash
ros2 pkg executables object_detection
```

## Troubleshooting

If the robot does not move:

```bash
ros2 topic echo /trolley_command
ros2 topic list | grep nav
```

Check that Nav2 is active and that the command was published exactly as `trolley_ready`.

If the zone markers do not appear in Gazebo:

```bash
ros2 service list | grep spawn_entity
ros2 run turtlebot_adapted spawn_zone_marks
```

If RViz does not show the zones, add `/delivery_zones` as a `MarkerArray` display and make sure the fixed frame is `map`.

If the object detector does not respond, first check that the mission node requested detection:

```bash
ros2 topic echo /detect_trolley_request
```

Then check which object-detection executable is available:

```bash
ros2 pkg executables object_detection
```
