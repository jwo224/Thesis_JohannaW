# Real Robot Mapping Workflow

These commands keep `rocket_ws` unchanged. Run them in separate terminals on the robot unless noted otherwise.

## 1. Start SLAM on the robot

```bash
~/gr4_ws/scripts/start_slam_mapping.sh
```

Wait until the lidars, `/odom`, `/merged`, and `slam_toolbox` are running. If CAN setup fails, bring CAN up manually:

```bash
sudo ip link set can0 up type can bitrate 250000
```

## 2. Open keyboard teleop

In a second terminal:

```bash
~/gr4_ws/scripts/start_keyboard_teleop.sh
```

Useful keys:

- `i` forward, `,` backward, `j` rotate left, `l` rotate right
- hold shift for holonomic movement: `J` strafe left, `L` strafe right
- `k` or any unmapped key stops
- `q/z`, `w/x`, `e/c` adjust speed while teleop is running

Drive slowly. For a clean map, trace the outside walls first, then fill the middle, and finish by returning near the start so loop closure can correct drift.

## 3. Watch the map in RViz

On a dev PC on the same ROS network:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=138
rviz2
```

Set fixed frame to `map`. Add displays for `/map`, `/merged`, `/odom`, and TF if they are not visible.

## 4. Save the map

When the map looks good:

```bash
~/gr4_ws/scripts/save_slam_map.sh mekagangen
```

This writes:

```text
/home/rocket/maps/mekagangen.yaml
/home/rocket/maps/mekagangen.pgm
```

The save path is interpreted by the running `slam_toolbox` process, so use a path that exists on the robot.

## 5. Test navigation with the saved map

Stop SLAM, then launch Nav2 with the map:

```bash
~/gr4_ws/scripts/start_nav_with_map.sh /home/rocket/maps/mekagangen.yaml
```

In RViz, set the robot initial pose with `2D Pose Estimate`, then send goals. The `rocket_ws` launch includes a restamp helper, so for command-line goals publish to `/goal_pose_raw`.

Example:

```bash
~/gr4_ws/scripts/send_map_goal.sh 1.8 0.8 1.57
```

Once this works, use the same `map` coordinates for the drop zones in the Nursing Home Robot command node.
