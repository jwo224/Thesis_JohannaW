# ROS 2 Humble Workspaces

This folder contains the ROS 2 Humble workspaces kept as working references.

## Workspaces

| Folder | Meaning | Build from |
| --- | --- | --- |
| `humble_robot_final_ws/` | Final robot workspace. Contains camera config, ArUco, YOLO, dropzone mission, elevator interaction, and related tools. | `ROS2/humble_robot_final_ws` |
| `humble_dev_ws/gr4_ws/` | Development robot workspace. Includes development packages such as `gr4_trolley_vision`. | `ROS2/humble_dev_ws/gr4_ws` |
| `Nursing-Home-Robot/Nursing-Home-Robot/` | Simulation workspace from the original repository. Windows refused renaming the outer folder, so the original container name was kept. | `ROS2/Nursing-Home-Robot/Nursing-Home-Robot` |

## Build Reminder On Linux

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build
source install/setup.bash
```
