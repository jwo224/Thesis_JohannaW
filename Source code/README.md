# Source Code Folder Index

Cleaned: 2026-07-04

This folder is organized so thesis reviewers and future students can find the
working code without digging through generated build output.

## Main Folders

| Folder | Purpose |
| --- | --- |
| `ROS2/` | ROS 2 Humble workspaces and ROS-related support files. |
| `Arduino/` | ESP32/Arduino elevator, barometer, FSR, BLE, Wi-Fi, and servo code. |
| `Datasets/` | YOLO trolley training notebooks, datasets, trained model outputs, and labelling tools. |
| `Assets/` | Floorplans and other non-code support assets. |
| `Results_And_Logs/` | Experiment logs, detection outputs, mission logs, CSVs, and result bundles moved out of workspaces. |
| `Methods_Appendix_Code_20260703/` | Curated thesis appendix snapshot created on the Linux development PC. |
| `Reference_Snapshots/` | Older/reference code snapshots and archives that are useful for traceability but are not the primary workspaces. |

## Cleanup Notes

- ROS generated `build/`, `install/`, `log/`, `__pycache__`, and `.pytest_cache`
  folders were removed from the ROS2 workspaces.
- Duplicate dated ROS2 workspace copies under the old nested `ROS2/Documents`
  tree were removed after result/log material was moved to `Results_And_Logs/`.
- ROS package-internal folder structures were preserved so relative imports,
  launch files, package resources, and model paths remain intact.
- YOLO model weights used directly by ROS packages were left inside those
  packages.
