# Barometer and Elevator Floor Detection

This guide covers the complete workflow:

1. Calibrate each elevator floor.
2. Save the calibration from the ESP32.
3. Load that calibration during normal robot operation.
4. Recalibrate atmospheric pressure from any known floor.
5. Save and understand the collected CSV data.

## Important rules

- Run only one serial node at a time. Do not run
  `esp32_serial_bridge` and `elevator_csv_calibration_logger` together.
- The ESP32 and ROS nodes use serial baud rate `115200`.
- Known-floor recalibration must use the robot's actual physical floor.
- Keep the robot still while calibrating or recalibrating.
- Wait until the automatic replacement measurement has completed before
  moving away from a floor during manual calibration.
- The normal calibration logger clears the ESP32 log when it starts. Dump any
  ESP32 data that you want to keep before restarting the logger.

## Build the ROS package

Run this after installing the package or changing its source:

```bash
cd ~/gr4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select elevator_interaction
source install/setup.bash
```

Source the workspace in every terminal that will use ROS:

```bash
source /opt/ros/humble/setup.bash
source ~/gr4_ws/install/setup.bash
```

## ROS topics

The barometer nodes use these topics:

| Topic | Direction | Purpose |
| --- | --- | --- |
| `/esp32_command` | ROS to ESP32 | Sends serial commands |
| `/esp32_rx` | ESP32 to ROS | Publishes every received serial line |
| `/csv_saved` | ROS output | Publishes the path of a saved CSV dump |
| `/elevator_logger_status` | ROS output | Reports calibration and recalibration status |

To monitor ESP32 output:

```bash
ros2 topic echo /esp32_rx
```

To monitor logger status:

```bash
ros2 topic echo /elevator_logger_status
```

To monitor saved file paths:

```bash
ros2 topic echo /csv_saved
```

The examples below use this command format:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'COMMAND'}"
```

## Part 1: Create a new floor calibration

Use `esp32_serial_bridge` for manual calibration. It forwards commands without
automatically loading an old calibration.

### 1. Start the serial bridge

```bash
ros2 run elevator_interaction esp32_serial_bridge
```

In a second sourced terminal, monitor the ESP32:

```bash
ros2 topic echo /esp32_rx
```

### 2. Enter barometer mode and clear old data

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'baro_on'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'resetcal'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'clearlog'}"
```

### 3. Calibrate floor 0

Move the robot to floor 0 and keep it still. Then run:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'floor0'}"
```

The ESP32 averages multiple sensor samples and stores the floor-0 pressure.

The ROS serial bridge deliberately sends `floor0` twice:

1. The first measurement warms and settles the BMP sensor/filter.
2. Three seconds later, the bridge automatically sends a replacement
   `floor0` measurement.

The CSV therefore contains two calibration rows for floor 0. This is expected.
The calibration loader uses the last row, not the warm-up row. Do not move the
robot until the terminal reports:

```text
floor0: sending settled replacement measurement
```

### 4. Calibrate every other floor

Move the robot to each floor, keep it still, and send the matching command:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'floor1'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'floor2'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'floor3'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'floor4'}"
```

Commands such as `floor 3` are also accepted by the ESP32.

The same warm-up/replacement behavior is applied to every `floorN` command.
Wait for the replacement measurement before moving to the next floor.

Check the stored values:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'overview'}"
```

### 5. Finish the calibration

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'end'}"
```

After `end`, the ESP32 starts estimating the current floor, printing live
barometer readings, and appending `read` rows to its CSV log.

### 6. Test the calibration

While physically at a known floor, send its `actual` command:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'actual0'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'actual1'}"
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'actual2'}"
```

The ESP32 compares the expected floor with its estimated floor and appends a
`test` row to the CSV.

### 7. Download the calibration CSV

Ask the ESP32 to print its complete CSV:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'dumpcsv'}"
```

The serial bridge captures everything between:

```text
========== CSV START ==========
========== CSV END ==========
```

It saves the result as:

```text
~/gr4_ws/elevator_logs/floor_log_YYYYMMDD_HHMMSS.csv
```

The exact saved path is also published on `/csv_saved`.

Stop the serial bridge before starting another serial node:

```text
Ctrl+C
```

### 8. Select the calibration used by normal operation

The normal logger expects:

```text
~/gr4_ws/elevator_logs/calibration.csv
```

Copy the newly verified calibration file to that name:

```bash
cp ~/gr4_ws/elevator_logs/floor_log_YYYYMMDD_HHMMSS.csv \
  ~/gr4_ws/elevator_logs/calibration.csv
```

Replace the timestamp with the actual filename. The ROS node reads rows whose
`type` is `calibration`, then extracts `calibrated_floor` and `pressure_Pa`.
If a file contains multiple calibration rows for one floor, the last row for
that floor is used.

## Part 2: Normal run with the saved calibration

Make sure this file exists first:

```bash
ls -l ~/gr4_ws/elevator_logs/calibration.csv
```

Start the calibration logger:

```bash
ros2 run elevator_interaction elevator_csv_calibration_logger
```

At startup it automatically:

1. Opens the ESP32 serial port.
2. Reads `~/gr4_ws/elevator_logs/calibration.csv`.
3. Sends `baro_on`.
4. Sends `resetcal`.
5. Sends `clearlog`.
6. Sends one `setcal FLOOR PRESSURE_PA` command for each calibrated floor.
7. Sends `overview`.
8. Sends `end`.

After this sequence, the ESP32 continuously prints and logs live barometer
readings.

To reload the original CSV calibration without restarting the ROS node:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String \
  "{data: 'load_csv_calibration'}"
```

### Record known-floor checks

At any known floor:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'actual0'}"
```

Change the number to the robot's physical floor.

### Save the normal-run data

Before stopping the node, dump the ESP32 log:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'dumpcsv'}"
```

The calibration logger saves it as:

```text
~/gr4_ws/elevator_logs/loaded_calibration_test_YYYYMMDD_HHMMSS.csv
```

The node does not automatically dump the ESP32 CSV when it shuts down.

Do not replace `calibration.csv` with a
`loaded_calibration_test_YYYYMMDD_HHMMSS.csv` normal-run dump. Normal startup
uses `setcal`, and the ESP32 does not write `setcal` values back as
`calibration` rows. Keep the verified manual-calibration file as the reusable
`calibration.csv`.

## Part 3: Recalibrate from a known floor

`recalibrate_known_floor` compensates for weather and pressure drift without
changing the pressure spacing between floors. Any floor present in
`calibration.csv` can be used as the reference.

### 1. Start normal operation

```bash
ros2 run elevator_interaction elevator_csv_calibration_logger
```

Wait until the calibration has loaded and live pressure output has started.

### 2. Place the robot at a known floor

The robot must be stationary, and the command's floor number must match its
actual physical floor. Supplying the wrong floor shifts the complete
calibration incorrectly.

### 3. Recalibrate using that floor

For example, while physically at floor 3:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String \
  "{data: 'recalibrate_known_floor 3'}"
```

The ROS node then:

1. Discards the first two fresh readings to avoid sensor/filter transients.
2. Collects seven additional pressure readings.
3. Waits until those readings have a spread of no more than `8 Pa`.
4. Uses the median of the seven stable readings.
5. Compares that median with the selected floor from the original
   `calibration.csv`.
6. Calculates:

   ```text
   pressure_shift =
       current_reference_pressure - original_reference_pressure
   ```

7. Applies the same shift to every original floor pressure:

   ```text
   adjusted_pressure[floor] =
       original_pressure[floor] + pressure_shift
   ```

8. Sends `resetcal`.
9. Sends the adjusted `setcal` values.
10. Sends `overview`.
11. Sends `end`.

Example:

```text
Known physical floor: 1
Original: floor0=101 Pa, floor1=103 Pa, floor2=106 Pa
Current floor1 pressure: 102 Pa
Shift: -1 Pa
Adjusted: floor0=100 Pa, floor1=102 Pa, floor2=105 Pa
```

Every known-floor recalibration starts from the original calibration held in
Python memory. Repeated commands do not accumulate previous shifts.

The sampling defaults can be changed with ROS parameters:

```bash
ros2 run elevator_interaction elevator_csv_calibration_logger --ros-args \
  -p recalibration_discard_samples:=2 \
  -p recalibration_sample_count:=7 \
  -p recalibration_max_spread_pa:=8.0 \
  -p recalibration_timeout_seconds:=30.0
```

Monitor the result:

```bash
ros2 topic echo /elevator_logger_status
```

A successful operation publishes a message similar to:

```text
recalibration_applied:reference_floor=3,shift_pa=-25.40
```

### Save data after recalibration

Use the same dump command:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'dumpcsv'}"
```

This saves another:

```text
~/gr4_ws/elevator_logs/loaded_calibration_test_YYYYMMDD_HHMMSS.csv
```

Known-floor recalibration does not overwrite `calibration.csv`. Its adjusted
pressures are runtime values in the ESP32 and Python node. Restarting the
logger loads the original calibration again. This preserves the measured
relationship between floors while allowing a fresh atmospheric shift on each
run.

## How CSV values are stored

The ESP32 stores its active log in LittleFS as:

```text
/floor_log.csv
```

The important row types are:

| `type` | Created when | Important values |
| --- | --- | --- |
| `calibration` | A physical `floorN` calibration is measured | Pressure and calibrated floor |
| `read` | Normal live barometer operation after `end` | Pressure, height and estimated floor |
| `test` | An `actualN` command is sent | Estimated floor, actual floor and correctness |

CSV columns:

| Column | Meaning |
| --- | --- |
| `type` | `calibration`, `read`, or `test` |
| `time_ms` | ESP32 milliseconds since boot |
| `temperature_C` | Sensor temperature |
| `pressure_Pa` | Pressure in pascals; used for calibration loading |
| `pressure_hPa` | The same pressure in hectopascals |
| `relative_height_m` | Calculated height relative to floor 0 |
| `estimated_floor` | Closest calibrated floor |
| `offset_from_estimated_floor_m` | Height difference from that floor |
| `actual_floor` | Known floor supplied using `actualN` |
| `correct` | `1` when estimated and actual floors match, otherwise `0` |
| `calibrated_floor` | Floor number saved by a physical `floorN` calibration |

Important storage behavior:

- `clearlog` permanently deletes the current ESP32 CSV and creates a new one.
- Physical `floorN` commands append `calibration` rows.
- Loading values with `setcal` changes runtime calibration but does not append
  calibration rows.
- Known-floor recalibration changes runtime calibration but does not overwrite
  `calibration.csv`.
- `dumpcsv` copies the current ESP32 LittleFS log to the robot computer.
- The ROS nodes publish the new computer-side filename on `/csv_saved`.

## Leaving barometer mode

The ESP32 rejects normal motor commands while barometer mode is active. To
return it to robot mode:

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'baro_off'}"
```

Live barometer logging stops in robot mode.

## Troubleshooting

### Calibration file not found

Check:

```bash
ls -l ~/gr4_ws/elevator_logs/calibration.csv
```

The logger reports the error on `/elevator_logger_status`.

### Serial port is busy

Stop any other program using the ESP32, especially the other serial ROS node.
Only one process can own the serial port.

### Known-floor recalibration times out

Confirm that:

- The ESP32 is connected.
- Calibration finished successfully.
- Live output contains lines such as `Pressure: 101325.00 Pa`.
- The robot is in barometer mode.

### Check the ESP32 mode

```bash
ros2 topic pub --once /esp32_command std_msgs/msg/String "{data: 'mode'}"
```

The response appears on `/esp32_rx` as either `MODE robot` or
`MODE barometer`.

## Copy changes from the development laptop to the robot

The source in this workspace is on the development laptop. Copy the changed
Python nodes to the robot:

```bash
scp \
  ~/gr4_ws/src/elevator_interaction/elevator_interaction/esp32_serial_bridge.py \
  ~/gr4_ws/src/elevator_interaction/elevator_interaction/elevator_csv_calibration_logger.py \
  rocket@192.168.8.221:~/gr4_ws/src/elevator_interaction/elevator_interaction/
```

Copy the package files:

```bash
scp \
  ~/gr4_ws/src/elevator_interaction/setup.py \
  ~/gr4_ws/src/elevator_interaction/package.xml \
  ~/gr4_ws/src/elevator_interaction/BAROMETER_README.md \
  rocket@192.168.8.221:~/gr4_ws/src/elevator_interaction/
```

Then connect to the robot and rebuild:

```bash
ssh rocket@192.168.8.221
cd ~/gr4_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select elevator_interaction
source install/setup.bash
```

Restart the running barometer node after rebuilding.

The automatic repeated `floorN` protection exists in the ROS serial bridge.
If calibration is performed directly through Arduino Serial Monitor instead,
the current ESP32 firmware still uses its first sensor samples. In that case,
manually issue each `floorN` command twice and use the second calibration row,
or add discard samples inside the ESP32 firmware.
