# Elevator Interaction Tests and Loggers

This directory contains separate tools for:

- Normal Wi-Fi operation
- Normal BLE operation
- Wi-Fi distance testing
- BLE distance testing
- Adaptive Wi-Fi FSR calibration
- Legacy serial calibration

Run all commands from the workspace:

```bash
cd ~/gr4_ws
```

## Quick selection guide

| What you want to do | Script |
|---|---|
| Press/read the normal ESP32-C3 over Wi-Fi and log it | `normal_elevator_wifi_logger.py` |
| Control the normal XIAO over BLE and log it | `normal_elevator_ble_logger.py` |
| Test Wi-Fi range from 1–10 m | `distance_check_wifi.py` |
| Test BLE range from 1–10 m | `distance_check_ble.py` |
| Run adaptive Wi-Fi threshold calibration | `fsr_calibration_wifi_logger.py` |
| Run the old USB-serial calibration firmware | `fsr_calibration_serial_logger.py` |

Do not use the adaptive calibration logger with the normal fixed-threshold
firmware. The normal firmware has no `/result` endpoint and does not ask for a
successful/unsuccessful judgment.

## Requirements

The Wi-Fi tools require `requests`:

```bash
python3 -m pip install requests
```

The BLE tools require `bleak` and a working Linux Bluetooth adapter:

```bash
python3 -m pip install bleak
```

The legacy serial logger requires `pyserial`:

```bash
python3 -m pip install pyserial
```

Only one application should connect to the BLE peripheral at a time. Close
phone apps, BLE terminals, and other logger instances before testing.

---

## 1. Normal Wi-Fi logger

File: `normal_elevator_wifi_logger.py`

Use this with the normal fixed-threshold ESP32-C3 Wi-Fi firmware exposing:

- `/press`
- `/status`
- `/fsr`

The default ESP32 address is configured near the top of the script:

```python
ESP32_IP = "192.168.8.226"
```

Start it with:

```bash
python3 normal_elevator_wifi_logger.py
```

Commands:

| Input | Action |
|---|---|
| Enter or `p` | Press the elevator button |
| `s` | Request status |
| `f` | Read the FSR |
| `q` | Save and quit |

CSV output:

```text
~/gr4_ws/elevator_logs/normal_elevator_wifi_YYYYMMDD_HHMMSS.csv
```

The CSV records HTTP success, response time, threshold result, trigger angle,
FSR values, status fields, and the complete response.

The current normal Wi-Fi firmware only records ESP32-to-router RSSI if its HTTP
responses include:

```text
wifi_rssi_dbm=-67
```

Otherwise, the `wifi_rssi_dbm` column remains empty.

### One-shot Wi-Fi commands

The existing ROS package client can be used when no CSV log is needed:

```bash
source install/setup.bash
ros2 run elevator_interaction esp32_elevator_client press
ros2 run elevator_interaction esp32_elevator_client status
ros2 run elevator_interaction esp32_elevator_client fsr
```

---

## 2. Normal BLE logger

File: `normal_elevator_ble_logger.py`

Use this with the XIAO firmware advertising:

```text
XIAO-FSR-SERVO
```

The logger auto-discovers the peripheral by name or Nordic UART service.

Start it with:

```bash
python3 normal_elevator_ble_logger.py
```

Commands:

| Input | BLE command |
|---|---|
| Enter or `p` | `PRESS` |
| `s` | `STATUS` |
| `f` | `FSR` |
| `o` | `OPEN` |
| `c` | `CLOSE` |
| `x` | `STOP` |
| `GOTO 90` | Move the servo to 90 degrees |
| `THRESH 100` | Change the FSR threshold to 100 |
| `q` | Save and quit |

Example threshold change:

```text
BLE command: THRESH 100
```

CSV output:

```text
~/gr4_ws/elevator_logs/ble_normal/normal_elevator_ble_YYYYMMDD_HHMMSS.csv
```

The CSV records BLE RSSI, discovery time, connection time, first-response time,
complete response time, threshold, FSR, servo angle/state, and errors.

### One-shot BLE commands

The existing ROS package client can be used without the logger:

```bash
source install/setup.bash
ros2 run elevator_interaction ble_elevator_client PRESS
ros2 run elevator_interaction ble_elevator_client STATUS
ros2 run elevator_interaction ble_elevator_client FSR
ros2 run elevator_interaction ble_elevator_client "THRESH 100"
```

---

## 3. Wi-Fi distance test

File: `distance_check_wifi.py`

This interactively tests distances from 1 through 10 metres. At every distance,
move the robot into position and press Enter. The script sends `/press` and
waits for the completed response.

Start it with:

```bash
python3 distance_check_wifi.py
```

Custom example:

```bash
python3 distance_check_wifi.py \
  --attempts 3 \
  --timeout 15 \
  --distances 1 2 3 4 5 6 7 8 9 10
```

CSV output:

```text
~/gr4_ws/elevator_logs/wifi_distance_checks/distance_check_wifi_YYYYMMDD_HHMMSS.csv
```

Important measurements include:

- TCP connection probe time
- Time until HTTP headers/first answer
- Response-body download time
- Total HTTP time
- Complete attempt time
- Threshold result, angle, and FSR values
- ESP32-to-router RSSI when the firmware returns `wifi_rssi_dbm`
- Robot-to-router Wi-Fi signal before and after the request

The robot Wi-Fi signal and ESP32 Wi-Fi RSSI are different measurements. For
ESP32-to-router RSSI, the ESP32 firmware must return `WiFi.RSSI()`.

---

## 4. BLE distance test

File: `distance_check_ble.py`

This interactively tests distances from 1 through 10 metres using `PRESS`.

Start it with:

```bash
python3 distance_check_ble.py
```

Custom example:

```bash
python3 distance_check_ble.py \
  --attempts 3 \
  --distances 1 2 3 4 5 6 7 8 9 10
```

CSV output:

```text
~/gr4_ws/elevator_logs/ble_distance_checks/distance_check_ble_YYYYMMDD_HHMMSS.csv
```

The logger records:

- BLE RSSI before and after the command
- Number and duration of discovery attempts
- BLE connection time
- Notification setup time
- Command-write time
- Time to first response
- Time from command write until `DONE`
- Disconnect time and total test time
- Threshold, hit result, trigger angle, and raw FSR
- Notification count, complete response, and failure stage

The XIAO firmware currently uses:

```cpp
const int BLE_TX_POWER = 0;
```

This README assumes it remains at `0 dBm`. No TX-power change is required to
run the tests. However, a reading around `-80 dBm` or lower is weak and may
cause slow connections, missed advertisements, merged BLE messages, and failed
trials.

The firmware advertises slowly, so the logger retries discovery and remembers
the address found during the first successful connection.

BLE response fields are compact:

| Response | Meaning |
|---|---|
| `DONE` | Press sequence finished |
| `R 1` / `R 0` | Threshold reached / not reached |
| `A 72` | Trigger/current angle |
| `F 110` | Raw FSR |
| `T 50` | Current threshold |

The BLE firmware does not send FSR voltage, so that value cannot be logged over
BLE without changing the firmware.

---

## 5. Adaptive Wi-Fi calibration logger

File: `fsr_calibration_wifi_logger.py`

Use this only with the adaptive Wi-Fi calibration firmware exposing:

- `/press`
- `/result?success=yes`
- `/result?success=no`
- `/status`
- `/fsr`
- `/reset_calibration`

Start it with:

```bash
python3 fsr_calibration_wifi_logger.py
```

Workflow:

1. Press Enter to run `/press`.
2. Inspect whether the physical elevator-button press succeeded.
3. Type `y` or `n`.
4. The logger sends `/result?success=yes` or `/result?success=no`.
5. `n` increases the adaptive threshold.
6. `y` accepts the threshold and finishes calibration.

Commands:

| Input | Action |
|---|---|
| Enter or `p` | Run a calibration trial |
| `s` | Status |
| `f` | FSR reading |
| `r` | Reset calibration to the initial threshold |
| `q` | Save and quit |

CSV output:

```text
~/gr4_ws/fsr_calibration_wifi_YYYYMMDD_HHMMSS.csv
```

The CSV includes trial number, operator answer, old/new threshold, final
successful threshold, FSR result, angle, RSSI, HTTP timing, and full responses.

After calibration, flash the normal fixed-threshold firmware and use the normal
Wi-Fi logger instead.

---

## 6. Legacy serial calibration logger

File: `fsr_calibration_serial_logger.py`

This is only for firmware that sends seven-column calibration rows over USB
serial:

```text
trial,time_ms,phase,threshold,angle,fsr_raw,fsr_voltage
```

It is not used with the current Wi-Fi or BLE firmware.

Configure the serial port at the top of the script:

```python
SERIAL_PORT = "COM3"
BAUD_RATE = 115200
```

Linux ports will normally look like:

```python
SERIAL_PORT = "/dev/ttyACM0"
```

Start it with:

```bash
python3 fsr_calibration_serial_logger.py
```

Press Enter to send the start character. Type `y` or `n` when that serial
firmware asks. The CSV is saved beside the script as:

```text
fsr_calibration_YYYYMMDD_HHMMSS.csv
```

---

## RSSI reference

RSSI is negative: values closer to zero are stronger.

| RSSI | General interpretation |
|---|---|
| `-30 dBm` | Excellent |
| `-50 dBm` | Strong |
| `-60 dBm` | Good |
| `-70 dBm` | Usable but weakening |
| `-80 dBm` | Weak and potentially unreliable |
| `-90 dBm` | Usually near disconnection |

RSSI varies with antenna orientation, walls, people, batteries, metal chassis,
servo wiring, radio interference, and the robot computer's Bluetooth adapter.

## Troubleshooting

### Wi-Fi request fails

- Confirm the robot and ESP32 use the same router.
- Verify the IP address in the script.
- Test `http://192.168.8.226/status`.
- Confirm the ESP32 firmware exposes the requested endpoint.

### BLE device is not found

- Confirm the XIAO is powered.
- Close phone BLE apps and other clients.
- Wait a few seconds after disconnecting.
- Keep the antenna clear of metal and wiring.
- Confirm the advertised name is `XIAO-FSR-SERVO`.
- Remember that weak RSSI can cause discovery failures before connection.

### BLE total time looks long

Total time includes discovery, connection, the physical servo movement, hold
time, return movement, disconnect, and RSSI scans. Use
`command_to_completion_ms` to evaluate the actual `PRESS` command duration.

### CSV safety

All logger files are flushed after every row. Ctrl+C may stop a test, but
completed rows should remain in the CSV.
