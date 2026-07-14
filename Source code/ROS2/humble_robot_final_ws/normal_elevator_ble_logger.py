#!/usr/bin/env python3
"""Interactive BLE controller/logger for the normal elevator presser firmware."""

import asyncio
import csv
import statistics
import time
from datetime import datetime
from pathlib import Path

try:
    from bleak import BleakClient, BleakScanner
except ImportError:
    print("ERROR: bleak is not installed.")
    print("Install it with: python3 -m pip install bleak")
    raise SystemExit(1)


# ============================== SETTINGS ==============================
DEVICE_NAME = "XIAO-FSR-SERVO"
SCAN_SECONDS = 6.0
CONNECT_TIMEOUT_SECONDS = 10.0
RESPONSE_TIMEOUT_SECONDS = 15.0
# ======================================================================

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

CSV_FIELDS = [
    "timestamp",
    "command",
    "device_name",
    "device_address",
    "success",
    "scan_ms",
    "rssi_last_dbm",
    "rssi_mean_dbm",
    "rssi_min_dbm",
    "rssi_max_dbm",
    "rssi_samples",
    "connect_ms",
    "notify_setup_ms",
    "command_write_ms",
    "time_to_first_response_ms",
    "response_wait_ms",
    "disconnect_ms",
    "total_ms",
    "notification_count",
    "response_line_count",
    "press_finished",
    "threshold",
    "threshold_reached",
    "trigger_angle",
    "fsr_raw",
    "current_angle",
    "servo_state",
    "response",
    "error_stage",
    "error",
]


def rssi_summary(samples):
    if not samples:
        return "", "", "", "", 0
    return (
        samples[-1],
        f"{statistics.fmean(samples):.2f}",
        min(samples),
        max(samples),
        len(samples),
    )


async def discover_device():
    found_device = None
    samples = []
    started = time.perf_counter()

    def detection_callback(device, advertisement_data):
        nonlocal found_device
        name = advertisement_data.local_name or device.name or ""
        services = {
            value.lower()
            for value in (advertisement_data.service_uuids or [])
        }
        if (
            name.casefold() == DEVICE_NAME.casefold()
            or UART_SERVICE_UUID in services
        ):
            found_device = device
            if advertisement_data.rssi is not None:
                samples.append(advertisement_data.rssi)

    scanner = BleakScanner(detection_callback=detection_callback)
    await scanner.start()
    try:
        await asyncio.sleep(SCAN_SECONDS)
    finally:
        await scanner.stop()

    return found_device, samples, (time.perf_counter() - started) * 1000


def command_complete(command, lines):
    base = command.split(maxsplit=1)[0]
    if base == "PRESS":
        return "DONE" in lines
    if base == "STATUS":
        return any(line in {"S ATTACHED", "S DETACHED"} for line in lines)
    if base == "FSR":
        return any(line.startswith("F ") for line in lines)
    if base == "OPEN":
        return "OPENED" in lines
    if base == "CLOSE":
        return "CLOSED" in lines
    if base == "STOP":
        return any(line in {"STOP", "STOPPED"} for line in lines)
    if base == "GOTO":
        return "GOTO_DONE" in lines and any(line.startswith("A ") for line in lines)
    if base == "THRESH":
        return any(line.startswith("THRESH ") for line in lines)
    return bool(lines)


def parse_response(lines):
    result = {
        "press_finished": "yes" if "DONE" in lines else "no",
        "threshold": "",
        "threshold_reached": "",
        "trigger_angle": "",
        "fsr_raw": "",
        "current_angle": "",
        "servo_state": "",
    }

    for line in lines:
        parts = line.split(maxsplit=1)
        if len(parts) != 2:
            continue
        label, value = parts
        if label == "R":
            result["threshold_reached"] = "yes" if value == "1" else "no"
        elif label == "A":
            result["trigger_angle"] = value
            result["current_angle"] = value
        elif label == "F":
            result["fsr_raw"] = value
        elif label == "T":
            result["threshold"] = value
        elif label == "THRESH":
            result["threshold"] = value
        elif label == "S":
            result["servo_state"] = value.lower()

    return result


async def send_command(command):
    total_started = time.perf_counter()
    row = {field: "" for field in CSV_FIELDS}
    row.update(
        {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "command": command,
            "device_name": DEVICE_NAME,
            "success": False,
        }
    )

    try:
        device, rssi_samples, scan_ms = await discover_device()
        row["scan_ms"] = f"{scan_ms:.2f}"
        (
            row["rssi_last_dbm"],
            row["rssi_mean_dbm"],
            row["rssi_min_dbm"],
            row["rssi_max_dbm"],
            row["rssi_samples"],
        ) = rssi_summary(rssi_samples)

        if device is None:
            row["error_stage"] = "scan"
            row["error"] = (
                f"Could not find {DEVICE_NAME} or Nordic UART service."
            )
            return row

        row["device_address"] = device.address

        lines = []
        buffer = ""
        notification_count = 0
        first_response_at = None
        command_written_at = None
        completion_event = asyncio.Event()

        def notification_handler(_sender, data):
            nonlocal buffer, notification_count, first_response_at
            notification_count += 1
            if first_response_at is None:
                first_response_at = time.perf_counter()

            buffer += data.decode("utf-8", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if line:
                    lines.append(line)
                    print(f"ESP32: {line}")
                    if command_complete(command, lines):
                        completion_event.set()

        client = BleakClient(device, timeout=CONNECT_TIMEOUT_SECONDS)

        try:
            connect_started = time.perf_counter()
            await client.connect()
            row["connect_ms"] = (
                f"{(time.perf_counter() - connect_started) * 1000:.2f}"
            )

            notify_started = time.perf_counter()
            await client.start_notify(UART_TX_CHAR_UUID, notification_handler)
            row["notify_setup_ms"] = (
                f"{(time.perf_counter() - notify_started) * 1000:.2f}"
            )

            write_started = time.perf_counter()
            await client.write_gatt_char(
                UART_RX_CHAR_UUID,
                f"{command}\n".encode("utf-8"),
                response=False,
            )
            command_written_at = time.perf_counter()
            row["command_write_ms"] = (
                f"{(command_written_at - write_started) * 1000:.2f}"
            )

            wait_started = time.perf_counter()
            try:
                await asyncio.wait_for(
                    completion_event.wait(),
                    timeout=RESPONSE_TIMEOUT_SECONDS,
                )
            except asyncio.TimeoutError:
                pass
            await asyncio.sleep(0.3)
            row["response_wait_ms"] = (
                f"{(time.perf_counter() - wait_started) * 1000:.2f}"
            )

            if buffer.strip():
                lines.append(buffer.strip())

            row["success"] = command_complete(command, lines)

            try:
                await client.stop_notify(UART_TX_CHAR_UUID)
            except Exception:
                pass

        except Exception as error:
            if not row["connect_ms"]:
                row["error_stage"] = "connect"
            elif not row["notify_setup_ms"]:
                row["error_stage"] = "notify_setup"
            elif not row["command_write_ms"]:
                row["error_stage"] = "command_write"
            else:
                row["error_stage"] = "response"
            row["error"] = str(error)
        finally:
            if client.is_connected:
                disconnect_started = time.perf_counter()
                try:
                    await client.disconnect()
                except Exception as error:
                    if not row["error"]:
                        row["error_stage"] = "disconnect"
                        row["error"] = str(error)
                row["disconnect_ms"] = (
                    f"{(time.perf_counter() - disconnect_started) * 1000:.2f}"
                )

        if first_response_at is not None and command_written_at is not None:
            row["time_to_first_response_ms"] = (
                f"{(first_response_at - command_written_at) * 1000:.2f}"
            )

        parsed = parse_response(lines)
        row.update(parsed)
        row["notification_count"] = notification_count
        row["response_line_count"] = len(lines)
        row["response"] = " | ".join(lines)

        if not row["success"] and not row["error"]:
            row["error_stage"] = "response"
            row["error"] = "Expected completion response was not received."

    except Exception as error:
        row["error_stage"] = row["error_stage"] or "unexpected"
        row["error"] = str(error)
    finally:
        row["total_ms"] = (
            f"{(time.perf_counter() - total_started) * 1000:.2f}"
        )

    return row


def normalize_command(user_input):
    value = user_input.strip().upper()
    if value == "":
        return "PRESS"
    if value in {"P", "PRESS"}:
        return "PRESS"
    if value in {"S", "STATUS"}:
        return "STATUS"
    if value in {"F", "FSR"}:
        return "FSR"
    if value in {"O", "OPEN"}:
        return "OPEN"
    if value in {"C", "CLOSE"}:
        return "CLOSE"
    if value in {"X", "STOP"}:
        return "STOP"
    if value in {"Q", "QUIT", "EXIT"}:
        return "QUIT"
    if value.startswith("GOTO "):
        return value
    if value.startswith("THRESH "):
        return value
    return ""


async def async_main():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_directory = (
        Path(__file__).resolve().parent / "elevator_logs" / "ble_normal"
    )
    output_directory.mkdir(parents=True, exist_ok=True)
    csv_path = output_directory / f"normal_elevator_ble_{stamp}.csv"

    print(f"Normal BLE controller: {DEVICE_NAME}")
    print(f"CSV: {csv_path}")
    print()
    print("ENTER/p = PRESS")
    print("s       = STATUS")
    print("f       = FSR")
    print("o       = OPEN")
    print("c       = CLOSE")
    print("x       = STOP")
    print("GOTO 90 = move to an angle")
    print("THRESH 100 = change FSR threshold")
    print("q       = save and quit")
    print()

    with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        csv_file.flush()

        while True:
            user_input = await asyncio.to_thread(input, "BLE command: ")
            command = normalize_command(user_input)
            if command == "QUIT":
                break
            if not command:
                print("Unknown command.")
                continue

            print(f"Sending {command}; scanning and connecting...")
            row = await send_command(command)
            writer.writerow(row)
            csv_file.flush()

            state = "OK" if row["success"] else "FAILED"
            print(
                f"{state}: RSSI {row['rssi_mean_dbm'] or 'unavailable'} dBm, "
                f"connect {row['connect_ms'] or 'n/a'} ms, "
                f"first response {row['time_to_first_response_ms'] or 'n/a'} ms, "
                f"total {row['total_ms']} ms"
            )
            if row["error"]:
                print(f"{row['error_stage']}: {row['error']}")
            print()

    print(f"CSV saved to: {csv_path}")


def main():
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
