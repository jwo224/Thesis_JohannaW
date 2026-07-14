#!/usr/bin/env python3
"""Interactive BLE distance, RSSI, response-time, and press-result logger."""

import argparse
import asyncio
import csv
import re
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


DEVICE_NAME = "XIAO-FSR-SERVO"
DEFAULT_ADDRESS = ""
DEFAULT_DISTANCES = list(range(1, 11))

UART_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
UART_RX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
UART_TX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

COMPLETION_MARKERS = (
    "DONE",
    "Press command finished",
    "PRESS command finished",
)

CSV_FIELDS = [
    "timestamp",
    "distance_m",
    "attempt",
    "command",
    "configured_address",
    "connected_address",
    "device_name",
    "success",
    "scan_before_found",
    "scan_before_attempts",
    "scan_before_ms",
    "rssi_before_last_dbm",
    "rssi_before_mean_dbm",
    "rssi_before_min_dbm",
    "rssi_before_max_dbm",
    "rssi_before_samples",
    "connect_ms",
    "notify_setup_ms",
    "command_write_ms",
    "time_to_first_notification_ms",
    "command_to_completion_ms",
    "response_wait_ms",
    "disconnect_ms",
    "total_attempt_ms",
    "notification_count",
    "response_line_count",
    "response_bytes",
    "completion_marker_received",
    "press_finished",
    "threshold",
    "threshold_reached",
    "trigger_angle",
    "fsr_raw",
    "fsr_voltage",
    "scan_after_found",
    "scan_after_ms",
    "rssi_after_last_dbm",
    "rssi_after_mean_dbm",
    "rssi_after_min_dbm",
    "rssi_after_max_dbm",
    "rssi_after_samples",
    "response",
    "error_stage",
    "error",
]


def rssi_summary(samples):
    if not samples:
        return {
            "last": "",
            "mean": "",
            "min": "",
            "max": "",
            "samples": 0,
        }

    return {
        "last": samples[-1],
        "mean": f"{statistics.fmean(samples):.2f}",
        "min": min(samples),
        "max": max(samples),
        "samples": len(samples),
    }


async def scan_target(address, name, duration):
    target_address = address.upper() if address else ""
    samples = []
    found_device = None
    started = time.perf_counter()

    def detection_callback(device, advertisement_data):
        nonlocal found_device
        advertised_name = advertisement_data.local_name or device.name or ""
        advertised_services = {
            service.lower() for service in (advertisement_data.service_uuids or [])
        }
        address_matches = bool(
            target_address and device.address.upper() == target_address
        )
        name_matches = advertised_name.casefold() == name.casefold()
        service_matches = UART_SERVICE_UUID in advertised_services

        if address_matches or name_matches or service_matches:
            found_device = device
            if advertisement_data.rssi is not None:
                samples.append(advertisement_data.rssi)

    scanner = BleakScanner(detection_callback=detection_callback)
    await scanner.start()
    try:
        await asyncio.sleep(duration)
    finally:
        await scanner.stop()

    elapsed_ms = (time.perf_counter() - started) * 1000
    return found_device, samples, elapsed_ms


def parse_key_values(text):
    values = {}
    for line in text.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key.strip().lower()] = value.strip()
    return values


def parse_ble_press_response(lines, raw_text=""):
    """Parse the compact BLE protocol: DONE, R 1, A n, F n, T n."""
    combined = raw_text or "\n".join(lines)
    values = {
        "press_finished": "yes" if "DONE" in combined else "no",
        "threshold": "",
        "threshold_reached": "",
        "trigger_angle": "",
        "fsr_raw": "",
        "fsr_voltage": "",
    }

    for line in lines:
        parts = line.strip().split(maxsplit=1)
        if len(parts) != 2:
            continue
        label, value = parts
        if label == "R":
            values["threshold_reached"] = "yes" if value == "1" else "no"
        elif label == "A":
            values["trigger_angle"] = value
        elif label == "F":
            values["fsr_raw"] = value
        elif label == "T":
            values["threshold"] = value

    # At weak signal strength, newlines can be lost and messages can arrive as
    # "RETURNDONER 0" or "F 0T 50". Recover the compact fields from raw text.
    matches = re.findall(r"R\s+([01])", combined)
    if matches:
        values["threshold_reached"] = "yes" if matches[-1] == "1" else "no"

    matches = re.findall(r"A\s+(-?\d+)", combined)
    if matches:
        values["trigger_angle"] = matches[-1]

    matches = re.findall(r"F\s+(-?\d+)", combined)
    if matches:
        values["fsr_raw"] = matches[-1]

    matches = re.findall(r"T\s+(-?\d+)", combined)
    if matches:
        values["threshold"] = matches[-1]

    return values


async def run_ble_attempt(args):
    started = time.perf_counter()
    result = {field: "" for field in CSV_FIELDS}
    result.update(
        {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "command": args.command,
            "configured_address": args.address,
            "success": False,
            "completion_marker_received": False,
            "press_finished": "no",
        }
    )

    try:
        device = None
        before_samples = []
        scan_before_ms = 0.0
        scan_attempts = 0

        for scan_attempts in range(1, args.scan_retries + 1):
            scanned_device, samples, elapsed_ms = await scan_target(
                args.address, args.name, args.scan_time
            )
            scan_before_ms += elapsed_ms
            before_samples.extend(samples)
            if scanned_device is not None:
                device = scanned_device
                break

        before = rssi_summary(before_samples)
        result.update(
            {
                "scan_before_found": device is not None,
                "scan_before_attempts": scan_attempts,
                "scan_before_ms": f"{scan_before_ms:.2f}",
                "rssi_before_last_dbm": before["last"],
                "rssi_before_mean_dbm": before["mean"],
                "rssi_before_min_dbm": before["min"],
                "rssi_before_max_dbm": before["max"],
                "rssi_before_samples": before["samples"],
            }
        )
    except Exception as error:
        device = None
        result["error_stage"] = "scan_before"
        result["error"] = str(error)

    if device is None:
        if args.address:
            # A weak peripheral may miss all advertisement windows while its
            # previously discovered address is still directly connectable.
            target = args.address
            result["connected_address"] = args.address
            result["device_name"] = args.name
        else:
            result["error_stage"] = result["error_stage"] or "scan_before"
            result["error"] = result["error"] or (
                f"Could not find BLE device named '{args.name}' or a device "
                f"advertising Nordic UART service {UART_SERVICE_UUID} after "
                f"{args.scan_retries} scan attempts."
            )
            result["total_attempt_ms"] = (
                f"{(time.perf_counter() - started) * 1000:.2f}"
            )
            return result
    else:
        target = device
        result["connected_address"] = device.address
        result["device_name"] = device.name or args.name
        # Remember the current address for later distances. This avoids
        # depending entirely on receiving a weak advertisement every time.
        args.address = device.address


    response_chunks = []
    response_lines = []
    notification_buffer = ""
    first_notification_at = None
    last_notification_at = None
    completion_event = asyncio.Event()
    command_written_at = None
    completion_at = None
    notification_count = 0

    def notification_handler(_sender, data):
        nonlocal notification_buffer
        nonlocal first_notification_at
        nonlocal last_notification_at
        nonlocal notification_count
        nonlocal completion_at

        now = time.perf_counter()
        if first_notification_at is None:
            first_notification_at = now
        last_notification_at = now
        notification_count += 1

        chunk = data.decode("utf-8", errors="replace")
        response_chunks.append(chunk)
        notification_buffer += chunk

        raw_response = "".join(response_chunks)
        if any(marker in raw_response for marker in COMPLETION_MARKERS):
            if completion_at is None:
                completion_at = now
            completion_event.set()

        while "\n" in notification_buffer:
            line, notification_buffer = notification_buffer.split("\n", 1)
            line = line.strip()
            if line:
                response_lines.append(line)
                print(f"    ESP32: {line}")

    client = BleakClient(target, timeout=args.connect_timeout)

    try:
        connect_started = time.perf_counter()
        await client.connect()
        result["connect_ms"] = f"{(time.perf_counter() - connect_started) * 1000:.2f}"

        if not client.is_connected:
            raise RuntimeError("BLE client did not report a connected state")

        notify_started = time.perf_counter()
        await client.start_notify(UART_TX_CHAR_UUID, notification_handler)
        result["notify_setup_ms"] = (
            f"{(time.perf_counter() - notify_started) * 1000:.2f}"
        )

        write_started = time.perf_counter()
        await client.write_gatt_char(
            UART_RX_CHAR_UUID,
            f"{args.command}\n".encode("utf-8"),
            response=False,
        )
        command_written_at = time.perf_counter()
        result["command_write_ms"] = (
            f"{(command_written_at - write_started) * 1000:.2f}"
        )

        response_started = time.perf_counter()
        try:
            await asyncio.wait_for(
                completion_event.wait(),
                timeout=args.response_timeout,
            )
        except asyncio.TimeoutError:
            pass

        # Allow a final notification chunk containing values after the marker.
        await asyncio.sleep(args.final_wait)
        result["response_wait_ms"] = (
            f"{(time.perf_counter() - response_started) * 1000:.2f}"
        )

        if notification_buffer.strip():
            response_lines.append(notification_buffer.strip())

        result["completion_marker_received"] = completion_event.is_set()
        result["press_finished"] = (
            "yes" if completion_event.is_set() else "no"
        )
        result["success"] = (
            completion_event.is_set()
            if args.command.strip().upper() == "PRESS"
            else bool(response_chunks)
        )

        try:
            await client.stop_notify(UART_TX_CHAR_UUID)
        except Exception:
            pass

    except Exception as error:
        if not result["connect_ms"]:
            result["error_stage"] = "connect"
        elif not result["notify_setup_ms"]:
            result["error_stage"] = "notify_setup"
        elif not result["command_write_ms"]:
            result["error_stage"] = "command_write"
        else:
            result["error_stage"] = "response"
        result["error"] = str(error) or repr(error)
    finally:
        if client.is_connected:
            disconnect_started = time.perf_counter()
            try:
                await client.disconnect()
            except Exception as error:
                if not result["error"]:
                    result["error_stage"] = "disconnect"
                    result["error"] = str(error)
            result["disconnect_ms"] = (
                f"{(time.perf_counter() - disconnect_started) * 1000:.2f}"
            )

    if first_notification_at is not None and command_written_at is not None:
        result["time_to_first_notification_ms"] = (
            f"{(first_notification_at - command_written_at) * 1000:.2f}"
        )

    if completion_at is not None and command_written_at is not None:
        result["command_to_completion_ms"] = (
            f"{(completion_at - command_written_at) * 1000:.2f}"
        )

    response_text = "\n".join(response_lines)
    raw_response_text = "".join(response_chunks)
    values = parse_key_values(response_text)
    compact_values = parse_ble_press_response(response_lines, raw_response_text)
    result.update(
        {
            "notification_count": notification_count,
            "response_line_count": len(response_lines),
            "response_bytes": len("".join(response_chunks).encode("utf-8")),
            "press_finished": compact_values["press_finished"],
            "threshold": (
                compact_values["threshold"] or values.get("threshold", "")
            ),
            "threshold_reached": (
                compact_values["threshold_reached"]
                or values.get("threshold_reached", "")
            ),
            "trigger_angle": (
                compact_values["trigger_angle"]
                or values.get("trigger_angle", "")
            ),
            "fsr_raw": compact_values["fsr_raw"] or values.get("fsr_raw", ""),
            "fsr_voltage": (
                compact_values["fsr_voltage"] or values.get("fsr_voltage", "")
            ),
            "response": response_text.replace("\r", " ").replace("\n", " | "),
        }
    )

    try:
        _, after_samples, scan_after_ms = await scan_target(
            result["connected_address"], args.name, args.after_scan_time
        )
        after = rssi_summary(after_samples)
        result.update(
            {
                "scan_after_found": bool(after_samples),
                "scan_after_ms": f"{scan_after_ms:.2f}",
                "rssi_after_last_dbm": after["last"],
                "rssi_after_mean_dbm": after["mean"],
                "rssi_after_min_dbm": after["min"],
                "rssi_after_max_dbm": after["max"],
                "rssi_after_samples": after["samples"],
            }
        )
    except Exception as error:
        if not result["error"]:
            result["error_stage"] = "scan_after"
            result["error"] = str(error)

    result["total_attempt_ms"] = f"{(time.perf_counter() - started) * 1000:.2f}"
    return result


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure BLE RSSI, reliability, and press response by distance."
    )
    parser.add_argument(
        "--address",
        default=DEFAULT_ADDRESS,
        help="Optional BLE address; discovery primarily uses name/service UUID",
    )
    parser.add_argument("--name", default=DEVICE_NAME)
    parser.add_argument("--command", default="PRESS")
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--scan-time", type=float, default=3.0)
    parser.add_argument("--scan-retries", type=int, default=3)
    parser.add_argument("--after-scan-time", type=float, default=2.0)
    parser.add_argument("--connect-timeout", type=float, default=10.0)
    parser.add_argument("--response-timeout", type=float, default=15.0)
    parser.add_argument("--final-wait", type=float, default=0.5)
    parser.add_argument(
        "--distances",
        type=float,
        nargs="+",
        default=DEFAULT_DISTANCES,
    )
    parser.add_argument("--output", type=Path)
    return parser.parse_args()


async def async_main():
    args = parse_args()
    if args.attempts < 1:
        raise SystemExit("--attempts must be at least 1")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or (
        Path(__file__).resolve().parent
        / "elevator_logs"
        / "ble_distance_checks"
        / f"distance_check_ble_{stamp}.csv"
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    address_text = args.address or "auto-discover"
    print(f"BLE distance check: {args.name} ({address_text})")
    print(f"Command: {args.command}")
    print(f"Distances: {', '.join(f'{value:g}' for value in args.distances)} m")
    print(f"CSV: {output}")

    with output.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
        writer.writeheader()
        csv_file.flush()

        for distance in args.distances:
            answer = await asyncio.to_thread(
                input,
                f"\nMove to {distance:g} m, then press Enter "
                "(or type q to finish): ",
            )
            if answer.strip().lower() in {"q", "quit", "stop"}:
                break

            successes = 0
            for attempt in range(1, args.attempts + 1):
                print(f"  Attempt {attempt}/{args.attempts}: scanning and connecting...")
                result = await run_ble_attempt(args)
                result["distance_m"] = distance
                result["attempt"] = attempt
                writer.writerow(result)
                csv_file.flush()

                successes += int(result["success"])
                state = "OK" if result["success"] else "FAILED"
                rssi = (
                    result["rssi_before_mean_dbm"]
                    or result["rssi_after_mean_dbm"]
                    or "unavailable"
                )
                print(
                    f"  {state}: RSSI {rssi} dBm, "
                    f"connect {result['connect_ms'] or 'n/a'} ms, "
                    f"first response "
                    f"{result['time_to_first_notification_ms'] or 'n/a'} ms, "
                    f"command complete "
                    f"{result['command_to_completion_ms'] or 'n/a'} ms, "
                    f"total {result['total_attempt_ms']} ms"
                )
                if result["error"]:
                    print(
                        f"    {result['error_stage']}: {result['error']}"
                    )

            print(
                f"Result at {distance:g} m: "
                f"{successes}/{args.attempts} successful"
            )

    print(f"\nFinished. Results saved to: {output}")


def main():
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("\nStopped by user.")


if __name__ == "__main__":
    main()
