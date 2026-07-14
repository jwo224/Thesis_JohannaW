#!/usr/bin/env python3
"""Normal-mode Wi-Fi controller/logger for the ESP32-C3 button presser."""

import csv
import time
from datetime import datetime
from pathlib import Path

try:
    import requests
except ImportError:
    print("ERROR: requests is not installed.")
    print("Install it with: python3 -m pip install requests")
    raise SystemExit(1)


# ============================== SETTINGS ==============================
ESP32_IP = "192.168.8.226"
REQUEST_TIMEOUT_SECONDS = 15.0
# ======================================================================

CSV_FIELDS = [
    "timestamp",
    "command",
    "url",
    "success",
    "http_status",
    "response_time_ms",
    "response_bytes",
    "press_finished",
    "energy_saving_mode",
    "threshold",
    "threshold_reached",
    "trigger_angle",
    "fsr_raw",
    "fsr_voltage",
    "status",
    "wifi_mode",
    "wifi_ssid",
    "ip",
    "wifi_connected",
    "current_angle",
    "fsr_threshold",
    "last_fsr_raw",
    "last_fsr_voltage",
    "last_trigger_angle",
    "last_press_reached_threshold",
    "wifi_rssi_dbm",
    "response",
    "error",
]


def parse_key_values(text):
    values = {}
    for line in text.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key.strip().lower()] = value.strip()
    return values


def request_esp32(command):
    url = f"http://{ESP32_IP}/{command}"
    started = time.perf_counter()

    try:
        response = requests.get(
            url,
            timeout=REQUEST_TIMEOUT_SECONDS,
            headers={"Connection": "close"},
        )
        elapsed_ms = (time.perf_counter() - started) * 1000
        text = response.text.strip()
        values = parse_key_values(text)

        return {
            "timestamp": datetime.now().isoformat(timespec="milliseconds"),
            "command": command,
            "url": url,
            "success": response.ok,
            "http_status": response.status_code,
            "response_time_ms": f"{elapsed_ms:.2f}",
            "response_bytes": len(response.content),
            "press_finished": "yes" if "Press command finished" in text else "",
            "energy_saving_mode": values.get("energy_saving_mode", ""),
            "threshold": values.get("threshold", ""),
            "threshold_reached": values.get("threshold_reached", ""),
            "trigger_angle": values.get("trigger_angle", ""),
            "fsr_raw": values.get("fsr_raw", ""),
            "fsr_voltage": values.get("fsr_voltage", ""),
            "status": values.get("status", ""),
            "wifi_mode": values.get("wifi_mode", ""),
            "wifi_ssid": values.get("wifi_ssid", ""),
            "ip": values.get("ip", ""),
            "wifi_connected": values.get("wifi_connected", ""),
            "current_angle": values.get("current_angle", ""),
            "fsr_threshold": values.get("fsr_threshold", ""),
            "last_fsr_raw": values.get("last_fsr_raw", ""),
            "last_fsr_voltage": values.get("last_fsr_voltage", ""),
            "last_trigger_angle": values.get("last_trigger_angle", ""),
            "last_press_reached_threshold": values.get(
                "last_press_reached_threshold", ""
            ),
            "wifi_rssi_dbm": values.get("wifi_rssi_dbm", ""),
            "response": text.replace("\r", " ").replace("\n", " | "),
            "error": "" if response.ok else f"HTTP {response.status_code}",
        }, text

    except requests.exceptions.RequestException as error:
        elapsed_ms = (time.perf_counter() - started) * 1000
        row = {field: "" for field in CSV_FIELDS}
        row.update(
            {
                "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                "command": command,
                "url": url,
                "success": False,
                "response_time_ms": f"{elapsed_ms:.2f}",
                "error": str(error),
            }
        )
        return row, ""


def command_from_input(value):
    value = value.strip().lower()
    if value in {"", "p", "press"}:
        return "press"
    if value in {"s", "status"}:
        return "status"
    if value in {"f", "fsr"}:
        return "fsr"
    if value in {"q", "quit", "exit"}:
        return "quit"
    return ""


def main():
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_directory = Path(__file__).resolve().parent / "elevator_logs"
    output_directory.mkdir(parents=True, exist_ok=True)
    csv_path = output_directory / f"normal_elevator_wifi_{stamp}.csv"

    print(f"Normal elevator controller: http://{ESP32_IP}")
    print(f"CSV: {csv_path}")
    print()
    print("ENTER/p = press and log")
    print("s       = status and log")
    print("f       = FSR reading and log")
    print("q       = save and quit")
    print()

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            csv_file.flush()

            while True:
                command = command_from_input(input("Command [ENTER/p/s/f/q]: "))
                if command == "quit":
                    break
                if not command:
                    print("Unknown command. Use ENTER, p, s, f, or q.")
                    continue

                if command == "press":
                    print("Pressing; waiting for the complete servo cycle...")

                row, text = request_esp32(command)
                writer.writerow(row)
                csv_file.flush()

                if row["success"]:
                    print(
                        f"[OK] /{command}: {row['response_time_ms']} ms "
                        f"(HTTP {row['http_status']})"
                    )
                    print(text)
                else:
                    print(
                        f"[FAILED] /{command}: {row['response_time_ms']} ms"
                    )
                    print(row["error"])
                print()

    except KeyboardInterrupt:
        print("\nStopping...")
    finally:
        print(f"CSV saved to: {csv_path}")


if __name__ == "__main__":
    main()
