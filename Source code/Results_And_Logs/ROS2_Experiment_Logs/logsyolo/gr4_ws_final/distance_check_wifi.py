#!/usr/bin/env python3
"""Interactive ESP32-C3 Wi-Fi distance and response-time test."""

import argparse
import csv
import re
import subprocess
import time
from datetime import datetime
from pathlib import Path

import requests


DEFAULT_IP = "192.168.8.226"
DEFAULT_DISTANCES = [1, 3, 5, 8, 10, 15, 20]
RSSI_PATTERN = re.compile(
    r"(?:rssi|signal(?:_strength)?)[^\d-]{0,12}(-?\d+(?:\.\d+)?)\s*(?:dbm)?",
    re.IGNORECASE,
)


def robot_wifi_signal():
    """Return the Linux Wi-Fi link signal; this may be the AP, not the ESP32."""
    try:
        result = subprocess.run(
            ["iw", "dev"],
            check=True,
            capture_output=True,
            text=True,
            timeout=2,
        )
        interfaces = re.findall(r"^\s*Interface\s+(\S+)", result.stdout, re.MULTILINE)
    except (FileNotFoundError, subprocess.SubprocessError):
        return "", "", ""

    for interface in interfaces:
        try:
            result = subprocess.run(
                ["iw", "dev", interface, "link"],
                check=False,
                capture_output=True,
                text=True,
                timeout=2,
            )
        except subprocess.SubprocessError:
            continue

        match = re.search(
            r"signal:\s*(-?\d+(?:\.\d+)?)\s*dBm", result.stdout, re.IGNORECASE
        )
        if match:
            ssid_match = re.search(r"SSID:\s*(.+)", result.stdout)
            ssid = ssid_match.group(1).strip() if ssid_match else ""
            return match.group(1), interface, ssid

    return "", "", ""


def esp32_rssi(response):
    """Extract RSSI from a JSON or text response when the firmware supplies it."""
    try:
        data = response.json()
        if isinstance(data, dict):
            for key, value in data.items():
                if key.lower() in {"rssi", "wifi_rssi", "signal", "signal_strength"}:
                    return value
    except ValueError:
        pass

    match = RSSI_PATTERN.search(response.text)
    return match.group(1) if match else ""


def parse_press_response(text):
    """Extract the values returned after the ESP32 finishes a press command."""
    values = {}
    for line in text.splitlines():
        key, separator, value = line.partition("=")
        if separator:
            values[key.strip().lower()] = value.strip()

    return {
        "press_finished": "yes" if "Press command finished" in text else "no",
        "threshold": values.get("threshold", ""),
        "threshold_reached": values.get("threshold_reached", ""),
        "trigger_angle": values.get("trigger_angle", ""),
        "fsr_raw": values.get("fsr_raw", ""),
        "fsr_voltage": values.get("fsr_voltage", ""),
    }


def test_request(session, url, timeout):
    started = time.perf_counter()
    try:
        response = session.get(url, timeout=timeout)
        elapsed_ms = (time.perf_counter() - started) * 1000
        result = {
            "success": response.ok,
            "elapsed_ms": f"{elapsed_ms:.2f}",
            "http_status": response.status_code,
            "esp32_rssi_dbm": esp32_rssi(response),
            "response": response.text.strip().replace("\r", " ").replace("\n", " "),
            "error": "" if response.ok else f"HTTP {response.status_code}",
        }
        result.update(parse_press_response(response.text))
        return result
    except requests.exceptions.RequestException as error:
        elapsed_ms = (time.perf_counter() - started) * 1000
        return {
            "success": False,
            "elapsed_ms": f"{elapsed_ms:.2f}",
            "http_status": "",
            "esp32_rssi_dbm": "",
            "press_finished": "no",
            "threshold": "",
            "threshold_reached": "",
            "trigger_angle": "",
            "fsr_raw": "",
            "fsr_voltage": "",
            "response": "",
            "error": str(error),
        }


def parse_args():
    parser = argparse.ArgumentParser(
        description="Measure ESP32 Wi-Fi reliability and response time by distance."
    )
    parser.add_argument("--ip", default=DEFAULT_IP, help="ESP32 IP address")
    parser.add_argument("--endpoint", default="press", help="HTTP endpoint to request")
    parser.add_argument("--attempts", type=int, default=1, help="Presses per distance")
    parser.add_argument("--timeout", type=float, default=15.0, help="Press timeout in seconds")
    parser.add_argument(
        "--pause", type=float, default=0.5, help="Seconds between requests"
    )
    parser.add_argument(
        "--distances",
        type=float,
        nargs="+",
        default=DEFAULT_DISTANCES,
        help="Test distances in metres",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="CSV output path (default: elevator_logs/wifi_distance_checks/...)",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    endpoint = args.endpoint.strip("/")
    url = f"http://{args.ip}/{endpoint}"

    if args.attempts < 1:
        raise SystemExit("--attempts must be at least 1")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output = args.output or (
        Path(__file__).resolve().parent
        / "elevator_logs"
        / "wifi_distance_checks"
        / f"distance_check_wifi_{stamp}.csv"
    )
    output.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = [
        "timestamp",
        "distance_m",
        "attempt",
        "url",
        "success",
        "elapsed_ms",
        "http_status",
        "press_finished",
        "threshold",
        "threshold_reached",
        "trigger_angle",
        "fsr_raw",
        "fsr_voltage",
        "esp32_rssi_dbm",
        "robot_wifi_signal_dbm",
        "robot_wifi_interface",
        "robot_wifi_ssid",
        "response",
        "error",
    ]

    print(f"ESP32 Wi-Fi distance check: {url}")
    print(f"{args.attempts} press command(s) per distance; timeout {args.timeout:g} s")
    print(f"CSV: {output}")
    print("ESP32 RSSI is recorded only if the HTTP response contains an RSSI value.")
    print("The robot Wi-Fi signal is the Linux link signal and may refer to the router/AP.")

    session = requests.Session()

    with output.open("w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        csv_file.flush()

        for distance in args.distances:
            answer = input(
                f"\nMove to {distance:g} m, then press Enter to test "
                "(or type q to finish): "
            ).strip().lower()
            if answer in {"q", "quit", "stop"}:
                break

            successful = 0
            threshold_reached_count = 0
            elapsed_values = []

            for attempt in range(1, args.attempts + 1):
                link_rssi, interface, ssid = robot_wifi_signal()
                result = test_request(session, url, args.timeout)
                successful += int(result["success"])
                threshold_reached_count += int(
                    result["threshold_reached"].strip().lower() == "yes"
                )
                if result["success"]:
                    elapsed_values.append(float(result["elapsed_ms"]))

                row = {
                    "timestamp": datetime.now().isoformat(timespec="milliseconds"),
                    "distance_m": distance,
                    "attempt": attempt,
                    "url": url,
                    **result,
                    "robot_wifi_signal_dbm": link_rssi,
                    "robot_wifi_interface": interface,
                    "robot_wifi_ssid": ssid,
                }
                writer.writerow(row)
                csv_file.flush()

                state = "OK" if result["success"] else "FAILED"
                rssi = result["esp32_rssi_dbm"] or link_rssi or "unavailable"
                print(
                    f"  {attempt}/{args.attempts}: {state}, "
                    f"{result['elapsed_ms']} ms, signal {rssi} dBm"
                )
                if result["success"]:
                    reached = result["threshold_reached"] or "not reported"
                    print(
                        f"    threshold reached: {reached}; "
                        f"threshold={result['threshold'] or 'n/a'}, "
                        f"angle={result['trigger_angle'] or 'n/a'}, "
                        f"fsr_raw={result['fsr_raw'] or 'n/a'}, "
                        f"fsr_voltage={result['fsr_voltage'] or 'n/a'}"
                    )
                if result["error"]:
                    print(f"    {result['error']}")

                if attempt < args.attempts:
                    time.sleep(args.pause)

            average = (
                f"{sum(elapsed_values) / len(elapsed_values):.2f} ms"
                if elapsed_values
                else "n/a"
            )
            print(
                f"Result at {distance:g} m: {successful}/{args.attempts} succeeded, "
                f"threshold reached {threshold_reached_count}/{args.attempts}, "
                f"average completed response {average}"
            )

            if successful == 0:
                answer = input(
                    "All requests failed. Press Enter to continue farther, "
                    "or type q to finish: "
                ).strip().lower()
                if answer in {"q", "quit", "stop"}:
                    break

    print(f"\nFinished. Results saved to {output}")


if __name__ == "__main__":
    main()
