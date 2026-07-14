#!/usr/bin/env python3
"""Wi-Fi controller/logger for the ESP32-C3 elevator button presser."""

import csv
import time
from datetime import datetime
from pathlib import Path

try:
    import requests
except ImportError:
    print("ERROR: The requests package is not installed.")
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
    "calibration_finished",
    "trial",
    "energy_saving_mode",
    "threshold",
    "current_threshold",
    "threshold_reached",
    "trigger_angle",
    "fsr_raw",
    "fsr_voltage",
    "wifi_rssi_dbm",
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
    "last_trial_valid",
    "operator_answer",
    "successful_threshold",
    "old_threshold",
    "new_threshold",
    "max_threshold_exceeded",
    "next_step",
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


def request_esp32(path, command=None, operator_answer=""):
    command = command or path
    url = f"http://{ESP32_IP}/{path}"
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
            "press_finished": (
                "yes" if "Press command finished" in text else ""
            ),
            "calibration_finished": values.get("calibration_finished", ""),
            "trial": values.get("trial", ""),
            "energy_saving_mode": values.get("energy_saving_mode", ""),
            "threshold": values.get("threshold", ""),
            "current_threshold": values.get("current_threshold", ""),
            "threshold_reached": values.get("threshold_reached", ""),
            "trigger_angle": values.get("trigger_angle", ""),
            "fsr_raw": values.get("fsr_raw", ""),
            "fsr_voltage": values.get("fsr_voltage", ""),
            "wifi_rssi_dbm": values.get("wifi_rssi_dbm", ""),
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
            "last_trial_valid": values.get("last_trial_valid", ""),
            "operator_answer": operator_answer,
            "successful_threshold": values.get("successful_threshold", ""),
            "old_threshold": values.get("old_threshold", ""),
            "new_threshold": values.get("new_threshold", ""),
            "max_threshold_exceeded": values.get("max_threshold_exceeded", ""),
            "next_step": values.get("next_step", ""),
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
                "operator_answer": operator_answer,
                "error": str(error),
            }
        )
        return row, ""


def show_result(row, text):
    print()
    if row["success"]:
        print(
            f"[OK] /{row['command']} completed in "
            f"{row['response_time_ms']} ms (HTTP {row['http_status']})"
        )
        if text:
            print(text)
    else:
        print(
            f"[FAILED] /{row['command']} after "
            f"{row['response_time_ms']} ms"
        )
        print(row["error"])
    print()


def command_from_input(user_input):
    value = user_input.strip().lower()
    if value in {"", "p", "press"}:
        return "press"
    if value in {"s", "status"}:
        return "status"
    if value in {"f", "fsr"}:
        return "fsr"
    if value in {"r", "reset"}:
        return "reset_calibration"
    if value in {"q", "quit", "exit"}:
        return "quit"
    return ""


def write_request(writer, csv_file, path, command=None, operator_answer=""):
    row, text = request_esp32(path, command, operator_answer)
    writer.writerow(row)
    csv_file.flush()
    show_result(row, text)
    return row


def ask_trial_result():
    while True:
        answer = input(
            "Was the physical elevator-button press successful? [y/n]: "
        ).strip().lower()
        if answer in {"y", "yes"}:
            return "yes"
        if answer in {"n", "no"}:
            return "no"
        print("Please type y or n.")


def main():
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    script_directory = Path(__file__).resolve().parent
    csv_path = script_directory / f"fsr_calibration_wifi_{timestamp}.csv"

    print(f"ESP32-C3 Wi-Fi controller: http://{ESP32_IP}")
    print(f"CSV will be saved to: {csv_path}")
    print()
    print("Commands:")
    print("  ENTER or p  Press the elevator button and log the result")
    print("  s           Read and log /status")
    print("  f           Read and log /fsr")
    print("  r           Reset adaptive calibration to threshold 20")
    print("  q           Save and quit")
    print()

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=CSV_FIELDS)
            writer.writeheader()
            csv_file.flush()

            while True:
                try:
                    user_input = input("Command [ENTER/p/s/f/r/q]: ")
                except EOFError:
                    break

                command = command_from_input(user_input)
                if command == "quit":
                    break
                if not command:
                    print("Unknown command. Use ENTER, p, s, f, r, or q.")
                    continue

                if command == "press":
                    print("Sending /press; waiting for the servo cycle to finish...")
                    press_row = write_request(writer, csv_file, "press")

                    if not press_row["success"] or press_row["press_finished"] != "yes":
                        continue

                    answer = ask_trial_result()
                    print(f"Sending calibration result: {answer}...")
                    result_row = write_request(
                        writer,
                        csv_file,
                        f"result?success={answer}",
                        command="result",
                        operator_answer=answer,
                    )

                    if answer == "yes" and result_row["success"]:
                        print(
                            "Calibration accepted and finished. "
                            f"Threshold: {result_row['successful_threshold'] or press_row['threshold']}"
                        )
                    elif answer == "no" and result_row["success"]:
                        print(
                            "Trial rejected. Next threshold: "
                            f"{result_row['new_threshold'] or 'not reported'}"
                        )
                else:
                    print(f"Sending /{command}...")
                    write_request(writer, csv_file, command)

    except KeyboardInterrupt:
        print("\nStopping logger...")
    except OSError as error:
        print(f"ERROR: Could not create or write the CSV file: {error}")
        raise SystemExit(1)
    finally:
        print(f"CSV saved to: {csv_path}")


if __name__ == "__main__":
    main()
